"""Final Project (Stair Ascent) — metric computation + plotting for the
`Final_Stair_Assist` CDC schema.

Reads a CSV produced by `PythonDecoder/CDC/cdc_selective_logger.py` with the
`Final_Stair_Assist` preset selected (Module 0xF0, 16 float channels streamed by
`XM_Apps/User_Algorithm/user_app.c`). Metadata columns `pc_time_s, seq_id,
module_id, status, tx_drops` precede the 16 named channels:

    EMG R env, EMG L env, EMG R act, EMG L act,
    FSR RH, FSR RT, FSR LH, FSR LT,
    Phase R, Phase L, Gait R, Gait L,
    Thigh R, Thigh L, Tau R, Tau L          (Tau negative = extension)

Cycle boundaries are detected from the stair-cycle phase clock: the firmware
resets `Phase R/L` to 0 at each loading-response (pull-up start), so a downward
jump in phase marks a new cycle — no separate event column is needed.

Metrics (Team-2 approved set; no heart rate):
  M1 — gluteus-maximus %MVC during pull-up (peak + RMS), from the MVIC-normalized
       activation channel  (%MVC = 100 * act)
  M2 — assistance timing error: |t(torque onset) - t(loading-response)|
  M3 — commanded torque: peak |tau| (band 1.5-2.5 N.m) + jerk RMS (< 30 N.m/s)
  M4 — encoder hip motion: thigh-angle peak/ROM + cadence
  + the 6-row shared-time-axis figure required by Problem 2(b)
  + paired t-test (B0 vs E1) across subjects when >= 2 CSVs are supplied

Usage
-----
    python final_project_analysis.py --csv B0_subject01.csv --labels B0
    python final_project_analysis.py --csv B0_subject01.csv E1_subject01.csv \\
        --labels B0 E1 --plot
    python final_project_analysis.py --csv data/B0_*.csv data/E1_*.csv --paired-test

Requires: numpy (scipy optional for stats, matplotlib optional for plots).

Author: Team 2 — Ahmed Hamza Dahioui
Date:   2026-06-03
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# Pull-up window in stair-cycle phase units (matches PHASE_PULSE_START/WINDOW in
# user_app.c; M1 is integrated over the gluteal power-generation window).
PULLUP_PHASE_LO = 0.05
PULLUP_PHASE_HI = 0.30

# Phase pulse window for plot shading (= PHASE_PULSE_START .. START + 3*TP).
PULSE_SHADE_LO = 0.05
PULSE_SHADE_HI = 0.35

TORQUE_ONSET_EPS_NM = 0.1


# =============================================================================
# Column mapping (logger display names -> internal keys)
# =============================================================================

COLUMN_MAP = {
    "time":      "pc_time_s",
    "emg_R_env": "EMG R env",
    "emg_L_env": "EMG L env",
    "emg_R_act": "EMG R act",
    "emg_L_act": "EMG L act",
    "fsr_RH":    "FSR RH",
    "fsr_RT":    "FSR RT",
    "fsr_LH":    "FSR LH",
    "fsr_LT":    "FSR LT",
    "phase_R":   "Phase R",
    "phase_L":   "Phase L",
    "gait_R":    "Gait R",
    "gait_L":    "Gait L",
    "thigh_R":   "Thigh R",
    "thigh_L":   "Thigh L",
    "tau_R":     "Tau R",
    "tau_L":     "Tau L",
}


@dataclass
class TrialData:
    label: str
    path: str
    dt: float
    t: np.ndarray
    emg_R_env: np.ndarray
    emg_L_env: np.ndarray
    emg_R_act: np.ndarray
    emg_L_act: np.ndarray
    fsr_RH: np.ndarray
    fsr_RT: np.ndarray
    fsr_LH: np.ndarray
    fsr_LT: np.ndarray
    phase_R: np.ndarray
    phase_L: np.ndarray
    thigh_R: np.ndarray
    thigh_L: np.ndarray
    tau_R: np.ndarray
    tau_L: np.ndarray


@dataclass
class TrialResult:
    label: str
    path: str
    n_cycles: int
    duration_s: float
    # M1 (primary)
    e_peak_pctMVC_R: float
    e_peak_pctMVC_L: float
    e_rms_pctMVC_R: float
    e_rms_pctMVC_L: float
    # M2
    timing_err_ms_median: float
    timing_err_ms_p95: float
    n_timing_pairs: int
    # M3
    tau_peak_nm_abs: float
    jerk_rms_nm_per_s: float
    # M4
    cadence_cycles_per_min: float
    thigh_peak_ext_deg_R: float   # min thigh angle reached per cycle (most extended)
    thigh_rom_deg_R: float


# =============================================================================
# CSV loading
# =============================================================================

def _to_float(s: Optional[str]) -> float:
    if s is None or s == "":
        return float("nan")
    try:
        return float(s)
    except ValueError:
        return float("nan")


def load_csv(path: Path, label: str) -> TrialData:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"CSV has no rows: {path}")

    headers = {h.strip(): h for h in rows[0].keys()}
    missing = [disp for disp in COLUMN_MAP.values() if disp not in headers]
    if missing:
        raise ValueError(
            f"{path}: missing expected columns {missing}. "
            f"Record with the 'Final_Stair_Assist' preset (all 16 channels checked)."
        )

    def col(disp_name: str) -> np.ndarray:
        key = headers[disp_name]
        return np.asarray([_to_float(r.get(key)) for r in rows])

    t = col(COLUMN_MAP["time"])
    diffs = np.diff(t)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
    dt = float(np.median(diffs)) if len(diffs) else 0.01

    return TrialData(
        label=label, path=str(path), dt=dt, t=t,
        emg_R_env=col(COLUMN_MAP["emg_R_env"]),
        emg_L_env=col(COLUMN_MAP["emg_L_env"]),
        emg_R_act=col(COLUMN_MAP["emg_R_act"]),
        emg_L_act=col(COLUMN_MAP["emg_L_act"]),
        fsr_RH=col(COLUMN_MAP["fsr_RH"]), fsr_RT=col(COLUMN_MAP["fsr_RT"]),
        fsr_LH=col(COLUMN_MAP["fsr_LH"]), fsr_LT=col(COLUMN_MAP["fsr_LT"]),
        phase_R=col(COLUMN_MAP["phase_R"]), phase_L=col(COLUMN_MAP["phase_L"]),
        thigh_R=col(COLUMN_MAP["thigh_R"]), thigh_L=col(COLUMN_MAP["thigh_L"]),
        tau_R=col(COLUMN_MAP["tau_R"]), tau_L=col(COLUMN_MAP["tau_L"]),
    )


# =============================================================================
# Detectors
# =============================================================================

def cycle_start_indices(phase: np.ndarray, drop: float = 0.5) -> np.ndarray:
    """Loading-response indices = downward jumps of the phase clock (reset to 0)."""
    out: List[int] = []
    for i in range(1, len(phase)):
        if np.isfinite(phase[i]) and np.isfinite(phase[i - 1]):
            if phase[i] - phase[i - 1] < -drop:
                out.append(i)
    return np.asarray(out, dtype=int)


def torque_onset_times(t: np.ndarray, tau: np.ndarray,
                       eps: float = TORQUE_ONSET_EPS_NM) -> np.ndarray:
    """Times where |tau| first crosses eps after being below (extension = negative)."""
    out: List[float] = []
    for i in range(1, len(tau)):
        if abs(tau[i]) > eps and abs(tau[i - 1]) <= eps:
            out.append(float(t[i]))
    return np.asarray(out)


def crop_steady_state(t: np.ndarray, drop_start_s: float = 20.0,
                      drop_end_s: float = 10.0) -> np.ndarray:
    if len(t) == 0:
        return np.zeros(0, dtype=bool)
    return (t >= t[0] + drop_start_s) & (t <= t[-1] - drop_end_s)


# =============================================================================
# Metric computation
# =============================================================================

def compute_metrics(d: TrialData) -> TrialResult:
    mask_ss = crop_steady_state(d.t)
    if mask_ss.sum() < 10:
        mask_ss = np.ones_like(d.t, dtype=bool)

    t = d.t[mask_ss]
    phase_R = d.phase_R[mask_ss]
    act_R = d.emg_R_act[mask_ss]
    act_L = d.emg_L_act[mask_ss]
    thigh_R = d.thigh_R[mask_ss]
    tau_R = d.tau_R[mask_ss]
    tau_L = d.tau_L[mask_ss]

    starts = cycle_start_indices(phase_R)
    cycles = [(starts[k], starts[k + 1]) for k in range(len(starts) - 1)]
    n_cycles = len(cycles)
    duration_s = float(t[-1] - t[0]) if len(t) else 0.0

    e_peak_R: List[float] = []
    e_peak_L: List[float] = []
    e_rms_R: List[float] = []
    e_rms_L: List[float] = []
    tau_peak_abs: List[float] = []
    thigh_ext: List[float] = []
    thigh_rom: List[float] = []

    tau_dot_R = np.gradient(tau_R, t) if len(tau_R) > 1 else np.zeros_like(tau_R)
    tau_dot_L = np.gradient(tau_L, t) if len(tau_L) > 1 else np.zeros_like(tau_L)
    jerk_vals: List[float] = []

    for (i0, i1) in cycles:
        seg = slice(i0, i1)
        if i1 - i0 < 5:
            continue
        pull = (phase_R[seg] >= PULLUP_PHASE_LO) & (phase_R[seg] <= PULLUP_PHASE_HI)
        if pull.sum() >= 2:
            aR = act_R[seg][pull]
            aL = act_L[seg][pull]
            e_peak_R.append(100.0 * float(np.nanmax(aR)))
            e_peak_L.append(100.0 * float(np.nanmax(aL)))
            e_rms_R.append(100.0 * float(np.sqrt(np.nanmean(aR ** 2))))
            e_rms_L.append(100.0 * float(np.sqrt(np.nanmean(aL ** 2))))
        tau_peak_abs.append(float(np.nanmax(np.abs(
            np.concatenate([tau_R[seg], tau_L[seg]])))))
        jerk_vals.append(float(np.sqrt(np.nanmean(tau_dot_R[seg] ** 2))))
        jerk_vals.append(float(np.sqrt(np.nanmean(tau_dot_L[seg] ** 2))))
        if np.any(np.isfinite(thigh_R[seg])):
            thigh_ext.append(float(np.nanmin(thigh_R[seg])))
            thigh_rom.append(float(np.nanmax(thigh_R[seg]) - np.nanmin(thigh_R[seg])))

    # M2 — timing: torque onset vs loading-response (cycle start) time
    lr_t = t[starts] if len(starts) else np.empty(0)
    onset_t = torque_onset_times(t, tau_R)
    timing_ms: List[float] = []
    for tt in lr_t:
        if len(onset_t) == 0:
            continue
        diff = onset_t - tt
        forward = diff[diff >= -0.05]  # accept up to 50 ms early
        if len(forward):
            timing_ms.append(float(np.min(np.abs(forward)) * 1000.0))

    def _mean(xs: List[float]) -> float:
        return float(np.mean(xs)) if xs else float("nan")

    cadence = 60.0 * n_cycles / max(duration_s, 1e-6)

    return TrialResult(
        label=d.label, path=d.path, n_cycles=n_cycles, duration_s=duration_s,
        e_peak_pctMVC_R=_mean(e_peak_R), e_peak_pctMVC_L=_mean(e_peak_L),
        e_rms_pctMVC_R=_mean(e_rms_R), e_rms_pctMVC_L=_mean(e_rms_L),
        timing_err_ms_median=(float(np.median(timing_ms)) if timing_ms else float("nan")),
        timing_err_ms_p95=(float(np.percentile(timing_ms, 95)) if timing_ms else float("nan")),
        n_timing_pairs=len(timing_ms),
        tau_peak_nm_abs=_mean(tau_peak_abs),
        jerk_rms_nm_per_s=_mean(jerk_vals),
        cadence_cycles_per_min=float(cadence),
        thigh_peak_ext_deg_R=_mean(thigh_ext),
        thigh_rom_deg_R=_mean(thigh_rom),
    )


# =============================================================================
# Output table
# =============================================================================

def print_table(results: List[TrialResult]) -> None:
    keys = [
        ("n_cycles", "{:>12d}"),
        ("duration_s", "{:>12.1f}"),
        ("e_peak_pctMVC_R", "{:>12.2f}"),
        ("e_peak_pctMVC_L", "{:>12.2f}"),
        ("e_rms_pctMVC_R", "{:>12.2f}"),
        ("e_rms_pctMVC_L", "{:>12.2f}"),
        ("timing_err_ms_median", "{:>12.2f}"),
        ("timing_err_ms_p95", "{:>12.2f}"),
        ("tau_peak_nm_abs", "{:>12.2f}"),
        ("jerk_rms_nm_per_s", "{:>12.2f}"),
        ("cadence_cycles_per_min", "{:>12.2f}"),
        ("thigh_peak_ext_deg_R", "{:>12.2f}"),
        ("thigh_rom_deg_R", "{:>12.2f}"),
    ]
    header = f"{'metric':24s} | " + " | ".join(f"{r.label:>12s}" for r in results)
    print(header)
    print("-" * len(header))
    for key, fmt in keys:
        cells: List[str] = []
        for r in results:
            v = getattr(r, key)
            try:
                cells.append(fmt.format(v))
            except (TypeError, ValueError):
                cells.append(f"{'nan':>12s}")
        print(f"{key:24s} | " + " | ".join(cells))


# =============================================================================
# 6-row figure required by Problem 2(b)
# =============================================================================

def plot_six_row_figure(d: TrialData, out_path: Path,
                        start_s: float = 40.0, window_s: float = 15.0) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib required for plotting", file=sys.stderr)
        return

    mask = (d.t >= start_s) & (d.t < start_s + window_s)
    if mask.sum() < 10:
        mask = np.zeros_like(d.t, dtype=bool)
        mask[: int(window_s / max(d.dt, 1e-6))] = True
    t = d.t[mask]
    if len(t) == 0:
        print("No samples in chosen window", file=sys.stderr)
        return
    tr = t - t[0]

    fig, ax = plt.subplots(6, 1, figsize=(12, 14), sharex=True)

    # Row 1: FSR loads
    for arr, lab in [(d.fsr_RH, "RH"), (d.fsr_RT, "RT"), (d.fsr_LH, "LH"), (d.fsr_LT, "LT")]:
        ax[0].plot(tr, arr[mask], linewidth=1.0, label=lab)
    ax[0].set_ylabel("FSR load"); ax[0].set_title("Row 1: FSR normalized loads")
    ax[0].grid(True, alpha=0.3); ax[0].legend(loc="upper right", fontsize=8)

    # Row 2: FSR + fuzzy threshold
    for arr, lab in [(d.fsr_RH, "RH"), (d.fsr_RT, "RT"), (d.fsr_LH, "LH"), (d.fsr_LT, "LT")]:
        ax[1].plot(tr, arr[mask], linewidth=1.0, label=lab)
    ax[1].axhline(0.35, color="red", linestyle=":", alpha=0.6, label="thr=0.35")
    ax[1].set_ylabel("load"); ax[1].set_ylim(-0.05, 1.6)
    ax[1].set_title("Row 2: FSR loads + fuzzy threshold")
    ax[1].grid(True, alpha=0.3); ax[1].legend(loc="upper right", fontsize=8)

    # Row 3: EMG %MVC (= 100 * activation)
    ax[2].plot(tr, 100.0 * d.emg_R_act[mask], linewidth=1.2, label="R glute %MVC")
    ax[2].plot(tr, 100.0 * d.emg_L_act[mask], linewidth=1.2, label="L glute %MVC")
    ax[2].set_ylabel("%MVC"); ax[2].set_title("Row 3: Gluteus-maximus activation (%MVC)")
    ax[2].grid(True, alpha=0.3); ax[2].legend(loc="upper right", fontsize=8)

    # Row 4: phase + loading-response markers
    ax[3].plot(tr, d.phase_R[mask], linewidth=1.2, label="phase R")
    ax[3].plot(tr, d.phase_L[mask], linewidth=1.2, label="phase L")
    for idx in cycle_start_indices(d.phase_R[mask]):
        ax[3].axvline(tr[idx], color="tab:red", alpha=0.4, linewidth=0.8)
    ax[3].set_ylabel("phase"); ax[3].set_ylim(-0.05, 1.05)
    ax[3].set_title("Row 4: stair-cycle phase + loading-response (red)")
    ax[3].grid(True, alpha=0.3); ax[3].legend(loc="upper right", fontsize=8)

    # Row 5: thigh angle
    ax[4].plot(tr, d.thigh_R[mask], linewidth=1.2, label="thigh R")
    ax[4].plot(tr, d.thigh_L[mask], linewidth=1.2, label="thigh L")
    ax[4].set_ylabel("deg"); ax[4].set_title("Row 5: hip thigh angle (encoder)")
    ax[4].grid(True, alpha=0.3); ax[4].legend(loc="upper right", fontsize=8)

    # Row 6: torque + shaded pull-up windows
    ax[5].plot(tr, d.tau_R[mask], linewidth=1.4, label="tau R (ext, neg)")
    ax[5].plot(tr, d.tau_L[mask], linewidth=1.4, label="tau L (ext, neg)")
    pr = d.phase_R[mask]
    active = (pr >= PULSE_SHADE_LO) & (pr < PULSE_SHADE_HI)
    in_run = False; run0 = 0
    for i, p in enumerate(active):
        if p and not in_run:
            in_run = True; run0 = i
        elif not p and in_run:
            in_run = False; ax[5].axvspan(tr[run0], tr[i - 1], alpha=0.15, color="tab:red")
    if in_run:
        ax[5].axvspan(tr[run0], tr[-1], alpha=0.15, color="tab:red")
    ax[5].set_ylabel("N.m"); ax[5].set_xlabel(f"time since t={t[0]:.1f}s")
    ax[5].set_title("Row 6: commanded extension torque + pull-up windows (R)")
    ax[5].grid(True, alpha=0.3); ax[5].legend(loc="upper right", fontsize=8)

    fig.suptitle(f"Trial: {d.label} — {Path(d.path).name}", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"saved {out_path}")


# =============================================================================
# Paired statistics (B0 vs E1 across subjects)
# =============================================================================

def paired_test(b0_vals: List[float], e1_vals: List[float], label: str) -> None:
    arr_b0 = np.asarray(b0_vals, dtype=float)
    arr_e1 = np.asarray(e1_vals, dtype=float)
    valid = np.isfinite(arr_b0) & np.isfinite(arr_e1)
    arr_b0, arr_e1 = arr_b0[valid], arr_e1[valid]
    n = len(arr_b0)
    if n < 2:
        print(f"{label:24s} | insufficient paired data (N={n})")
        return
    pct = 100.0 * np.mean(arr_e1 - arr_b0) / max(abs(np.mean(arr_b0)), 1e-9)
    try:
        from scipy import stats
        t_stat, p_t = stats.ttest_rel(arr_b0, arr_e1)
        line = f"t={t_stat:6.3f} p={p_t:.4f}"
    except ImportError:
        line = "(scipy not installed)"
    print(f"{label:24s} | N={n:2d} | B0={np.mean(arr_b0):8.3f} | "
          f"E1={np.mean(arr_e1):8.3f} | d={pct:+6.2f}% | {line}")


def run_paired_analysis(results: List[TrialResult]) -> None:
    b0: Dict[str, TrialResult] = {}
    e1: Dict[str, TrialResult] = {}
    for r in results:
        stem = Path(r.path).stem
        if r.label == "B0" or "B0" in stem:
            b0[stem.replace("B0_", "").replace("B0", "")] = r
        elif r.label == "E1" or "E1" in stem:
            e1[stem.replace("E1_", "").replace("E1", "")] = r
    common = sorted(set(b0) & set(e1))
    if not common:
        print("\nNo matched B0/E1 pairs (expected names B0_subjectXX.csv / E1_subjectXX.csv)")
        return
    print(f"\nPaired analysis across {len(common)} subject(s):")
    print("-" * 96)
    for key in ["e_peak_pctMVC_R", "e_rms_pctMVC_R", "tau_peak_nm_abs",
                "jerk_rms_nm_per_s", "cadence_cycles_per_min", "thigh_rom_deg_R"]:
        paired_test([getattr(b0[s], key) for s in common],
                    [getattr(e1[s], key) for s in common], key)


# =============================================================================
# Entry point
# =============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(description="Final Project stair-ascent metrics + plots")
    ap.add_argument("--csv", nargs="+", required=True, help="Trial CSV file(s).")
    ap.add_argument("--labels", nargs="+", default=None, help="Label per CSV (else inferred).")
    ap.add_argument("--plot", action="store_true", help="Write the 6-row figure per CSV.")
    ap.add_argument("--plot-start", type=float, default=40.0)
    ap.add_argument("--plot-window", type=float, default=15.0)
    ap.add_argument("--out-dir", default="plots")
    ap.add_argument("--paired-test", action="store_true")
    args = ap.parse_args()

    csv_paths = [Path(p) for p in args.csv]
    labels = args.labels or [
        "B0" if "B0" in p.stem else ("E1" if "E1" in p.stem else p.stem) for p in csv_paths
    ]
    if len(labels) != len(csv_paths):
        print("--labels must match --csv count", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    results: List[TrialResult] = []
    for path, lbl in zip(csv_paths, labels):
        d = load_csv(path, lbl)
        results.append(compute_metrics(d))
        if args.plot:
            plot_six_row_figure(d, out_dir / f"{path.stem}_6row.png",
                                start_s=args.plot_start, window_s=args.plot_window)

    print()
    print_table(results)
    if args.paired_test or (len(results) >= 2
                            and any(r.label == "B0" for r in results)
                            and any(r.label == "E1" for r in results)):
        run_paired_analysis(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
