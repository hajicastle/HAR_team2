# Final Project — Design of Wearable Robot Assistance Strategy
### Bilateral EMG-Gated, FSR-Phase-Shaped Hip-Extension Assist for Stair Ascent

**Course:** Human Assistive Robotics (ME491A), Department of Mechanical Engineering, KAIST
**Team:** Team 2 **Author:** Ahmed Hamza Dahioui
**Platform:** Angel Suit H10 hip exoskeleton + XM10 rev 2.0 expansion module (firmware branch `ghlee_02`)
**Date:** 2026-06-03

> **Status note on data.** This report fully specifies the scenario, algorithm, control law, evaluation metrics, and experimental protocol. The Problem 2(b) plot and the Problem 3 result numbers must be produced from a recorded lab trial (the firmware streams every required signal; the analysis pipeline is provided in the appendix). Where numeric outcomes are anticipated, they are labelled **[expected — literature-grounded]** and paired with a blank **[measured]** column to be filled after the lab session. No simulated value is presented as a measurement.

---

## Abstract

We design a hip-extension assistance strategy for **stair ascent**. A 4-channel FSR insole drives a fuzzy gait-phase detector that identifies the **pull-up (weight-acceptance) phase**; a 2-channel surface EMG measures **hamstring (hip-extensor) activation**. During the pull-up window the robot delivers a smooth, phase-shaped hip-extension torque pulse whose magnitude is proportional to the user's own hamstring effort (assist-as-needed), capped at the firmware safety ceiling of 2.5 N·m and ramped/slew-limited for comfort. The whole chain — scenario → FSR/EMG state estimation → phase-gated proportional torque → evaluation metric → two-condition experiment — is built to be logically self-consistent, which is the stated objective of the project rather than controller perfection.

---

## Problem 1 — Assistance Scenario and Evaluation Metric

### 1(a) Assistance scenario

- **Target user.** Adults whose stair-climbing ability is *preserved but costly*: older adults (60+) with mild gluteal sarcopenia, early post-operative rehabilitation patients (e.g. 4–12 weeks post hip/knee arthroplasty), and workers who repeatedly climb loaded stairs. The defining feature is that the user *can* climb stairs unaided but pays a disproportionately high per-step muscular cost.
- **Motion.** Continuous stair ascent on a stair treadmill at a self-selected cadence (~30 stair-cycles/min, i.e. ~1 step/s per leg), in 3–5 minute bouts.
- **Difficulty experienced.** Stair ascent imposes a hip-extensor moment of roughly **1.5× body weight**, versus ~0.5× BW in level walking (Ch.5-4). The demand is concentrated in the brief **pull-up phase** (~5–30 % of the stair cycle), where the hip extensors (gluteus maximus and hamstrings) must extend the hip to lift the body onto the next step. For the target users this phase is the limiting factor — it drives early fatigue, trunk lean, and reluctance to use stairs.
- **Why robotic assistance is needed.** A passive spring cannot deliver a *timed* extension burst synchronized to each step, and the assistance must scale with how hard the user is actually working (to remain assist-as-needed rather than motion-replacing). The H10 is bilateral, hip-only, and back-drivable — anatomically matched to the hip-extension task with no knee/ankle coupling. Its 2.5 N·m working ceiling is ~10 % of the biological hip-extensor peak, so it *supplements* the hip-extensor group during the short pull-up window rather than overriding the user.
- **Expected effect of assistance.** A reduction in hamstring (hip-extensor) activation during pull-up (literature on hip-extension exosuits reports 10–40 % reductions in extensor effort; Mooney 2014, Lee 2017), delayed onset of fatigue across a bout, maintained cadence, and reduced compensatory trunk lean — at a torque small enough to be perceived as natural.

### 1(b) Evaluation metric

Following the Team 2 TA feedback, **no heart-rate metric is used** (extra HR sensors are not permitted, for fairness). All four metrics are computed from sensor/robot data already streamed by the firmware. M1 is the primary outcome; M2–M4 are process/quality/guard checks that make the causal chain auditable.

| ID | Metric | What it measures | How it is computed | Better |
|----|--------|------------------|--------------------|--------|
| **M1 (primary)** | **Normalized hamstring EMG RMS** | Pull-up muscular effort the assist is meant to offload | Per stair cycle, RMS of the hamstring EMG **envelope** over the pull-up window φ∈[0.05, 0.30], normalized by per-leg **MVIC** (`EMG_MVIC_R/L`) → %MVC. Report per-cycle mean ± SD over the steady-state window. | **Lower** |
| **M2** | **Assistance timing error** | Whether the torque fires at the right moment | Δt = (time of first sample with \|τ_cmd\| > 0.1 N·m) − (time of FSR loading-response event), per cycle. Report median and 95th percentile of \|Δt\|. | **Smaller \|Δt\|** (target median < 30 ms, p95 < 80 ms) |
| **M3** | **Commanded torque — magnitude & smoothness** | That assistance is delivered, and is smooth/safe | Peak \|τ_cmd\| per cycle (target band 1.5–2.5 N·m), and jerk RMS `J = sqrt(mean((dτ/dt)²))` (target < 30 N·m/s). | Magnitude **in band**; jerk **lower** |
| **M4** | **Encoder-based hip motion** | That assistance does not distort natural gait | From the H10 thigh-angle encoder: peak hip extension per cycle, thigh-angle ROM, and cadence. Compare baseline vs assist. | **Unchanged** (assist should not deform kinematics; small extension increase acceptable) |

**Rationale.** M1 directly quantifies the scenario goal (offloading the hip extensors during pull-up, measured at the hamstring). M2 verifies the FSR phase gate fires the torque in the intended window — if M1 improves but M2 is poor, the improvement is not attributable to correct timing. M3 confirms the robot actually delivered safe, smooth torque (a null result with near-zero torque would be uninformative). M4 guards against the failure mode of "improvement by gait modification" — we want the user to climb the *same way* with less effort. Statistical comparison: paired t-test (Wilcoxon if N ≥ 5), α = 0.05, within-subject.

---

## Problem 2 — Algorithm Design and Implementation

### 2(a) State-estimation algorithm

**Sensors used.**
- **External — 4-ch FSR insole** (PF5 R-heel, PF6 R-toe, PF7 L-heel, PF8 L-toe): gait phase / events.
- **External — 2-ch sEMG** (PF3 R hamstring / biceps femoris, PF4 L hamstring, SENIAM placement): effort magnitude.
- **Internal — H10 thigh-angle encoder** (`rightThighAngle`, `leftThighAngle`) and **pelvic tilt** (`pelvicAngle`): safety gating and the M4 kinematic metric. Wearer body data is sent at startup via `XM_SendUserBodyData()` so these H10-estimated angles are accurate.

**Signal processing.**
- *FSR* (per channel, 1 kHz): voltage → first-order low-pass (fc = 8 Hz) → two-point calibration `load = clamp((v_lpf − off)/(on − off), 0, 1.5)` (off = unloaded capture, on = loaded capture, span floored at 0.05 V).
- *EMG* (per channel, 1 kHz): voltage → resting-bias removal → pre-rectification LPF (fc = 80 Hz) → full-wave rectification → envelope LPF (fc = 5 Hz) → deadband (0.02 V) → MVIC normalization → first-order **activation dynamics** `a[k] = γ·a[k−1] + (1−γ)·(env/MVIC)`, γ = 0.95 (electromechanical-delay model, from the sEMG demo / HW5).

**State estimation rule.**
1. **Fuzzy gait phase (per foot), as an ordered FSM.** Each foot's heel and toe loads pass through tanh membership functions `μ_large(x) = ½(tanh(s·(x − θ)) + 1)` (s = `fuzzy_sensitivity` = 12, θ = 0.35) to make robust "loaded" decisions (the upgrade over hard thresholds — Ch.5-4, Demo_FSR). These drive a **single-direction finite-state machine** `SWING → HEEL_STRIKE → LOAD_RESPONSE → TERMINAL_STANCE → SWING`, anchored on foot contact, with a 60 ms minimum dwell per phase (FSM structure per `examples/17_FSM_Gait_Intent`). Single-direction + dwell guarantees `LOAD_RESPONSE` (the pull-up trigger) fires **exactly once per stair cycle** — re-armed only by passing through `SWING` — avoiding the chatter / out-of-order firing a per-sample max-membership argmax can produce.
2. **Stair-cycle phase clock (per leg).** Entry into `LOAD_RESPONSE` is the pull-up trigger: it resets a continuous phase `φ ∈ [0,1)` to 0 and estimates the stair-cycle period from the interval between consecutive ipsilateral loading-response events, clamped to [1.0, 4.0] s. Between events, `φ` advances at `dt/period`.
3. **Activation estimate.** `a_ham_R/L` from the EMG pipeline above.

The fused state per leg is `(φ, gait_phase, a_ham)`, plus a bilateral 4-bit contact mask and a standing flag (mask = all-contact held ≥ 0.7 s).

**Why this fits the scenario.** The FSR fuzzy detector gives a noise-robust, calibration-tolerant identification of the exact phase (pull-up) where hip-extensor demand peaks — providing the *timing*. The EMG activation provides the *magnitude*, keeping the device assist-as-needed. The continuous phase clock lets us shape a smooth torque pulse instead of a hard on/off step.

### 2(b) Assistance input and control law

**Control law (per leg ζ ∈ {R, L}):**

```
τ_cmd_ζ(t) = − G_φ(φ_ζ) · K_EMG · a_ham_ζ · Ramp(t) · 1[ safe_ζ ]
```

- **Leading minus sign → hip extension** (negative on the H10 convention) — the hip extensors the robot supplements (gluteus maximus and hamstrings) extend the hip to pull the body upward.
- **Phase envelope (F-vector pulse):** `G_φ(φ) = x·exp(1 − x)`, `x = (φ − 0.05)/0.10`, nonzero only for φ ∈ [0.05, 0.35], **peak = 1 at φ = 0.15** (mid pull-up). C¹-smooth, single-peaked, grounded in motor-recruitment shape.
- **Magnitude:** `K_EMG · a_ham`, with `K_EMG = 2.0 N·m` (start low), scaled by activation a ∈ [0, ~1].
- **Direction & magnitude of assistance:** hip extension, peak ~1.5–2.5 N·m at full pull-up effort.

**Activation / deactivation conditions** (`safe_ζ`, all must hold or torque = 0):
`control_ON == 1` · all four calibrations done (`calibration_ready`) · H10 mode == ASSIST · first loading-response seen · thigh angle ∈ [−10°, 90°] · stair period ∈ [1, 4] s · not standing · `pelvicAngle ≥ pelvic_incline_min_deg` (default permissive; can be raised to refuse assistance on level ground). Releasing ASSIST mode, a BTN3 long-press, or CM disconnect immediately zero the torque.

**Safety elements.**
- Hard saturation to extension-only `[−2.5, 0] N·m` (`HARD_MAX_ASSIST_TORQUE_NM`, the instructor ceiling); the student limit `assist_torque_limit_nm` (default 2.0) is additionally clamped under it.
- Slew-rate limit `|dτ/dt| ≤ 30 N·m/s`.
- Ramp-in over 2 s on ACTIVE entry; symmetric ramp-out via the same gate.
- Pure feedforward (no integrator → no wind-up); 1 kHz loop.

**Required plot — sensors, estimated state, hip angle, commanded torque on a shared time axis.**
Generated from a recorded trial by `final_project_analysis.py --plot` (6-row figure, ~15 s window): Row 1–2 FSR loads + hysteresis thresholds, Row 3 hamstring EMG envelope (%MVC), Row 4 stair-cycle phase φ_R/φ_L with loading-response markers, Row 5 thigh angle + pelvic tilt, Row 6 commanded extension torque with the pull-up window shaded. **→ Insert `plots/E1_subjectXX_6row.png` here after the lab trial.**

---

## Problem 3 — Experimental Validation and Evaluation

### 3(a) Experimental design (≥ 2 conditions)

- **B0 — baseline:** robot worn, `control_ON = 0` (zero torque). Isolates the assistance effect from the mass/donning penalty.
- **E1 — assist:** `control_ON = 1`, `K_EMG = 2.0`, `assist_torque_limit_nm = 2.0`.
- *(Optional E2 — conservative gain `K_EMG = 1.0`)* for a gain comparison if time permits.

**Subjects & protocol.** N = 2–3, ages 20–35, BMI 18–28; safety harness recommended on the stair treadmill. Session (~80 min): don device (6 min) → MVIC (3×5 s/leg, prone hip-extended) → BTN1 EMG bias + MVIC, BTN2 FSR off + on calibration → familiarization (3 min) → two 4-min trials, **counterbalanced order**. For each condition record the full 16-channel CSV; segment on right loading-response; drop first 20 s + last 10 s; require ≥ 80 steady-state cycles. Compute M1–M4 per condition.

**Quality gates:** mean |τ_cmd| in E1 > 0.5 N·m; ≤ 5 safety trips; no fall/harness catch.

### 3(b) Evaluation, limitations, and improvement

**Result table (fill after lab).** Direction expectations are literature-grounded, not measurements.

| Metric | B0 [measured] | E1 [measured] | Δ | Expected direction [literature] |
|--------|---------------|---------------|---|----------------------------------|
| M1 hamstring peak %MVC (pull-up) | ____ | ____ | ____ | E1 lower by ~20–35 % |
| M1 hamstring RMS %MVC | ____ | ____ | ____ | E1 lower |
| M2 timing error median / p95 (ms) | n/a | ____ | — | median < 30 ms, p95 < 80 ms |
| M3 peak \|τ_cmd\| (N·m) | 0 | ____ | — | ~1.5–2.5 N·m, in band |
| M3 jerk RMS (N·m/s) | 0 | ____ | — | < 30 |
| M4 peak hip extension (deg) | ____ | ____ | ____ | unchanged or slight ↑ |
| M4 cadence (cycles/min) | ____ | ____ | ____ | unchanged (±2) |

**Assessment criteria.** The scenario is fulfilled if M1 drops significantly (paired test, p < 0.05) **while** M2 confirms correct pull-up timing, M3 confirms safe smooth torque, and M4 confirms cadence/kinematics are preserved (effort reduced without changing how the user climbs). The numbers must be consistent with the 2(b) plot — e.g. the EMG-envelope reduction in Row 3 should coincide with the shaded torque windows in Row 6.

**Limitations (≥ 1) and concrete improvements.**
1. **Fatigue-induced EMG decay weakens proportional assist when most needed.** As the muscle fatigues, EMG amplitude can fall even as effort rises, shrinking `a_ham` and thus the assist. *Improvement:* add a fatigue-compensation factor from a rolling EMG median-frequency estimate, `Φ(t) = clamp(f_med0 / max(f_med, 0.5·f_med0), 1.0, 2.0)`, multiplying the torque so assistance is maintained as spectral fatigue sets in.
2. **Stair loading-response detection** can mis-fire on metallic stair plates / metatarsal-first contact. *Improvement:* require a ≥ 50 ms contact-bit hold before accepting a loading-response, and/or fuse the hip-encoder angle threshold (`Final_Encoder_ex01` style) as a confirming trigger (TA-suggested).
3. **Pelvic incline gate semantics.** `pelvicAngle` is a tilt estimate, not a true stair-incline signal; default gate is permissive. *Improvement:* learn a per-session pelvic-pitch baseline at quiet standing and gate a few degrees above it, or use the per-hip IMU sagittal pitch.

---

## Appendix A — Firmware ↔ report mapping

- **Source:** `XM_Apps/User_Algorithm/user_app.c` (this is the file the build compiles; `CMakeLists.txt` line 240).
- **Channel map:** PF3/PF4 = R/L hamstring EMG; PF5/PF6 = R heel/toe FSR; PF7/PF8 = L heel/toe FSR (DIO_1..6 → ADC_5..10, 5 V).
- **Key tunables (Live Expressions):** `control_ON`, `K_EMG`, `assist_torque_limit_nm`, `EMG_MVIC_R/L`, `fuzzy_heel_threshold`, `fuzzy_toe_threshold`, `fuzzy_sensitivity`, `pelvic_incline_min_deg`, `cdc_stream_period_ms`, `use_assist_level_scale` (0 = deterministic full torque; 1 = scale by the H10 assist-level button, for demos).
- **Verified API correctness (vs. previous-lab code):** uses `rightThighAngle`/`leftThighAngle` (not `thighAngleR/L`), `pelvicAngle` (not `pelvicTilt`), and the 2.5 N·m firmware ceiling — the three mismatches that broke the old `user_app.c` on `ghlee_02`.

## Appendix B — Data logging & analysis workflow

1. Flash `user_app.c` (CubeIDE import, or VSCode + CMake: `cmake --preset Debug && cmake --build --preset Debug`).
2. Calibrate: BTN1 (EMG bias → MVIC), BTN2 (FSR off → on), confirm `calibration_ready == 1`.
3. Record: `python PythonDecoder/CDC/cdc_selective_logger.py`, select module `0xF0`, choose the **`Final_Stair_Assist`** preset (added for this project — 16 channels: EMG R/L env, EMG R/L act, FSR RH/RT/LH/LT, Phase R/L, Gait R/L, Thigh R/L, Tau R/L). Save `B0_subjectXX.csv` and `E1_subjectXX.csv`.
4. Analyse/plot: `final_project_analysis.py` reads this exact 16-channel schema (`pc_time_s, EMG R env, …, Tau R, Tau L`), detects stair cycles from the phase-clock resets, and emits the M1–M4 table + the Problem 2(b) 6-row figure. It has been run end-to-end on synthetic data (table + both plots generated); only the real recorded CSVs are still needed.

## Appendix C — Build status

The firmware **builds clean** with the STM32CubeIDE 2.1.1 bundled toolchain (arm-none-eabi-gcc 14.3, CMake 3.30, Ninja): `cmake --preset Debug && cmake --build --preset Debug` → `Extension_Module.elf` / `.bin` (FLASH 30 %, RAM_D2 99 %). `user_app.c` compiles with no errors/warnings. One stock-repo fix was required to link: the top-level `Core/Src/system_stm32h7xx.c` was missing the CubeMX-6.13+ `ExitRun0Mode()` power-supply shim that `startup_stm32h743xx.s` calls (the function exists in the `XM10_SDK/Rev1.1` copy and is documented in the CHANGELOG) — it was added back. The build needs `pyyaml` in the Python used by the pre-build codegen step.

Two refinements were folded in from `examples/17_FSM_Gait_Intent` (TA-recommended) and re-verified to build clean: (1) the pull-up trigger is now an **ordered single-direction gait FSM** with a 60 ms per-phase dwell (fires once per cycle, no argmax chatter); (2) `XM_SendUserBodyData()` is called at startup so the H10 thigh/pelvic-angle estimates (safety bounds + M4 metric) are accurate. A `use_assist_level_scale` flag (default 0) optionally scales torque by the suit's assist-level button for live demos without affecting the deterministic experiment torque. On-device USB-MSC logging was deliberately not added — the Python CDC logger is the data path and no spare button is free.

## Appendix D — Control-law parameter summary

| Parameter | Symbol / macro | Value |
|-----------|----------------|-------|
| Loop rate | `CONTROL_DT_S` | 1 ms (1 kHz) |
| EMG pre-LPF / envelope LPF | `EMG_PRE_LPF_FC_HZ` / `EMG_ENV_LPF_FC_HZ` | 80 / 5 Hz |
| EMG activation dynamics | `EMG_ACT_GAMMA` (γ) | 0.95 |
| FSR LPF | `FSR_LPF_FC_HZ` | 8 Hz |
| Fuzzy threshold / sensitivity | `fuzzy_*_threshold` / `fuzzy_sensitivity` | 0.35 / 12 |
| Pulse window / peak | `PHASE_PULSE_START`, `_TP`, `_WINDOW` | φ ∈ [0.05, 0.35], peak 0.15 |
| Gain / student limit / hard cap | `K_EMG` / `assist_torque_limit_nm` / `HARD_MAX_ASSIST_TORQUE_NM` | 2.0 / 2.0 / 2.5 N·m |
| Slew / ramp-in | `SLEW_RATE_LIMIT_NM_PER_S` / `RAMP_DURATION_S` | 30 N·m/s / 2 s |
| Stair period (init/min/max) | `STAIR_PERIOD_*_S` | 2 / 1 / 4 s |
