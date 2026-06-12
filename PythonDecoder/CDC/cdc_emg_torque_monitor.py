"""
XM10 EMG + Torque-decomposition live monitor — Stair-Ascent Final Project (Team 2).

Reads the 16-channel Final_Stair_Assist stream on USB-CDC module 0xF0 and plots
the EMG measurement AND the full torque expression broken into its factors:

    tau_z = - G(phi_z) * K_EMG * a_z * Ramp(t)        [extension, negative N.m]

Stream slots used (zero-indexed):
    0 EMG R env   1 EMG L env   2 EMG R act   3 EMG L act
    8 Phase R     9 Phase L     14 Tau R      15 Tau L

G(phi) and the magnitude product are reconstructed on the PC with the exact
firmware math (final_user_app.c). K_EMG / limit are passed in (the PC can't read
the live firmware globals over CDC); set them to match your Live-Expression
values. Ramp is ~1 a couple seconds after ACTIVE entry, so the reconstructed
command (-min(mag,limit)) should overlay the streamed Tau in steady state.

Run:
    python cdc_emg_torque_monitor.py --port COM4 --kemg 2.0 --limit 2.0
    python cdc_emg_torque_monitor.py --list-ports

Requires: pyserial, PyQt5, pyqtgraph.
"""

import argparse
import csv
import math
import queue
import struct
import sys
import threading
import time
from datetime import datetime

import serial
import serial.tools.list_ports
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

PHAI_SOF = 0xAA
PHAI_HEADER_SIZE = 6
PHAI_CRC_SIZE = 2
DEFAULT_BAUD = 921600
MODULE_ID = 0xF0
N_WIN = 1000            # rolling window (samples) ~10 s @ 100 Hz

# CSV schema — IDENTICAL to cdc_selective_logger.py (Final_Stair_Assist) so the
# recording is interchangeable and works directly in final_project_analysis.py.
STREAM_CHANNELS = [
    "EMG R env", "EMG L env", "EMG R act", "EMG L act",
    "FSR RH", "FSR RT", "FSR LH", "FSR LT",
    "Phase R", "Phase L", "Gait R", "Gait L",
    "Thigh R", "Thigh L", "Tau R", "Tau L",
]
CSV_HEADER = ["pc_time_s", "seq_id", "module_id", "status", "tx_drops"] + STREAM_CHANNELS
BASE_TITLE = "XM10 EMG + Torque (Team 2) — press R to start/stop recording"
HARD_MAX = 2.5          # firmware HARD_MAX_ASSIST_TORQUE_NM (for plot Y-scales)

# ---- phase envelope constants (final_user_app.c) ----
P_START = 0.05
P_TP = 0.10
P_WIN = 0.30


def phase_env(phi):
    rel = phi - P_START
    if rel <= 0.0 or rel >= P_WIN:
        return 0.0
    x = rel / P_TP
    return x * math.exp(1.0 - x)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(encoded: bytes) -> bytes:
    out = bytearray(); idx = 0
    while idx < len(encoded):
        code = encoded[idx]; idx += 1
        if code == 0:
            raise ValueError("zero in COBS frame")
        end = idx + code - 1
        if end > len(encoded):
            raise ValueError("truncated COBS frame")
        out.extend(encoded[idx:end]); idx = end
        if code < 0xFF and idx < len(encoded):
            out.append(0)
    return bytes(out)


def parse_phai_frame(frame: bytes):
    decoded = cobs_decode(frame)
    if len(decoded) < PHAI_HEADER_SIZE + PHAI_CRC_SIZE or decoded[0] != PHAI_SOF:
        return None
    len_units = decoded[1]
    total = PHAI_HEADER_SIZE + len_units * 4 + PHAI_CRC_SIZE
    if len_units == 0 or len(decoded) < total:
        return None
    pe = PHAI_HEADER_SIZE + len_units * 4
    crc_rx = decoded[pe] | (decoded[pe + 1] << 8)
    if crc_rx != crc16_ccitt(decoded[:pe]):
        return None
    seq = decoded[2] | (decoded[3] << 8)
    mid = decoded[4]
    status = decoded[5]
    vals = struct.unpack(f"<{len_units}f", decoded[PHAI_HEADER_SIZE:pe])
    return seq, mid, status, vals


def reader_thread(port, baud, q, stop):
    """Robust reader: auto-reconnects if the USB-CDC port drops/re-enumerates."""
    while not stop.is_set():
        use_port = port
        if use_port in (None, "", "AUTO", "auto"):
            avail = [p.device for p in serial.tools.list_ports.comports()]
            use_port = avail[0] if avail else None
        if use_port is None:                       # no COM yet -> keep window open, rescan
            for _ in range(15):
                if stop.is_set():
                    break
                time.sleep(0.1)
            continue
        ser = None
        try:
            ser = serial.Serial(use_port, baud, timeout=0.02)
            ser.dtr = True; ser.rts = True
            ser.reset_input_buffer()
            ser.write(b"AGRB MON START\r\n")
            print(f"[reader] connected to {use_port}")
            buf = bytearray()
            while not stop.is_set():
                try:
                    chunk = ser.read(4096)
                except (serial.SerialException, OSError) as e:
                    print(f"[reader] port dropped ({e}); will retry...")
                    break
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    d = buf.find(b"\x00")
                    if d < 0:
                        break
                    raw = bytes(buf[:d]); del buf[:d + 1]
                    if not raw:
                        continue
                    try:
                        pkt = parse_phai_frame(raw)
                    except ValueError:
                        pkt = None
                    if not pkt:
                        continue
                    seq, mid, status, vals = pkt
                    if mid == MODULE_ID and len(vals) >= 16:
                        q.put((time.perf_counter(), seq, mid, status, vals))
        except (serial.SerialException, OSError) as e:
            print(f"[reader] open failed ({e}); retrying in 1.5 s...")
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        # wait before reconnect attempt (responsive to stop)
        for _ in range(15):
            if stop.is_set():
                break
            time.sleep(0.1)


class Monitor(pg.GraphicsLayoutWidget):
    def __init__(self, q, k_emg, limit, record_path=None,
                 onset_mode="angle", pullup_angle=40.0, pullup_end=5.0, base=0.0, rate=100.0):
        super().__init__(show=False)        # embedded in MainWindow (button bar + plots)
        self.setMinimumSize(1000, 820)
        self.q = q
        self.k_emg = k_emg
        self.limit = limit
        self.base = base                    # match firmware assist_base_nm (fixed floor)
        self.rec_dt = 1.0 / rate            # uniform CSV time base (avoids bursty USB timestamps)
        self.onset_mode = onset_mode        # 'angle' (matches firmware use_angle_onset=1) or 'phase'
        self.pullup_angle = pullup_angle
        self.pullup_end = pullup_end
        self._thr_prev = [None, None]       # for thigh-rate (extending) detection
        self._pu = [False, False]           # reconstructed pull-up latch (R, L)

        # --- recording state (CSV schema matches cdc_selective_logger.py) ---
        self.recording = False
        self.writer = None
        self._csv_fh = None
        self.t0 = None
        self.rec_count = 0
        self.rec_path = None
        self.on_state = None   # callback(recording: bool, path) set by MainWindow
        self.setWindowTitle(BASE_TITLE)
        self._pending_record_path = record_path

        self.buf = {k: np.zeros(N_WIN) for k in
                    ("fsrRH", "fsrRT", "fsrLH", "fsrLT",
                     "envR", "envL", "actR", "actL", "GR", "GL",
                     "magR", "magL", "tauR", "tauL", "cmdR", "cmdL")}
        self.x = np.arange(N_WIN)

        R = pg.mkPen("#d62728", width=2)   # right
        L = pg.mkPen("#1f77b4", width=2)   # left
        Rd = pg.mkPen("#d62728", width=1, style=QtCore.Qt.DashLine)
        Ld = pg.mkPen("#1f77b4", width=1, style=QtCore.Qt.DashLine)
        lim = pg.mkPen("#888888", width=1, style=QtCore.Qt.DotLine)
        # FSR pens (4 distinct colours)
        pRH = pg.mkPen("#d62728", width=2)   # right heel  - red
        pRT = pg.mkPen("#ff7f0e", width=2)   # right toe   - orange
        pLH = pg.mkPen("#1f77b4", width=2)   # left heel   - blue
        pLT = pg.mkPen("#17becf", width=2)   # left toe    - cyan

        def plot(title, ylabel):
            p = self.addPlot(title=title); p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", ylabel); p.addLegend(offset=(10, 5))
            self.nextRow()
            return p

        pf = plot("FSR loads (after swap: RH=PF6, RT=PF5, LH=PF4, LT=PF3)", "0..1.5")
        self.c_fsrRH = pf.plot(self.x, self.buf["fsrRH"], pen=pRH, name="RH")
        self.c_fsrRT = pf.plot(self.x, self.buf["fsrRT"], pen=pRT, name="RT")
        self.c_fsrLH = pf.plot(self.x, self.buf["fsrLH"], pen=pLH, name="LH")
        self.c_fsrLT = pf.plot(self.x, self.buf["fsrLT"], pen=pLT, name="LT")
        pf.addLine(y=0.35, pen=lim)   # fuzzy_heel/toe_threshold

        p1 = plot("EMG envelope (measurement)", "V")
        self.c_envR = p1.plot(self.x, self.buf["envR"], pen=R, name="EMG R env")
        self.c_envL = p1.plot(self.x, self.buf["envL"], pen=L, name="EMG L env")

        p2 = plot("a = EMG activation (/MVIC)", "0..1")
        self.c_actR = p2.plot(self.x, self.buf["actR"], pen=R, name="a_R")
        self.c_actL = p2.plot(self.x, self.buf["actL"], pen=L, name="a_L")

        p3title = ("angle-onset gate (thigh >= %g deg)" % self.pullup_angle
                   if self.onset_mode == "angle" else "G(phi) = phase envelope")
        p3 = plot(p3title, "0..1")
        self.c_GR = p3.plot(self.x, self.buf["GR"], pen=R,
                            name=("gate_R" if self.onset_mode == "angle" else "G(phi_R)"))
        self.c_GL = p3.plot(self.x, self.buf["GL"], pen=L,
                            name=("gate_L" if self.onset_mode == "angle" else "G(phi_L)"))

        self._gate_lbl = "gate(angle)" if self.onset_mode == "angle" else "G(phi)"
        self.p4 = p4 = plot(self._mag_title(), "N.m")
        self.c_magR = p4.plot(self.x, self.buf["magR"], pen=R, name="mag_R")
        self.c_magL = p4.plot(self.x, self.buf["magL"], pen=L, name="mag_L")
        self._p4_limit_line = p4.addLine(y=self.limit, pen=lim)

        p5 = plot("Torque: Tau streamed (solid) vs reconstructed -min(mag,limit) (dashed)", "N.m")
        self.c_tauR = p5.plot(self.x, self.buf["tauR"], pen=R, name="Tau R")
        self.c_tauL = p5.plot(self.x, self.buf["tauL"], pen=L, name="Tau L")
        self.c_cmdR = p5.plot(self.x, self.buf["cmdR"], pen=Rd, name="recon R")
        self.c_cmdL = p5.plot(self.x, self.buf["cmdL"], pen=Ld, name="recon L")

        # Fixed Y-scales (consistent with the FSR monitor / phai_receiver conventions)
        pf.setYRange(0.0, 1.55)     # FSR load  (firmware clamps 0..1.5)
        p1.setYRange(0.0, 2.0)      # EMG envelope (V)
        p2.setYRange(0.0, 1.5)      # activation (~0..1 once MVIC is calibrated)
        p3.setYRange(0.0, 1.1)      # gate / G(phi)
        p4.setYRange(0.0, HARD_MAX) # magnitude (N.m), up to the hard ceiling
        p5.setYRange(-HARD_MAX, 1.6)  # torque: extension (neg) .. optional flexion (+1.5)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(40)

        if self._pending_record_path:
            self._start_recording(self._pending_record_path)

    def _mag_title(self):
        return (f"magnitude = (base + K_EMG*a)*{self._gate_lbl}   "
                f"(base={self.base:g}, K_EMG={self.k_emg:g}, limit={self.limit:g})")

    def set_recon_params(self, k_emg, base, limit):
        """Update reconstruction params (to match the robot's live values) + refresh labels."""
        self.k_emg = k_emg
        self.base = base
        self.limit = limit
        self.p4.setTitle(self._mag_title())
        self._p4_limit_line.setValue(limit)

    def _toggle_recording(self):
        if self.recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self, path=None):
        if self.recording:
            return
        if path is None:
            path = f"emg_torque_{datetime.now():%Y%m%d_%H%M%S}.csv"
        try:
            self._csv_fh = open(path, "w", newline="", encoding="utf-8")
        except OSError as e:
            print(f"[rec] cannot open {path}: {e}")
            return
        self.writer = csv.writer(self._csv_fh)
        self.writer.writerow(CSV_HEADER)
        self.t0 = None
        self.rec_count = 0
        self.rec_path = path
        self.recording = True
        self.setWindowTitle(f"● REC  {path}   (press R to stop)")
        print(f"[rec] recording -> {path}")
        if self.on_state:
            self.on_state(True, path)

    def _stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        try:
            self._csv_fh.flush()
            self._csv_fh.close()
        except Exception:
            pass
        self.writer = None
        print(f"[rec] stopped ({self.rec_count} rows) -> {self.rec_path}")
        self.setWindowTitle(BASE_TITLE)
        if self.on_state:
            self.on_state(False, self.rec_path)

    def _push(self, key, v):
        b = self.buf[key]; b[:-1] = b[1:]; b[-1] = v

    def _angle_gate(self, i, thigh, gait):
        """Reconstruct the firmware angle-onset latch from streamed Thigh + Gait."""
        prev = self._thr_prev[i]
        extending = (prev is not None) and (thigh < prev - 0.02)
        self._thr_prev[i] = thigh
        swing = (round(gait) == 3)          # GAIT_PHASE_SWING == 3
        if (not self._pu[i]) and thigh >= self.pullup_angle and extending:
            self._pu[i] = True
        if self._pu[i] and (thigh <= self.pullup_end or swing):
            self._pu[i] = False
        return 1.0 if self._pu[i] else 0.0

    def update(self):
        drained = 0
        while True:
            try:
                item = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            pc_t, seq, mid, status, vals = item
            if self.recording and self.writer is not None:
                self.writer.writerow(
                    [f"{self.rec_count * self.rec_dt:.4f}", seq, mid, status, status & 0x7F]
                    + [f"{v:.6f}" for v in vals[:16]])
                self.rec_count += 1
            fsrRH, fsrRT, fsrLH, fsrLT = vals[4], vals[5], vals[6], vals[7]
            envR, envL, actR, actL = vals[0], vals[1], vals[2], vals[3]
            phiR, phiL = vals[8], vals[9]
            gaitR, gaitL = vals[10], vals[11]
            thR, thL = vals[12], vals[13]
            tauR, tauL = vals[14], vals[15]
            if self.onset_mode == "angle":
                GR = self._angle_gate(0, thR, gaitR)
                GL = self._angle_gate(1, thL, gaitL)
            else:
                GR, GL = phase_env(phiR), phase_env(phiL)
            magR = min((self.base + self.k_emg * actR) * GR, self.limit)
            magL = min((self.base + self.k_emg * actL) * GL, self.limit)
            for k, v in (("fsrRH", fsrRH), ("fsrRT", fsrRT), ("fsrLH", fsrLH), ("fsrLT", fsrLT),
                         ("envR", envR), ("envL", envL), ("actR", actR), ("actL", actL),
                         ("GR", GR), ("GL", GL), ("magR", magR), ("magL", magL),
                         ("tauR", tauR), ("tauL", tauL), ("cmdR", -magR), ("cmdL", -magL)):
                self._push(k, v)
        if drained:
            self.c_fsrRH.setData(self.x, self.buf["fsrRH"]); self.c_fsrRT.setData(self.x, self.buf["fsrRT"])
            self.c_fsrLH.setData(self.x, self.buf["fsrLH"]); self.c_fsrLT.setData(self.x, self.buf["fsrLT"])
            self.c_envR.setData(self.x, self.buf["envR"]); self.c_envL.setData(self.x, self.buf["envL"])
            self.c_actR.setData(self.x, self.buf["actR"]); self.c_actL.setData(self.x, self.buf["actL"])
            self.c_GR.setData(self.x, self.buf["GR"]);     self.c_GL.setData(self.x, self.buf["GL"])
            self.c_magR.setData(self.x, self.buf["magR"]); self.c_magL.setData(self.x, self.buf["magL"])
            self.c_tauR.setData(self.x, self.buf["tauR"]); self.c_tauL.setData(self.x, self.buf["tauL"])
            self.c_cmdR.setData(self.x, self.buf["cmdR"]); self.c_cmdL.setData(self.x, self.buf["cmdL"])


class MainWindow(QtWidgets.QWidget):
    """Container: condition selector (B0/E1) + subject + Record button over the plot grid."""
    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor
        self.setWindowTitle("XM10 FSR + EMG + Torque monitor (Team 2)")
        self.resize(1150, 940)

        self.cond = QtWidgets.QComboBox()
        self.cond.addItems(["B0  (no assist, control_ON=0)",
                            "E1  (assist, control_ON=1)"])
        self.subj = QtWidgets.QLineEdit("01")
        self.subj.setMaximumWidth(70)
        self.btn = QtWidgets.QPushButton("Record  (R)")
        self.btn.setMinimumWidth(130)
        self.btn.setStyleSheet("font-weight:bold; padding:6px 14px;")
        self.btn.clicked.connect(self._toggle)
        self.status = QtWidgets.QLabel("pick condition + subject, then Record -> file B0_/E1_subjectNN.csv")

        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Condition:"))
        bar.addWidget(self.cond)
        bar.addWidget(QtWidgets.QLabel("Subject:"))
        bar.addWidget(self.subj)
        bar.addWidget(self.btn)
        bar.addWidget(self.status, 1)

        # reconstruction params — editable so you can match the robot's live values
        self.sp_k = QtWidgets.QDoubleSpinBox(); self.sp_k.setRange(0.0, 10.0); self.sp_k.setSingleStep(0.1)
        self.sp_k.setDecimals(2); self.sp_k.setValue(self.monitor.k_emg); self.sp_k.setPrefix("K_EMG ")
        self.sp_b = QtWidgets.QDoubleSpinBox(); self.sp_b.setRange(0.0, 2.5); self.sp_b.setSingleStep(0.1)
        self.sp_b.setDecimals(2); self.sp_b.setValue(self.monitor.base); self.sp_b.setPrefix("base ")
        self.sp_l = QtWidgets.QDoubleSpinBox(); self.sp_l.setRange(0.0, 2.5); self.sp_l.setSingleStep(0.1)
        self.sp_l.setDecimals(2); self.sp_l.setValue(self.monitor.limit); self.sp_l.setPrefix("limit ")
        for _sp in (self.sp_k, self.sp_b, self.sp_l):
            _sp.valueChanged.connect(self._params_changed)
        bar2 = QtWidgets.QHBoxLayout()
        bar2.addWidget(QtWidgets.QLabel("Reconstruction (set to match robot live values):"))
        bar2.addWidget(self.sp_k); bar2.addWidget(self.sp_b); bar2.addWidget(self.sp_l)
        bar2.addStretch(1)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.addLayout(bar)
        lay.addLayout(bar2)
        lay.addWidget(self.monitor)

        sc = QtWidgets.QShortcut(QtGui.QKeySequence("R"), self)
        sc.setContext(QtCore.Qt.ApplicationShortcut)
        sc.activated.connect(self._toggle)

        self.monitor.on_state = self._on_state
        self._on_state(self.monitor.recording, self.monitor.rec_path)

    def _params_changed(self):
        self.monitor.set_recon_params(self.sp_k.value(), self.sp_b.value(), self.sp_l.value())

    def _toggle(self):
        if self.monitor.recording:
            self.monitor._stop_recording()
        else:
            cond = "E1" if self.cond.currentIndex() == 1 else "B0"
            subj = (self.subj.text().strip() or "01")
            self.monitor._start_recording(f"{cond}_subject{subj}.csv")

    def _on_state(self, recording, path):
        self.cond.setEnabled(not recording)
        self.subj.setEnabled(not recording)
        if recording:
            self.btn.setText("Stop  (R)")
            self.btn.setStyleSheet("font-weight:bold; padding:6px 14px; background:#c0392b; color:white;")
            self.status.setText(f"RECORDING -> {path}   (make sure control_ON matches on the robot!)")
        else:
            self.btn.setText("Record  (R)")
            self.btn.setStyleSheet("font-weight:bold; padding:6px 14px;")
            if path:
                self.status.setText(f"stopped - {self.monitor.rec_count} rows saved to {path}")
            else:
                self.status.setText("pick condition + subject, then Record -> file B0_/E1_subjectNN.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--kemg", type=float, default=2.0, help="match firmware K_EMG")
    ap.add_argument("--base", type=float, default=0.0, help="match firmware assist_base_nm (fixed floor)")
    ap.add_argument("--rate", type=float, default=100.0, help="nominal stream rate (Hz) for the CSV time base")
    ap.add_argument("--limit", type=float, default=2.0, help="match assist_torque_limit_nm")
    ap.add_argument("--record", nargs="?", const="", default=None,
                    help="record CSV from start; optional path (auto-named if omitted)")
    ap.add_argument("--onset", choices=["angle", "phase"], default="angle",
                    help="reconstruction mode: 'angle' (matches firmware use_angle_onset=1) or 'phase' (G(phi))")
    ap.add_argument("--pullup-angle", type=float, default=40.0, help="match firmware pullup_angle_deg")
    ap.add_argument("--pullup-end", type=float, default=5.0, help="match firmware pullup_end_deg")
    ap.add_argument("--list-ports", action="store_true")
    args = ap.parse_args()

    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(p.device, "-", p.description)
        return
    if not args.port:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        args.port = ports[0] if len(ports) == 1 else None   # None -> AUTO: reader scans + waits

    print(f"Opening {args.port or 'AUTO (waiting for a COM port)'} @ {args.baud}, "
          f"module 0x{MODULE_ID:02X}, K_EMG={args.kemg}, limit={args.limit}")
    q = queue.Queue()
    stop = threading.Event()
    th = threading.Thread(target=reader_thread, args=(args.port, args.baud, q, stop), daemon=True)
    th.start()

    rec_path = None
    if args.record is not None:
        rec_path = args.record or f"emg_torque_{datetime.now():%Y%m%d_%H%M%S}.csv"

    app = pg.mkQApp()
    mon = Monitor(q, args.kemg, args.limit, record_path=rec_path,
                  onset_mode=args.onset, pullup_angle=args.pullup_angle, pullup_end=args.pullup_end,
                  base=args.base, rate=args.rate)
    win = MainWindow(mon)
    win.show()
    try:
        app.exec_()
    finally:
        mon._stop_recording()
        stop.set()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
