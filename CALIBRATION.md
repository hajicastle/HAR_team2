# Calibration Guide — Stair-Ascent Hip-Extension Assist (Team 2)

Full procedure for the four sensor calibration captures the firmware needs before it will
ever command torque. Each one fills a global the firmware later uses to scale or threshold
sensor data. Do all four every session — calibration is `.bss` state and zeroes on every
hot-reflash.

> Companion to `LAB_GUIDE.md §7` (which lists the buttons but not the *why* or the
> verification checks). This document is the full reference: physical protocol, what the
> firmware does internally, how to verify, and how to fix every failure mode.

---

## 0. What gets calibrated and why

| Capture | Variable filled | Purpose |
|---|---|---|
| **BTN1 (1st press) — EMG bias** | `s_emg_bias[R, L]` (internal; not a Live Expression) | Subtracts the resting DC offset from raw EMG. Without this the envelope is permanently inflated and `act` reads high even at rest. |
| **BTN1 (2nd press) — EMG MVIC** | `EMG_MVIC_R`, `EMG_MVIC_L` (live, observable) | Per-leg maximum-effort envelope voltage. Normalizes `act = env / MVIC` so M1 reads as %MVC. The M1 metric — the primary outcome of the project — depends entirely on this. |
| **BTN2 (1st press) — FSR off** | `s_fsr_off[4]` (internal) | Captures the unloaded FSR voltage per sensor. Normalizes `load = (v − off) / (on − off)` so 0 = unloaded. |
| **BTN2 (2nd press) — FSR on** | `s_fsr_on[4]` (internal) | Captures the loaded FSR voltage per sensor (standing weight). Sets the `1.0` point of the normalized load. Drives the fuzzy gait detector. |

All four must complete (`calibration_ready == 1`) before `Active_Loop` ever lets safety pass
and torque can fire. The gate is at `user_app.c:413`.

---

## 1. Prerequisites

Before pressing any button:

1. **Firmware flashed** with `user_app.c` (this file's controller).
2. **CubeIDE Debug session running** — calibration values only matter if you can read them back.
3. **H10 in ASSIST mode** (calibration runs in STANDBY, which requires a connected Control Module).
4. **Wearer is donned** with EMG electrodes on the biceps femoris (SENIAM placement —
   `LAB_GUIDE.md §5` or `PROJECT_GUIDE.md §F1`) and the FSR insole in each shoe (heel under the
   calcaneus, toe under the metatarsal heads — `PROJECT_GUIDE.md §F2`).
5. **Live Expressions added** for monitoring (see §2 below).
6. **Skin prep done** — shave if hairy, lightly abrade, alcohol wipe, press electrodes firmly.
   Bad skin contact is the #1 cause of low MVIC and shaky bias readings.

Bench safety: don't have the wearer up on the stairs yet. Calibration happens in a chair or
standing on flat ground.

---

## 2. Live Expressions to add before calibrating

Minimum set so you can verify each step. Add via *Add new expression* in the Live Expressions
pane:

```
control_ON
calibration_ready
cal_emg_bias_done
cal_emg_mvic_done
cal_fsr_off_done
cal_fsr_on_done
EMG_MVIC_R
EMG_MVIC_L
emg_R_raw_v
emg_L_raw_v
emg_R_env_v
emg_L_env_v
emg_R_act
emg_L_act
fsr_RH_raw_v
fsr_RT_raw_v
fsr_LH_raw_v
fsr_LT_raw_v
fsr_RH_load
fsr_RT_load
fsr_LH_load
fsr_LT_load
```

Confirm `control_ON = 0` before starting — calibration should not fire torque.

---

## 3. Calibration A — EMG resting bias  (BTN1, 1st press)

### What the firmware will do (`user_app.c:753–764`)
For 3 seconds (`EMG_CAL_MS = 3000`), accumulate `s_v[CH_EMG_R]` and `s_v[CH_EMG_L]` (raw
volts read by `_SampleAdc`), divide by the sample count to get the mean, and store as
`s_emg_bias[0]` and `s_emg_bias[1]`. After this, every iteration of `_ProcessEmg`
subtracts the bias before rectifying: `centered = s_v[i] - s_emg_bias[i]`.

### Wearer posture
- **Standing relaxed** with both feet on the floor. Arms hanging loose at sides.
- Hamstrings completely loose — wearer should not be bracing knees or "holding posture."
- Mouth, jaw, shoulders relaxed (involuntary tension in nearby muscles can show up in
  hamstring EMG as crosstalk).

### Steps
1. Confirm: `control_ON = 0`, `cal_emg_bias_done = 0`.
2. Tell the wearer "stand relaxed, don't move for 3 seconds."
3. **Press BTN1** on the XM10 (single quick press).
4. **LED2 blinks** for 3 seconds while capturing.
5. Don't talk to the wearer during the capture — startle responses contaminate the signal.

### Verify
| Variable | Expected after capture |
|---|---|
| `cal_emg_bias_done` | `1` |
| `emg_R_raw_v`, `emg_L_raw_v` | After capture, these stay near the bias value — **roughly 1.5–1.8 V** on a 3.3 V ADC mid-rail front end |
| `emg_R_env_v`, `emg_L_env_v` | Should now sit very close to **0.000–0.020 V** (the deadband). If they hover at 0.05+ V even at rest, the bias is wrong |

### Fixes if it failed
- **`cal_emg_bias_done` stays `0`** — the button event didn't register. Make sure
  `XM_IO_Update()` is being called (the firmware does this at `user_app.c:384`). If it's
  there, check the XM10 LEDs blinked. If LED2 never blinked, the button event was lost.
  Press BTN1 again.
- **`emg_*_env_v` is still high at rest** — the wearer wasn't actually relaxed. Repeat:
  press BTN3 (reset), then BTN1 again. Common cause: the wearer was talking, fidgeting, or
  holding their breath.

---

## 4. Calibration B — EMG MVIC (Maximum Voluntary Isometric Contraction)  (BTN1, 2nd press)

This is the most important capture for the M1 metric. **The whole report's primary outcome
depends on this number being a real, repeatable max** — not a half-effort, not a fake
software value.

### What the firmware will do (`user_app.c:765–775`)
For 3 seconds, **track the maximum envelope value reached** per leg:
```c
if (s_emg_env[0] > s_cal_max_env[0]) s_cal_max_env[0] = s_emg_env[0];
if (s_emg_env[1] > s_cal_max_env[1]) s_cal_max_env[1] = s_emg_env[1];
```
After 3 s, write:
```c
EMG_MVIC_R = (s_cal_max_env[0] > EMG_MVIC_MIN_V) ? s_cal_max_env[0] : EMG_MVIC_MIN_V;
EMG_MVIC_L = (s_cal_max_env[1] > EMG_MVIC_MIN_V) ? s_cal_max_env[1] : EMG_MVIC_MIN_V;
```
The floor `EMG_MVIC_MIN_V = 0.050 V` (`user_app.c:115`) prevents divide-by-near-zero
later. If your captured peak is below `0.050`, the MVIC is meaningless and gets clipped to
the floor — that's the safety net, not the desired outcome.

### Wearer posture options

Pick one of these — they both isolate the biceps femoris reasonably well. Resisted knee
flexion is preferred because it's the easiest to apply maximum force against.

**Option 1 (preferred) — resisted knee flexion, seated**
- Wearer **seated** with knees at ~90°.
- A spotter holds the wearer's ankle / lower shin firmly.
- Wearer attempts to **flex the knee** (pull the heel back toward the buttock).
- The spotter prevents motion — this becomes an **isometric** contraction.
- Both legs simultaneously for the simultaneous-MVIC pattern this firmware supports.

**Option 2 — resisted hip extension, prone**
- Wearer **prone** (face-down) on a mat.
- Knee bent ~30°.
- Spotter presses down on the lower thigh just above the knee.
- Wearer attempts to **extend the hip** (lift the thigh) against resistance.

**Option 3 — standing, isometric "press"**
- Wearer **standing** with one foot on a low step (~10 cm).
- Wearer transfers weight forward and **pushes down through the heel** as hard as possible
  without raising the body up. The hamstring co-activates with the glute to hold the
  posture.
- Use this if you don't have a spotter or a chair.

### Steps
1. Confirm: `cal_emg_bias_done = 1` (the firmware uses this flag at `user_app.c:725` to
   pick MVIC mode for the 2nd press). If it's 0, do calibration A first.
2. Warm up: have the wearer do **2–3 sub-maximal contractions** (~50% effort) for ~3 s each,
   with 10 s rest between. This wakes up the motor units and gives a higher max.
3. Tell the wearer: "Push as hard as you can for **3 full seconds**. Build to max in the first
   second and then hold *as hard as possible* through the whole second and third seconds."
4. **Press BTN1** — this is the 2nd press; the firmware enters `CAL_EMG_MVIC_RUN`.
5. **LED2 blinks** for 3 seconds while capturing.
6. Wearer pushes maximum effort the entire time. **Watch `emg_R_env_v` and `emg_L_env_v`**
   in Live Expressions — they should climb sharply within the first ~500 ms and peak.
7. After 3 s, capture ends. `cal_emg_mvic_done` flips to 1.

### Verify
| Variable | Expected |
|---|---|
| `cal_emg_mvic_done` | `1` |
| `EMG_MVIC_R` | **Clearly > 0.10 V, ideally 0.20–0.80 V.** Subject-dependent. |
| `EMG_MVIC_L` | Same range. |
| Ratio `EMG_MVIC_R / EMG_MVIC_L` | Within `[0.7, 1.4]`. If outside, electrode placement is asymmetric. |
| `emg_R_act`, `emg_L_act` at rest *after* capture | Drop to ~0 again (because rest envelope ≪ new MVIC). |

### Fixes if it failed

- **MVIC reads exactly `0.050 V`** (the floor) — the captured peak was below floor.
  - The contraction wasn't hard enough (most common). Try Option 1 with a stronger spotter
    or a heavier resistance band.
  - Electrode contact is bad. Re-prep skin, reposition.
  - Reset and recapture: press **BTN3 click** (resets all four flags), then BTN1 (bias), then
    BTN1 again (MVIC).
- **MVIC reads exactly `1.000 V`** (the boot default) — capture never ran.
  - `cal_emg_mvic_done` will be `0`. Press BTN1 again. If still no response, check `BTN1`
    isn't stuck (read `btn1_state` if you added it).
- **L/R asymmetry > 40%** — likely electrode misplacement on the weaker side.
  - Visually check both placements against the SENIAM landmarks. The pair should sit on
    the muscle belly, not over the tendon near the knee, not on the midline (sciatic nerve
    area between the two hamstring heads).
  - Re-prep skin on the weaker side, reposition, recalibrate.
- **MVIC drops on repeat capture** — wearer is fatigued. Rest 5+ minutes, recapture. Capture
  MVIC **once at the start of the session**, before any trial.

---

## 5. Calibration C — FSR unloaded  (BTN2, 1st press)

### What the firmware will do (`user_app.c:776–791`)
For 1 second (`FSR_CAL_MS = 1000`), accumulate the four `s_fsr_lpf[i]` values per loop, then
take the mean and store as `s_fsr_off[i]`. After this, every iteration computes:
```c
load = clamp((s_fsr_lpf[i] - s_fsr_off[i]) / span, 0, 1.5)
```
where `span = max(s_fsr_on[i] - s_fsr_off[i], 0.05)`.

### Wearer posture
- **Wearer sits down** in a chair.
- **Both feet airborne** — knees extended just enough that the insoles bear zero weight.
- This is critical: any residual pressure during the capture makes the "unloaded" reading
  non-zero, which then incorrectly raises the loaded threshold.

### Steps
1. Confirm: `cal_fsr_off_done = 0`. Wearer is seated, feet lifted.
2. **Press BTN2** on the XM10 (single quick press).
3. **LED3 blinks** for 1 second while capturing.
4. Wearer keeps feet airborne the whole time.

### Verify
| Variable | Expected |
|---|---|
| `cal_fsr_off_done` | `1` |
| `fsr_RH_raw_v` etc. | The actual unloaded voltages. Subject- and shoe-dependent — could be 0.05–0.5 V. |
| `fsr_RH_load` etc. **with feet still airborne** | Should be near **0.00** (within ~0.05) |

### Fixes if it failed
- **`fsr_*_load` is > 0.1 when wearer is sitting feet-up** — the off capture wasn't truly
  unloaded. Repeat with the wearer holding feet completely off, or sitting with feet on a
  chair to be sure.
- **One channel always reads ~0** even when stepping — that FSR is disconnected or broken.
  Check the cable at the XM10. Re-seat the insole connector.

---

## 6. Calibration D — FSR loaded  (BTN2, 2nd press)

### What the firmware will do
Same loop as Calibration C but stores the result as `s_fsr_on[i]` (`user_app.c:783–784`).
After both off and on captures, the normalized load span is `on − off`. The firmware floors
this at `FSR_MIN_SPAN_V = 0.05 V` (`user_app.c:591`) — if you accidentally captured nearly
the same voltage for off and on, the span gets floored to 0.05 V and `load` becomes
extremely noisy.

### Wearer posture
- **Wearer stands evenly on both feet**, knees slightly bent, weight centered.
- All four FSRs (heel and toe of each foot) should be loaded — wearer is **not** on tiptoes
  and **not** on heels alone.
- Wearer should not lean to one side.

### Steps
1. Confirm: `cal_fsr_off_done = 1`. Wearer is now standing evenly on both feet.
2. **Press BTN2** on the XM10.
3. **LED3 blinks** for 1 second while capturing.
4. Wearer keeps weight evenly distributed the whole time.

### Verify
| Variable | Expected |
|---|---|
| `cal_fsr_on_done` | `1` |
| `cal_emg_bias_done + cal_emg_mvic_done + cal_fsr_off_done + cal_fsr_on_done` | All `1` |
| `calibration_ready` | `1` ⟹ all four done; firmware now allows ACTIVE entry |
| `fsr_*_load` **with wearer still standing** | Near **1.00** ± 0.2 on all four |
| `fsr_*_load` **when wearer lifts a foot** | The two channels for that foot drop to ~0 |

### Fixes if it failed
- **`fsr_*_load` reads above 1.5 (saturating at 1.5)** when stepping on stairs — the loaded
  voltage from the cal was below what stair stepping actually produces. Acceptable —
  the FSR is over the cal max, but the load is clamped. Won't break the controller, just
  means the wearer is putting more pressure than during cal.
- **`fsr_*_load` reads near 0 even when standing** — the on capture didn't get a full load.
  Repeat: reset (`BTN3 click`), then both BTN2 presses with the wearer fully bearing weight.
- **One channel reads very different from its pair on the same foot** — insole crease,
  bad sensor placement under the foot, or one sensor connector loose.

---

## 7. Final verification — all four done

| Variable | Expected |
|---|---|
| `cal_emg_bias_done` | `1` |
| `cal_emg_mvic_done` | `1` |
| `cal_fsr_off_done` | `1` |
| `cal_fsr_on_done` | `1` |
| `calibration_ready` | `1` |
| `EMG_MVIC_R`, `EMG_MVIC_L` | Both clearly > `0.10 V` (subject-dependent; floor is `0.050`) |
| LED2 | **Solid on** (both EMG captures done) |
| LED3 | **Solid on** (both FSR captures done) |

**At this point the firmware will permit ACTIVE entry** when the H10 mode transitions to
ASSIST. The actual torque output is still gated by `control_ON == 1`, the safety predicates,
and the bench torque-sign check (Pre-lab #1) — those come next, see `LAB_GUIDE.md §9`.

---

## 8. Reset and re-do

### Reset all four calibrations
**BTN3 (single click)** in STANDBY clears all four flags and restores defaults:
- `s_emg_bias[i] = EMG_BIAS_DEFAULT_V` (= 1.65 V)
- `EMG_MVIC_R = EMG_MVIC_L = EMG_MVIC_DEFAULT_V` (= 1.0 V)
- `s_fsr_off[i] = 0`, `s_fsr_on[i] = FSR_ON_DEFAULT_V` (= 1.0 V)

After reset, do all four captures again.

### Re-do just one capture
You **can't** redo just one — the firmware's two-stage state machine relies on the order:
- BTN1 1st press → bias (only if bias not yet done)
- BTN1 2nd press → MVIC (only if bias already done)

If you want to redo only MVIC, you'd have to (a) reset all, then (b) redo bias and then MVIC.
Same for FSR off / on.

### Restart of Debug session, hot-reflash
**Calibration flags zero on every reset / reflash.** Always redo all four after a reflash.

---

## 9. Quality criteria — what counts as "good calibration"

A calibration session is good if **all** of the following hold:

| Check | Threshold |
|---|---|
| `calibration_ready` | `1` |
| `EMG_MVIC_R` | ≥ `0.10 V` (preferably ≥ `0.20`) |
| `EMG_MVIC_L` | ≥ `0.10 V` (preferably ≥ `0.20`) |
| MVIC L/R ratio | between `0.7` and `1.4` |
| `fsr_*_load` with feet airborne | < `0.05` |
| `fsr_*_load` standing evenly | between `0.7` and `1.3` |
| `emg_*_env_v` at rest *after* bias | < `0.030 V` |

If any of these fail, redo the offending capture(s). It's a 30-second cost; a bad
calibration corrupts every metric of every recorded trial after it.

---

## 10. Best practices

1. **Calibrate once at the start of the session, before any trial.** Don't recalibrate
   mid-session — the act of re-doing MVIC under fatigue lowers `EMG_MVIC_R/L` and inflates
   the apparent %MVC in subsequent trials.
2. **Warm up** before MVIC: 2–3 sub-maximal contractions, ~50% effort, 3 s each, 10 s rest.
   This recruits motor units and avoids under-counting the peak.
3. **Skin prep** is the single highest leverage on EMG quality. Shave → light abrasion →
   alcohol → press firmly. Test by tapping near the electrode — you should see a clean
   spike on `emg_*_env_v`.
4. **Tape the cables down** so they don't sway during stepping. Cable sway = motion artifact.
5. **Insoles flat, no creases**, sensor side up. Heel sensor under the calcaneus, toe under
   the metatarsal heads.
6. **Repeat MVIC at least once** if the first capture looks low. The wearer often
   under-shoots on the first try.
7. **Don't write `EMG_MVIC_R/L` by hand.** The whole point is per-subject normalization.

---

## 11. Quick-reference card (print and bring to the lab)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          CALIBRATION CHECKLIST                          │
├─────────────────────────────────────────────────────────────────────────┤
│  Prereqs                                                                │
│    [ ] firmware flashed, Debug session active                           │
│    [ ] H10 in ASSIST mode                                               │
│    [ ] control_ON = 0                                                   │
│    [ ] EMG electrodes + FSR insoles placed                              │
├─────────────────────────────────────────────────────────────────────────┤
│  A. EMG bias       BTN1 1st        Wearer: relaxed standing, 3 s        │
│     check: cal_emg_bias_done=1, emg_*_env_v ≈ 0                         │
│                                                                          │
│  B. EMG MVIC       BTN1 2nd        Wearer: resisted knee flexion, 3 s    │
│     check: cal_emg_mvic_done=1, EMG_MVIC_R/L > 0.10 V                   │
│                                                                          │
│  C. FSR off        BTN2 1st        Wearer: sit, feet airborne, 1 s      │
│     check: cal_fsr_off_done=1, fsr_*_load near 0                        │
│                                                                          │
│  D. FSR on         BTN2 2nd        Wearer: stand evenly, 1 s            │
│     check: cal_fsr_on_done=1, fsr_*_load near 1                         │
├─────────────────────────────────────────────────────────────────────────┤
│  Final:  calibration_ready = 1  →  ready for ACTIVE                     │
│  Reset:  BTN3 click   (restores all four to defaults)                   │
│  E-stop: BTN3 long-press (only in ACTIVE)                               │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 12. What's actually stored — for the curious

The four calibrations write to two kinds of state:

| Capture | Storage location | Visible as Live Expression? |
|---|---|---|
| EMG bias | `s_emg_bias[2]` (static array in `user_app.c`) | No — internal only |
| EMG MVIC | `EMG_MVIC_R`, `EMG_MVIC_L` (file-scope globals) | **Yes** |
| FSR off | `s_fsr_off[4]` (static array) | No — internal only |
| FSR on | `s_fsr_on[4]` (static array) | No — internal only |

If you want to see the internal arrays in CubeIDE: drop into the Expressions pane (not Live
Expressions) and type `s_emg_bias[0]`, `s_fsr_off[2]` etc. They'll show but require manual
refresh.

The four `cal_*_done` flags and `calibration_ready` are the public surface — they're enough
to confirm everything is in place without exposing the underlying arrays.

---

## 13. After calibration — what's next

Once `calibration_ready = 1`:

1. **Bench torque-sign check** (Pre-lab #1) — `LAB_GUIDE.md §9` or `PROJECT_GUIDE.md §G4`.
   Confirm negative `tau_*_cmd_nm` physically extends the hip on this specific unit.
2. **Raise `pelvic_incline_min_deg` to `+5`** (Pre-lab #2) before going onto the stair treadmill.
3. **Record B0 and E1 trials** — `LAB_GUIDE.md §11` or `COMMANDS.md §10`.

---

**End of `CALIBRATION.md`.** For the *operational quick steps* see `LAB_GUIDE.md §7`. For
*what the firmware actually computes* see `code.md §1.11`. For *why each metric needs which
calibration* see `ANALYSIS.md §6`.
