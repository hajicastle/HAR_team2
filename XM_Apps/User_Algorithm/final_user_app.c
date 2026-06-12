/**
 ******************************************************************************
 * @file    user_app.c
 * @brief   Final Project — Stair-Ascent Hip-Extension Assist (Team 2).
 *          FINAL COMBINED CONTROLLER:
 *            EMG-gated, FSR-fuzzy-phase-shaped hip-extension pulse
 *            + kinematic angle latch (stair-vs-flat discrimination)
 *            + optional swing-flexion assist.
 * @details
 * ============================================================================
 * Provenance — best of both source controllers
 * ============================================================================
 *  From the fuzzy/phase backbone ("Code 2", original user_app.c):
 *    - 4-state fuzzy gait FSM (SWING->HEEL_STRIKE->LOAD_RESPONSE->TERM_STANCE)
 *      with anti-chatter dwell; one LOAD_RESPONSE event per stride.
 *    - Stair-cycle phase clock phi in [0,1), period validated [1..4] s.
 *    - F-vector pull-up envelope  G(phi) = x*exp(1-x)  for a smooth pulse.
 *    - Full safety suite (cal-ready, thigh bounds, pelvic gate, period
 *      bounds, first-LR seen, NOT standing) and robust 4-step calibration.
 *    - Clean standing detection + _ResetRuntime() on ACTIVE entry.
 *    - Conservative limits (HARD_MAX 2.5 N.m, slew 30 N.m/s, ramp 2 s).
 *
 *  From the kinematic-latch controller ("Code 1", user_stair_climb):
 *    - CORRECTED hardware channel map (FSR PF3..PF6, EMG PF7..PF8 — see below).
 *    - Kinematic angle latch: at LOAD_RESPONSE, confirm "stair" only if the
 *      thigh is flexed >= STANCE_EXT_MIN_DEG. Prevents flat-ground walking
 *      from triggering the extension pulse.
 *    - Optional swing-flexion assist (use_flexion_assist, default OFF).
 *
 * ============================================================================
 * Sensor channel allocation  (corrected wiring — verified hardware)
 *   All channels 5 V powered, read in millivolts via XM_AnalogReadMillivolts.
 * ============================================================================
 *   PF3 (DIO_1 -> XM_EXT_ADC_8)   FSR   L toe
 *   PF4 (DIO_2 -> XM_EXT_ADC_7)   FSR   L heel
 *   PF5 (DIO_3 -> XM_EXT_ADC_6)   FSR   R toe
 *   PF6 (DIO_4 -> XM_EXT_ADC_5)   FSR   R heel
 *   PF7 (DIO_5 -> XM_EXT_ADC_9)   sEMG  R hamstring (biceps femoris)
 *   PF8 (DIO_6 -> XM_EXT_ADC_10)  sEMG  L hamstring (biceps femoris)
 *
 * Control law per leg z in {R, L}:
 *   STANCE (stair confirmed):
 *     tau_cmd_z(t) = - G_phi(phi_z) * K_EMG * a_ham_z * Ramp(t) * 1[safe_z]
 *       - leading minus sign => hip EXTENSION torque (negative on H10)
 *       - stair confirmed    = FSM LOAD_RESPONSE AND thigh_z >= STANCE_EXT_MIN_DEG
 *       - G_phi(phi)         => smooth F-vector pulse over the pull-up window
 *       - EMG activation a   => HOW MUCH (assist-as-needed, hamstring effort)
 *   SWING (optional, use_flexion_assist == 1):
 *     tau_cmd_z(t) = + FLEX_ASSIST_NM * Ramp(t)   when FLEX_START < thigh_z < FLEX_END
 *
 * ============================================================================
 * Calibration (in STANDBY, before ASSIST) — 4 captures over 3 buttons
 * ============================================================================
 *   BTN1 : step 1 = EMG resting bias (relax hamstrings, 3 s)
 *          step 2 = EMG MVIC effort  (max contraction, 3 s)
 *   BTN2 : step 1 = FSR off / unloaded (feet airborne, 1 s)
 *          step 2 = FSR on  / loaded   (stand on both feet, 1 s)
 *   BTN3 : reset all calibration.  BTN3 long-press in ACTIVE = emergency stop.
 *   calibration_ready == 1 only after all four captures complete.
 *
 * ============================================================================
 * USB-CDC telemetry  (Module 0xF0, 16 floats)  -> PC logger preset
 *   "Final_Stair_Assist" in PythonDecoder/CDC/cdc_selective_logger.py
 * ============================================================================
 *   EMG R env, EMG L env, EMG R act, EMG L act,
 *   FSR RH load, FSR RT load, FSR LH load, FSR LT load,
 *   Phase R, Phase L, Gait R, Gait L,
 *   Thigh R, Thigh L, Tau R, Tau L      (Tau negative = extension)
 *   (is_stair_R/L and use_flexion_assist are Live-Expression observables,
 *    kept OUT of the stream so the 16-float analysis pipeline is unchanged.)
 *
 * @version 2.0  (final combined)
 * @date    2026-06-11
 ******************************************************************************
 */

#include "xm_api.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * COMPILE-TIME CONSTANTS
 * ============================================================================ */
#define CONTROL_DT_S              0.001f      /* 1 kHz ACTIVE loop */
#define TWO_PI                    6.28318530718f
#define USB_MODULE_ID             0xF0U
#define USB_DEBUG_PERIOD_MS       500U
#define HARD_MAX_ASSIST_TORQUE_NM 2.5f        /* instructor absolute ceiling */

/* User body data — improves H10 internal thigh/pelvic-angle estimates.
 * Edit per subject: [0] = weight kg x10, [1] = height cm x10. */
#define USER_WEIGHT_KG_X10        700U        /* 70.0 kg */
#define USER_HEIGHT_CM_X10        1750U       /* 175.0 cm */

/* ADC channel order — corrected hardware wiring (matches s_adc_pins).
 * FSR run in reverse ADC order (RH->ADC8 .. LT->ADC5); EMG follow (R->ADC9, L->ADC10). */
typedef enum {
    CH_FSR_RH = 0,  /* PF6 (DIO_4) -> XM_EXT_ADC_5  Right Heel FSR */
    CH_FSR_RT,      /* PF5 (DIO_3) -> XM_EXT_ADC_6  Right Toe  FSR */
    CH_FSR_LH,      /* PF4 (DIO_2) -> XM_EXT_ADC_7  Left  Heel FSR */
    CH_FSR_LT,      /* PF3 (DIO_1) -> XM_EXT_ADC_8  Left  Toe  FSR */
    CH_EMG_R,       /* PF7 (DIO_5) -> XM_EXT_ADC_9  Right hamstring EMG */
    CH_EMG_L,       /* PF8 (DIO_6) -> XM_EXT_ADC_10 Left  hamstring EMG */
    ADC_CH_COUNT
} AdcCh_t;

#define FSR_CH_COUNT              4
#define EMG_CH_COUNT              2

/* EMG pipeline */
#define EMG_BIAS_DEFAULT_V        1.65f
#define EMG_PRE_LPF_FC_HZ         80.0f
#define EMG_ENV_LPF_FC_HZ         5.0f
#define EMG_ENV_DEADBAND_V        0.020f
#define EMG_MVIC_DEFAULT_V        1.0f
#define EMG_MVIC_MIN_V            0.050f
#define EMG_ACT_GAMMA             0.95f       /* Hill activation dynamics */
#define EMG_CAL_MS                3000U

/* FSR pipeline */
#define FSR_LPF_FC_HZ             8.0f
#define FSR_MIN_SPAN_V            0.05f
#define FSR_ON_DEFAULT_V          1.0f
#define FSR_CAL_MS                1000U

/* Fuzzy gait detection */
#define FUZZY_PHASE_COUNT         4
#define PHASE_MIN_DWELL_MS        60U         /* min time in a phase (anti-chatter) */
#define MEMBERSHIP_ON             0.5f        /* fuzzy membership "loaded" boundary */

/* Stair-cycle phase estimator */
#define STAIR_PERIOD_INIT_S       2.0f        /* ~30 cycles/min */
#define STAIR_PERIOD_MIN_S        1.0f
#define STAIR_PERIOD_MAX_S        4.0f
#define STANDING_DWELL_S          0.7f

/* F-vector pull-up envelope: G(phi)=(x)exp(1-x), x=(phi-START)/TP */
#define PHASE_PULSE_START         0.05f
#define PHASE_PULSE_TP            0.10f       /* peak at phi=0.15 */
#define PHASE_PULSE_WINDOW        0.30f       /* zero past START + 3*TP = 0.35 */

/* Kinematic angle latch — stair discriminator (Code 1).
 * thigh must be flexed at least this much at LOAD_RESPONSE to count as a stair. */
#define STANCE_EXT_MIN_DEG        25.0f

/* Optional swing-flexion assist (Code 1) */
#define FLEX_ASSIST_NM            1.5f        /* fixed flexion torque, N.m (positive) */
#define FLEX_START_DEG            30.0f       /* start lifting assist above this angle */
#define FLEX_END_DEG              45.0f       /* cut off above this angle */

/* Angle-onset envelope (thigh-angle-triggered extension assist) */
#define THIGH_RATE_LPF_FC_HZ      5.0f        /* LPF for the thigh angular-rate estimate */
#define EXT_RATE_THRESH_DEG_MS    0.02f       /* |rate| > this (deg/ms, ~20 deg/s) = "extending" */

/* Torque shaping / safety */
#define SLEW_RATE_LIMIT_NM_PER_S  30.0f
#define RAMP_DURATION_S           2.0f
#define THIGH_ANGLE_MIN_DEG       (-10.0f)
#define THIGH_ANGLE_MAX_DEG       (90.0f)

typedef enum {
    GAIT_PHASE_HEEL_STRIKE = 0,
    GAIT_PHASE_LOAD_RESPONSE,
    GAIT_PHASE_TERMINAL_STANCE,
    GAIT_PHASE_SWING
} FootGaitPhase_t;

typedef enum {
    GAIT_EVENT_NONE            = 0U,
    GAIT_EVENT_HEEL_STRIKE     = 1U << 0,
    GAIT_EVENT_LOAD_RESPONSE   = 1U << 1,
    GAIT_EVENT_TERMINAL_STANCE = 1U << 2,
    GAIT_EVENT_TOE_OFF         = 1U << 3
} FootGaitEvent_t;

typedef struct {
    float           mu[FUZZY_PHASE_COUNT];
    FootGaitPhase_t phase;
    FootGaitPhase_t prev_phase;
    uint32_t        phase_since_ms;     /* tick of last phase change (dwell timing) */
    bool            initialized;
} FootFuzzy_t;

typedef struct {
    float    phase;          /* [0,1) stair-cycle phase */
    float    period_s;
    uint32_t last_lr_ms;
    bool     has_first_lr;
} StairEstim_t;

typedef enum {
    CAL_IDLE = 0,
    CAL_EMG_BIAS_RUN,
    CAL_EMG_MVIC_RUN,
    CAL_FSR_OFF_RUN,
    CAL_FSR_ON_RUN
} CalState_t;

/* 16-float CDC telemetry record (order must match logger preset). */
typedef struct {
    float emg_R_env, emg_L_env, emg_R_act, emg_L_act;
    float fsr_RH_load, fsr_RT_load, fsr_LH_load, fsr_LT_load;
    float phase_R, phase_L, gait_R, gait_L;
    float thigh_R, thigh_L, tau_R, tau_L;
} StairStreamData_t;

/* ============================================================================
 * STUDENT-TUNABLE LIVE-EXPRESSION GLOBALS
 * ============================================================================ */
uint16_t control_ON          = 0U;          /* master enable (set 1 after bench check) */
float    K_EMG               = 2.0f;        /* magnitude scalar, N.m (start low) */
float    assist_base_nm      = 0.0f;        /* fixed extension floor added to EMG term when gate open (N.m) */
float    assist_torque_limit_nm = 2.0f;     /* project limit, <= HARD_MAX */
float    EMG_MVIC_R          = EMG_MVIC_DEFAULT_V;
float    EMG_MVIC_L          = EMG_MVIC_DEFAULT_V;

float    fuzzy_heel_threshold = 0.35f;
float    fuzzy_toe_threshold  = 0.35f;
float    fuzzy_sensitivity    = 12.0f;

float    pelvic_incline_min_deg = -90.0f;   /* permissive default; raise to gate on incline */
uint16_t use_flexion_assist     = 0U;       /* 1 = enable swing-flexion assist (allows + torque) */
uint16_t use_assist_level_scale = 0U;       /* 1 = scale torque by H10 assist-level (0..10); 0 = full (deterministic) */

/* Angle-onset extension assist (replaces the G(phi) FSR-phase timing when on).
 * Pulse turns on as soon as the thigh is flexed past pullup_angle_deg AND the hip
 * is extending, and runs through the pull-up until the thigh extends below
 * pullup_end_deg (or the foot swings). Earlier/lower-latency than the phase clock. */
uint16_t use_angle_onset        = 1U;       /* 1 = angle-onset envelope; 0 = G(phi) FSR-phase envelope */
float    pullup_angle_deg       = 40.0f;    /* onset: assist when thigh flexed past this (deg) */
float    pullup_end_deg         = 5.0f;     /* release: pull-up done once thigh extends below this */
uint16_t pullup_require_extending = 1U;     /* 1 = also require hip to be extending at onset */

uint16_t cdc_stream_enable    = 1U;
uint16_t cdc_stream_period_ms = 10U;        /* 100 Hz */

/* Observe-only live expressions */
float    emg_R_raw_v, emg_L_raw_v;
float    emg_R_env_v, emg_L_env_v;
float    emg_R_act, emg_L_act;
float    fsr_RH_load, fsr_RT_load, fsr_LH_load, fsr_LT_load;
float    fsr_RH_raw_v, fsr_RT_raw_v, fsr_LH_raw_v, fsr_LT_raw_v;
uint16_t mask_bilateral;
uint16_t standing_flag;
uint16_t calibration_ready;
uint16_t app_state;          /* TSM state readout for Live Expressions: 0=OFF, 1=STANDBY, 2=ACTIVE */
uint16_t cal_emg_bias_done, cal_emg_mvic_done, cal_fsr_off_done, cal_fsr_on_done;
uint16_t left_gait_phase, right_gait_phase;
float    phase_R, phase_L;
float    period_R_s, period_L_s;
float    thigh_R_deg, thigh_L_deg, pelvic_angle_deg;
float    tau_R_cmd_nm, tau_L_cmd_nm;        /* negative = extension */
uint16_t assist_enable;
uint16_t is_stair_R, is_stair_L;            /* live stair-confirmation flags (bench debug) */
uint16_t pullup_active_R, pullup_active_L;  /* live: angle-onset pull-up gate active */
float    thigh_rate_R, thigh_rate_L;        /* live: thigh angular rate, deg/ms (negative = extending) */

/* ============================================================================
 * STATIC STATE
 * ============================================================================ */
static XmTsmHandle_t s_tsm;

/* ADC pin table — order matches AdcCh_t enum (corrected wiring). */
static const XmAdcPin_t s_adc_pins[ADC_CH_COUNT] = {
    XM_EXT_ADC_5,   /* CH_FSR_RH : PF6 Right Heel  (swapped: was ADC_8) */
    XM_EXT_ADC_6,   /* CH_FSR_RT : PF5 Right Toe   (swapped: was ADC_7) */
    XM_EXT_ADC_7,   /* CH_FSR_LH : PF4 Left  Heel  (swapped: was ADC_6) */
    XM_EXT_ADC_8,   /* CH_FSR_LT : PF3 Left  Toe   (swapped: was ADC_5) */
    XM_EXT_ADC_9,   /* CH_EMG_R  : PF7 Right hamstring */
    XM_EXT_ADC_10   /* CH_EMG_L  : PF8 Left  hamstring */
};

static float s_v[ADC_CH_COUNT];             /* latest raw volts per channel */

/* EMG state (index 0=R, 1=L) */
static float s_emg_bias[EMG_CH_COUNT] = { EMG_BIAS_DEFAULT_V, EMG_BIAS_DEFAULT_V };
static float s_emg_pre[EMG_CH_COUNT];
static float s_emg_env[EMG_CH_COUNT];
static float s_emg_act[EMG_CH_COUNT];

/* FSR state (index per AdcCh: 0=RH, 1=RT, 2=LH, 3=LT) */
static float s_fsr_lpf[FSR_CH_COUNT];
static float s_fsr_off[FSR_CH_COUNT];
static float s_fsr_on[FSR_CH_COUNT]  = { FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V,
                                         FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V };
static float s_fsr_load[FSR_CH_COUNT];
static bool  s_filter_init;

/* Fuzzy detectors + stair phase estimators */
static FootFuzzy_t  s_right, s_left;
static StairEstim_t s_stair_R = { 0.0f, STAIR_PERIOD_INIT_S, 0U, false };
static StairEstim_t s_stair_L = { 0.0f, STAIR_PERIOD_INIT_S, 0U, false };

/* Hybrid stair-confirmation latches (Code 1 angle check x Code 2 FSM event) */
static bool s_stair_latch_R = false;
static bool s_stair_latch_L = false;

/* Angle-onset pull-up state (index 0=R, 1=L) */
static float s_thigh_prev[2];
static float s_thigh_rate[2];
static bool  s_pullup[2];

/* Standing detection */
static uint8_t  s_prev_mask;
static uint32_t s_mask_since_ms;
static bool     s_standing;

/* Calibration */
static CalState_t s_cal_state = CAL_IDLE;
static uint32_t   s_cal_start_ms;
static uint32_t   s_cal_count;
static double     s_cal_sum_emg[EMG_CH_COUNT];
static double     s_cal_sum_fsr[FSR_CH_COUNT];
static float      s_cal_max_env[EMG_CH_COUNT];

/* Torque output history */
static float    s_tau_prev[2];              /* 0=R, 1=L */
static uint32_t s_active_entry_ms;

/* Telemetry */
static uint32_t s_stream_ms;
static uint32_t s_usb_dbg_ms;
static StairStreamData_t s_stream;

/* CDC metadata for module 0xF0 (16 channels). */
static const char s_cdc_meta[] =
    "[{\"name\":\"EMG R env\",\"unit\":\"V\"},"
     "{\"name\":\"EMG L env\",\"unit\":\"V\"},"
     "{\"name\":\"EMG R act\",\"unit\":\"-\"},"
     "{\"name\":\"EMG L act\",\"unit\":\"-\"},"
     "{\"name\":\"FSR RH\",\"unit\":\"-\"},"
     "{\"name\":\"FSR RT\",\"unit\":\"-\"},"
     "{\"name\":\"FSR LH\",\"unit\":\"-\"},"
     "{\"name\":\"FSR LT\",\"unit\":\"-\"},"
     "{\"name\":\"Phase R\",\"unit\":\"-\"},"
     "{\"name\":\"Phase L\",\"unit\":\"-\"},"
     "{\"name\":\"Gait R\",\"unit\":\"id\"},"
     "{\"name\":\"Gait L\",\"unit\":\"id\"},"
     "{\"name\":\"Thigh R\",\"unit\":\"deg\"},"
     "{\"name\":\"Thigh L\",\"unit\":\"deg\"},"
     "{\"name\":\"Tau R\",\"unit\":\"Nm\"},"
     "{\"name\":\"Tau L\",\"unit\":\"Nm\"}]";

_Static_assert(sizeof(s_cdc_meta) <= 513U, "CDC metadata exceeds 512-byte limit");

/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */
static void  Off_Loop(void);
static void  Standby_Loop(void);
static void  Active_Entry(void);
static void  Active_Loop(void);
static void  Active_Exit(void);

static float _LpfAlphaRC(float fc_hz);
static float _Clamp(float x, float lo, float hi);
static float _Abs(float x);
static float _MembershipLarge(float v, float thr, float sens);
static float _PhaseEnvelope(float phi);

static void  _SampleAdc(void);
static void  _ProcessEmg(void);
static void  _ProcessFsr(void);
static uint16_t _UpdateFoot(FootFuzzy_t *d, float heel_load, float toe_load, uint32_t now_ms);
static void  _UpdateStairEstim(StairEstim_t *e, bool loading_response, uint32_t now_ms);
static void  _ResetRuntime(void);

static void  _HandleCalibration(void);
static bool  _AllCalDone(void);

static void  _PublishSignals(void);
static void  _SendStream(void);
static void  _UsbDebug(uint32_t now_ms);

/* ============================================================================
 * REQUIRED ENTRY POINTS
 * ============================================================================ */
void User_Setup(void)
{
    /* Send wearer body data so the H10's internal thigh/pelvic-angle estimates
     * (used for the safety bounds and the M4 kinematic metric) are accurate. */
    uint32_t bodyData[8] = { USER_WEIGHT_KG_X10, USER_HEIGHT_CM_X10, 0, 0, 0, 0, 0, 0 };
    XM_SendUserBodyData(bodyData);

    XM_SetExtPowerVoltage(XM_EXT_PWR_5V);

    XM_SwitchDioToAdc(XM_EXT_DIO_1);
    XM_SwitchDioToAdc(XM_EXT_DIO_2);
    XM_SwitchDioToAdc(XM_EXT_DIO_3);
    XM_SwitchDioToAdc(XM_EXT_DIO_4);
    XM_SwitchDioToAdc(XM_EXT_DIO_5);
    XM_SwitchDioToAdc(XM_EXT_DIO_6);

    s_tsm = XM_TSM_Create(XM_STATE_OFF);

    XmStateConfig_t off_conf = { .id = XM_STATE_OFF,     .on_loop = Off_Loop };
    XM_TSM_AddState(s_tsm, &off_conf);
    XmStateConfig_t sb_conf  = { .id = XM_STATE_STANDBY, .on_loop = Standby_Loop };
    XM_TSM_AddState(s_tsm, &sb_conf);
    XmStateConfig_t act_conf = { .id = XM_STATE_ACTIVE,
                                 .on_entry = Active_Entry,
                                 .on_loop  = Active_Loop,
                                 .on_exit  = Active_Exit };
    XM_TSM_AddState(s_tsm, &act_conf);

    XM_SetUsbCustomMeta(USB_MODULE_ID, s_cdc_meta);
    XM_SetUsbTotalDataStream(false);
    XM_SetH10AssistExistingMode(false);   /* default built-in H10 assist OFF — suit passive unless our controller drives it */
    XM_SetControlMode(XM_CTRL_MONITOR);

    XM_SendUsbDebugMessage("[FP-STAIR] boot — fuzzy-phase + kinematic-latch hip-extension assist\r\n");
}

void User_Loop(void)
{
    if (!s_tsm) {
        return;
    }
    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
    }
    XM_TSM_Run(s_tsm);

    /* REQUIRED: debounce buttons + tick LED timers, else BTN events stay NONE. */
    XM_IO_Update();
}

/* ============================================================================
 * TSM STATES
 * ============================================================================ */
static void Off_Loop(void)
{
    app_state = 0U;   /* OFF */
    if (XM_IsCmConnected()) {
        XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
        XM_SendUsbDebugMessage("[FP-STAIR] CM connected -> STANDBY (calibrate, then ASSIST)\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
    }
}

static void Standby_Loop(void)
{
    app_state = 1U;   /* STANDBY */
    /* Calibration is performed here, before entering ASSIST. */
    _SampleAdc();
    _ProcessEmg();   /* needed so MVIC capture sees a live envelope */
    _ProcessFsr();
    _HandleCalibration();

    XM_SetLedState(XM_LED_2, (cal_emg_bias_done && cal_emg_mvic_done) ? XM_ON : XM_OFF);
    XM_SetLedState(XM_LED_3, (cal_fsr_off_done  && cal_fsr_on_done)   ? XM_ON : XM_OFF);

    _PublishSignals();   /* publishes encoder/thigh in STANDBY too (bench check) */
    _SendStream();

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST && _AllCalDone()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    app_state = 2U;   /* ACTIVE */
    XM_SetH10AssistExistingMode(false);   /* take over from built-in assist */
    XM_SetControlMode(XM_CTRL_TORQUE);

    _ResetRuntime();
    s_active_entry_ms = XM_GetTick();
    s_usb_dbg_ms      = XM_GetTick();

    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 200);
    XM_SendUsbDebugMessage("[FP-STAIR] ACTIVE — stair extension + optional swing flexion\r\n");
}

static void Active_Loop(void)
{
    if (XM.status.h10.h10Mode != XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[FP-STAIR] ASSIST released -> STANDBY\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
        return;
    }
    if (XM_GetButtonEvent(XM_BTN_3) == XM_BTN_LONG_PRESS) {
        XM_SendUsbDebugMessage("[FP-STAIR] BTN3 long-press -> EMERGENCY STOP\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
        return;
    }

    uint32_t now_ms = XM_GetTick();

    /* 1. sense + signal processing -------------------------------------- */
    _SampleAdc();
    _ProcessEmg();
    _ProcessFsr();

    /* 2. bilateral fuzzy gait phase ------------------------------------- */
    uint16_t ev_R = _UpdateFoot(&s_right, s_fsr_load[CH_FSR_RH], s_fsr_load[CH_FSR_RT], now_ms);
    uint16_t ev_L = _UpdateFoot(&s_left,  s_fsr_load[CH_FSR_LH], s_fsr_load[CH_FSR_LT], now_ms);

    /* 3. bilateral contact mask + standing detection -------------------- */
    bool rh = s_fsr_load[CH_FSR_RH] >= fuzzy_heel_threshold;
    bool rt = s_fsr_load[CH_FSR_RT] >= fuzzy_toe_threshold;
    bool lh = s_fsr_load[CH_FSR_LH] >= fuzzy_heel_threshold;
    bool lt = s_fsr_load[CH_FSR_LT] >= fuzzy_toe_threshold;
    uint8_t mask = (uint8_t)((lh << 3) | (lt << 2) | (rh << 1) | (rt << 0));
    if (mask != s_prev_mask) {
        s_mask_since_ms = now_ms;
    }
    s_standing = (mask == 0x0FU) &&
                 ((now_ms - s_mask_since_ms) >= (uint32_t)(STANDING_DWELL_S * 1000.0f));
    s_prev_mask = mask;

    /* 4. stair-cycle phase clock (loading-response = pull-up start) ------ */
    _UpdateStairEstim(&s_stair_R, (ev_R & GAIT_EVENT_LOAD_RESPONSE) != 0U, now_ms);
    _UpdateStairEstim(&s_stair_L, (ev_L & GAIT_EVENT_LOAD_RESPONSE) != 0U, now_ms);

    /* 5. kinematic stair-vs-flat latch ---------------------------------- *
     * The angle check fires once per step at LOAD_RESPONSE (no chatter) and
     * re-arms when the foot returns to SWING.  Stair ascent lands with large
     * thigh flexion; flat-ground walking does not — so the extension pulse is
     * gated to stairs only.                                                */
    float thigh_R = XM.status.h10.rightThighAngle;
    float thigh_L = XM.status.h10.leftThighAngle;
    float pelvic  = XM.status.h10.pelvicAngle;

    if ((ev_R & GAIT_EVENT_LOAD_RESPONSE) != 0U) {
        s_stair_latch_R = (thigh_R >= STANCE_EXT_MIN_DEG);
    }
    if (s_right.phase == GAIT_PHASE_SWING) {
        s_stair_latch_R = false;
    }
    if ((ev_L & GAIT_EVENT_LOAD_RESPONSE) != 0U) {
        s_stair_latch_L = (thigh_L >= STANCE_EXT_MIN_DEG);
    }
    if (s_left.phase == GAIT_PHASE_SWING) {
        s_stair_latch_L = false;
    }

    /* 5b. angle-onset pull-up gate (thigh-angle-triggered timing) -------- *
     * Low-pass the thigh angular rate; "extending" = thigh angle decreasing.
     * Latch ON when thigh flexed past pullup_angle_deg AND extending; latch
     * OFF when the thigh extends below pullup_end_deg or the foot swings.   */
    float a_rate = _LpfAlphaRC(THIGH_RATE_LPF_FC_HZ);
    s_thigh_rate[0] += a_rate * ((thigh_R - s_thigh_prev[0]) - s_thigh_rate[0]);
    s_thigh_rate[1] += a_rate * ((thigh_L - s_thigh_prev[1]) - s_thigh_rate[1]);
    s_thigh_prev[0] = thigh_R;
    s_thigh_prev[1] = thigh_L;
    bool ext_R = (pullup_require_extending == 0U) || (s_thigh_rate[0] < -EXT_RATE_THRESH_DEG_MS);
    bool ext_L = (pullup_require_extending == 0U) || (s_thigh_rate[1] < -EXT_RATE_THRESH_DEG_MS);
    if (!s_pullup[0] && thigh_R >= pullup_angle_deg && ext_R) s_pullup[0] = true;
    if (s_pullup[0] && (thigh_R <= pullup_end_deg || s_right.phase == GAIT_PHASE_SWING)) s_pullup[0] = false;
    if (!s_pullup[1] && thigh_L >= pullup_angle_deg && ext_L) s_pullup[1] = true;
    if (s_pullup[1] && (thigh_L <= pullup_end_deg || s_left.phase == GAIT_PHASE_SWING)) s_pullup[1] = false;
    pullup_active_R = s_pullup[0] ? 1U : 0U;
    pullup_active_L = s_pullup[1] ? 1U : 0U;
    thigh_rate_R = s_thigh_rate[0];
    thigh_rate_L = s_thigh_rate[1];

    /* 6. safety predicates ---------------------------------------------- */
    bool safe_R = (control_ON == 1U) && _AllCalDone() && s_stair_R.has_first_lr &&
                  (thigh_R >= THIGH_ANGLE_MIN_DEG) && (thigh_R <= THIGH_ANGLE_MAX_DEG) &&
                  (pelvic >= pelvic_incline_min_deg) &&
                  (s_stair_R.period_s >= STAIR_PERIOD_MIN_S) &&
                  (s_stair_R.period_s <= STAIR_PERIOD_MAX_S) &&
                  !s_standing;
    bool safe_L = (control_ON == 1U) && _AllCalDone() && s_stair_L.has_first_lr &&
                  (thigh_L >= THIGH_ANGLE_MIN_DEG) && (thigh_L <= THIGH_ANGLE_MAX_DEG) &&
                  (pelvic >= pelvic_incline_min_deg) &&
                  (s_stair_L.period_s >= STAIR_PERIOD_MIN_S) &&
                  (s_stair_L.period_s <= STAIR_PERIOD_MAX_S) &&
                  !s_standing;

    float limit = _Clamp(assist_torque_limit_nm, 0.0f, HARD_MAX_ASSIST_TORQUE_NM);

    /* 7. control law ---------------------------------------------------- *
     * STANCE (stair confirmed): tau = -G_phi * K_EMG * a * Ramp  (extension)
     * SWING  (optional)       : tau = +FLEX_ASSIST_NM            (flexion)   */
    float tau_R_raw = 0.0f;
    float tau_L_raw = 0.0f;

    /* Right leg */
    if (safe_R) {
        if (s_right.phase != GAIT_PHASE_SWING) {
            if (s_stair_latch_R) {
                float g   = use_angle_onset ? (s_pullup[0] ? 1.0f : 0.0f)
                                            : _PhaseEnvelope(s_stair_R.phase);
                float mag = _Clamp((assist_base_nm + K_EMG * s_emg_act[0]) * g, 0.0f, limit);
                if (use_assist_level_scale == 1U) {
                    mag *= (float)XM.status.h10.h10AssistLevel * 0.1f;
                }
                tau_R_raw = -mag;   /* extension = negative */
            }
        } else if (use_flexion_assist == 1U) {
            if (thigh_R > FLEX_START_DEG && thigh_R < FLEX_END_DEG) {
                tau_R_raw = FLEX_ASSIST_NM;   /* flexion = positive */
            }
        }
    }

    /* Left leg */
    if (safe_L) {
        if (s_left.phase != GAIT_PHASE_SWING) {
            if (s_stair_latch_L) {
                float g   = use_angle_onset ? (s_pullup[1] ? 1.0f : 0.0f)
                                            : _PhaseEnvelope(s_stair_L.phase);
                float mag = _Clamp((assist_base_nm + K_EMG * s_emg_act[1]) * g, 0.0f, limit);
                if (use_assist_level_scale == 1U) {
                    mag *= (float)XM.status.h10.h10AssistLevel * 0.1f;
                }
                tau_L_raw = -mag;
            }
        } else if (use_flexion_assist == 1U) {
            if (thigh_L > FLEX_START_DEG && thigh_L < FLEX_END_DEG) {
                tau_L_raw = FLEX_ASSIST_NM;
            }
        }
    }

    /* 8. ramp-in x slew-rate limit x final clamp ------------------------ */
    float ramp = _Clamp((float)(now_ms - s_active_entry_ms) * 1e-3f / RAMP_DURATION_S,
                        0.0f, 1.0f);
    tau_R_raw *= ramp;
    tau_L_raw *= ramp;

    const float max_step = SLEW_RATE_LIMIT_NM_PER_S * CONTROL_DT_S;
    s_tau_prev[0] += _Clamp(tau_R_raw - s_tau_prev[0], -max_step, max_step);
    s_tau_prev[1] += _Clamp(tau_L_raw - s_tau_prev[1], -max_step, max_step);

    /* Extension-only by default; allow positive only when swing-flexion is enabled. */
    float tau_hi = (use_flexion_assist == 1U) ? limit : 0.0f;
    s_tau_prev[0] = _Clamp(s_tau_prev[0], -limit, tau_hi);
    s_tau_prev[1] = _Clamp(s_tau_prev[1], -limit, tau_hi);

    XM_SetAssistTorqueRH(s_tau_prev[0]);
    XM_SetAssistTorqueLH(s_tau_prev[1]);

    assist_enable  = (safe_R || safe_L) ? 1U : 0U;
    mask_bilateral = mask;
    is_stair_R     = s_stair_latch_R ? 1U : 0U;
    is_stair_L     = s_stair_latch_L ? 1U : 0U;

    /* 9. publish + stream ----------------------------------------------- */
    tau_R_cmd_nm = s_tau_prev[0]; tau_L_cmd_nm = s_tau_prev[1];
    _PublishSignals();   /* also publishes thigh/pelvic (in STANDBY + ACTIVE) */
    _SendStream();
    _UsbDebug(now_ms);
}

static void Active_Exit(void)
{
    XM_SetAssistTorqueRH(0.0f);
    XM_SetAssistTorqueLH(0.0f);
    XM_SetControlMode(XM_CTRL_MONITOR);
    XM_SetH10AssistExistingMode(false);   /* default built-in H10 assist OFF — suit passive unless our controller drives it */
    s_tau_prev[0] = s_tau_prev[1] = 0.0f;
    tau_R_cmd_nm = tau_L_cmd_nm = 0.0f;
    assist_enable = 0U;
    is_stair_R = is_stair_L = 0U;
    XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
    XM_SendUsbDebugMessage("[FP-STAIR] ACTIVE exit — torque cleared\r\n");
}

/* ============================================================================
 * SIGNAL PROCESSING
 * ============================================================================ */
static void _SampleAdc(void)
{
    for (int i = 0; i < ADC_CH_COUNT; i++) {
        s_v[i] = (float)XM_AnalogReadMillivolts(s_adc_pins[i]) * 0.001f;
    }
}

static void _ProcessEmg(void)
{
    float a_pre = _LpfAlphaRC(EMG_PRE_LPF_FC_HZ);
    float a_env = _LpfAlphaRC(EMG_ENV_LPF_FC_HZ);
    for (int i = 0; i < EMG_CH_COUNT; i++) {
        /* s_v[CH_EMG_R + i]: i=0 -> ADC_9 (R), i=1 -> ADC_10 (L) */
        float centered = s_v[CH_EMG_R + i] - s_emg_bias[i];
        s_emg_pre[i] += a_pre * (centered - s_emg_pre[i]);
        float rect = _Abs(s_emg_pre[i]);
        s_emg_env[i] += a_env * (rect - s_emg_env[i]);

        float mvic = (i == 0) ? EMG_MVIC_R : EMG_MVIC_L;
        if (mvic < EMG_MVIC_MIN_V) mvic = EMG_MVIC_MIN_V;
        float active = s_emg_env[i] - EMG_ENV_DEADBAND_V;
        if (active < 0.0f) active = 0.0f;
        float act_raw = active / mvic;            /* ~0..1 fraction of MVIC */
        s_emg_act[i] = EMG_ACT_GAMMA * s_emg_act[i] + (1.0f - EMG_ACT_GAMMA) * act_raw;
    }
}

static void _ProcessFsr(void)
{
    float a = _LpfAlphaRC(FSR_LPF_FC_HZ);
    for (int i = 0; i < FSR_CH_COUNT; i++) {
        float raw = s_v[CH_FSR_RH + i];           /* RH, RT, LH, LT (indices 0..3) */
        if (!s_filter_init) {
            s_fsr_lpf[i] = raw;
        } else {
            s_fsr_lpf[i] += a * (raw - s_fsr_lpf[i]);
        }
        float span = s_fsr_on[i] - s_fsr_off[i];
        if (span < FSR_MIN_SPAN_V) span = FSR_MIN_SPAN_V;
        s_fsr_load[i] = _Clamp((s_fsr_lpf[i] - s_fsr_off[i]) / span, 0.0f, 1.5f);
    }
    s_filter_init = true;
}

/* Ordered, single-direction gait FSM anchored on foot contact:
 *   SWING -> HEEL_STRIKE -> LOAD_RESPONSE -> TERMINAL_STANCE -> SWING -> ...
 * Each transition needs a minimum dwell (anti-chatter), so LOAD_RESPONSE — the
 * pull-up trigger that resets the stair-phase clock — fires exactly once per
 * cycle (re-armed only by passing through SWING), instead of relying on a
 * per-sample max-membership argmax that can chatter or fire out of order.
 * Fuzzy memberships still set the heel/toe "loaded" decisions and d->mu (kept
 * for telemetry/inspection). */
static uint16_t _UpdateFoot(FootFuzzy_t *d, float heel_load, float toe_load, uint32_t now_ms)
{
    float heel_large = _MembershipLarge(heel_load, fuzzy_heel_threshold, fuzzy_sensitivity);
    float toe_large  = _MembershipLarge(toe_load,  fuzzy_toe_threshold,  fuzzy_sensitivity);
    bool  heel_on = (heel_large > MEMBERSHIP_ON);
    bool  toe_on  = (toe_large  > MEMBERSHIP_ON);
    bool  contact = heel_on || toe_on;

    d->mu[GAIT_PHASE_HEEL_STRIKE]     = heel_large * (1.0f - toe_large);
    d->mu[GAIT_PHASE_LOAD_RESPONSE]   = heel_large * toe_large;
    d->mu[GAIT_PHASE_TERMINAL_STANCE] = (1.0f - heel_large) * toe_large;
    d->mu[GAIT_PHASE_SWING]           = (1.0f - heel_large) * (1.0f - toe_large);

    if (!d->initialized) {
        d->phase = GAIT_PHASE_SWING;
        d->prev_phase = GAIT_PHASE_SWING;
        d->phase_since_ms = now_ms;
        d->initialized = true;
        return GAIT_EVENT_NONE;
    }

    FootGaitPhase_t next = d->phase;
    if ((now_ms - d->phase_since_ms) >= PHASE_MIN_DWELL_MS) {
        switch (d->phase) {
        case GAIT_PHASE_SWING:
            if (contact) next = GAIT_PHASE_HEEL_STRIKE;             /* foot lands */
            break;
        case GAIT_PHASE_HEEL_STRIKE:
            if (heel_on && toe_on) {
                next = GAIT_PHASE_LOAD_RESPONSE;                    /* weight accepted */
            } else if (contact &&
                       (now_ms - d->phase_since_ms) >= (PHASE_MIN_DWELL_MS * 2U)) {
                next = GAIT_PHASE_LOAD_RESPONSE;                    /* metatarsal-first fallback */
            }
            break;
        case GAIT_PHASE_LOAD_RESPONSE:
            if (!heel_on) next = GAIT_PHASE_TERMINAL_STANCE;        /* heel-off */
            break;
        case GAIT_PHASE_TERMINAL_STANCE:
            if (!contact) next = GAIT_PHASE_SWING;                  /* toe-off */
            break;
        default:
            next = GAIT_PHASE_SWING;
            break;
        }
    }

    uint16_t ev = GAIT_EVENT_NONE;
    if (next != d->phase) {
        d->prev_phase = d->phase;
        d->phase = next;
        d->phase_since_ms = now_ms;
        switch (next) {
        case GAIT_PHASE_HEEL_STRIKE:     ev |= GAIT_EVENT_HEEL_STRIKE;     break;
        case GAIT_PHASE_LOAD_RESPONSE:   ev |= GAIT_EVENT_LOAD_RESPONSE;   break;
        case GAIT_PHASE_TERMINAL_STANCE: ev |= GAIT_EVENT_TERMINAL_STANCE; break;
        case GAIT_PHASE_SWING:           ev |= GAIT_EVENT_TOE_OFF;         break;
        default: break;
        }
    }
    return ev;
}

static void _UpdateStairEstim(StairEstim_t *e, bool loading_response, uint32_t now_ms)
{
    if (loading_response) {
        if (e->has_first_lr) {
            float elapsed = (float)(now_ms - e->last_lr_ms) * 1e-3f;
            e->period_s = _Clamp(elapsed, STAIR_PERIOD_MIN_S, STAIR_PERIOD_MAX_S);
        } else {
            e->has_first_lr = true;
        }
        e->phase = 0.0f;
        e->last_lr_ms = now_ms;
    }
    e->phase += CONTROL_DT_S / e->period_s;
    if (e->phase >= 1.0f) e->phase -= 1.0f;
}

static void _ResetRuntime(void)
{
    for (int i = 0; i < EMG_CH_COUNT; i++) {
        s_emg_pre[i] = s_emg_env[i] = s_emg_act[i] = 0.0f;
    }
    memset(&s_right, 0, sizeof(s_right));
    memset(&s_left,  0, sizeof(s_left));
    s_right.phase = s_right.prev_phase = GAIT_PHASE_SWING;
    s_left.phase  = s_left.prev_phase  = GAIT_PHASE_SWING;
    s_stair_R.phase = s_stair_L.phase = 0.0f;
    s_stair_R.period_s = s_stair_L.period_s = STAIR_PERIOD_INIT_S;
    s_stair_R.has_first_lr = s_stair_L.has_first_lr = false;
    s_stair_latch_R = s_stair_latch_L = false;
    s_pullup[0] = s_pullup[1] = false;
    s_thigh_rate[0] = s_thigh_rate[1] = 0.0f;
    s_thigh_prev[0] = XM.status.h10.rightThighAngle;   /* seed to avoid a first-loop rate spike */
    s_thigh_prev[1] = XM.status.h10.leftThighAngle;
    s_prev_mask = 0U;
    s_mask_since_ms = XM_GetTick();
    s_standing = false;
    s_tau_prev[0] = s_tau_prev[1] = 0.0f;
}

/* ============================================================================
 * CALIBRATION (runs in STANDBY) — BTN1 EMG (bias->MVIC), BTN2 FSR (off->on)
 * ============================================================================ */
static void _HandleCalibration(void)
{
    XmBtnEvent_t b1 = XM_GetButtonEvent(XM_BTN_1);
    XmBtnEvent_t b2 = XM_GetButtonEvent(XM_BTN_2);
    XmBtnEvent_t b3 = XM_GetButtonEvent(XM_BTN_3);
    uint32_t now = XM_GetTick();
    bool busy = (s_cal_state != CAL_IDLE);

    if (!busy) {
        if (b3 == XM_BTN_CLICK) {
            cal_emg_bias_done = cal_emg_mvic_done = 0U;
            cal_fsr_off_done = cal_fsr_on_done = 0U;
            for (int i = 0; i < EMG_CH_COUNT; i++) s_emg_bias[i] = EMG_BIAS_DEFAULT_V;
            EMG_MVIC_R = EMG_MVIC_L = EMG_MVIC_DEFAULT_V;
            for (int i = 0; i < FSR_CH_COUNT; i++) { s_fsr_off[i] = 0.0f; s_fsr_on[i] = FSR_ON_DEFAULT_V; }
            XM_SetLedEffect(XM_LED_1, XM_LED_ONESHOT, 500);
            XM_SendUsbDebugMessage("[CAL] reset to defaults\r\n");
            return;
        }
        if (b1 == XM_BTN_CLICK) {
            s_cal_state = cal_emg_bias_done ? CAL_EMG_MVIC_RUN : CAL_EMG_BIAS_RUN;
            s_cal_start_ms = now; s_cal_count = 0U;
            s_cal_sum_emg[0] = s_cal_sum_emg[1] = 0.0;
            s_cal_max_env[0] = s_cal_max_env[1] = 0.0f;
            XM_SetLedEffect(XM_LED_2, XM_LED_BLINK, 100);
            XM_SendUsbDebugMessage(cal_emg_bias_done
                ? "[CAL] EMG MVIC — max hamstring contraction (3 s)\r\n"
                : "[CAL] EMG bias — relax hamstrings (3 s)\r\n");
            return;
        }
        if (b2 == XM_BTN_CLICK) {
            if (!cal_fsr_off_done) {
                s_cal_state = CAL_FSR_OFF_RUN;
                XM_SendUsbDebugMessage("[CAL] FSR off — lift both feet (1 s)\r\n");
            } else {
                s_cal_state = CAL_FSR_ON_RUN;
                XM_SendUsbDebugMessage("[CAL] FSR on — stand on both feet (1 s)\r\n");
            }
            s_cal_start_ms = now; s_cal_count = 0U;
            for (int i = 0; i < FSR_CH_COUNT; i++) s_cal_sum_fsr[i] = 0.0;
            XM_SetLedEffect(XM_LED_3, XM_LED_BLINK, 100);
            return;
        }
        return;
    }

    /* accumulate while a capture is running */
    switch (s_cal_state) {
    case CAL_EMG_BIAS_RUN:
        s_cal_sum_emg[0] += s_v[CH_EMG_R];
        s_cal_sum_emg[1] += s_v[CH_EMG_L];
        s_cal_count++;
        if ((now - s_cal_start_ms) >= EMG_CAL_MS && s_cal_count > 0U) {
            s_emg_bias[0] = (float)(s_cal_sum_emg[0] / (double)s_cal_count);
            s_emg_bias[1] = (float)(s_cal_sum_emg[1] / (double)s_cal_count);
            cal_emg_bias_done = 1U;
            s_cal_state = CAL_IDLE;
            XM_SendUsbDebugMessage("[CAL] EMG bias done\r\n");
        }
        break;
    case CAL_EMG_MVIC_RUN:
        if (s_emg_env[0] > s_cal_max_env[0]) s_cal_max_env[0] = s_emg_env[0];
        if (s_emg_env[1] > s_cal_max_env[1]) s_cal_max_env[1] = s_emg_env[1];
        if ((now - s_cal_start_ms) >= EMG_CAL_MS) {
            EMG_MVIC_R = (s_cal_max_env[0] > EMG_MVIC_MIN_V) ? s_cal_max_env[0] : EMG_MVIC_MIN_V;
            EMG_MVIC_L = (s_cal_max_env[1] > EMG_MVIC_MIN_V) ? s_cal_max_env[1] : EMG_MVIC_MIN_V;
            cal_emg_mvic_done = 1U;
            s_cal_state = CAL_IDLE;
            XM_SendUsbDebugMessage("[CAL] EMG MVIC done\r\n");
        }
        break;
    case CAL_FSR_OFF_RUN:
    case CAL_FSR_ON_RUN:
        for (int i = 0; i < FSR_CH_COUNT; i++) s_cal_sum_fsr[i] += s_fsr_lpf[i];
        s_cal_count++;
        if ((now - s_cal_start_ms) >= FSR_CAL_MS && s_cal_count > 0U) {
            for (int i = 0; i < FSR_CH_COUNT; i++) {
                float avg = (float)(s_cal_sum_fsr[i] / (double)s_cal_count);
                if (s_cal_state == CAL_FSR_OFF_RUN) s_fsr_off[i] = avg;
                else                                s_fsr_on[i]  = avg;
            }
            if (s_cal_state == CAL_FSR_OFF_RUN) cal_fsr_off_done = 1U;
            else                                cal_fsr_on_done  = 1U;
            s_cal_state = CAL_IDLE;
            XM_SendUsbDebugMessage("[CAL] FSR capture done\r\n");
        }
        break;
    default:
        s_cal_state = CAL_IDLE;
        break;
    }
}

static bool _AllCalDone(void)
{
    return cal_emg_bias_done && cal_emg_mvic_done && cal_fsr_off_done && cal_fsr_on_done;
}

/* ============================================================================
 * PUBLISH + TELEMETRY
 * ============================================================================ */
static void _PublishSignals(void)
{
    emg_R_raw_v = s_v[CH_EMG_R];   emg_L_raw_v = s_v[CH_EMG_L];
    emg_R_env_v = s_emg_env[0];    emg_L_env_v = s_emg_env[1];
    emg_R_act   = s_emg_act[0];    emg_L_act   = s_emg_act[1];

    fsr_RH_raw_v = s_v[CH_FSR_RH]; fsr_RT_raw_v = s_v[CH_FSR_RT];
    fsr_LH_raw_v = s_v[CH_FSR_LH]; fsr_LT_raw_v = s_v[CH_FSR_LT];
    fsr_RH_load = s_fsr_load[0];   fsr_RT_load = s_fsr_load[1];
    fsr_LH_load = s_fsr_load[2];   fsr_LT_load = s_fsr_load[3];

    right_gait_phase = (uint16_t)s_right.phase;
    left_gait_phase  = (uint16_t)s_left.phase;
    phase_R = s_stair_R.phase; phase_L = s_stair_L.phase;
    period_R_s = s_stair_R.period_s; period_L_s = s_stair_L.period_s;
    standing_flag = s_standing ? 1U : 0U;
    calibration_ready = _AllCalDone() ? 1U : 0U;

    /* Hip/thigh encoder + pelvic angle — published in BOTH STANDBY and ACTIVE so
     * the encoder can be monitored on the bench before the controller goes ACTIVE. */
    thigh_R_deg = XM.status.h10.rightThighAngle;
    thigh_L_deg = XM.status.h10.leftThighAngle;
    pelvic_angle_deg = XM.status.h10.pelvicAngle;
}

static void _SendStream(void)
{
    uint32_t now = XM_GetTick();
    if (cdc_stream_enable != 1U || cdc_stream_period_ms == 0U ||
        (now - s_stream_ms) < cdc_stream_period_ms) {
        return;
    }
    s_stream_ms = now;

    s_stream.emg_R_env = s_emg_env[0];  s_stream.emg_L_env = s_emg_env[1];
    s_stream.emg_R_act = s_emg_act[0];  s_stream.emg_L_act = s_emg_act[1];
    s_stream.fsr_RH_load = s_fsr_load[0]; s_stream.fsr_RT_load = s_fsr_load[1];
    s_stream.fsr_LH_load = s_fsr_load[2]; s_stream.fsr_LT_load = s_fsr_load[3];
    s_stream.phase_R = s_stair_R.phase; s_stream.phase_L = s_stair_L.phase;
    s_stream.gait_R = (float)s_right.phase; s_stream.gait_L = (float)s_left.phase;
    s_stream.thigh_R = XM.status.h10.rightThighAngle;
    s_stream.thigh_L = XM.status.h10.leftThighAngle;
    s_stream.tau_R = s_tau_prev[0]; s_stream.tau_L = s_tau_prev[1];

    XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), USB_MODULE_ID);
}

static void _UsbDebug(uint32_t now_ms)
{
    if (now_ms - s_usb_dbg_ms < USB_DEBUG_PERIOD_MS) return;
    s_usb_dbg_ms = now_ms;
    char buf[180];
    snprintf(buf, sizeof(buf),
        "FP-STAIR on:%u rdy:%u mask:0x%X st:%u stair R/L:%u/%u | aR/L:%.2f/%.2f phR/L:%.2f/%.2f | T R/L:%.2f/%.2f\r\n",
        (unsigned)control_ON, (unsigned)calibration_ready, (unsigned)s_prev_mask,
        (unsigned)standing_flag, (unsigned)is_stair_R, (unsigned)is_stair_L,
        (double)s_emg_act[0], (double)s_emg_act[1],
        (double)s_stair_R.phase, (double)s_stair_L.phase,
        (double)s_tau_prev[0], (double)s_tau_prev[1]);
    XM_SendUsbDebugMessage(buf);
}

/* ============================================================================
 * MATH UTILITIES
 * ============================================================================ */
static float _LpfAlphaRC(float fc_hz)
{
    float rc = 1.0f / (TWO_PI * fc_hz);
    return CONTROL_DT_S / (rc + CONTROL_DT_S);
}

static float _Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float _Abs(float x) { return (x < 0.0f) ? -x : x; }

static float _MembershipLarge(float v, float thr, float sens)
{
    return 0.5f * (tanhf(sens * (v - thr)) + 1.0f);
}

/* F-vector pull-up envelope: peak 1 at phi = START + TP (=0.15), zero outside. */
static float _PhaseEnvelope(float phi)
{
    float rel = phi - PHASE_PULSE_START;
    if (rel <= 0.0f || rel >= PHASE_PULSE_WINDOW) return 0.0f;
    float x = rel / PHASE_PULSE_TP;
    return x * expf(1.0f - x);
}

/* ============================================================================
 * END
 * ============================================================================ */
