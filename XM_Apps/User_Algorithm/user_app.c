/**
 ******************************************************************************
 * @file    user_app.c
 * @brief   Final Project — EMG-gated, FSR-phase-shaped hip-extension assist for
 *          STAIR ASCENT (combined Final_FSR_Fuzzy_Logic + Final_EMG backbone).
 * @details
 * ============================================================================
 * Scenario (KAIST ME491A Final Project, Team 2 — stair ascent)
 * ============================================================================
 * Target user : adults with reduced hip-extensor capacity (older adults with mild
 *               sarcopenia, early post-op rehab, stair-loaded workers) who can
 *               climb stairs but at a disproportionately high per-step effort.
 * Motion      : continuous stair ascent (~1 step/s/leg) on a stair treadmill.
 * Difficulty  : the pull-up phase (weight acceptance, ~5-30% of the stair
 *               cycle) demands a large hip-extensor moment (gluteus maximus and
 *               hamstrings). EMG is recorded from the HAMSTRING (biceps femoris)
 *               as the accessible hip-extensor under the suit.
 * Assistance  : a brief, phase-locked hip-EXTENSION torque pulse during pull-up
 *               whose MAGNITUDE is scaled by the user's own hamstring (hip-
 *               extensor) EMG (assist-as-needed), and whose TIMING is gated by
 *               an FSR fuzzy gait-phase detector.
 *
 * Control law per leg z in {R, L}:
 *     tau_cmd_z(t) = - G_phi(phi_z) * K_EMG * a_ham_z * Ramp(t) * 1[safe_z]
 *
 *   - leading minus sign  => hip EXTENSION torque (negative on H10 convention)
 *   - FSR fuzzy detector  => WHEN to assist (loading-response = pull-up start)
 *   - G_phi(phi)          => smooth F-vector pulse over the pull-up window
 *   - EMG activation a    => HOW MUCH (proportional to hamstring/hip-extensor effort)
 *
 * ============================================================================
 * Sensor channel allocation (all 5 V powered, read in millivolts)
 * ============================================================================
 *   PF3 (DIO_1 -> XM_EXT_ADC_5)   sEMG  R hamstring (biceps femoris)
 *   PF4 (DIO_2 -> XM_EXT_ADC_6)   sEMG  L hamstring (biceps femoris)
 *   PF5 (DIO_3 -> XM_EXT_ADC_7)   FSR   R heel
 *   PF6 (DIO_4 -> XM_EXT_ADC_8)   FSR   R toe
 *   PF7 (DIO_5 -> XM_EXT_ADC_9)   FSR   L heel
 *   PF8 (DIO_6 -> XM_EXT_ADC_10)  FSR   L toe
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
 * Safety (per leg, all must hold or torque = 0)
 * ============================================================================
 *   control_ON == 1 ; calibration_ready ; H10 mode == ASSIST ;
 *   thigh angle within [THIGH_MIN, THIGH_MAX] ; NOT standing ;
 *   stair period within [PERIOD_MIN, PERIOD_MAX] ; pelvic >= pelvic_incline_min.
 *   Output is extension-only, hard-clamped to [-HARD_MAX, 0], slew-limited,
 *   and ramped in over RAMP_DURATION_S on ACTIVE entry.
 *
 * ============================================================================
 * USB-CDC telemetry  (Module 0xF0, 16 floats)  -> PC logger preset
 *   "Final_Stair_Assist" in PythonDecoder/CDC/cdc_selective_logger.py
 * ============================================================================
 *   EMG R env, EMG L env, EMG R act, EMG L act,
 *   FSR RH load, FSR RT load, FSR LH load, FSR LT load,
 *   Phase R, Phase L, Gait R, Gait L,
 *   Thigh R, Thigh L, Tau R, Tau L      (Tau negative = extension)
 *
 * @version 1.0
 * @date    2026-06-03
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

/* ADC channel order (matches s_adc_pins / DIO_1..6) */
typedef enum {
    CH_EMG_R = 0,   /* PF3 */
    CH_EMG_L,       /* PF4 */
    CH_FSR_RH,      /* PF5 */
    CH_FSR_RT,      /* PF6 */
    CH_FSR_LH,      /* PF7 */
    CH_FSR_LT,      /* PF8 */
    ADC_CH_COUNT
} AdcCh_t;

#define EMG_CH_COUNT              2
#define FSR_CH_COUNT              4

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
float    assist_torque_limit_nm = 2.0f;     /* project limit, <= HARD_MAX */
float    EMG_MVIC_R          = EMG_MVIC_DEFAULT_V;
float    EMG_MVIC_L          = EMG_MVIC_DEFAULT_V;

float    fuzzy_heel_threshold = 0.35f;
float    fuzzy_toe_threshold  = 0.35f;
float    fuzzy_sensitivity    = 12.0f;

float    pelvic_incline_min_deg = -90.0f;   /* permissive default; raise to gate on incline */
uint16_t use_assist_level_scale = 0U;       /* 1 = scale torque by H10 assist-level (0..10); 0 = full (deterministic) */

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
uint16_t cal_emg_bias_done, cal_emg_mvic_done, cal_fsr_off_done, cal_fsr_on_done;
uint16_t left_gait_phase, right_gait_phase;
float    phase_R, phase_L;
float    period_R_s, period_L_s;
float    thigh_R_deg, thigh_L_deg, pelvic_angle_deg;
float    tau_R_cmd_nm, tau_L_cmd_nm;        /* negative = extension */
uint16_t assist_enable;

/* ============================================================================
 * STATIC STATE
 * ============================================================================ */
static XmTsmHandle_t s_tsm;

static const XmAdcPin_t s_adc_pins[ADC_CH_COUNT] = {
    XM_EXT_ADC_5,   /* PF3 EMG R */
    XM_EXT_ADC_6,   /* PF4 EMG L */
    XM_EXT_ADC_7,   /* PF5 FSR RH */
    XM_EXT_ADC_8,   /* PF6 FSR RT */
    XM_EXT_ADC_9,   /* PF7 FSR LH */
    XM_EXT_ADC_10   /* PF8 FSR LT */
};

static float s_v[ADC_CH_COUNT];             /* latest raw volts per channel */

/* EMG state (index 0=R, 1=L) */
static float s_emg_bias[EMG_CH_COUNT] = { EMG_BIAS_DEFAULT_V, EMG_BIAS_DEFAULT_V };
static float s_emg_pre[EMG_CH_COUNT];
static float s_emg_env[EMG_CH_COUNT];
static float s_emg_act[EMG_CH_COUNT];

/* FSR state (index per AdcCh offset: RH,RT,LH,LT) */
static float s_fsr_lpf[FSR_CH_COUNT];
static float s_fsr_off[FSR_CH_COUNT];
static float s_fsr_on[FSR_CH_COUNT]  = { FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V,
                                         FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V };
static float s_fsr_load[FSR_CH_COUNT];
static bool  s_filter_init;

/* Fuzzy detectors + bilateral mask */
static FootFuzzy_t  s_right, s_left;
static StairEstim_t s_stair_R = { 0.0f, STAIR_PERIOD_INIT_S, 0U, false };
static StairEstim_t s_stair_L = { 0.0f, STAIR_PERIOD_INIT_S, 0U, false };
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
    XM_SetH10AssistExistingMode(true);
    XM_SetControlMode(XM_CTRL_MONITOR);

    XM_SendUsbDebugMessage("[FP-STAIR] boot — EMG-gated FSR-phase hip-extension assist\r\n");
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
    if (XM_IsCmConnected()) {
        XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
        XM_SendUsbDebugMessage("[FP-STAIR] CM connected -> STANDBY (calibrate, then ASSIST)\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
    }
}

static void Standby_Loop(void)
{
    /* Calibration is performed here, before entering ASSIST. */
    _SampleAdc();
    _ProcessEmg();   /* needed so MVIC capture sees a live envelope */
    _ProcessFsr();
    _HandleCalibration();

    XM_SetLedState(XM_LED_2, (cal_emg_bias_done && cal_emg_mvic_done) ? XM_ON : XM_OFF);
    XM_SetLedState(XM_LED_3, (cal_fsr_off_done  && cal_fsr_on_done)   ? XM_ON : XM_OFF);

    _PublishSignals();
    _SendStream();

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST && _AllCalDone()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    XM_SetH10AssistExistingMode(false);   /* take over from built-in assist */
    XM_SetControlMode(XM_CTRL_TORQUE);

    _ResetRuntime();
    s_active_entry_ms = XM_GetTick();
    s_usb_dbg_ms      = XM_GetTick();

    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 200);
    XM_SendUsbDebugMessage("[FP-STAIR] ACTIVE — pull-up extension assist\r\n");
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
    uint16_t ev_R = _UpdateFoot(&s_right, s_fsr_load[0], s_fsr_load[1], now_ms); /* RH, RT */
    uint16_t ev_L = _UpdateFoot(&s_left,  s_fsr_load[2], s_fsr_load[3], now_ms); /* LH, LT */

    /* 3. bilateral contact mask + standing detection -------------------- */
    bool rh = s_fsr_load[0] >= fuzzy_heel_threshold;
    bool rt = s_fsr_load[1] >= fuzzy_toe_threshold;
    bool lh = s_fsr_load[2] >= fuzzy_heel_threshold;
    bool lt = s_fsr_load[3] >= fuzzy_toe_threshold;
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

    /* 5. safety predicates ---------------------------------------------- */
    float thigh_R = XM.status.h10.rightThighAngle;
    float thigh_L = XM.status.h10.leftThighAngle;
    float pelvic  = XM.status.h10.pelvicAngle;

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

    /* 6. control law: tau = -G_phi * K_EMG * a * Ramp ------------------- */
    float limit = _Clamp(assist_torque_limit_nm, 0.0f, HARD_MAX_ASSIST_TORQUE_NM);
    float g_R = _PhaseEnvelope(s_stair_R.phase);
    float g_L = _PhaseEnvelope(s_stair_L.phase);

    float mag_R = safe_R ? _Clamp(K_EMG * s_emg_act[0] * g_R, 0.0f, limit) : 0.0f;
    float mag_L = safe_L ? _Clamp(K_EMG * s_emg_act[1] * g_L, 0.0f, limit) : 0.0f;

    /* Optional: let the wearer's H10 assist-level button (0..10) scale magnitude.
     * Off by default so experiment torque stays deterministic (B0/E1 not confounded). */
    if (use_assist_level_scale == 1U) {
        float lvl = (float)XM.status.h10.h10AssistLevel * 0.1f;
        mag_R *= lvl;
        mag_L *= lvl;
    }

    float tau_R_raw = -mag_R;   /* extension = negative */
    float tau_L_raw = -mag_L;

    /* ramp-in */
    float ramp = _Clamp((float)(now_ms - s_active_entry_ms) * 1e-3f / RAMP_DURATION_S,
                        0.0f, 1.0f);
    tau_R_raw *= ramp;
    tau_L_raw *= ramp;

    /* slew-rate limit */
    const float max_step = SLEW_RATE_LIMIT_NM_PER_S * CONTROL_DT_S;
    s_tau_prev[0] += _Clamp(tau_R_raw - s_tau_prev[0], -max_step, max_step);
    s_tau_prev[1] += _Clamp(tau_L_raw - s_tau_prev[1], -max_step, max_step);

    /* extension-only final saturation */
    s_tau_prev[0] = _Clamp(s_tau_prev[0], -limit, 0.0f);
    s_tau_prev[1] = _Clamp(s_tau_prev[1], -limit, 0.0f);

    XM_SetAssistTorqueRH(s_tau_prev[0]);
    XM_SetAssistTorqueLH(s_tau_prev[1]);

    assist_enable = (safe_R || safe_L) ? 1U : 0U;
    mask_bilateral = mask;

    /* 7. publish + stream ----------------------------------------------- */
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
    XM_SetH10AssistExistingMode(true);
    s_tau_prev[0] = s_tau_prev[1] = 0.0f;
    tau_R_cmd_nm = tau_L_cmd_nm = 0.0f;
    assist_enable = 0U;
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
        float centered = s_v[i] - s_emg_bias[i];
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
        float raw = s_v[CH_FSR_RH + i];           /* RH, RT, LH, LT */
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
    char buf[160];
    snprintf(buf, sizeof(buf),
        "FP-STAIR on:%u rdy:%u mask:0x%X st:%u | aR/L:%.2f/%.2f phR/L:%.2f/%.2f | T R/L:%.2f/%.2f\r\n",
        (unsigned)control_ON, (unsigned)calibration_ready, (unsigned)s_prev_mask,
        (unsigned)standing_flag, (double)s_emg_act[0], (double)s_emg_act[1],
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
