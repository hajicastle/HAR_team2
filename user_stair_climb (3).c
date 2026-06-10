// user_app.c
/**

───

• @file user_app.c
• @brief Stair Ascent Assist — Kinematic Latch & EMG Proportional Controller
• @details
◦ 상체 기울기를 배제하고 착지 순간의 고관절 각도를 래치(Latch)하여 계단을 판단합니다.
◦ 프로젝트 요구사항에 맞춰, 계단을 오를 때 PF7, PF8에 연결된 EMG 센서 데이터에
• 비례(K_EMG)하는 신전 토크를 양 다리에 각각 독립적으로 인가합니다.

───

*/

#include "xm_api.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
• SYSTEM CONSTANTS
• ============================================================================ */
#define CONTROL_DT_S 0.001f
#define TWO_PI 6.28318530718f
#define USB_MODULE_ID 0xF0U
#define USB_DEBUG_PERIOD_MS 500U
#define HARD_MAX_ASSIST_TORQUE_NM 3.5f

#define USER_WEIGHT_KG_X10 700U
 #define USER_HEIGHT_CM_X10 1750U

/* ----------------------------------------------------------------------------
• 핀 매핑: FSR (PF3~PF6), EMG (PF7~PF8)
• ---------------------------------------------------------------------------- /
typedef enum {
CH_FSR_RH = 0, / PF3 (DIO 1): Right Heel FSR /
CH_FSR_RT, / PF4 (DIO 2): Right Toe FSR /
CH_FSR_LH, / PF5 (DIO 3): Left Heel FSR /
CH_FSR_LT, / PF6 (DIO 4): Left Toe FSR /
CH_EMG_R, / PF7 (DIO 5): Right EMG /
CH_EMG_L, / PF8 (DIO 6): Left EMG */
ADC_CH_COUNT
} AdcCh_t;

#define FSR_CH_COUNT 4
#define EMG_CH_COUNT 2

/* 파이프라인 설정 */
#define FSR_LPF_FC_HZ 8.0f
#define FSR_MIN_SPAN_V 0.05f
#define FSR_ON_DEFAULT_V 1.0f
#define FSR_CAL_MS 1000U

#define EMG_BIAS_DEFAULT_V 1.65f
#define EMG_PRE_LPF_FC_HZ 80.0f
#define EMG_ENV_LPF_FC_HZ 5.0f
#define EMG_ENV_DEADBAND_V 0.020f
#define EMG_MVIC_DEFAULT_V 1.0f
#define EMG_MVIC_MIN_V 0.050f
#define EMG_ACT_GAMMA 0.95f
#define EMG_CAL_MS 3000U

#define STANDING_DWELL_S 0.7f

/* ============================================================================
• EXPERIMENT TUNING PARAMETERS (튜닝 파라미터)
• ============================================================================ /
#define SWING_FLEXION_START_DEG 30.0f / 다리 들어 올릴 때 밀어주기 시작할 각도 /
#define SWING_FLEXION_END_DEG 45.0f / 이 각도를 넘으면 다리 들기 토크 끔 (저항 제거) /
#define FLEXION_ASSIST_TORQUE 1.5f / 다리 들어 올리는 고정 굴곡 보조력 (Nm, +) */

#define STANCE_EXTENSION_MIN_DEG 25.0f /* [핵심] 땅을 디딘 순간, 고관절이 이 각도 이상이어야 계단으로 인정! */

/* Safety Limits */
#define SLEW_RATE_LIMIT_NM_PER_S 40.0f
 #define RAMP_DURATION_S 1.5f
#define THIGH_ANGLE_MIN_DEG (-10.0f)
#define THIGH_ANGLE_MAX_DEG (90.0f)

typedef enum { CAL_IDLE = 0, CAL_FSR_OFF_RUN, CAL_FSR_ON_RUN, CAL_EMG_BIAS_RUN, CAL_EMG_MVIC_RUN } CalState_t;

typedef struct {
float emg_R_act, emg_L_act;
float fsr_RH_load, fsr_RT_load, fsr_LH_load, fsr_LT_load;
float thigh_R, thigh_L;
float tau_R, tau_L;
float is_stair_R, is_stair_L;
} StairStreamData_t;

/* ============================================================================
• LIVE EXPRESSION GLOBALS
• ============================================================================ /
uint16_t control_ON = 0U;
 float assist_torque_limit_nm = 3.0f;
 float K_EMG = 2.5f; / [튜닝 필수] EMG 비례 제어 이득. 높일수록 세게 밀어줍니다. */

float fuzzy_heel_threshold = 0.35f;
 float fuzzy_toe_threshold = 0.35f;

/* 모니터링 변수 */
float emg_R_act, emg_L_act;
float fsr_RH_load, fsr_RT_load, fsr_LH_load, fsr_LT_load;
uint16_t cal_fsr_done, cal_emg_done;
float thigh_R_deg, thigh_L_deg;
float tau_R_cmd_nm, tau_L_cmd_nm;
 uint16_t standing_flag = 0U;

uint16_t cdc_stream_enable = 1U;
uint16_t cdc_stream_period_ms = 10U;

/* ============================================================================
• STATIC RUNTIME STATE
• ============================================================================ */
static XmTsmHandle_t s_tsm;

static const XmAdcPin_t s_adc_pins[ADC_CH_COUNT] = {
XM_EXT_ADC_5, XM_EXT_ADC_6, XM_EXT_ADC_7, XM_EXT_ADC_8,
XM_EXT_ADC_9, XM_EXT_ADC_10
};

static float s_v[ADC_CH_COUNT];

/* FSR 상태 */
static float s_fsr_lpf[FSR_CH_COUNT];
static float s_fsr_off[FSR_CH_COUNT] = {0};
static float s_fsr_on[FSR_CH_COUNT] = { FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V, FSR_ON_DEFAULT_V };
static float s_fsr_load[FSR_CH_COUNT];
static bool s_filter_init;

/* EMG 상태 */
static float s_emg_bias[EMG_CH_COUNT] = { EMG_BIAS_DEFAULT_V, EMG_BIAS_DEFAULT_V };
static float s_emg_pre[EMG_CH_COUNT];
static float s_emg_env[EMG_CH_COUNT];
static float s_emg_act[EMG_CH_COUNT];
static float s_emg_mvic[EMG_CH_COUNT] = { EMG_MVIC_DEFAULT_V, EMG_MVIC_DEFAULT_V };

/* 캘리브레이션 */
static CalState_t s_cal_state = CAL_IDLE;
static uint32_t s_cal_start_ms;
static uint32_t s_cal_count;
static double s_cal_sum[ADC_CH_COUNT];
static float s_cal_max_env[EMG_CH_COUNT];

/* 정지 및 각도 래치 상태 */
static uint8_t s_prev_mask;
static uint32_t s_mask_since_ms;
static bool s_standing;
static uint32_t s_not_standing_since_ms = 0U;

/* 캘리브레이션 순서 추적 플래그 */
static bool s_bias_done = false;
static bool s_fsr_off_done = false;

static bool s_r_stance_latch = false;
static bool s_is_stair_step_R = false;
static bool s_l_stance_latch = false;
static bool s_is_stair_step_L = false;

static float s_tau_prev[2];
 static uint32_t s_active_entry_ms;
static uint32_t s_stream_ms;
static uint32_t s_usb_dbg_ms;
static StairStreamData_t s_stream;

/* ============================================================================
• INTERNAL FUNCTION DECLARATIONS
• ============================================================================ */
static void Off_Loop(void);
static void Standby_Loop(void);
static void Active_Entry(void);
static void Active_Loop(void);
static void Active_Exit(void);

static float _LpfAlphaRC(float fc_hz);
static float _Clamp(float x, float lo, float hi);
static float _Abs(float x);
static void _SampleAdc(void);
static void _ProcessFsr(void);
static void _ProcessEmg(void);
static void _HandleCalibration(void);
static void _PublishSignals(void);
static void _SendStream(void);
static void _UsbDebug(uint32_t now_ms);

/* ============================================================================
• ENTRY POINTS
• ============================================================================ */
void User_Setup(void)
{
uint32_t bodyData[8] = { USER_WEIGHT_KG_X10, USER_HEIGHT_CM_X10, 0, 0, 0, 0, 0, 0 };
XM_SendUserBodyData(bodyData);
XM_SetExtPowerVoltage(XM_EXT_PWR_5V);

/* 6개 포트 모두 아날로그 ADC 전환 */
XM_SwitchDioToAdc(XM_EXT_DIO_1);
XM_SwitchDioToAdc(XM_EXT_DIO_2);
XM_SwitchDioToAdc(XM_EXT_DIO_3);
XM_SwitchDioToAdc(XM_EXT_DIO_4);
XM_SwitchDioToAdc(XM_EXT_DIO_5);
XM_SwitchDioToAdc(XM_EXT_DIO_6);

s_tsm = XM_TSM_Create(XM_STATE_OFF);
XmStateConfig_t off_conf = { .id = XM_STATE_OFF, .on_loop = Off_Loop };
XM_TSM_AddState(s_tsm, &off_conf);
XmStateConfig_t sb_conf = { .id = XM_STATE_STANDBY, .on_loop = Standby_Loop };
XM_TSM_AddState(s_tsm, &sb_conf);
XmStateConfig_t act_conf = { .id = XM_STATE_ACTIVE, .on_entry = Active_Entry, .on_loop = Active_Loop, .on_exit = Active_Exit };
XM_TSM_AddState(s_tsm, &act_conf);

XM_SetUsbCustomMeta(USB_MODULE_ID, "[{"name":"EMG_R_Act","unit":"-"}, ...]");
XM_SetUsbTotalDataStream(false);
XM_SetH10AssistExistingMode(true);
XM_SetControlMode(XM_CTRL_MONITOR);
}

void User_Loop(void)
{
if (!s_tsm) return;
if (!XM_IsCmConnected()) { XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF); }
XM_TSM_Run(s_tsm);
XM_IO_Update();
}

/* ============================================================================
• TSM STATE LOOPS
• ============================================================================ */
static void Off_Loop(void) {
if (XM_IsCmConnected()) {
XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
}
}

static void Standby_Loop(void) {
_SampleAdc();
_ProcessFsr();
_ProcessEmg();
_HandleCalibration();

cal_fsr_done = (s_fsr_on[0] > 0.1f) ? 1U : 0U;
cal_emg_done = (s_emg_mvic[0] > 0.1f) ? 1U : 0U;

XM_SetLedState(XM_LED_2, cal_emg_done ? XM_ON : XM_OFF);
XM_SetLedState(XM_LED_3, cal_fsr_done ? XM_ON : XM_OFF);

_PublishSignals();
_SendStream();

if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST && cal_fsr_done && cal_emg_done) {
XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
}
}

static void Active_Entry(void) {
XM_SetH10AssistExistingMode(false);
 XM_SetControlMode(XM_CTRL_TORQUE);

s_tau_prev[0] = s_tau_prev[1] = 0.0f;
s_prev_mask = 0U;
s_mask_since_ms = XM_GetTick();
s_standing = false;
s_not_standing_since_ms = XM_GetTick();

s_r_stance_latch = false; s_is_stair_step_R = false;
s_l_stance_latch = false; s_is_stair_step_L = false;

s_active_entry_ms = XM_GetTick();
s_usb_dbg_ms = XM_GetTick();
XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 200);
}

static void Active_Loop(void) {
if (XM.status.h10.h10Mode != XM_H10_MODE_ASSIST) {
XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY); return;
}
if (XM_GetButtonEvent(XM_BTN_3) == XM_BTN_LONG_PRESS) {
XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF); return;
}

uint32_t now_ms = XM_GetTick();

_SampleAdc();
_ProcessFsr();
_ProcessEmg();

/* Standing Detection (채터링 방지) */
bool rh = s_fsr_load[0] >= fuzzy_heel_threshold;
bool rt = s_fsr_load[1] >= fuzzy_toe_threshold;
bool lh = s_fsr_load[2] >= fuzzy_heel_threshold;
bool lt = s_fsr_load[3] >= fuzzy_toe_threshold;
uint8_t mask = (uint8_t)((lh << 3) | (lt << 2) | (rh << 1) | (rt << 0));

if (mask == 0x0FU) s_not_standing_since_ms = now_ms;
if (mask != 0x0FU && (now_ms - s_not_standing_since_ms) > 100U) s_mask_since_ms = now_ms;
s_standing = (now_ms - s_mask_since_ms) >= (uint32_t)(STANDING_DWELL_S * 1000.0f);
s_prev_mask = mask;

float thigh_R = XM.status.h10.rightThighAngle;
float thigh_L = XM.status.h10.leftThighAngle;

bool safe_R = (thigh_R >= THIGH_ANGLE_MIN_DEG) && (thigh_R <= THIGH_ANGLE_MAX_DEG);
bool safe_L = (thigh_L >= THIGH_ANGLE_MIN_DEG) && (thigh_L <= THIGH_ANGLE_MAX_DEG);

float limit = _Clamp(assist_torque_limit_nm, 0.0f, HARD_MAX_ASSIST_TORQUE_NM);
float tau_R_raw = 0.0f;
float tau_L_raw = 0.0f;

/* ============================================================================
* [핵심 로직] 각도 래치 기반 계단/평지 분리 및 양발 EMG 비례 토크
* ============================================================================ */

/* --- 우측 다리 제어 --- */
if (safe_R && !s_standing) {
bool r_is_stance = (rh || rt);

if (!r_is_stance) {
/* [SWING 구간] /
s_r_stance_latch = false;
if (thigh_R > SWING_FLEXION_START_DEG && thigh_R < SWING_FLEXION_END_DEG) {
tau_R_raw = FLEXION_ASSIST_TORQUE;
}
}
else {
/ [STANCE 구간] */
if (!s_r_stance_latch) {
s_is_stair_step_R = (thigh_R >= STANCE_EXTENSION_MIN_DEG);
s_r_stance_latch = true;
}

if (s_is_stair_step_R) {
/* [EMG 비례 제어 활성화] 우측 햄스트링/대둔근 EMG 활성도에 비례하는 토크 인가 */
float ext_mag = K_EMG * s_emg_act[0];
tau_R_raw = -_Clamp(ext_mag, 0.0f, limit);
} else {
tau_R_raw = 0.0f;
}
}
}

/* --- 좌측 다리 제어 --- */
if (safe_L && !s_standing) {
bool l_is_stance = (lh || lt);

if (!l_is_stance) {
s_l_stance_latch = false;
if (thigh_L > SWING_FLEXION_START_DEG && thigh_L < SWING_FLEXION_END_DEG) {
tau_L_raw = FLEXION_ASSIST_TORQUE;
}
}
else {
if (!s_l_stance_latch) {
s_is_stair_step_L = (thigh_L >= STANCE_EXTENSION_MIN_DEG);
s_l_stance_latch = true;
}

if (s_is_stair_step_L) {
/* [EMG 비례 제어 활성화] 좌측 햄스트링/대둔근 EMG 활성도에 비례하는 토크 인가 */
float ext_mag = K_EMG * s_emg_act[1];
tau_L_raw = -_Clamp(ext_mag, 0.0f, limit);
} else {
tau_L_raw = 0.0f;
}
}
}

/* 안전 필터 및 토크 출력 */
float ramp = _Clamp((float)(now_ms - s_active_entry_ms) * 1e-3f / RAMP_DURATION_S, 0.0f, 1.0f);
tau_R_raw *= ramp; tau_L_raw *= ramp;

const float max_step = SLEW_RATE_LIMIT_NM_PER_S * CONTROL_DT_S;
s_tau_prev[0] += _Clamp(tau_R_raw - s_tau_prev[0], -max_step, max_step);
s_tau_prev[1] += _Clamp(tau_L_raw - s_tau_prev[1], -max_step, max_step);

s_tau_prev[0] = _Clamp(s_tau_prev[0], -limit, limit);
s_tau_prev[1] = _Clamp(s_tau_prev[1], -limit, limit);

if (control_ON == 1U) {
XM_SetAssistTorqueRH(s_tau_prev[0]);
XM_SetAssistTorqueLH(s_tau_prev[1]);
} else {
XM_SetAssistTorqueRH(0.0f);
XM_SetAssistTorqueLH(0.0f);
}

tau_R_cmd_nm = s_tau_prev[0]; tau_L_cmd_nm = s_tau_prev[1];
_PublishSignals();
 _SendStream();
_UsbDebug(now_ms);
}

static void Active_Exit(void) {
XM_SetAssistTorqueRH(0.0f);
XM_SetAssistTorqueLH(0.0f);
XM_SetControlMode(XM_CTRL_MONITOR);
XM_SetH10AssistExistingMode(true);
s_tau_prev[0] = s_tau_prev[1] = 0.0f;
tau_R_cmd_nm = tau_L_cmd_nm = 0.0f;
XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
}

/* ============================================================================
• 센서 프로세싱 및 캘리브레이션
• ============================================================================ */
static void _SampleAdc(void) {
for (int i = 0; i < ADC_CH_COUNT; i++) {
s_v[i] = (float)XM_AnalogReadMillivolts(s_adc_pins[i]) * 0.001f;
}
}

static void _ProcessFsr(void) {
float a = _LpfAlphaRC(FSR_LPF_FC_HZ);
for (int i = 0; i < FSR_CH_COUNT; i++) {
float raw = s_v[i]; // PF3~PF6
 if (!s_filter_init) { s_fsr_lpf[i] = raw; }
else { s_fsr_lpf[i] += a * (raw - s_fsr_lpf[i]); }

float span = s_fsr_on[i] - s_fsr_off[i];
if (span < FSR_MIN_SPAN_V) span = FSR_MIN_SPAN_V;
s_fsr_load[i] = _Clamp((s_fsr_lpf[i] - s_fsr_off[i]) / span, 0.0f, 1.5f);
}
s_filter_init = true;
}

static void _ProcessEmg(void) {
float a_pre = _LpfAlphaRC(EMG_PRE_LPF_FC_HZ);
float a_env = _LpfAlphaRC(EMG_ENV_LPF_FC_HZ);
for (int i = 0; i < EMG_CH_COUNT; i++) {
float raw = s_v[CH_EMG_R + i]; // PF7~PF8
float centered = raw - s_emg_bias[i];
s_emg_pre[i] += a_pre * (centered - s_emg_pre[i]);
float rect = _Abs(s_emg_pre[i]);
s_emg_env[i] += a_env * (rect - s_emg_env[i]);

float mvic = s_emg_mvic[i];
if (mvic < EMG_MVIC_MIN_V) mvic = EMG_MVIC_MIN_V;
float active = s_emg_env[i] - EMG_ENV_DEADBAND_V;
if (active < 0.0f) active = 0.0f;
float act_raw = active / mvic;
s_emg_act[i] = EMG_ACT_GAMMA * s_emg_act[i] + (1.0f - EMG_ACT_GAMMA) * act_raw;
}
}

static void _HandleCalibration(void) {
XmBtnEvent_t b1 = XM_GetButtonEvent(XM_BTN_1);
XmBtnEvent_t b2 = XM_GetButtonEvent(XM_BTN_2);
XmBtnEvent_t b3 = XM_GetButtonEvent(XM_BTN_3);
uint32_t now = XM_GetTick();

if (s_cal_state == CAL_IDLE) {
if (b3 == XM_BTN_CLICK) {
for (int i = 0; i < FSR_CH_COUNT; i++) { s_fsr_off[i]=0; s_fsr_on[i]=1.0f; }
for (int i = 0; i < EMG_CH_COUNT; i++) { s_emg_bias[i]=EMG_BIAS_DEFAULT_V; s_emg_mvic[i]=EMG_MVIC_DEFAULT_V; }
s_bias_done = false;      /* 순서 플래그 초기화 */
s_fsr_off_done = false;   /* 순서 플래그 초기화 */
XM_SetLedEffect(XM_LED_1, XM_LED_ONESHOT, 500);
return;
}
if (b1 == XM_BTN_CLICK) {
s_cal_state = s_bias_done ? CAL_EMG_MVIC_RUN : CAL_EMG_BIAS_RUN;
s_bias_done = !s_bias_done;

s_cal_start_ms = now; s_cal_count = 0U;
for (int i=0; i<ADC_CH_COUNT; i++) s_cal_sum[i] = 0.0;
s_cal_max_env[0] = s_cal_max_env[1] = 0.0f;
XM_SetLedEffect(XM_LED_2, XM_LED_BLINK, 100);
return;
}
if (b2 == XM_BTN_CLICK) {
s_cal_state = s_fsr_off_done ? CAL_FSR_ON_RUN : CAL_FSR_OFF_RUN;
s_fsr_off_done = !s_fsr_off_done;

s_cal_start_ms = now; s_cal_count = 0U;
for (int i=0; i<ADC_CH_COUNT; i++) s_cal_sum[i] = 0.0;
XM_SetLedEffect(XM_LED_3, XM_LED_BLINK, 100);
return;
}
return;
}

if (s_cal_state == CAL_EMG_BIAS_RUN || s_cal_state == CAL_EMG_MVIC_RUN) {
s_cal_sum[0] += s_v[CH_EMG_R]; s_cal_sum[1] += s_v[CH_EMG_L];
if (s_emg_env[0] > s_cal_max_env[0]) s_cal_max_env[0] = s_emg_env[0];
if (s_emg_env[1] > s_cal_max_env[1]) s_cal_max_env[1] = s_emg_env[1];
s_cal_count++;

if ((now - s_cal_start_ms) >= EMG_CAL_MS) {
if (s_cal_state == CAL_EMG_BIAS_RUN) {
s_emg_bias[0] = (float)(s_cal_sum[0]/s_cal_count);
s_emg_bias[1] = (float)(s_cal_sum[1]/s_cal_count);
} else {
s_emg_mvic[0] = s_cal_max_env[0]; s_emg_mvic[1] = s_cal_max_env[1];
}
s_cal_state = CAL_IDLE;
}
}
else if (s_cal_state == CAL_FSR_OFF_RUN || s_cal_state == CAL_FSR_ON_RUN) {
for (int i=0; i<FSR_CH_COUNT; i++) s_cal_sum[i] += s_fsr_lpf[i];
s_cal_count++;
if ((now - s_cal_start_ms) >= FSR_CAL_MS) {
for (int i=0; i<FSR_CH_COUNT; i++) {
float avg = (float)(s_cal_sum[i]/s_cal_count);
if (s_cal_state == CAL_FSR_OFF_RUN) s_fsr_off[i] = avg;
else s_fsr_on[i] = avg;
}
s_cal_state = CAL_IDLE;
}
}
}

static void _PublishSignals(void) {
emg_R_act = s_emg_act[0]; emg_L_act = s_emg_act[1];
fsr_RH_load = s_fsr_load[0]; fsr_RT_load = s_fsr_load[1];
fsr_LH_load = s_fsr_load[2]; fsr_LT_load = s_fsr_load[3];
standing_flag = s_standing ? 1U : 0U;
thigh_R_deg = XM.status.h10.rightThighAngle;
thigh_L_deg = XM.status.h10.leftThighAngle;
}

static void _SendStream(void) {
uint32_t now = XM_GetTick();
if (cdc_stream_enable != 1U || (now - s_stream_ms) < cdc_stream_period_ms) return;
s_stream_ms = now;

s_stream.emg_R_act = s_emg_act[0]; s_stream.emg_L_act = s_emg_act[1];
s_stream.fsr_RH_load = s_fsr_load[0]; s_stream.fsr_RT_load = s_fsr_load[1];
s_stream.fsr_LH_load = s_fsr_load[2]; s_stream.fsr_LT_load = s_fsr_load[3];
s_stream.thigh_R = thigh_R_deg; s_stream.thigh_L = thigh_L_deg;
s_stream.tau_R = s_tau_prev[0]; s_stream.tau_L = s_tau_prev[1];
s_stream.is_stair_R = s_is_stair_step_R ? 1.0f : 0.0f;
s_stream.is_stair_L = s_is_stair_step_L ? 1.0f : 0.0f;

XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), USB_MODULE_ID);
}

static void _UsbDebug(uint32_t now_ms) {
if (now_ms - s_usb_dbg_ms < USB_DEBUG_PERIOD_MS) return;
s_usb_dbg_ms = now_ms;
char buf[140];
snprintf(buf, sizeof(buf),
"STAIR-LATCH | On:%u St:%u | Stair R/L:%d/%d | EMG aR/L:%.2f/%.2f | T R/L:%.2f/%.2f\r\n",
control_ON, standing_flag, s_is_stair_step_R, s_is_stair_step_L,
emg_R_act, emg_L_act, s_tau_prev[0], s_tau_prev[1]);
XM_SendUsbDebugMessage(buf);
}

static float _LpfAlphaRC(float fc_hz) { return CONTROL_DT_S / ((1.0f / (TWO_PI * fc_hz)) + CONTROL_DT_S); }
static float _Clamp(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }
static float _Abs(float x) { return (x < 0.0f) ? -x : x; }