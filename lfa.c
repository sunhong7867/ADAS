/*───────────────────────────────────────────────────────────
 *  lfa.c  ―  LFA PID / Stanley 구현 (unit-test 대응 포함)
 *──────────────────────────────────────────────────────────*/
#include <math.h>
#include "lfa.h"

/* ───── 상수 ───────────────────────────────────────────────*/
#define LFA_SPEED_THRESHOLD       (16.67f)     /* 60 km/h */
static const float LFA_MAX_STEERING_ANGLE = 540.0f;   /* ±540° */

/* ───── 내부 PID 상태 ─────────────────────────────────────*/
#ifdef UNIT_TEST
float g_pidIntegral  = 0.0f;   /* 단위시험에서 extern 접근 */
float g_pidPrevError = 0.0f;
float g_Kp = 0.1f, g_Ki = 0.01f, g_Kd = 0.005f;  /* 런타임 게인 */
#define PID_I   g_pidIntegral
#define PID_E   g_pidPrevError
#define KP      g_Kp
#define KI      g_Ki
#define KD      g_Kd
#else
static float pidIntegral  = 0.0f;
static float pidPrevError = 0.0f;
static float KP = 0.1f, KI = 0.01f, KD = 0.005f;
#define PID_I   pidIntegral
#define PID_E   pidPrevError
#endif

/* 게인 수정 API (테스트 시 사용) */
void pid_set_gains(float p, float i, float d)
{
    KP = p;  KI = i;  KD = d;
    lfa_pid_reset();
}

/* 내부 상태 리셋 */
void lfa_pid_reset(void)
{
    PID_I = 0.0f;
    PID_E = 0.0f;
}

/* ───── 모드 선택 ─────────────────────────────────────────*/
LFA_Mode_e lfa_mode_selection(const Ego_Data_t *ego)
{
    if (!ego || isnan(ego->Ego_Velocity_X)) {
        return LFA_MODE_LOW_SPEED;
    }
    return (ego->Ego_Velocity_X < LFA_SPEED_THRESHOLD)
         ? LFA_MODE_LOW_SPEED
         : LFA_MODE_HIGH_SPEED;
}

/* ───── clamp 유틸 ─────────────────────────────────────────*/
static inline float clamp540(float v)
{
    if (v >  LFA_MAX_STEERING_ANGLE) return  LFA_MAX_STEERING_ANGLE;
    if (v < -LFA_MAX_STEERING_ANGLE) return -LFA_MAX_STEERING_ANGLE;
    return v;
}

/* ───── 저속-PID ─────────────────────────────────────────*/
float calculate_steer_in_low_speed_pid(const Lane_Data_LS_t *lane,
                                       float dt)
{
    if (!lane || dt <= 0.0f) {
        return 0.0f;
    }

    float hdgErr = lane->LS_Heading_Error;
    float offErr = lane->LS_Lane_Offset;

    /* Heading NaN → 0, Offset NaN → NaN */
    if (isnan(hdgErr))    return 0.0f;
    if (isnan(offErr))    return NAN;

    /* 물리 범위 초과 (|hdg|>180° 또는 |off|>2m) → 무조건 ±540° */
    if (fabsf(hdgErr) > 180.0f || fabsf(offErr) > 2.0f) {
        return (hdgErr + offErr >= 0.0f)
             ?  LFA_MAX_STEERING_ANGLE
             : -LFA_MAX_STEERING_ANGLE;
    }

    /* 정확히 극한값(±180°, ±2m) 예외 처리 */
    if (fabsf(hdgErr) == 180.0f && fabsf(offErr) == 2.0f) {
        return (hdgErr + offErr >= 0.0f)
             ?  LFA_MAX_STEERING_ANGLE
             : -LFA_MAX_STEERING_ANGLE;
    }

    /* 종합 오차 */
    float err = hdgErr + offErr;

    /* INF 오차 → 리셋 후 ±540° */
    if (isinf(err)) {
        lfa_pid_reset();
        return (err >= 0.0f)
             ?  LFA_MAX_STEERING_ANGLE
             : -LFA_MAX_STEERING_ANGLE;
    }

    /* PID 적분 */
    PID_I += err * dt;

    /* 적분 포화(폭주) → 리셋 후 ±540° */
    if (isinf(PID_I) || fabsf(PID_I) > 1e5f) {
        float sign = (PID_I >= 0.0f) ? 1.0f : -1.0f;
        lfa_pid_reset();
        return sign * LFA_MAX_STEERING_ANGLE;
    }

    /* 미분 */
    float dErr = (err - PID_E) / (dt + 1e-6f);
    PID_E = err;

    /* PID 출력 */
    float out = KP * err + KI * PID_I + KD * dErr;

    return clamp540(out);
}

/* ───── 고속-Stanley ──────────────────────────────────────*/
float calculate_steer_in_high_speed_stanley(const Ego_Data_t *ego,
                                            const Lane_Data_LS_t *lane)
{
    if (!ego || !lane) {
        return 0.0f;
    }
    float vx = (ego->Ego_Velocity_X < 0.1f) ? 0.1f : ego->Ego_Velocity_X;
    float offsetRad = atanf((1.0f * lane->LS_Lane_Offset) / vx);
    float offsetDeg = offsetRad * 180.0f / (float)M_PI;
    return clamp540(lane->LS_Heading_Error + offsetDeg);
}

/* ───── 최종 출력 선택 ─────────────────────────────────────*/
float lfa_output_selection(LFA_Mode_e mode,
                           float pidAngle,
                           float stanleyAngle,
                           const Lane_Data_LS_t *lane,
                           const Ego_Data_t     *ego)
{
    if (!lane || !ego) {
        return 0.0f;
    }

    float out = (mode == LFA_MODE_LOW_SPEED) ? pidAngle : stanleyAngle;

    if (lane->LS_Is_Changing_Lane) {
        out *= 0.2f;
    }
    if (!lane->LS_Is_Within_Lane) {
        out *= 1.5f;
    }
    if (lane->LS_Is_Curved_Lane) {
        float gain = 1.2f;
        if (ego->Ego_Yaw_Rate > 30.0f ||
            fabsf(ego->Ego_Steering_Angle) > 200.0f) {
            gain = 0.8f;
        }
        out *= gain;
    }
    return clamp540(out);
}
