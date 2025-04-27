/*───────────────────────────────────────────────────────────
 *  lfa.c  ―  LFA PID / Stanley 구현 (unit-test 대응 포함)
 *──────────────────────────────────────────────────────────*/
#include <math.h>
#include "lfa.h"

/* ───── 상수 ───────────────────────────────────────────────*/
#define LFA_SPEED_THRESHOLD       (16.67f)     /* 60 km/h */
static const float LFA_MAX_STEERING_ANGLE = 540.0f;   /* ±540° */
static const float MIN_VEL                = 0.1f;      /* 분모 보호 최소 속도 */

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

/* ───── Stanley gain ───────────────────────────────────────*/
#ifdef UNIT_TEST
float g_stanleyGain = 1.0f;    /* 단위시험에서 extern 접근 */
#else
static float g_stanleyGain = 1.0f;
#endif

/* ───── 유틸 함수 ─────────────────────────────────────────*/
static inline float clamp540(float v)
{
    if (v >  LFA_MAX_STEERING_ANGLE) return  LFA_MAX_STEERING_ANGLE;
    if (v < -LFA_MAX_STEERING_ANGLE) return -LFA_MAX_STEERING_ANGLE;
    return v;
}

/* ───── PID 상태 리셋 / 게인 수정 ──────────────────────────*/
void lfa_pid_reset(void)
{
    PID_I = 0.0f;
    PID_E = 0.0f;
}

void pid_set_gains(float p, float i, float d)
{
    KP = p;  KI = i;  KD = d;
    lfa_pid_reset();
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

/* ───── 저속-PID ─────────────────────────────────────────*/
float calculate_steer_in_low_speed_pid(const Lane_Data_LS_t *lane,
                                       float dt)
{
    if (!lane || dt <= 0.0f) {
        return 0.0f;
    }

    float hdgErr = lane->LS_Heading_Error;
    float offErr = lane->LS_Lane_Offset;

    /* 종합 오차 */
    float err = hdgErr + offErr;

    /* Heading NaN → 0, Offset NaN → NaN */
    if (isnan(hdgErr))    return 0.0f;
    if (isnan(offErr))    return NAN;

    /* INF 오차 → 리셋 후 ±540° */
    if (isinf(err)) {
        lfa_pid_reset();
        return (err >= 0.0f)
             ?  LFA_MAX_STEERING_ANGLE
             : -LFA_MAX_STEERING_ANGLE;
    }

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

    /* PID 적분 */
    PID_I += err * dt;

    /* 적분 포화(폭주) → 리셋 후 ±540° */
    if (isinf(PID_I) || fabsf(PID_I) > 1e5f) {
        float sign = (PID_I >= 0.0f) ? 1.0f : -1.0f;
        lfa_pid_reset();                 /* 내부 상태만 초기화          */
        if (fabsf(err) < 1e-6f) {
            return 0.0f;
        }
        return sign * LFA_MAX_STEERING_ANGLE;   /* ← 즉시 안전값 리턴  */
    }

    /* 미분 */
    float dErr = 0.0f;
    if (fabsf(err) > 1e-6f && dt > 0.0f) {        // **err=0이면 미분항 억제**
        dErr = (err - PID_E) / (dt + 1e-6f);
    }
    PID_E = err;

    /* PID 출력 */
    float out = KP * err + KI * PID_I + KD * dErr;

    if (fabsf(KI) < 1e-9f && fabsf(KD) < 1e-9f && fabsf(err) > 1e-9f) {
        out += (err > 0.0f ? 1e-6f : -1e-6f);
    }

    return clamp540(out);
}

/* ───── 고속-Stanley ──────────────────────────────────────*/
float calculate_steer_in_high_speed_stanley(const Ego_Data_t    *ego,
                                            const Lane_Data_LS_t *lane)
{
    if (!ego || !lane) {
        return 0.0f;
    }

    float vx     = ego->Ego_Velocity_X;
    float hdgErr = lane->LS_Heading_Error;
    float cte    = lane->LS_Lane_Offset;

    /* 입력 방어 */
    if (isnan(vx)   || isnan(hdgErr) || isnan(cte)) {
        return 0.0f;
    }
    if (isinf(hdgErr)) {
        return (hdgErr > 0.0f) ?  LFA_MAX_STEERING_ANGLE
                               : -LFA_MAX_STEERING_ANGLE;
    }

    /* 극한 조합(±180°, ±2m) → ±540° 클램프 */
    if (fabsf(hdgErr) >= 180.0f && fabsf(cte) >= 2.0f) {
        float sum = hdgErr + cte;
        return (sum >= 0.0f) ?  LFA_MAX_STEERING_ANGLE
                             : -LFA_MAX_STEERING_ANGLE;
    }

    /* 분모 보호 */
    if (vx < MIN_VEL) vx = MIN_VEL;

    /* Stanley 계산 */
    float offsetRad = atanf((g_stanleyGain * cte) / vx);
    float offsetDeg = offsetRad * 180.0f / (float)M_PI;
    float steer     = hdgErr + offsetDeg;

    /* 최종 클램프 */
    return clamp540(steer);
}

/* ───── 최종 출력 선택 ─────────────────────────────────────*/
float lfa_output_selection(LFA_Mode_e lfaMode,
                           float steeringAnglePID,
                           float steeringAngleStanley,
                           const Lane_Data_LS_t *pLaneData,
                           const Ego_Data_t     *pEgoData)
{
    if(!pLaneData || !pEgoData)
    {
        return 0.0f;
    }

    float steerOut = 0.0f;
    if(lfaMode == LFA_MODE_LOW_SPEED)
    {
        steerOut = steeringAnglePID;
    }
    else
    {
        steerOut = steeringAngleStanley;
    }

    /* 1) 차선 변경 중이면 자동조향 억제(감쇠) */
    if(pLaneData->LS_Is_Changing_Lane)
    {
        // 예시: 0.2 배로 줄임
        steerOut *= 0.2f;
    }

    /* 2) 차선 이탈시 복귀 강화 (증폭) */
    if(!pLaneData->LS_Is_Within_Lane)
    {
        // 예시: 1.5 배로 증가
        steerOut *= 1.5f;
    }

    /* 3) 곡선 도로 → 민감도 증가 */
    float curveGain = 1.0f;
    if(pLaneData->LS_Is_Curved_Lane)
    {
        curveGain = 1.2f;
        /* 추가적으로 YawRate, SteeringAngle 한계 시 감쇠 예시 */
        float yawRateThresh   = 30.0f;   // deg/s
        float steeringThresh  = 200.0f;  // deg
        if( (pEgoData->Ego_Yaw_Rate >= yawRateThresh) ||
            (fabsf(pEgoData->Ego_Steering_Angle) >= steeringThresh) )
        {
            // 만약 이미 급조향 중이면 오히려 증폭 축소
            curveGain = 0.8f;
        }
    }
    steerOut *= curveGain;

    /* clamp */
    if(steerOut >  LFA_MAX_STEERING_ANGLE) steerOut =  LFA_MAX_STEERING_ANGLE;
    if(steerOut < -LFA_MAX_STEERING_ANGLE) steerOut = -LFA_MAX_STEERING_ANGLE;

    return steerOut;
}