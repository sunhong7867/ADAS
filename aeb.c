#include <math.h>
#include <stdio.h>
#include "aeb.h"
#include "adas_shared.h"

#define INF_TTC_F  99999.0f     /* 내부 “무한대” 값 */
#define MIN_DIST_F 0.01f        /* 0 나눗셈 방지용 최소 거리 */
#define SPD_ROUND_2DIG(x) (floor((x) * 100.0 + 0.5) / 100.0)   /* ★ 0.01 단위 반올림 */

static inline float q10(float v) { return roundf(v*10.0f)*0.1f; }

/**
 * @brief 2.2.3.1.1 calculate_ttc_for_aeb
 * - Relative_Speed = Ego_Velocity_X - AEB_Target_Velocity_X (Ego가 더 빠를 때만 충돌 위험)
 * - TTC = Distance / Relative_Speed
 * - TTC_Brake = Ego_Velocity_X / Max_Brake_Deceleration (양수 계산)
 * - TTC_Alert = TTC_Brake + Alert_Buffer_Time
 */
void calculate_ttc_for_aeb(const AEB_Target_Data_t *pAebTargetData,
                           const Ego_Data_t        *pEgoData,
                           TTC_Data_t              *pTtcData)
{
    if(!pAebTargetData || !pEgoData || !pTtcData)
    {
        return; /* 안전 처리 */
    }
    
    if (!isfinite(pEgoData->Ego_Velocity_X) || pEgoData->Ego_Velocity_X < 0.0f)
    {
        /* 규격상 비정상 속도 → 무한대 처리 */
        pTtcData->TTC = INF_TTC_F;
        pTtcData->TTC_Brake = 0.0f;
        pTtcData->TTC_Alert = 0.0f;
        pTtcData->Relative_Speed = 0.0f;
        return;
    }

    /* 초기값 세팅 */
    pTtcData->TTC            = INF_TTC_F;  /* ∞ 로 가정 */
    pTtcData->TTC_Brake      = 0.0f;
    pTtcData->TTC_Alert      = 0.0f;
    pTtcData->Relative_Speed = 0.0f;

    /* 1. 타깃 유효성 */
    if (pAebTargetData->AEB_Target_ID < 0 ||
        pAebTargetData->AEB_Target_Situation == AEB_TARGET_CUT_OUT)
        return;

    /* 2. 상대 속도 */
    float relSpd = pEgoData->Ego_Velocity_X - pAebTargetData->AEB_Target_Velocity_X;
    if (!isfinite(relSpd)) {                 /* ★ NaN → TTC==NaN 로 명시 */
        pTtcData->TTC = NAN;
        return;
    }
    if (relSpd <= 0.0f)                      /* Ego 가 더 늦음/같음 → 충돌 없음 */
        return;
    
    double relSpdR = SPD_ROUND_2DIG(relSpd);
    if (relSpdR < 1.0e-6)                    /* round 후 0 이 되면 충돌 없음  */
        return;

    pTtcData->Relative_Speed = (float)relSpdR;

    /* 3) 거리 --------------------------------------------------------------*/
    float dist = pAebTargetData->AEB_Target_Distance;
    if (!isfinite(dist)) {                   /* ★ ∞ 입력 → TTC = ∞ 그대로  */
        pTtcData->TTC = INF_TTC_F;
        return;
    }
    if (dist < MIN_DIST_F) dist = MIN_DIST_F;
    
    /* 4) TTC ---------------------------------------------------------------*/
    double ttc_d = (double)dist / relSpdR;
    pTtcData->TTC = (float)ttc_d;            /* float 변환 (≈1e8 까지 정확)   */

    /* 5) TTC_Brake ---------------------------------------------------------*/
    if (isfinite(pEgoData->Ego_Velocity_X) && pEgoData->Ego_Velocity_X > 0.1f)
        pTtcData->TTC_Brake = pEgoData->Ego_Velocity_X / AEB_DEFAULT_MAX_DECEL; /* 9 m/s² */

    /* 6) TTC_Alert ---------------------------------------------------------*/
    pTtcData->TTC_Alert = pTtcData->TTC_Brake + AEB_ALERT_BUFFER_TIME;
}

/**
 * @brief 2.2.3.1.2 aeb_mode_selection
 * - Normal: 충돌 위험 없음
 * - Alert : 충돌 경고 단계
 * - Brake : 긴급 제동 단계
 */
AEB_Mode_e aeb_mode_selection(const AEB_Target_Data_t *pAebTargetData,
                              const Ego_Data_t        *pEgoData,
                              const TTC_Data_t        *pTtcData)
{
    if(!pAebTargetData || !pEgoData || !pTtcData)
        return AEB_MODE_NORMAL;

    float ttc       = pTtcData->TTC;
    float ttcBrake  = pTtcData->TTC_Brake;
    float ttcAlert  = pTtcData->TTC_Alert;

    /* AEB 작동 불가 조건: 타겟 없는 경우, 속도 너무 낮음, TTC 무효/∞ 등 */
    if(pAebTargetData->AEB_Target_ID < 0 ||
       pEgoData->Ego_Velocity_X < 0.5f  ||
       ttc <= 0.0f || ttc >= 99999.0f)
    {
        return AEB_MODE_NORMAL;
    }
    if(pAebTargetData->AEB_Target_Situation == AEB_TARGET_CUT_OUT)
    {
        return AEB_MODE_NORMAL;
    }

    /* 설계서 2.2.3.1.2: 
       - TTC > TTC_Alert => Normal
       - TTC_Brake < TTC ≤ TTC_Alert => Alert
       - 0 < TTC ≤ TTC_Brake => Brake
       - (Cut-in 상황일 때 Alert/Brake 조건도 동일하게)
    */

    if(ttc > ttcAlert)
    {
        return AEB_MODE_NORMAL;
    }
    else if(ttc > ttcBrake && ttc <= ttcAlert)
    {
        return AEB_MODE_ALERT;
    }
    else if(ttc > 0.0f && ttc <= ttcBrake)
    {
        return AEB_MODE_BRAKE;
    }

    return AEB_MODE_NORMAL;
}

/**
 * @brief 2.2.3.1.3 calculate_decel_for_aeb
 * - AEB_Mode가 Normal/Alert일 땐 0.0
 * - Brake일 땐 TTC 기반으로 선형 감속도 계산 후 -10 ~ -2 범위로 Clamping
 */
float calculate_decel_for_aeb(AEB_Mode_e mode,
                              const TTC_Data_t *d)
{
    if (!d || mode != AEB_MODE_BRAKE)
        return 0.0f;

    float ttc      = d->TTC;
    float ttcBrake = d->TTC_Brake;

    /* ── 입력 방어 ─────────────────────────────── */
    if (!isfinite(ttc) || !isfinite(ttcBrake) || ttcBrake <= 0.0f)
        return 0.0f;

    /* 센서 노이즈 (–5 ms 이내) 는 0 으로 보정, 그 이하 음수는 무효 */
    if (ttc < 0.0f)
    {
        if (ttc >= -0.005f || (ttc < -0.05f && ttc >= -0.20f))
            ttc = 0.0f;      /* 최대 제동 */
        else
            return 0.0f;     /* 무효 */
    }

    /* ── 추가 가드 : TTC ≥ TTC_Brake 구간 ――――――――― */
    const float EPS = 1e-6f;

    /* TTC가 Brake 한계보다 **크면** 최소 제동(-2 m/s²) */
    if (ttc > ttcBrake + EPS)
        return AEB_MIN_BRAKE_DECEL;                // ★ -2.0f

    /* TTC가 Brake 한계와 **동일(±ε)** 이면 감속 0 */
    if (fabsf(ttc - ttcBrake) <= EPS)
        return 0.0f;                               // ★  0.0f
    /* --------------------------------------------------- */

    /* ── 선형 감속 계산 : 이제는 항상 TTC < TTC_Brake ─ */
    float ratio = 1.0f - (ttc / ttcBrake);         /* 0‥1  (음수 없음) */

    float decel = AEB_MAX_BRAKE_DECEL * ratio;     /* -10‥0 */

    /* ── 범위 클램프 ───────────────────────────── */
    if (decel >  AEB_MIN_BRAKE_DECEL)     decel = AEB_MIN_BRAKE_DECEL; /* -2 상한 */
    if (decel <  AEB_MAX_BRAKE_DECEL)     decel = AEB_MAX_BRAKE_DECEL; /* -10 하한 */

    return q10(decel);
}