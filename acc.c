#include <math.h>
#include <stdio.h>
#include "acc.h"

/* Distance PID 적분, 과거오차 저장 */
float s_distIntegral  = 0.0f;
float s_distPrevError = 0.0f;

/* Speed PID 적분, 과거오차 저장 */
float s_speedIntegral  = 0.0f;
float s_speedPrevError = 0.0f;

/* 이전 시간 저장(예: 거리 PID에서 Delta Time 계산용) */
float s_prevTimeDistance = 0.0f;

/**
 * @brief 2.2.4.1.1 ACC 모드 결정
 */
ACC_Mode_e acc_mode_selection(
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    const Lane_Data_t       *pLaneData
)
{
    if(!pAccTargetData || !pEgoData || !pLaneData) return ACC_MODE_SPEED;
    if(pAccTargetData->ACC_Target_ID < 0)      return ACC_MODE_SPEED;

    float dist = pAccTargetData->ACC_Target_Distance;
    if(dist > 55.0f) {
        // 멀리 있으면 무조건 Speed
        return ACC_MODE_SPEED;
    }
    else if(dist < 45.0f) {
        /* (추가) 45m 미만에서도 Stopped + Ego 정지이면 Stop 모드 */
        if ((pAccTargetData->ACC_Target_Status == ACC_TARGET_STOPPED) &&
            (pEgoData->Ego_Velocity_X < 0.5f))
        {
            return ACC_MODE_STOP;
        }
        return ACC_MODE_DISTANCE;
    }
    else {
        // 중간 구간(45~55)에서만 STOP/CUT-IN 적용
        if(pAccTargetData->ACC_Target_Status == ACC_TARGET_STOPPED &&
           pEgoData->Ego_Velocity_X            < 0.5f)
        {
            return ACC_MODE_STOP;
        }
        if(pAccTargetData->ACC_Target_Situation == ACC_TARGET_CUT_IN) {
            return ACC_MODE_DISTANCE;
        }
        // 디폴트
        return ACC_MODE_SPEED;
    }
}
/**
 * @brief 2.2.4.1.2 거리 PID 계산
 */
float calculate_accel_for_distance_pid(
    ACC_Mode_e               accMode,
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    float                    current_time
)
{
    /* 간단 유효성 체크 */
    if((pAccTargetData == NULL) || (pEgoData == NULL))
    {
        return 0.0f;
    }

    /* Distance 모드나 Stop 모드일 때만 거리 PID 유효 */
    if((accMode != ACC_MODE_DISTANCE) && (accMode != ACC_MODE_STOP))
    {
        return 0.0f;
    }

    /* Delta Time 계산 (예: current_time ms단위 가정) */
    float deltaTime_s = (current_time - s_prevTimeDistance) / 1000.0f;
    if(deltaTime_s <= 0.0f) deltaTime_s = 0.01f;
    s_prevTimeDistance = current_time;

    /* 기준거리 = 40m (설계서에서) */
    float targetDist = 40.0f;
    float distErr    = pAccTargetData->ACC_Target_Distance - targetDist;

    /* PID Gains */
    float Kp = 0.4f, Ki = 0.05f, Kd = 0.1f;

    s_distIntegral += distErr * deltaTime_s;
    float dErr      = (distErr - s_distPrevError) / deltaTime_s;
    s_distPrevError = distErr;

    float accelDist = Kp*distErr + Ki*s_distIntegral + Kd*dErr;

    if(accelDist > 10.0f)  accelDist = 10.0f;
    if(accelDist < -10.0f) accelDist = -10.0f;

    /* Stop 모드의 경우 (정지 유지) / Stopped 타겟과 ego도 거의 0이면 강제 제동 */
    if(accMode == ACC_MODE_STOP) {
        if (pAccTargetData->ACC_Target_Status == ACC_TARGET_STOPPED &&
            pEgoData->Ego_Velocity_X        < 0.5f)
        {
            if (pAccTargetData->ACC_Target_Velocity_X > 0.5f) {
                accelDist = 1.2f;          /* 재출발 */
            } else {
                accelDist = -3.0f;         /* 정지 유지 */
            }
        }
    }
    return accelDist;
}

/**
 * @brief 2.2.4.1.3 속도 PID 계산
 */
float calculate_accel_for_speed_pid(
    const Ego_Data_t  *pEgoData,
    const Lane_Data_t *pLaneData,
    float              delta_time
)
{
    if((pEgoData == NULL) || (pLaneData == NULL) || (delta_time <= 0.0f))
    {
        return 0.0f;
    }

    /* 기본 목표 속도: 80 km/h = 22.22 m/s */
    float baseTargetSpeed = 22.22f;

    /* 곡선 차선이면 속도 제한: 15 m/s */
    if(pLaneData->LS_Is_Curved_Lane)
    {
        if(baseTargetSpeed > 15.0f)
        {
            baseTargetSpeed = 15.0f;
        }
    }

    /* 오차 = 목표속도 - 현재속도 */
    float speedErr = baseTargetSpeed - pEgoData->Ego_Velocity_X;

    /* PID 게인 */
    float Kp = 0.5f;
    float Ki = 0.1f;
    float Kd = 0.05f;

    s_speedIntegral      += speedErr * delta_time;
    float dErr            = (speedErr - s_speedPrevError) / (delta_time + 1e-5f);
    s_speedPrevError      = speedErr;

    float accelSpeed = (Kp * speedErr) + (Ki * s_speedIntegral) + (Kd * dErr);

    return accelSpeed;
}

/**
 * @brief 2.2.4.1.4 최종 ACC 가속도 선택
 * - Speed 모드 => Accel_Speed_X
 * - Distance 모드 => Accel_Distance_X
 * - Stop 모드 => 0.0
 */
float acc_output_selection(
    ACC_Mode_e accMode,
    float      Accel_Distance_X,
    float      Accel_Speed_X
)
{
    if(accMode == ACC_MODE_SPEED)
    {
        return Accel_Speed_X;
    }
    else if(accMode == ACC_MODE_DISTANCE)
    {
        return Accel_Distance_X;
    }
    else if(accMode == ACC_MODE_STOP)
    {
        /* 정지 유지 */
        return 0.0f;
    }

    return 0.0f;
}