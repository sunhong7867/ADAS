/*---------------------------------------------------------
 * File: lfa.h
 *---------------------------------------------------------*/
#ifndef LFA_H
#define LFA_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LFA 모드 구분 (저속/고속)
 * - 속도 임계값: 16.67 m/s (≈60 km/h)
 */
typedef enum {
    LFA_MODE_LOW_SPEED = 0,
    LFA_MODE_HIGH_SPEED
} LFA_Mode_e;

/**
 * @brief 차선 오차 데이터 (Low-Speed 모드용)
 */
typedef struct {
    float LS_Heading_Error;    /* (-180 ~ 180) [°] */
    float LS_Lane_Offset;      /* (-2.0 ~ 2.0)   [m] */
    int   LS_Is_Changing_Lane; /* (True=1, False=0) */
    int   LS_Is_Within_Lane;   /* (True=1, False=0) */
    int   LS_Is_Curved_Lane;   /* (True=1, False=0) */
} Lane_Data_LS_t;

/**
 * @brief Ego 차량 상태 데이터 (LFA 고속 모드용)
 */
typedef struct {
    float Ego_Velocity_X;      /* (0 ~ 100)    [m/s] */
    float Ego_Yaw_Rate;        /* (-180~180)   [°/s] */
    float Ego_Steering_Angle;  /* (-540~540)   [°]   */
} Ego_Data_t;

/**
 * @brief LFA 모드 선택 함수
 *        - 입력 속도 NaN 및 NULL 처리 포함
 */
LFA_Mode_e lfa_mode_selection(const Ego_Data_t *pEgoData);

/**
 * @brief 저속 모드 PID 기반 조향각 계산
 *        - NaN/INF/극한 입력 방어 로직 포함
 * @param pLaneData 차선 오차
 * @param deltaTime 제어 주기 (s)
 * @return Steering_Angle_PID (-540 ~ 540) [°]
 */
float calculate_steer_in_low_speed_pid(const Lane_Data_LS_t *pLaneData,
                                       float deltaTime);

/**
 * @brief 고속 모드 Stanley 제어 기반 조향각 계산
 */
float calculate_steer_in_high_speed_stanley(const Ego_Data_t    *pEgoData,
                                            const Lane_Data_LS_t *pLaneData);

/**
 * @brief 최종 LFA 출력 선택 (PID vs Stanley + 감쇠/증폭)
 */
float lfa_output_selection(LFA_Mode_e lfaMode,
                           float steeringAnglePID,
                           float steeringAngleStanley,
                           const Lane_Data_LS_t *pLaneData,
                           const Ego_Data_t     *pEgoData);

/**
 * @brief 테스트용: PID 내부 상태 초기화
 */
void lfa_pid_reset(void);
void pid_set_gains(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif
#endif /* LFA_H */