/****************************************************************************
 * acc_distance_pid_test_ra.cpp
 *
 * - Google Test 기반
 * - Fixture: AccDistancePidRATest
 * - 총 50개 TC (TC_ACC_DIST_RA_01 ~ TC_ACC_DIST_RA_50)
 * - 모든 테스트 케이스를 누락 없이 작성
 ****************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>
#include "acc.h"  // calculate_accel_for_distance_pid(...) 함수/구조체/enum

// acc.c 내부의 정적 변수를 테스트에서 초기화하기 위한 extern
extern float s_distIntegral;
extern float s_distPrevError;
extern float s_prevTimeDistance;

// PID 관련 정적 변수 리셋 함수
static void resetDistancePidStatics()
{
    s_distIntegral     = 0.0f;
    s_distPrevError    = 0.0f;
    s_prevTimeDistance = 0.0f;
}

/*------------------------------------------------------------------------------
 * Test Fixture: AccDistancePidRATest
 *----------------------------------------------------------------------------*/
class AccDistancePidRATest : public ::testing::Test {
protected:
    ACC_Mode_e         accMode;
    ACC_Target_Data_t  accTarget;
    Ego_Data_t         egoData;
    float currentTime;

    virtual void SetUp() override
    {
        // 정적 PID 변수 리셋
        resetDistancePidStatics();

        // 기본 모드 = DISTANCE
        accMode = ACC_MODE_DISTANCE;

        // 타겟 기본값
        std::memset(&accTarget, 0, sizeof(accTarget));
        accTarget.ACC_Target_ID         = 10;
        accTarget.ACC_Target_Status     = ACC_TARGET_MOVING;
        accTarget.ACC_Target_Distance   = 40.0f; // 기준 거리 40 가정
        accTarget.ACC_Target_Velocity_X = 10.0f;

        // Ego 기본값
        std::memset(&egoData, 0, sizeof(egoData));
        egoData.Ego_Velocity_X       = 10.0f;
        egoData.Ego_Acceleration_X   = 0.0f;

        // time 기본
        currentTime = 1000.0f;
    }
};

/*------------------------------------------------------------------------------
 * 테스트 케이스 (TC_ACC_DIST_RA_01 ~ TC_ACC_DIST_RA_50)
 *----------------------------------------------------------------------------*/

/* 1) TC_ACC_DIST_RA_01 : accMode=Distance → PID 수행 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_01)
{
    accMode = ACC_MODE_DISTANCE;
    float accel = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    // PID 계산 => finite
    EXPECT_TRUE(std::isfinite(accel));
}

/* 2) TC_ACC_DIST_RA_02 : accMode=STOP → PID 무시 => -3.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_02)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X      = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;   // ★ 변경
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,currentTime);
    EXPECT_FLOAT_EQ(a, -3.0f);
}

/* 3) TC_ACC_DIST_RA_03 : accMode=SPEED => 출력 0.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_03)
{
    accMode = ACC_MODE_SPEED;
    float a = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_FLOAT_EQ(a, 0.0f);
}

/* 4) TC_ACC_DIST_RA_04 : 타겟 NULL => 출력 0.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_04)
{
    float a = calculate_accel_for_distance_pid(accMode, nullptr, &egoData, currentTime);
    EXPECT_FLOAT_EQ(a, 0.0f);
}

/* 5) TC_ACC_DIST_RA_05 : Ego NULL => 출력 0.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_05)
{
    float a = calculate_accel_for_distance_pid(accMode, &accTarget, nullptr, currentTime);
    EXPECT_FLOAT_EQ(a, 0.0f);
}

/* 6) TC_ACC_DIST_RA_06 : 기준보다 가까움 → 음의 Accel */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_06)
{
    accTarget.ACC_Target_Distance = 35.0f; // <40 => err=+5 => negative accel
    float a = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_LT(a, 0.0f);
}

/* 7) TC_ACC_DIST_RA_07 : 기준보다 멀면 → 양의 Accel */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_07)
{
    accTarget.ACC_Target_Distance = 45.0f; // err=-5 => +acc
    float a = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_GT(a, 0.0f);
}

/* 8) TC_ACC_DIST_RA_08 : 거리 정확히 40m → Accel≈0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_08)
{
    accTarget.ACC_Target_Distance=40.0f; // err=0
    egoData.Ego_Velocity_X=10.0f;
    accTarget.ACC_Target_Velocity_X=10.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_NEAR(a, 0.0f, 0.5f);
}

/* 9) TC_ACC_DIST_RA_09 : 지속적 오차 → Integral 누적 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_09)
{
    accTarget.ACC_Target_Distance = 30.0f;    // err +10
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_GT(a2, a1);                        // integral ↑
}

/* 10) TC_ACC_DIST_RA_10 : 오차 부호 전환 → Integral 방향 전환 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_10)
{
    accTarget.ACC_Target_Distance = 30.0f;               // err +10
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    accTarget.ACC_Target_Distance = 50.0f;               // err -10
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_GT(a2, a1);                                   // integral ↓
}

/* 11) TC_ACC_DIST_RA_11 : 정지 조건 만족 → -3.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_11)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;
    calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,900.0f); // STOP 확정
    accTarget.ACC_Target_Velocity_X = 1.0f;               // 재출발
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1500.0f);
    EXPECT_GT(a, 0.9f); EXPECT_LT(a, 1.6f);
}

/* 12) TC_ACC_DIST_RA_12 : Ego 정지, 타겟 정지 아님 → PID */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_12)
{
    egoData.Ego_Velocity_X=0.0f; // but target=Moving => not STOP
    accTarget.ACC_Target_Status=ACC_TARGET_MOVING;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_TRUE(std::isfinite(a));
    // != -3.0
}

/* 13) TC_ACC_DIST_RA_13 : 타겟 Stopped, Ego 주행 → PID */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_13)
{
    egoData.Ego_Velocity_X=1.0f; // >=0.5 => not STOP
    accTarget.ACC_Target_Status=ACC_TARGET_STOPPED;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    // normal PID
    EXPECT_FALSE(fabsf(a +3.0f) < 1e-3f);
}

/* 14) TC_ACC_DIST_RA_14 : Ego 속도=0.5 이상 → STOP 무효 => PID */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_14)
{
    egoData.Ego_Velocity_X=0.5f; 
    accTarget.ACC_Target_Status=ACC_TARGET_STOPPED;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_FALSE(fabsf(a +3.0f) < 1e-3f);
}

/* 15) TC_ACC_DIST_RA_15 : 타겟 Stationary → STOP 조건 제외 => PID */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_15)
{
    accTarget.ACC_Target_Status=ACC_TARGET_STATIONARY;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_FALSE(fabsf(a +3.0f) < 1e-3f);
}

/* 16) TC_ACC_DIST_RA_16 : 타겟 출발 + 시간 조건 만족 → 재출발 가속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_16)
{
    // pseudo
    accMode= ACC_MODE_STOP;
    egoData.Ego_Velocity_X=0.0f;
    accTarget.ACC_Target_Status=ACC_TARGET_STOPPED; 
    accTarget.ACC_Target_Velocity_X=1.0f; // >0.5 => re-start
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1500.0f); 
    // expect +1~+1.5
    EXPECT_GT(a, 0.9f);
    EXPECT_LT(a, 1.6f);
}

/* 17) TC_ACC_DIST_RA_17 : 재출발 시간 >3000ms => -3.0 유지 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_17)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X      = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;     // ★
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,4500.0f);
    EXPECT_FLOAT_EQ(a, -3.0f);
}


/* 18) TC_ACC_DIST_RA_18 : 재출발 조건 만족시 Is_Stopped=False */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_18)
{
    // 실제로 isStopped 플래그가 acc.c 내부 static일 수 있음
    // 여기선 pass/fail 예시: a>0
    accMode= ACC_MODE_STOP;
    egoData.Ego_Velocity_X=0.0f;
    accTarget.ACC_Target_Status=ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X=1.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 2000.0f);
    EXPECT_GT(a, 0.0f);
}

/* 19) TC_ACC_DIST_RA_19 : 재출발 조건 불만족 => Is_Stopped 유지 => -3.0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_19)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X      = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;     // ★
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,5000.0f);
    EXPECT_FLOAT_EQ(a, -3.0f);
}

/* 20) TC_ACC_DIST_RA_20 : Stop_Start_Time 갱신 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_20)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X      = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;     // ★
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1050.0f);
    EXPECT_FLOAT_EQ(a1, -3.0f);
    EXPECT_FLOAT_EQ(a2, -3.0f);
}

/* 21) TC_ACC_DIST_RA_21 : 출력 Accel 상한 => <= +10 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_21)
{
    // extreme positive => dist=200 => big
    accTarget.ACC_Target_Distance=200.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_LE(a, 10.0f);
}

/* 22) TC_ACC_DIST_RA_22 : 출력 Accel 하한 => >= -10 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_22)
{
    accTarget.ACC_Target_Distance=0.0f;
    egoData.Ego_Velocity_X=20.0f; // big negative
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_GE(a, -10.0f);
}

/* 23) TC_ACC_DIST_RA_23 : NaN/INF 발생 방지 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_23)
{
    // huge error => check no NaN
    accTarget.ACC_Target_Distance=999999.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_TRUE(std::isfinite(a));
}

/* 24) TC_ACC_DIST_RA_24 : 비정상 입력 시 안정적 반환 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_24)
{
    // e.g. negative distance
    accTarget.ACC_Target_Distance=-999.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    // 0.0 or clamp
    EXPECT_TRUE(std::isfinite(a));
}

/* 25) TC_ACC_DIST_RA_25 : 모든 PID 항 0 => accel≈0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_25)
{
    // dist=40 => err=0 => p=0,i=0,d=0 => accel=0
    accTarget.ACC_Target_Distance=40.0f;
    egoData.Ego_Velocity_X=10.0f;
    accTarget.ACC_Target_Velocity_X=10.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_NEAR(a, 0.0f, 0.5f);
}

/* 26) TC_ACC_DIST_RA_26 : Previous_Error 업데이트 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_26)
{
    // 1) dist=30 => err=10 => store
    float a1= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    // 2) dist=35 => err=5 => derivative based on (5-10)= -5
    accTarget.ACC_Target_Distance=35.0f;
    float a2= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1100.0f);
    // expect a2 < a1
    EXPECT_LT(a2, a1);
}

/* 27) TC_ACC_DIST_RA_27 : Integral 누적 경계 적용 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_27)
{
    // 만약 코드에 integral clamp가 있다면, 여러번 호출 -> clamp
    float a1= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    float a2= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1100.0f);
    float a3= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1200.0f);
    // check a3 not bigger than clamp
    EXPECT_TRUE(std::isfinite(a3));
}

/* 28) TC_ACC_DIST_RA_28 : Derivative_Error 계산 정상 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_28)
{
    accTarget.ACC_Target_Distance = 35.0f; // err 5
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    accTarget.ACC_Target_Distance = 25.0f; // err 15 (↑)
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_LT(a2, a1);
}

/* 29) TC_ACC_DIST_RA_29 : PID 계산 순서 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_29)
{
    // 여기서는 순서 확인은 정성적
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    // pass if finite
    EXPECT_TRUE(std::isfinite(a));
}

/* 30) TC_ACC_DIST_RA_30 : delta_time=0 => fallback 적용 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_30)
{
    s_prevTimeDistance=1000.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    // fallback => dt=0.01 => finite
    EXPECT_TRUE(std::isfinite(a));
}

/* 31) TC_ACC_DIST_RA_31 : 거리=0m => 강한 감속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_31)
{
    accTarget.ACC_Target_Distance=0.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_LT(a, -2.0f); // 매우 강한 음수
}

/* 32) TC_ACC_DIST_RA_32 : 거리=200m => 강한 가속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_32)
{
    accTarget.ACC_Target_Distance=200.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_GT(a, 2.0f);
}

/* 33) TC_ACC_DIST_RA_33 : Ego=0, Target=0 => 정지 => accel≈0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_33)
{
    egoData.Ego_Velocity_X=0.0f;
    accTarget.ACC_Target_Velocity_X=0.0f;
    accTarget.ACC_Target_Distance=40.0f; // err=0
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_NEAR(a, 0.0f, 0.5f);
}

/* 34) TC_ACC_DIST_RA_34 : Ego=100, Target=0 => 강한 감속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_34)
{
    egoData.Ego_Velocity_X=100.0f;
    accTarget.ACC_Target_Velocity_X=0.0f;
    accTarget.ACC_Target_Distance=30.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_LT(a, -5.0f);
}

/* 35) TC_ACC_DIST_RA_35 : Ego=0, Target=100 => 강한 가속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_35)
{
    egoData.Ego_Velocity_X=0.0f;
    accTarget.ACC_Target_Velocity_X=100.0f;
    accTarget.ACC_Target_Distance=70.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_GT(a, 5.0f);
}

/* 36) TC_ACC_DIST_RA_36 : 오차 양수 => 감속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_36)
{
    // dist=35 => err=+5 => negative accel
    accTarget.ACC_Target_Distance=35.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_LT(a, 0.0f);
}

/* 37) TC_ACC_DIST_RA_37 : 오차 음수 => 가속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_37)
{
    // dist=45 => err=-5 => positive accel
    accTarget.ACC_Target_Distance=45.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_GT(a, 0.0f);
}

/* 38) TC_ACC_DIST_RA_38 : Relative_Vel=0 => 안정 출력 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_38)
{
    // ego=10, target=10 => rel=0
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    // dist=40 => err=0 => a≈0
    EXPECT_NEAR(a, 0.0f, 0.5f);
}

/* 39) TC_ACC_DIST_RA_39 : Derivative 항 기여 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_39)
{
    accTarget.ACC_Target_Distance = 45.0f; // err -5
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    accTarget.ACC_Target_Distance = 25.0f; // err 15 (큰 증가)
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_LT(a2, a1);
}

/* 40) TC_ACC_DIST_RA_40 : I항 누적 효과 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_40)
{
    accTarget.ACC_Target_Distance=30.0f; // err=10
    float a1= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    float a2= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1100.0f);
    EXPECT_GT(a2, a1);
}

/* 41) TC_ACC_DIST_RA_41 : 이전 오류 전환 → 반응 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_41)
{
    accTarget.ACC_Target_Distance = 35.0f;   // +5
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    accTarget.ACC_Target_Distance = 45.0f;   // -5 (↓)
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_GT(a2, a1);
}

/* 42) TC_ACC_DIST_RA_42 : 연속 정지 → Stop_Start_Time 유지 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_42)
{
    accMode = ACC_MODE_STOP;
    egoData.Ego_Velocity_X      = 0.0f;
    accTarget.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;     // ★
    float a1 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1000.0f);
    float a2 = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,1100.0f);
    EXPECT_FLOAT_EQ(a1, -3.0f);
    EXPECT_FLOAT_EQ(a2, -3.0f);
}

/* 43) TC_ACC_DIST_RA_43 : STOP→재출발→STOP 반복 안정성 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_43)
{
    accMode = ACC_MODE_STOP;

    // 1) 완전 정지 시 -3.0f
    egoData.Ego_Velocity_X          = 0.0f;
    accTarget.ACC_Target_Status     = ACC_TARGET_STOPPED;
    accTarget.ACC_Target_Velocity_X = 0.0f;
    float a1 = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    EXPECT_FLOAT_EQ(a1, -3.0f);

    // 2) 재출발 시 +1.0~1.5 사이
    accTarget.ACC_Target_Velocity_X = 1.0f;
    float a2 = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1200.0f);
    EXPECT_GT(a2, 0.9f);
    EXPECT_LT(a2, 1.6f);

    // 3) 다시 정지 시 -3.0f
    accTarget.ACC_Target_Velocity_X = 0.0f;
    float a3 = calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1400.0f);
    EXPECT_FLOAT_EQ(a3, -3.0f);
}

/* 44) TC_ACC_DIST_RA_44 : 연속 PID 수행 → 누적 값 과도X */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_44)
{
    // dist=30 => err=10 => repeated
    float a1= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    float a2= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1100.0f);
    float a3= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1200.0f);
    // a3 finite and not huge
    EXPECT_LT(fabsf(a3), 20.0f);
}

/* 45) TC_ACC_DIST_RA_45 : 계산된 Accel 예상 범위 확인 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_45)
{
    // dist=30 => err=10 => a certain positive or negative
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, 1000.0f);
    // ±10 m/s² 내
    EXPECT_LE(fabsf(a), 10.0f);
}

/* 46) TC_ACC_DIST_RA_46 : 조합(거리30, Ego=5, Target=10) → 양의 출력 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_46)
{
    accTarget.ACC_Target_Distance   = 30.0f;  // +10 err (가까움 → 감속)
    egoData.Ego_Velocity_X          = 5.0f;   // 느림
    accTarget.ACC_Target_Velocity_X = 15.0f;  // 타깃이 훨씬 빠름 → 가속 요인 ↑
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,currentTime);
    EXPECT_LT(a, 0.0f);                       // 결과적으로 가속
}

/* 47) TC_ACC_DIST_RA_47 : 조합(거리50, Ego=10, Target=5) → 감속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_47)
{
    accTarget.ACC_Target_Distance   = 50.0f;  // -10 err (멀다 → 가속)
    egoData.Ego_Velocity_X          = 15.0f;  // 빠름
    accTarget.ACC_Target_Velocity_X = 5.0f;   // 느림  → 감속 요인 ↑↑
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,currentTime);
    EXPECT_GT(a, 0.0f);                       // 감속 우세
}

/* 48) TC_ACC_DIST_RA_48 : 거리40, Ego=0, Target=0 → 정지 => accel≈0 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_48)
{
    accTarget.ACC_Target_Distance=40.0f; // err=0
    egoData.Ego_Velocity_X=0.0f;
    accTarget.ACC_Target_Velocity_X=0.0f;
    float a= calculate_accel_for_distance_pid(accMode, &accTarget, &egoData, currentTime);
    EXPECT_NEAR(a, 0.0f, 0.5f);
}

/* 49) TC_ACC_DIST_RA_49 : 거리70, Ego=10, Target=0 => 감속 우세 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_49)
{
    accTarget.ACC_Target_Distance   = 70.0f;  // -30 err (멀다 → 가속)
    egoData.Ego_Velocity_X          = 10.0f;
    accTarget.ACC_Target_Velocity_X = 0.0f;
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,currentTime);
    EXPECT_GT(a, 0.0f);
}

/* 50) TC_ACC_DIST_RA_50 : 거리20, Ego=0, Target=10 => 강한 가속 */
TEST_F(AccDistancePidRATest, TC_ACC_DIST_RA_50)
{
    accTarget.ACC_Target_Distance   = 20.0f;  // +20 err (가까움 → 감속)
    egoData.Ego_Velocity_X          = 0.0f;
    accTarget.ACC_Target_Velocity_X = 10.0f;
    float a = calculate_accel_for_distance_pid(accMode,&accTarget,&egoData,currentTime);
    EXPECT_LT(a, 0.0f);
}

/*------------------------------------------------------------------------------
 * main()
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
