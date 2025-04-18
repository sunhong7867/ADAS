/********************************************************************************
 * acc_mode_selection_test.cpp
 *
 * - Google Test 기반
 * - Fixture: AccModeSelectionTest
 * - 총 60개 TC (EQ=20, BV=20, RA=20)
 * - 모든 테스트 케이스를 누락 없이 구현
 * - 실제 회사에서 운영한다고 가정하여 상세 작성
 ********************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>
#include "acc.h"   // acc_mode_selection, ACC_Mode_e, etc.

/*------------------------------------------------------------------------------
 * Test Fixture
 *----------------------------------------------------------------------------*/
class AccModeSelectionTest : public ::testing::Test {
protected:
    ACC_Target_Data_t accTargetData;  // 테스트 대상: 타겟 정보
    Ego_Data_t        egoData;        // 테스트 대상: 자차 정보
    Lane_Data_t       laneData;       // 테스트 대상: 차선 정보

    virtual void SetUp() override
    {
        // 구조체 기본값 초기화
        std::memset(&accTargetData, 0, sizeof(accTargetData));
        std::memset(&egoData,       0, sizeof(egoData));
        std::memset(&laneData,      0, sizeof(laneData));

        // ACC Target 기본값
        accTargetData.ACC_Target_ID         = 10;  // 유효 타겟
        accTargetData.ACC_Target_Distance   = 50.0f;
        accTargetData.ACC_Target_Status     = ACC_TARGET_MOVING;
        accTargetData.ACC_Target_Situation  = ACC_TARGET_NORMAL;
        accTargetData.ACC_Target_Velocity_X = 30.0f;

        // Ego 기본값
        egoData.Ego_Velocity_X       = 20.0f;
        egoData.Ego_Acceleration_X   = 0.0f;

        // LaneData 기본값
        laneData.Lane_Curvature       = 1000.0f;
        laneData.Next_Lane_Curvature  = 1000.0f;
        laneData.LS_Heading_Error     = 0.0f;
        laneData.LS_Is_Curved_Lane    = 0;   // false
    }
};

/*------------------------------------------------------------------------------
 * 동등 분할 (EQ) 테스트 케이스 20개
 *------------------------------------------------------------------------------
 * TC_ACC_MS_EQ_01 ~ TC_ACC_MS_EQ_20
 *----------------------------------------------------------------------------*/

/* 1) TC_ACC_MS_EQ_01 : 타겟 없음 상태 → SPEED 모드 출력 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_01)
{
    accTargetData.ACC_Target_ID = -1;  // 타겟 없음
    // 함수 호출
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 기대 SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 2) TC_ACC_MS_EQ_02 : 타겟 유효 + 거리 60m → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_02)
{
    accTargetData.ACC_Target_ID       = 10;
    accTargetData.ACC_Target_Distance = 60.0f; // >55
    accTargetData.ACC_Target_Status   = ACC_TARGET_MOVING;
    // 호출
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 3) TC_ACC_MS_EQ_03 : 타겟 유효 + 거리 30m → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_03)
{
    accTargetData.ACC_Target_Distance = 30.0f; // <45
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 4) TC_ACC_MS_EQ_04 : 거리 50m + 정지 타겟 + Ego 주행 중 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_04)
{
    accTargetData.ACC_Target_Distance = 50.0f; // 45~55
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED; // 정지
    egoData.Ego_Velocity_X            = 1.0f;  // 주행(>=0.5)
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // STOP 조건 미충족 => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 5) TC_ACC_MS_EQ_05 : 거리 50m + 정지 타겟 + Ego 정지 중 → STOP 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_05)
{
    accTargetData.ACC_Target_Distance = 50.0f;
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    egoData.Ego_Velocity_X            = 0.3f; // <0.5 => 정지
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/* 6) TC_ACC_MS_EQ_06 : 타겟 Moving 상태 + Ego 주행 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_06)
{
    accTargetData.ACC_Target_Status = ACC_TARGET_MOVING;
    egoData.Ego_Velocity_X          = 20.0f; // 주행
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 7) TC_ACC_MS_EQ_07 : 타겟 Stopped + 거리 멀고 Ego 정지 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_07)
{
    accTargetData.ACC_Target_Distance = 60.0f; // >55
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    egoData.Ego_Velocity_X            = 0.3f;  // 정지
    // dist>55 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 8) TC_ACC_MS_EQ_08 : Cut-out 상황 → SPEED 모드 강제 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_08)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_OUT; 
    // cut-out => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 9) TC_ACC_MS_EQ_09 : Cut-in 상황 + 거리 가까움 → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_09)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 30.0f; // <45
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 10) TC_ACC_MS_EQ_10 : Cut-in 상황 + 거리 멀리 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_10)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 70.0f; // >55
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 11) TC_ACC_MS_EQ_11 : Stationary 타겟 + Ego 정지 → STOP 모드 미적용 => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_11)
{
    accTargetData.ACC_Target_Status = ACC_TARGET_STATIONARY; 
    egoData.Ego_Velocity_X          = 0.3f; 
    // Stationary이면 STOP 안 된다 -> SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 12) TC_ACC_MS_EQ_12 : Oncoming 타겟 → 무시하고 SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_12)
{
    accTargetData.ACC_Target_Status = ACC_TARGET_ONCOMING;
    // 무조건 SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 13) TC_ACC_MS_EQ_13 : Ego 속도 0.3m/s + 타겟 정지 → STOP 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_13)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f; // 45~55
    egoData.Ego_Velocity_X            = 0.3f;  // <0.5 => 정지
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/* 14) TC_ACC_MS_EQ_14 : Ego 속도 0.7m/s + 타겟 정지 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_14)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f;  // 45~55
    egoData.Ego_Velocity_X            = 0.7f;   // >=0.5
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 15) TC_ACC_MS_EQ_15 : 곡률 반경 500m → Is_Tight_Curve = True (내부) */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_15)
{
    laneData.Lane_Curvature = 500.0f; // <800 => Tight
    // 모드 결정은 타겟 정보로, 여기선 MOVING + dist=50 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 내부에서 Tight Curve 등 처리 가능. 최종모드는 SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 16) TC_ACC_MS_EQ_16 : Next_Lane_Curvature < 800 → 곡선 간주, 모드 영향 없음 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_16)
{
    laneData.Next_Lane_Curvature = 700.0f; 
    // 그래도 모드는 타겟 info에 의해 SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 17) TC_ACC_MS_EQ_17 : Heading Error > 5도 → 곡선 주행으로 인식, 모드 영향 없음 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_17)
{
    laneData.LS_Heading_Error = 5.5f; 
    // 모드는 타겟이 Moving, dist=50 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 18) TC_ACC_MS_EQ_18 : Heading Error < 5도 → 직선 주행 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_18)
{
    laneData.LS_Heading_Error = 4.0f; 
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 여전히 dist=50, Moving => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 19) TC_ACC_MS_EQ_19 : LS_Is_Curved_Lane = True → 모드 영향 없음 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_19)
{
    laneData.LS_Is_Curved_Lane = 1; // true
    // dist=50, Moving => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 20) TC_ACC_MS_EQ_20 : 정지 타겟 + 거리 40m + Ego 정지 → STOP 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_EQ_20)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 40.0f; // <45
    egoData.Ego_Velocity_X            = 0.3f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/*------------------------------------------------------------------------------
 * 경계값 분석 (BV) 테스트 케이스 20개
 *------------------------------------------------------------------------------
 * TC_ACC_MS_BV_01 ~ TC_ACC_MS_BV_20
 *----------------------------------------------------------------------------*/

/* 1) TC_ACC_MS_BV_01 : 거리 44.0m → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_01)
{
    accTargetData.ACC_Target_Distance = 44.0f; // <45
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 2) TC_ACC_MS_BV_02 : 거리 45.0m → 유지 or SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_02)
{
    accTargetData.ACC_Target_Distance = 45.0f; 
    // 설계상 "45m <= dist <= 55m"면 이전 모드 유지 or SPEED
    // 여기서는 기본 모드가 SPEED이므로 SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 테스트 목적: SPEED인지 확인
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 3) TC_ACC_MS_BV_03 : 거리 46.0m → 유지 or SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_03)
{
    accTargetData.ACC_Target_Distance = 46.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 45~55 사이라도 "기본 SPEED" 가능
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 4) TC_ACC_MS_BV_04 : 거리 54.0m → 유지 or SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_04)
{
    accTargetData.ACC_Target_Distance = 54.0f;
    // 마찬가지로 45~55 범위 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 5) TC_ACC_MS_BV_05 : 거리 55.0m → 유지 or SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_05)
{
    accTargetData.ACC_Target_Distance = 55.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 경계 => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 6) TC_ACC_MS_BV_06 : 거리 56.0m → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_06)
{
    accTargetData.ACC_Target_Distance = 56.0f; // >55 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 7) TC_ACC_MS_BV_07 : Ego 속도 0.49m/s → 정지 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_07)
{
    egoData.Ego_Velocity_X = 0.49f;
    // dist=50, target=Stopped => STOP?
    accTargetData.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance=50.0f;
    // 45~55 + ego<0.5 => STOP
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/* 8) TC_ACC_MS_BV_08 : Ego 속도 0.50m/s → 주행 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_08)
{
    egoData.Ego_Velocity_X = 0.50f;
    // dist=50, target=Stopped => STOP?
    // But ego=0.5 => STOP 미충족 => SPEED
    accTargetData.ACC_Target_Status = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance=50.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 9) TC_ACC_MS_BV_09 : Ego 속도 0.51m/s → 주행 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_09)
{
    egoData.Ego_Velocity_X = 0.51f; 
    accTargetData.ACC_Target_Status=ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance=50.0f;
    // ego>=0.5 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 10) TC_ACC_MS_BV_10 : Heading Error 4.9° → 직선 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_10)
{
    laneData.LS_Heading_Error = 4.9f; 
    // 모드는 target=Moving, dist=50 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 11) TC_ACC_MS_BV_11 : Heading Error 5.0° → 경계값 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_11)
{
    laneData.LS_Heading_Error = 5.0f;
    // dist=50, target=Moving => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 경계여도 모드 영향X => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 12) TC_ACC_MS_BV_12 : Heading Error 5.1° → 곡선 인식 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_12)
{
    laneData.LS_Heading_Error = 5.1f;
    // 모드는 target=Moving => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 13) TC_ACC_MS_BV_13 : Lane_Curvature = 799.0m → Tight Curve 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_13)
{
    laneData.Lane_Curvature = 799.0f; // <800 => tight
    // 모드 => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 14) TC_ACC_MS_BV_14 : Lane_Curvature = 800.0m → 경계값 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_14)
{
    laneData.Lane_Curvature = 800.0f; 
    // target=Moving => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 15) TC_ACC_MS_BV_15 : Lane_Curvature = 801.0m → 직선 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_15)
{
    laneData.Lane_Curvature = 801.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 16) TC_ACC_MS_BV_16 : Next_Lane_Curvature=799.0m → 곡선 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_16)
{
    laneData.Next_Lane_Curvature = 799.0f;
    // 모드 => target info => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 17) TC_ACC_MS_BV_17 : Next_Lane_Curvature=800.0m → 경계값 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_17)
{
    laneData.Next_Lane_Curvature = 800.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 18) TC_ACC_MS_BV_18 : Next_Lane_Curvature=801.0m → 직선 간주 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_18)
{
    laneData.Next_Lane_Curvature = 801.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 19) TC_ACC_MS_BV_19 : Cut-in + 거리 44.9m → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_19)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 44.9f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 20) TC_ACC_MS_BV_20 : Cut-out + 거리 56.0m → SPEED 모드 강제 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_BV_20)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_OUT;
    accTargetData.ACC_Target_Distance  = 56.0f; 
    // cut-out => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/*------------------------------------------------------------------------------
 * 요구사항 분석 (RA) 테스트 케이스 20개
 *------------------------------------------------------------------------------
 * TC_ACC_MS_RA_01 ~ TC_ACC_MS_RA_20
 *----------------------------------------------------------------------------*/

/* 1) TC_ACC_MS_RA_01 : 타겟 없음 → 무조건 SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_01)
{
    accTargetData.ACC_Target_ID = -1;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 2) TC_ACC_MS_RA_02 : Cut-out 상황 → 무조건 SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_02)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_OUT;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 3) TC_ACC_MS_RA_03 : 거리 < 45m → DISTANCE 모드 강제 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_03)
{
    accTargetData.ACC_Target_Distance = 40.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 4) TC_ACC_MS_RA_04 : 거리 > 55m → SPEED 모드 강제 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_04)
{
    accTargetData.ACC_Target_Distance = 60.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 5) TC_ACC_MS_RA_05 : 거리 50m + STOP 조건 만족 → STOP 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_05)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f; // 45~55
    egoData.Ego_Velocity_X            = 0.3f;  // <0.5
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/* 6) TC_ACC_MS_RA_06 : 거리 50m + STOP 조건 불만족 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_06)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f;
    egoData.Ego_Velocity_X            = 0.7f; 
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 7) TC_ACC_MS_RA_07 : 타겟 상태가 Stationary → STOP 조건 제외 => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_07)
{
    accTargetData.ACC_Target_Status = ACC_TARGET_STATIONARY;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 8) TC_ACC_MS_RA_08 : 타겟 상태가 Oncoming → 무시 => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_08)
{
    accTargetData.ACC_Target_Status = ACC_TARGET_ONCOMING;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 9) TC_ACC_MS_RA_09 : 곡선 여부는 ACC 모드 결정에 영향 없음 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_09)
{
    laneData.LS_Is_Curved_Lane = 1;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // dist=50 => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 10) TC_ACC_MS_RA_10 : Ego만 정지 상태, 타겟 없음 → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_10)
{
    egoData.Ego_Velocity_X    = 0.3f;
    accTargetData.ACC_Target_ID = -1;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    // 타겟 없음 => SPEED
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 11) TC_ACC_MS_RA_11 : Cut-in + 거리 30m → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_11)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 30.0f; // <45
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 12) TC_ACC_MS_RA_12 : Cut-in + 거리 60m → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_12)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 60.0f; // >55
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 13) TC_ACC_MS_RA_13 : 타겟 Moving + 거리 50m → SPEED 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_13)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_MOVING;
    accTargetData.ACC_Target_Distance = 50.0f;
    egoData.Ego_Velocity_X            = 20.0f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 14) TC_ACC_MS_RA_14 : Stopped 타겟 + Ego 속도 0.6 → STOP 아님 => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_14)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f;
    egoData.Ego_Velocity_X            = 0.6f; // >=0.5 => stop 미충족
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 15) TC_ACC_MS_RA_15 : Stopped 타겟 + Ego 속도 0.3 → STOP 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_15)
{
    accTargetData.ACC_Target_Status   = ACC_TARGET_STOPPED;
    accTargetData.ACC_Target_Distance = 50.0f;
    egoData.Ego_Velocity_X            = 0.3f; // <0.5 => STOP
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_STOP);
}

/* 16) TC_ACC_MS_RA_16 : 곡률 반경 < 800 → Is_Tight_Curve True, 모드 영향 없음 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_16)
{
    laneData.Lane_Curvature = 750.0f; 
    // target=Moving => SPEED
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 17) TC_ACC_MS_RA_17 : Heading Error > 5도 → 곡선 판단 → 모드 영향 없음 => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_17)
{
    laneData.LS_Heading_Error = 5.5f;
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 18) TC_ACC_MS_RA_18 : Lane_Curvature = 801 → 직선으로 판단 => 모드: target info => SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_18)
{
    laneData.Lane_Curvature = 801.0f;
    // dist=50 => speed
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/* 19) TC_ACC_MS_RA_19 : 타겟 유효 + 거리 50m + Cut-in → DISTANCE 모드 */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_19)
{
    accTargetData.ACC_Target_Situation = ACC_TARGET_CUT_IN;
    accTargetData.ACC_Target_Distance  = 50.0f;
    // 45~55 + cut-in => ???

    // 설계서(=사용자 요구)에는 "Cut-in + 45 이하 => DISTANCE" 라고 기재되어 있다면,
    // 50m는 45~55 사이지만 Cut-in이므로 DISTANCE로 정함 (사용자 정의)
    // 아래는 사용자가 지정한 요구사항에 따라
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_DISTANCE);
}

/* 20) TC_ACC_MS_RA_20 : 모든 조건 정상 (Moving, 50m, Ego 20m/s) → SPEED */
TEST_F(AccModeSelectionTest, TC_ACC_MS_RA_20)
{
    // 기본 세팅: dist=50, moving => speed
    ACC_Mode_e mode = acc_mode_selection(&accTargetData, &egoData, &laneData);
    EXPECT_EQ(mode, ACC_MODE_SPEED);
}

/*------------------------------------------------------------------------------
 * main()
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
