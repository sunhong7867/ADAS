/********************************************************************************
 * lane_selection_test_all.cpp
 *
 *   Google Test 기반
 *   Lane Selection 모듈에 대한 60개 테스트 케이스 (동등 분할(EQ) 20개, 경계값(BV) 20개, 요구사항 기반(RA) 20개)
 *   ※ 제공된 adas_shared.h 및 lane_selection.h의 자료형에 맞춰 작성됨.
 *
 *   주의: Lane_Type과 Lane_Change_Status는 열거형(enum) 상수를 사용합니다.
 *         (예: LANE_TYPE_STRAIGHT, LANE_TYPE_CURVE, LANE_CHANGE_KEEP, LANE_CHANGE_CHANGING, LANE_CHANGE_DONE)
 ********************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>

// 실제 프로젝트 환경에 맞게 "lane_selection.h"를 포함합니다.
#include "lane_selection.h"

//========================================================================
//  Test Fixture
//========================================================================
class LaneSelectionTest : public ::testing::Test {
protected:
    LaneData_t   laneData;
    EgoData_t    egoData;
    LaneSelectOutput_t lsOutput;

    virtual void SetUp() override {
        // 모든 구조체 메모리를 0 초기화
        std::memset(&laneData, 0, sizeof(laneData));
        std::memset(&egoData,  0, sizeof(egoData));
        std::memset(&lsOutput, 0, sizeof(lsOutput));

        // 기본 초기값 (SetUp)
        laneData.Lane_Curvature      = 1000.0f;
        laneData.Next_Lane_Curvature = 1000.0f;
        laneData.Lane_Offset         = 0.0f;
        laneData.Lane_Width          = 3.5f;
        laneData.Lane_Heading        = 0.0f;
        laneData.Lane_Change_Status  = LANE_CHANGE_KEEP;
        laneData.Lane_Type           = LANE_TYPE_STRAIGHT; // 기본은 직선으로 가정

        egoData.Ego_Heading          = 0.0f;
    }
    
    // LaneSelection 함수 호출 wrapper
    void ExecLaneSelection() {
        int ret = LaneSelection(&laneData, &egoData, &lsOutput);
        // 테스트에서는 올바른 입력이면 ret==0이어야 함.
        EXPECT_EQ(ret, 0) << "LaneSelection 함수 호출 실패 (invalid argument 등)";
    }
};

/********************************************************************************
 * 1) 동등 분할 (EQ) 테스트 케이스 20개
 ********************************************************************************/

/* TC_LS_EQ_01 : 직선 차선 상태 판별
   조건: Lane_Curvature = 1000 (즉, 1000 이상이면 직선으로 판별)
   기대: LS_Is_Curved_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_EQ_01) {
    laneData.Lane_Curvature = 1000.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_EQ_01 실패] 곡률 1000이면 직선으로 판별되어야 함.";
}

/* TC_LS_EQ_02 : 곡선 차선 상태 판별
   조건: Lane_Curvature = 300 (800 미만이면 곡선)
   기대: LS_Is_Curved_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_EQ_02) {
    laneData.Lane_Curvature = 300.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_EQ_02 실패] 곡률 300이면 곡선(true)이어야 함.";
}

/* TC_LS_EQ_03 : 곡률 전이 감지 없음
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 990 (차이가 미미)
   기대: LS_Curve_Transition_Flag == false */
TEST_F(LaneSelectionTest, TC_LS_EQ_03) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 990.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_EQ_03 실패] 곡률 차이가 미미하므로 전이 플래그는 false여야 함.";
}

/* TC_LS_EQ_04 : 곡률 전이 감지 발생
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 500 (충분한 차이)
   기대: LS_Curve_Transition_Flag == true */
TEST_F(LaneSelectionTest, TC_LS_EQ_04) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 500.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_EQ_04 실패] 곡률 차이가 커야 전이 플래그는 true여야 함.";
}

/* TC_LS_EQ_05 : 진행 방향 오차 계산 (10°)
   조건: Ego_Heading = 10°, Lane_Heading = 0°
   기대: LS_Heading_Error == 10° */
TEST_F(LaneSelectionTest, TC_LS_EQ_05) {
    egoData.Ego_Heading   = 10.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 10.0f, 0.001)
        << "[TC_LS_EQ_05 실패] Heading error는 10°여야 함.";
}

/* TC_LS_EQ_06 : Heading Error 음수값 처리 (-170°)
   조건: Ego_Heading = -170°, Lane_Heading = 0°
   기대: LS_Heading_Error == -170° */
TEST_F(LaneSelectionTest, TC_LS_EQ_06) {
    egoData.Ego_Heading   = -170.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, -170.0f, 0.001)
        << "[TC_LS_EQ_06 실패] Heading error는 -170°여야 함.";
}

/* TC_LS_EQ_07 : 차선 중앙 주행
   조건: Lane_Offset = 0.0, Lane_Width = 3.5m
   기대: LS_Is_Within_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_EQ_07) {
    laneData.Lane_Offset = 0.0f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_07 실패] 오프셋 0.0이면 중앙(내부, true)이어야 함.";
}

/* TC_LS_EQ_08 : 차선 이탈 판별
   조건: Lane_Offset = 1.9m, Lane_Width = 3.5m (임계치: 1.75m)
   기대: LS_Is_Within_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_EQ_08) {
    laneData.Lane_Offset = 1.9f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_08 실패] 오프셋 1.9m는 차선을 벗어난 것으로 판별되어야 함.";
}

/* TC_LS_EQ_09 : 차선 변경 상태 (유지)
   조건: Lane_Change_Status = LANE_CHANGE_KEEP
   기대: LS_Is_Changing_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_EQ_09) {
    laneData.Lane_Change_Status = LANE_CHANGE_KEEP;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_EQ_09 실패] Lane_Change_Status가 KEEP이면, 변경 중 플래그는 false여야 함.";
}

/* TC_LS_EQ_10 : 차선 변경 상태 (변경 중)
   조건: Lane_Change_Status = LANE_CHANGE_CHANGING
   기대: LS_Is_Changing_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_EQ_10) {
    laneData.Lane_Change_Status = LANE_CHANGE_CHANGING;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_EQ_10 실패] Lane_Change_Status가 CHANGING이면, 변경 중 플래그는 true여야 함.";
}

/* TC_LS_EQ_11 : 차선 변경 상태 (변경 완료)
   조건: Lane_Change_Status = LANE_CHANGE_DONE
   기대: LS_Is_Changing_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_EQ_11) {
    laneData.Lane_Change_Status = LANE_CHANGE_DONE;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_EQ_11 실패] Lane_Change_Status가 DONE이면, 변경 중 플래그는 true여야 함.";
}

/* TC_LS_EQ_12 : 곡률 0 처리
   조건: Lane_Curvature = 0
   기대: LS_Is_Curved_Lane == false (직선으로 판단) */
TEST_F(LaneSelectionTest, TC_LS_EQ_12) {
    laneData.Lane_Curvature = 0.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_EQ_12 실패] 곡률 0이면 직선(true가 아님)이어야 함.";
}

/* TC_LS_EQ_13 : Heading wrap-around
   조건: Ego_Heading = 180°, Lane_Heading = -180°
   기대: LS_Heading_Error == 0° */
TEST_F(LaneSelectionTest, TC_LS_EQ_13) {
    egoData.Ego_Heading   = 180.0f;
    laneData.Lane_Heading = -180.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 0.0f, 0.001)
        << "[TC_LS_EQ_13 실패] 180°와 -180°의 차이는 0°이어야 함.";
}

/* TC_LS_EQ_14 : 차선 중심 오차 판별 (케이스1)
   조건: Lane_Width = 3.5m, Lane_Offset = 1.5m
   → 구현에서는 offset threshold = 3.5/2 = 1.75, 그러므로 1.5 < 1.75 → LS_Is_Within_Lane == true
   (설계 의도와 불일치할 수 있으므로, 결함 발견용으로 EXPECT 메시지 포함)
*/
TEST_F(LaneSelectionTest, TC_LS_EQ_14) {
    laneData.Lane_Width  = 3.5f;
    laneData.Lane_Offset = 1.5f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_14 실패] 오프셋 1.5m는 3.5m 차선에서 내부로 판정되어야 함 (threshold=1.75m).";
}

/* TC_LS_EQ_15 : 좁은 차선에서의 오프셋 판별
   조건: Lane_Width = 2.5m, Lane_Offset = 1.0m → threshold = 2.5/2 = 1.25, 1.0 < 1.25 → true
*/
TEST_F(LaneSelectionTest, TC_LS_EQ_15) {
    laneData.Lane_Width  = 2.5f;
    laneData.Lane_Offset = 1.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_15 실패] 좁은 차선(2.5m)에서 오프셋 1.0m는 내부(true)로 판정되어야 함.";
}

/* TC_LS_EQ_16 : Heading Error 계산 (음수 최대)
   조건: Ego_Heading = -90°, Lane_Heading = 90°
   기대: LS_Heading_Error == -180° */
TEST_F(LaneSelectionTest, TC_LS_EQ_16) {
    egoData.Ego_Heading   = -90.0f;
    laneData.Lane_Heading = 90.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, -180.0f, 0.001)
        << "[TC_LS_EQ_16 실패] -90°와 90°의 차이는 -180°여야 함.";
}

/* TC_LS_EQ_17 : 곡률 전이 판별 (차이가 임계치 미만)
   조건: Lane_Curvature = 850, Next_Lane_Curvature = 500 → diff = 350 (<400)
   기대: LS_Curve_Transition_Flag == false
*/
TEST_F(LaneSelectionTest, TC_LS_EQ_17) {
    laneData.Lane_Curvature      = 850.0f;
    laneData.Next_Lane_Curvature = 500.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_EQ_17 실패] 곡률 차이 350 (<400)이므로 전이 플래그는 false여야 함.";
}

/* TC_LS_EQ_18 : 음수 오프셋 처리
   조건: Lane_Offset = -1.0m, Lane_Width = 3.5m (threshold=1.75)
   기대: LS_Is_Within_Lane == true
*/
TEST_F(LaneSelectionTest, TC_LS_EQ_18) {
    laneData.Lane_Offset = -1.0f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_18 실패] 오프셋 -1.0m는 내부로 판정되어야 함.";
}

/* TC_LS_EQ_19 : 양수 오프셋 처리
   조건: Lane_Offset = +1.0m (기본 Lane_Width=3.5)
   기대: LS_Is_Within_Lane == true (1.0 < 1.75)
*/
TEST_F(LaneSelectionTest, TC_LS_EQ_19) {
    laneData.Lane_Offset = 1.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_EQ_19 실패] 오프셋 +1.0m는 내부로 판정되어야 함.";
}

/* TC_LS_EQ_20 : Heading wrap-around 예제
   조건: Ego_Heading = 5°, Lane_Heading = 355° → 5 - 355 = -350, 정규화하면 10°
   기대: LS_Heading_Error == 10° */
TEST_F(LaneSelectionTest, TC_LS_EQ_20) {
    egoData.Ego_Heading   = 5.0f;
    laneData.Lane_Heading = 355.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 10.0f, 0.001)
        << "[TC_LS_EQ_20 실패] 5°와 355°의 차이는 10°로 정규화되어야 함.";
}

/********************************************************************************
 * 2) 경계값 분석 (BV) 테스트 케이스 20개
 ********************************************************************************/

/* TC_LS_BV_01 : 곡선 판단 경계
   조건: Lane_Curvature = 799 (<800)
   기대: LS_Is_Curved_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_BV_01) {
    laneData.Lane_Curvature = 799.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_BV_01 실패] 곡률 799이면 곡선(true)이어야 함.";
}

/* TC_LS_BV_02 : 직선 경계
   조건: Lane_Curvature = 800
   기대: LS_Is_Curved_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_BV_02) {
    laneData.Lane_Curvature = 800.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_BV_02 실패] 곡률 800이면 직선(false)이어야 함.";
}

/* TC_LS_BV_03 : 곡률 801 (직선)
   기대: LS_Is_Curved_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_BV_03) {
    laneData.Lane_Curvature = 801.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_BV_03 실패] 곡률 801이면 직선(false)이어야 함.";
}

/* TC_LS_BV_04 : 곡률 차 399 → 전이 false
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 601 (diff = 399)
   기대: LS_Curve_Transition_Flag == false */
TEST_F(LaneSelectionTest, TC_LS_BV_04) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 601.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_BV_04 실패] 곡률 차 399이면 전이 플래그는 false여야 함.";
}

/* TC_LS_BV_05 : 곡률 차 400 → 경계 (false)
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 600 (diff = 400)
   기대: LS_Curve_Transition_Flag == false (구현에서는 diff > threshold여야 true) */
TEST_F(LaneSelectionTest, TC_LS_BV_05) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 600.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_BV_05 실패] 곡률 차 400이면 전이 플래그는 false여야 함.";
}

/* TC_LS_BV_06 : 곡률 차 401 → 전이 true
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 599 (diff = 401)
   기대: LS_Curve_Transition_Flag == true */
TEST_F(LaneSelectionTest, TC_LS_BV_06) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 599.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_BV_06 실패] 곡률 차 401이면 전이 플래그는 true여야 함.";
}

/* TC_LS_BV_07 : Heading Error 경계 (179°)
   조건: Ego_Heading = 179°, Lane_Heading = 0°
   기대: LS_Heading_Error == 179° */
TEST_F(LaneSelectionTest, TC_LS_BV_07) {
    egoData.Ego_Heading   = 179.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 179.0f, 0.001)
        << "[TC_LS_BV_07 실패] Heading error는 179°여야 함.";
}

/* TC_LS_BV_08 : Heading Error 경계 (180° 또는 -180°)
   조건: Ego_Heading = 180°, Lane_Heading = 0°
   기대: LS_Heading_Error == 180° 또는 -180° */
TEST_F(LaneSelectionTest, TC_LS_BV_08) {
    egoData.Ego_Heading   = 180.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    float val = lsOutput.LS_Heading_Error;
    bool pass = (fabs(val - 180.0f) < 0.001f) || (fabs(val + 180.0f) < 0.001f);
    EXPECT_TRUE(pass)
        << "[TC_LS_BV_08 실패] Heading error는 180° 또는 -180°여야 함. 현재: " << val;
}

/* TC_LS_BV_09 : Heading wrap-around (181° → -179°)
   조건: Ego_Heading = 181°, Lane_Heading = 0°
   기대: LS_Heading_Error == -179° */
TEST_F(LaneSelectionTest, TC_LS_BV_09) {
    egoData.Ego_Heading   = 181.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, -179.0f, 0.001)
        << "[TC_LS_BV_09 실패] Heading error는 -179°여야 함.";
}

/* TC_LS_BV_10 : 내부 판별 (좁은 차선)
   조건: Lane_Offset = 1.24m, Lane_Width = 2.5m → threshold = 1.25
   기대: LS_Is_Within_Lane == true (1.24 < 1.25)
*/
TEST_F(LaneSelectionTest, TC_LS_BV_10) {
    laneData.Lane_Width  = 2.5f;
    laneData.Lane_Offset = 1.24f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_10 실패] 오프셋 1.24m는 2.5m 차선 내에 있어야 함.";
}

/* TC_LS_BV_11 : 경계값 (Offset == threshold)
   조건: Lane_Offset = 1.25m, Lane_Width = 2.5m → threshold = 1.25, (1.25 < 1.25는 false)
   기대: LS_Is_Within_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_BV_11) {
    laneData.Lane_Width  = 2.5f;
    laneData.Lane_Offset = 1.25f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_11 실패] 오프셋이 경계값일 경우 내부 판별 조건을 만족하지 않아야 함.";
}

/* TC_LS_BV_12 : 외부 판별
   조건: Lane_Offset = 1.26m, Lane_Width = 2.5m → 1.26 > 1.25
   기대: LS_Is_Within_Lane == false */
TEST_F(LaneSelectionTest, TC_LS_BV_12) {
    laneData.Lane_Width  = 2.5f;
    laneData.Lane_Offset = 1.26f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_12 실패] 오프셋 1.26m는 차선 외부로 판별되어야 함.";
}

/* TC_LS_BV_13 : Heading Error (-179° 그대로)
   조건: Ego_Heading = -179°, Lane_Heading = 0°
   기대: LS_Heading_Error == -179° */
TEST_F(LaneSelectionTest, TC_LS_BV_13) {
    egoData.Ego_Heading   = -179.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, -179.0f, 0.001)
        << "[TC_LS_BV_13 실패] Heading error는 -179°여야 함.";
}

/* TC_LS_BV_14 : Heading Error (-180° 처리)
   조건: Ego_Heading = -180°, Lane_Heading = 0°
   기대: LS_Heading_Error == -180° 또는 180° */
TEST_F(LaneSelectionTest, TC_LS_BV_14) {
    egoData.Ego_Heading   = -180.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    float val = lsOutput.LS_Heading_Error;
    bool pass = (fabs(val + 180.0f) < 0.001f) || (fabs(val - 180.0f) < 0.001f);
    EXPECT_TRUE(pass)
        << "[TC_LS_BV_14 실패] Heading error는 -180° 또는 180°여야 함. 현재: " << val;
}

/* TC_LS_BV_15 : Heading wrap-around (-181° → +179°)
   조건: Ego_Heading = -181°, Lane_Heading = 0°
   기대: LS_Heading_Error == +179° */
TEST_F(LaneSelectionTest, TC_LS_BV_15) {
    egoData.Ego_Heading   = -181.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 179.0f, 0.001)
        << "[TC_LS_BV_15 실패] Heading error는 +179°여야 함.";
}

/* TC_LS_BV_16 : Lane Width 기준 미달 (2.49m)
   조건: Lane_Width = 2.49m
   기대: 설계서 기준에 따르면 처리 오류 또는 false 판별 (여기서는 EXPECT_FALSE로 지정)
   ※ 구현에서는 최소 폭 체크가 없으므로, 이 테스트가 실패할 경우 결함 검출 가능
*/
TEST_F(LaneSelectionTest, TC_LS_BV_16) {
    laneData.Lane_Width = 2.49f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_16 실패] 차선 폭이 2.49m이면 최소 요구 폭 미달로 처리되어야 함.";
}

/* TC_LS_BV_17 : Lane Width = 2.5m → 정상 처리
   조건: Lane_Width = 2.5m, (예: offset 기본 0이면 내부)
   기대: LS_Is_Within_Lane == true
*/
TEST_F(LaneSelectionTest, TC_LS_BV_17) {
    laneData.Lane_Width = 2.5f;
    laneData.Lane_Offset = 0.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_17 실패] 2.5m 차선은 정상 범위로 처리되어야 함.";
}

/* TC_LS_BV_18 : Lane Width = 2.51m → 정상 처리
   기대: LS_Is_Within_Lane == true
*/
TEST_F(LaneSelectionTest, TC_LS_BV_18) {
    laneData.Lane_Width = 2.51f;
    laneData.Lane_Offset = 0.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_18 실패] 2.51m 차선도 정상 범위로 처리되어야 함.";
}

/* TC_LS_BV_19 : 중앙 정렬 확인
   조건: Lane_Offset = 0.0
   기대: LS_Is_Within_Lane == true
*/
TEST_F(LaneSelectionTest, TC_LS_BV_19) {
    laneData.Lane_Offset = 0.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_19 실패] 오프셋 0.0이면 반드시 내부(true) 판정이어야 함.";
}

/* TC_LS_BV_20 : 음수 최대 경계 처리
   조건: Lane_Offset = -2.0, Lane_Width = 3.5 → threshold = 1.75 → | -2.0 | = 2.0 > 1.75
   기대: LS_Is_Within_Lane == false
*/
TEST_F(LaneSelectionTest, TC_LS_BV_20) {
    laneData.Lane_Offset = -2.0f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_BV_20 실패] 오프셋 -2.0m는 차선 외부로 판별되어야 함.";
}

/********************************************************************************
 * 3) 요구사항 기반 (RA) 테스트 케이스 20개
 ********************************************************************************/

/* TC_LS_RA_01 : 곡선 판단 기준 적용
   조건: Lane_Curvature = 300
   기대: LS_Is_Curved_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_01) {
    laneData.Lane_Curvature = 300.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_RA_01 실패] 곡률 300이면 곡선(true)이어야 함.";
}

/* TC_LS_RA_02 : 곡률 전이 기준 (>400m)
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 550 → diff = 450 (>400)
   기대: LS_Curve_Transition_Flag == true */
TEST_F(LaneSelectionTest, TC_LS_RA_02) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 550.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_RA_02 실패] 곡률 차 450는 전이 플래그 true여야 함.";
}

/* TC_LS_RA_03 : 진행 방향 오차 정규화
   조건: Ego_Heading = 180°, Lane_Heading = -180°
   기대: LS_Heading_Error == 0° */
TEST_F(LaneSelectionTest, TC_LS_RA_03) {
    egoData.Ego_Heading   = 180.0f;
    laneData.Lane_Heading = -180.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 0.0f, 0.001)
        << "[TC_LS_RA_03 실패] Heading error는 0°여야 함.";
}

/* TC_LS_RA_04 : 차선 중심 오차 기준 (반차폭)
   조건: Lane_Width = 3.5m, Lane_Offset = 1.74m (1.74 < 1.75, 경계 포함로 true)
   기대: LS_Is_Within_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_04) {
    laneData.Lane_Width  = 3.5f;
    laneData.Lane_Offset = 1.74f; // 1.74 < 1.75
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_RA_04 실패] 오프셋 1.74m는 내부(true)로 판별되어야 함.";
}

/* TC_LS_RA_05 : 차선 변경 중 상태 해석 반영
   조건: Lane_Change_Status = LANE_CHANGE_CHANGING
   기대: LS_Is_Changing_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_05) {
    laneData.Lane_Change_Status = LANE_CHANGE_CHANGING;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_RA_05 실패] 변경 중 상태이면 변경 플래그가 true여야 함.";
}

/* TC_LS_RA_06 : Ego 위치 기준 해석 (여기서는 Ego_Heading 기준)
   조건: Ego_Heading = 0 (기본), Lane_Offset = 0 등
   기대: LS_Is_Within_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_06) {
    egoData.Ego_Heading = 0.0f;
    laneData.Lane_Offset = 0.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_RA_06 실패] 기본 값이면 내부 판별(true)이어야 함.";
}

/* TC_LS_RA_07 : 동일 루프 내 계산 수행 확인
   조건: 동일 입력을 2회 호출하여 결과가 동일해야 함.
   기대: 두 호출 결과가 일치 */
TEST_F(LaneSelectionTest, TC_LS_RA_07) {
    laneData.Lane_Offset = 0.5f;
    ExecLaneSelection();
    bool firstVal = lsOutput.LS_Is_Within_Lane;
    ExecLaneSelection();
    bool secondVal = lsOutput.LS_Is_Within_Lane;
    EXPECT_EQ(firstVal, secondVal)
        << "[TC_LS_RA_07 실패] 동일 입력에 대해 결과가 일치해야 함.";
}

/* TC_LS_RA_08 : 곡률 0일 때 전이 판단 생략
   조건: Lane_Curvature = 0
   기대: LS_Curve_Transition_Flag == false */
TEST_F(LaneSelectionTest, TC_LS_RA_08) {
    laneData.Lane_Curvature = 0.0f;
    ExecLaneSelection();
    EXPECT_FALSE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_RA_08 실패] 곡률 0이면 전이 플래그는 false여야 함.";
}

/* TC_LS_RA_09 : 오프셋 기준 판별
   조건: Lane_Offset = 0.8m, Lane_Width = 3.5m → 0.8 < 1.75
   기대: LS_Is_Within_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_09) {
    laneData.Lane_Offset = 0.8f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_RA_09 실패] 오프셋 0.8m는 내부로 판별되어야 함.";
}

/* TC_LS_RA_10 : Heading Error 보정용
   조건: Ego_Heading = 10°, Lane_Heading = 0°
   기대: LS_Heading_Error == 10° */
TEST_F(LaneSelectionTest, TC_LS_RA_10) {
    egoData.Ego_Heading   = 10.0f;
    laneData.Lane_Heading = 0.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 10.0f, 0.001)
        << "[TC_LS_RA_10 실패] Heading error는 10°여야 함.";
}

/* TC_LS_RA_11 : 차선 변경 상태 외부 전달 활용
   조건: Lane_Change_Status = LANE_CHANGE_CHANGING
   기대: LS_Is_Changing_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_11) {
    laneData.Lane_Change_Status = LANE_CHANGE_CHANGING;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_RA_11 실패] 변경 중 상태이면 변경 플래그가 true여야 함.";
}

/* TC_LS_RA_12 : Heading Error 절대 오차 값 확인
   조건: Ego_Heading = 20°, Lane_Heading = 10°
   기대: LS_Heading_Error == 10° */
TEST_F(LaneSelectionTest, TC_LS_RA_12) {
    egoData.Ego_Heading   = 20.0f;
    laneData.Lane_Heading = 10.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 10.0f, 0.001)
        << "[TC_LS_RA_12 실패] 20°-10°는 10°여야 함.";
}

/* TC_LS_RA_13 : LS_Lane_Offset 외부 연동 신호 확인
   조건: Lane_Offset = 1.0m
   기대: LS_Lane_Offset (출력 변수)가 입력 값과 동일 (즉, 1.0m)
   ※ 구현에서는 단순 복사 처리하므로, lsOutput.LS_Lane_Offset == laneData.Lane_Offset */
TEST_F(LaneSelectionTest, TC_LS_RA_13) {
    laneData.Lane_Offset = 1.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Lane_Offset, 1.0f, 0.001)
        << "[TC_LS_RA_13 실패] LS_Lane_Offset이 입력 값(1.0m)와 동일해야 함.";
}

/* TC_LS_RA_14 : Lane_Type 출력 확인
   조건: laneData.Lane_Type를 LANE_TYPE_STRAIGHT로 세팅
   기대: lsOutput.LS_Lane_Type == LANE_TYPE_STRAIGHT
*/
TEST_F(LaneSelectionTest, TC_LS_RA_14) {
    laneData.Lane_Type = LANE_TYPE_STRAIGHT;
    ExecLaneSelection();
    EXPECT_EQ(lsOutput.LS_Lane_Type, LANE_TYPE_STRAIGHT)
        << "[TC_LS_RA_14 실패] Lane_Type이 LANE_TYPE_STRAIGHT여야 함.";
}

/* TC_LS_RA_15 : 곡률 전이 영향 확인
   조건: Lane_Curvature = 1000, Next_Lane_Curvature = 400 (diff = 600)
   기대: LS_Curve_Transition_Flag == true */
TEST_F(LaneSelectionTest, TC_LS_RA_15) {
    laneData.Lane_Curvature      = 1000.0f;
    laneData.Next_Lane_Curvature = 400.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Curve_Transition_Flag)
        << "[TC_LS_RA_15 실패] 곡률 전이 조건을 만족하면 전이 플래그가 true여야 함.";
}

/* TC_LS_RA_16 : 곡선 여부 활용 확인
   조건: Lane_Curvature = 300
   기대: LS_Is_Curved_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_16) {
    laneData.Lane_Curvature = 300.0f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Curved_Lane)
        << "[TC_LS_RA_16 실패] 곡률 300이면 LS_Is_Curved_Lane는 true여야 함.";
}

/* TC_LS_RA_17 : 차선 변경 상태 공유 확인
   조건: Lane_Change_Status = LANE_CHANGE_CHANGING
   기대: LS_Is_Changing_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_17) {
    laneData.Lane_Change_Status = LANE_CHANGE_CHANGING;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Changing_Lane)
        << "[TC_LS_RA_17 실패] 변경 중 상태이면 변경 플래그는 true여야 함.";
}

/* TC_LS_RA_18 : 진행 방향 일치 → 오차 0
   조건: Ego_Heading = 10°, Lane_Heading = 10°
   기대: LS_Heading_Error == 0° */
TEST_F(LaneSelectionTest, TC_LS_RA_18) {
    egoData.Ego_Heading   = 10.0f;
    laneData.Lane_Heading = 10.0f;
    ExecLaneSelection();
    EXPECT_NEAR(lsOutput.LS_Heading_Error, 0.0f, 0.001)
        << "[TC_LS_RA_18 실패] 동일 Heading이면 error는 0°여야 함.";
}

/* TC_LS_RA_19 : 오프셋 절대값 비교
   조건: Lane_Offset = 1.0m, Lane_Width = 3.5m → 1.0 < 1.75
   기대: LS_Is_Within_Lane == true */
TEST_F(LaneSelectionTest, TC_LS_RA_19) {
    laneData.Lane_Offset = 1.0f;
    laneData.Lane_Width  = 3.5f;
    ExecLaneSelection();
    EXPECT_TRUE(lsOutput.LS_Is_Within_Lane)
        << "[TC_LS_RA_19 실패] 오프셋 1.0m는 내부로 판별되어야 함.";
}

/* TC_LS_RA_20 : 100Hz 주기 내 계산 일관성 확인
   조건: 동일 입력으로 2회 호출 시 결과 동일
   기대: 두 호출 결과 동일 */
TEST_F(LaneSelectionTest, TC_LS_RA_20) {
    laneData.Lane_Offset = 0.5f;
    ExecLaneSelection();
    bool first = lsOutput.LS_Is_Within_Lane;
    ExecLaneSelection();
    bool second = lsOutput.LS_Is_Within_Lane;
    EXPECT_EQ(first, second)
        << "[TC_LS_RA_20 실패] 동일 입력 시 결과는 일관되어야 함.";
}

/******************************************************************************
 * main()
 ******************************************************************************/
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
