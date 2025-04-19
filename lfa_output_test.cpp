/******************************************************************************
 * lfa_output_selection_test.cpp
 *
 *  Google‑Test 기반 LFA 최종 조향각 선택 로직 단위시험
 *  - Test‑Fixture : LfaOutputSelectionTest
 *  - TC 구성      : EQ 20 + BV 20 + RA 20  = 총 60 Case
 *  - 중간 생략‑없음 (실제 프로젝트 컴파일 시 lfa.h / adas_shared.h 경로만 조정)
 *  - 테스트 대상  : float lfa_output_selection( LFA_Mode_t       mode,
 *                                             float            steerPID,
 *                                             float            steerStanley,
 *                                             const Lane_Data_LS_t* pLane,
 *                                             const Ego_Data_t*         pEgo );
 ******************************************************************************/

 #include <gtest/gtest.h>
 #include <cstring>
 #include <cmath>
 
 #include "lfa.h"            /* LFA_Mode_t, lfa_output_selection(...)             */
 #include "adas_shared.h"    /* Ego_Data_t, Lane_Data_LS_t, etc.               */
 
 static constexpr float kEps = 1e-3f;
 
 /*==========================================================================*/
 /*  Test Fixture                                                            */
 /*==========================================================================*/
 class LfaOutputSelectionTest : public ::testing::Test
 {
 protected:
	 /* 입력 */
	 LFA_Mode_e           mode;
	 float                steerPID;
	 float                steerStanley;
	 Lane_Data_LS_t   lane;
	 Ego_Data_t            ego;
 
	 /* 출력 */
	 float                steerOut;
 
	 void SetUp() override
	 {
		 std::memset(&lane, 0, sizeof(lane));
		 std::memset(&ego , 0, sizeof(ego ));
 
		 /*‑‑ 기본값 --------------------------------------------------------*/
		 mode            = LFA_MODE_LOW_SPEED;
		 steerPID        = 0.0f;
		 steerStanley    = 0.0f;
		 lane.LS_Is_Changing_Lane = false;
		 lane.LS_Is_Within_Lane   = true;
		 lane.LS_Is_Curved_Lane   = false;
		 ego.Ego_Yaw_Rate         = 0.0f;
		 ego.Ego_Steering_Angle   = 0.0f;
 
		 steerOut = 0.0f;
	 }
 
	 /* 호출 래퍼 */
	 float Call()
	 {
		 steerOut = lfa_output_selection(mode,
										 steerPID,
										 steerStanley,
										 &lane,
										 &ego);
		 return steerOut;
	 }
 };
 
 /*==========================================================================
  *  EQ (Test Cases 01 ~ 20) ― 동등 분할
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_01_LowSpeedUsesPID)
 {
	 mode         = LFA_MODE_LOW_SPEED;
	 steerPID     = 50.0f;
	 steerStanley = 100.0f;
 
	 EXPECT_NEAR(Call(), 50.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_02_HighSpeedUsesStanley)
 {
	 mode         = LFA_MODE_HIGH_SPEED;
	 steerPID     = 50.0f;
	 steerStanley = 60.0f;
 
	 EXPECT_NEAR(Call(), 60.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_03_ChangingLaneAttenuation)
 {
	 steerPID                   = 100.0f;
	 lane.LS_Is_Changing_Lane   = true;
 
	 EXPECT_NEAR(Call(), 100.0f * 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_04_NoAttenuationWhenNotChanging)
 {
	 steerPID                   = 80.0f;
	 lane.LS_Is_Changing_Lane   = false;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_05_WithinLaneFalseAmplify)
 {
	 steerPID                 = 40.0f;
	 lane.LS_Is_Within_Lane   = false;
 
	 EXPECT_NEAR(Call(), 40.0f * 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_06_WithinLaneTrueNoAmplify)
 {
	 steerStanley               = 70.0f;
	 mode                       = LFA_MODE_HIGH_SPEED;
	 lane.LS_Is_Within_Lane     = true;
 
	 EXPECT_NEAR(Call(), 70.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_07_CurvedLaneGain12)
 {
	 steerPID                   = 80.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Yaw_Rate           = 20.0f;   /* 제한 미충족 */
 
	 EXPECT_NEAR(Call(), 80.0f * 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_08_NotCurvedGain1)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = 80.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_09_CurvePlusHighYawRateGain08)
 {
	 mode                       = LFA_MODE_HIGH_SPEED;
	 steerStanley               = 100.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Yaw_Rate           = 35.0f;   /* >30°/s */
 
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_10_CurvePlusHighSteerAngleGain08)
 {
	 mode                       = LFA_MODE_HIGH_SPEED;
	 steerStanley               = 120.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Steering_Angle     = 210.0f;  /* abs >200 */
 
	 EXPECT_NEAR(Call(), 120.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_11_OnlyChangeLaneRule)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;
 
	 EXPECT_NEAR(Call(), 100.0f * 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_12_OnlyWithinLaneFalseRule)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
 
	 EXPECT_NEAR(Call(), 100.0f * 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_13_OnlyCurvedGain12)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 20.0f;
 
	 EXPECT_NEAR(Call(), 100.0f * 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_14_YawRateOverLimitGain08)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;
 
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_15_SteerAngleOverLimitGain08)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = -210.0f;
 
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_16_AllRulesSequential)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;      /* ×0.2 → 20 */
	 lane.LS_Is_Within_Lane   = false;     /* ×1.5 → 30 */
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;     /* gain 0.8 → 24 */
 
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_17_PositiveClamp)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = 600.0f;      /* >540 */
 
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_18_NegativeClamp)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = -600.0f;
 
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_19_NoRuleApplied)
 {
	 steerPID = 100.0f;
 
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_20_AllZeroSafeOutputZero)
 {
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 /*==========================================================================
  *  BV (Test Cases 21 ~ 40) ― 경계값
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_01_ClampHighEdge5399)
 {
	 steerPID = 539.9f;
	 EXPECT_NEAR(Call(), 539.9f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_02_ClampHighEdge5401)
 {
	 steerPID = 540.1f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_03_ClampLowEdgeMinus5401)
 {
	 steerStanley = -540.1f;
	 mode         = LFA_MODE_HIGH_SPEED;
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_04_ZeroSteerNoRules)
 {
	 steerPID = 0.0f;
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_05_WithinLaneTrueNoAmplifyBoundary)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = true;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_06_WithinLaneFalseAmplifyBoundary)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
	 EXPECT_NEAR(Call(), 150.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_07_Heading180NoRules)
 {
	 steerPID = 180.0f;
	 EXPECT_NEAR(Call(), 180.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_08_YawRate29_9_NoAttenuate)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 29.9f;
 
	 EXPECT_NEAR(Call(), 120.0f, kEps);      /* ×1.2 */
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_09_YawRate30_Attenuate)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 30.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);       /* ×0.8 */
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_10_SteerAngle199_9_NoAttenuate)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 199.9f;
 
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_11_SteerAngle200_Attenuate)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 200.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_12_CurveGainUpperBound12)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
 
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_13_CurveGainLowerBound08)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 31.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_14_ChangingLaneSmallSteer)
 {
	 steerPID                 = 1.0f;
	 lane.LS_Is_Changing_Lane = true;
 
	 EXPECT_NEAR(Call(), 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_15_WithinLaneFalseSmallSteer)
 {
	 steerPID               = 1.0f;
	 lane.LS_Is_Within_Lane = false;
 
	 EXPECT_NEAR(Call(), 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_16_CurvedLaneSmallSteerGain12)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 1.0f;
	 lane.LS_Is_Curved_Lane   = true;
 
	 EXPECT_NEAR(Call(), 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_17_CurvedLaneSmallSteerGain08)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 1.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 210.0f;
 
	 EXPECT_NEAR(Call(), 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_18_AllRulesSmallSteer)
 {
	 steerPID                 = 1.0f;
	 lane.LS_Is_Changing_Lane = true;   /* ×0.2 →0.2 */
	 lane.LS_Is_Within_Lane   = false;  /* ×1.5 →0.3 */
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;  /* ×0.8 →0.24 */
 
	 EXPECT_NEAR(Call(), 0.24f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_19_Input1000Clamp)
 {
	 steerPID = 1000.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_20_InputMinus1000Clamp)
 {
	 steerPID = -1000.0f;
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 /*==========================================================================
  *  RA (Test Cases 41 ~ 60) ― 요구사항 기반
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_01_LowSpeedSelectPID)
 {
	 mode      = LFA_MODE_LOW_SPEED;
	 steerPID  = 80.0f;
	 steerStanley = 120.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_02_HighSpeedSelectStanley)
 {
	 mode         = LFA_MODE_HIGH_SPEED;
	 steerPID     = 80.0f;
	 steerStanley = 100.0f;
 
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_03_ChangingLane20Percent)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;
 
	 EXPECT_NEAR(Call(), 20.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_04_WithinLaneTrueNoAmplify)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = true;
 
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_05_WithinLaneFalseAmplify)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
 
	 EXPECT_NEAR(Call(), 150.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_06_CurvedGain12)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
 
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_07_CurvedGain08ByYawRate)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_08_CurvedGain08BySteerAngle)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 210.0f;
 
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_09_AllRulesSequenceCheck)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;     // ×0.2  =20
	 lane.LS_Is_Within_Lane   = false;    // ×1.5  =30
	 lane.LS_Is_Curved_Lane   = true;     // ×1.2  =36 (yaw <30, steer <200)
	 ego.Ego_Yaw_Rate         = 25.0f;
 
	 EXPECT_NEAR(Call(), 36.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_10_FinalClampAlways)
 {
	 steerPID = 700.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_11_BothInputsZero)
 {
	 steerPID     = 0.0f;
	 steerStanley = 0.0f;
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_12_ChangePlusCurvePlusExitLane)
 {
	 mode                         = LFA_MODE_HIGH_SPEED;
	 steerStanley                 = 100.0f;
	 lane.LS_Is_Changing_Lane     = true;   // ×0.2 =20
	 lane.LS_Is_Within_Lane       = false;  // ×1.5 =30
	 lane.LS_Is_Curved_Lane       = true;
	 ego.Ego_Yaw_Rate             = 35.0f;  // ×0.8 =24
 
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_13_ClampExact540)
 {
	 steerPID = 600.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_14_CurveOvershootStillBelowClamp)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 200.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 25.0f;   /* gain 1.2 => 240 */
 
	 EXPECT_NEAR(Call(), 240.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_15_CorrectionSequenceOrder)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;   // ×0.2 =20
	 lane.LS_Is_Within_Lane   = false;  // ×1.5 =30
	 lane.LS_Is_Curved_Lane   = true;   // ×1.2 =36
	 EXPECT_NEAR(Call(), 36.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_16_CurveGain08ConditionMet)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_17_GainOrderConflictCheck)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;   // ×0.2
	 lane.LS_Is_Within_Lane   = false;  // ×1.5
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;  // ×0.8
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_18_NullLanePointerSafeZero)
 {
	 /* API 호출 시 NULL 전달 케이스 를 별도 함수 랩퍼 로 검증 */
	 float out =
		 lfa_output_selection(LFA_MODE_LOW_SPEED, 100.0f, 50.0f,
							  /* NULL */ nullptr,
							  &ego);
	 EXPECT_NEAR(out, 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_19_CurveGainVariants)
 {
	 /* (a) gain 1.2 */
	 steerPID               = 50.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 20.0f;
	 float out12 = Call();          /* 50×1.2 = 60 */
 
	 /* (b) gain 0.8 */
	 SetUp();                       /* fixture reset */
	 steerPID               = 50.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
	 float out08 = Call();          /* 50×0.8 = 40 */
 
	 EXPECT_NEAR(out12, 60.0f, kEps);
	 EXPECT_NEAR(out08, 40.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_20_AllFalseRulesOriginalOut)
 {
	 steerPID = 100.0f;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 /*==========================================================================*/
 /*  main()                                                                  */
 /*==========================================================================*/
 int main(int argc, char** argv)
 {
	 ::testing::InitGoogleTest(&argc, argv);
	 return RUN_ALL_TESTS();
 }
 