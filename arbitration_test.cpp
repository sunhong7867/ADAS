/*******************************************************************************
 * arbitration_test.cpp
 *
 *  - Google Test 기반
 *  - Test‑Fixture : ArbitrationTest
 *  - 총 60 TC  (EQ 20, BV 20, RA 20)
 *  - 대상 함수 : Arbitration()
 *               (arbitration.c / arbitration.h, adas_shared.h 포함)
 ******************************************************************************/

 #include <gtest/gtest.h>
 #include <cstring>
 #include <cmath>
 
 #include "arbitration.h"     /* Arbitration(), VehicleControl_t               */
 #include "adas_shared.h"     /* AEB_Mode_e                                    */
 
 static constexpr float kEPS = 1e-4f;
 
 /*==============================================================================
  * Test Fixture
  *============================================================================*/
 class ArbitrationTest : public ::testing::Test
 {
 protected:
	 /* 입력 파라미터 */
	 float      accAccel;      /* ACC  [m/s²]  (-10 ~ +10) */
	 float      aebDecel;      /* AEB  [m/s²]  (-10 ~    0) */
	 float      lfaSteer;      /* LFA  [deg]   (-540~+540) */
	 AEB_Mode_e aebMode;
 
	 /* 출력 */
	 VehicleControl_t cmd;
 
	 void SetUp() override
	 {
		 accAccel = 0.0f;
		 aebDecel = 0.0f;
		 lfaSteer = 0.0f;
		 aebMode  = AEB_MODE_NORMAL;
		 std::memset(&cmd, 0, sizeof(cmd));
	 }
 
	 void CallArbitration()
	 {
		 Arbitration(accAccel, aebDecel, lfaSteer, aebMode, &cmd);
	 }
 };
 
 /*==============================================================================
  *  EQ (동등분할) : TC_ARB_EQ_01 ~ TC_ARB_EQ_20
  *============================================================================*/
 TEST_F(ArbitrationTest, TC_ARB_EQ_01_AebBrakePriority)
 {
	 accAccel = +5.0f;     /* 무시됨 */
	 aebDecel = -5.0f;
	 aebMode  = AEB_MODE_BRAKE;
	 CallArbitration();
 
	 EXPECT_NEAR(cmd.brake,    0.5f, kEPS);
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_02_AccAccelUsedInNormal)
 {
	 accAccel = +5.0f;
	 aebDecel = -5.0f;
	 aebMode  = AEB_MODE_NORMAL;
	 CallArbitration();
 
	 EXPECT_NEAR(cmd.throttle, 0.5f, kEPS);
	 EXPECT_NEAR(cmd.brake,    0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_03_AccDecelSelected)
 {
	 accAccel = -3.0f;   /* AEB NORMAL => ACC 사용 */
	 CallArbitration();
 
	 EXPECT_NEAR(cmd.brake,    0.3f, kEPS);
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_04_ZeroAccelNoThrottleBrake)
 {
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
	 EXPECT_NEAR(cmd.brake,    0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_05_SteerPositiveHalf)
 {
	 lfaSteer = 270.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 0.5f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_06_SteerNegativeFull)
 {
	 lfaSteer = -540.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, -1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_07_SteerClampHigh)
 {
	 lfaSteer = 600.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_08_MaxBrake)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_09_MaxThrottle)
 {
	 accAccel = +10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_10_ThrottleClamp)
 {
	 accAccel = +12.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_11_BrakeClamp)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -12.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_12_AebBrakeZeroDecel)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = 0.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake,    0.0f, kEPS);
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_13_AccNegativeDecel)
 {
	 accAccel = -5.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.5f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_14_AebAlertUsesAcc)
 {
	 accAccel = +6.0f;
	 aebMode  = AEB_MODE_ALERT;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.6f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_15_MutualExclusiveThrottleBrake)
 {
	 accAccel = +4.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.0f, kEPS);
 
	 accAccel = -4.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_16_NullPointerSafe)
 {
	 /* EXPECT : 함수가 크래시 없이 리턴 */
	 EXPECT_NO_FATAL_FAILURE(
		 Arbitration(5.0f, 0.0f, 100.0f, AEB_MODE_NORMAL, nullptr));
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_17_AebOverridesAcc)
 {
	 accAccel = +8.0f;
	 aebDecel = -2.0f;
	 aebMode  = AEB_MODE_BRAKE;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.2f, kEPS);
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_18_AccSelectedWhenAebNormal)
 {
	 accAccel = +2.0f;
	 aebDecel = -8.0f;
	 aebMode  = AEB_MODE_NORMAL;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.2f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_19_SteerZeroRatioZero)
 {
	 lfaSteer = 0.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_EQ_20_CompositeCase)
 {
	 accAccel = +6.0f;          /* throttle 0.6 */
	 lfaSteer = -270.0f;        /* steer -0.5   */
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.6f, kEPS);
	 EXPECT_NEAR(cmd.steer,   -0.5f, kEPS);
 }
 
 /*==============================================================================
  *  BV (경계값) : TC_ARB_BV_01 ~ TC_ARB_BV_20
  *============================================================================*/
 TEST_F(ArbitrationTest, TC_ARB_BV_01_AccelZero)
 {
	 accAccel = 0.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
	 EXPECT_NEAR(cmd.brake,    0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_02_AccelPoint01)
 {
	 accAccel = 0.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.001f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_03_AccelMinusPoint01)
 {
	 accAccel = -0.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.001f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_04_Accel9_99)
 {
	 accAccel = 9.99f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.999f, 1e-3f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_05_Accel10)
 {
	 accAccel = 10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_06_Accel10_01Clamp)
 {
	 accAccel = 10.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_07_DecelMinus9_99)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -9.99f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.999f, 1e-3f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_08_DecelMinus10)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_09_DecelMinus10_01Clamp)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -10.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_10_Steer539_9)
 {
	 lfaSteer = 539.9f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 0.9998f, 1e-4f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_11_Steer540)
 {
	 lfaSteer = 540.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_12_Steer540_1Clamp)
 {
	 lfaSteer = 540.1f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_13_SteerMinus539_9)
 {
	 lfaSteer = -539.9f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, -0.9998f, 1e-4f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_14_SteerMinus540)
 {
	 lfaSteer = -540.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, -1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_15_SteerMinus540_1Clamp)
 {
	 lfaSteer = -540.1f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, -1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_16_AebBrakeSmallDecel)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = -0.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.001f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_17_AebBrakeZero)
 {
	 aebMode  = AEB_MODE_BRAKE;
	 aebDecel = 0.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_18_AccSmallPositive)
 {
	 accAccel = 0.01f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.001f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_19_OutputRangePositive)
 {
	 accAccel = 20.0f;      /* 클램프 */
	 lfaSteer = 1000.0f;    /* 클램프 */
	 CallArbitration();
	 EXPECT_LE(cmd.throttle, 1.0f);
	 EXPECT_GE(cmd.throttle, 0.0f);
	 EXPECT_LE(cmd.steer, 1.0f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_BV_20_OutputRangeNegative)
 {
	 accAccel = -20.0f;
	 lfaSteer = -1000.0f;
	 CallArbitration();
	 EXPECT_LE(cmd.brake, 1.0f);
	 EXPECT_GE(cmd.brake, 0.0f);
	 EXPECT_GE(cmd.steer, -1.0f);
 }
 
 /*==============================================================================
  *  RA (요구사항기반) : TC_ARB_RA_01 ~ TC_ARB_RA_20
  *============================================================================*/
 TEST_F(ArbitrationTest, TC_ARB_RA_01_AebBrakeAlwaysDominant)
 {
	 /* accAccel 양수여도 AEB BRAKE 우선 */
	 accAccel = +9.0f;
	 aebDecel = -1.0f;
	 aebMode  = AEB_MODE_BRAKE;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.1f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_02_AccNormalFlow)
 {
	 accAccel = +3.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.3f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_03_LinearThrottleMapping)
 {
	 accAccel = +7.5f;  /* 75 % */
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.75f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_04_LinearBrakeMapping)
 {
	 accAccel = -7.5f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.75f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_05_ThrottleFullAt10)
 {
	 accAccel = +10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_06_BrakeFullAtMinus10)
 {
	 accAccel = -10.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_07_NoThrottleWhenAccelNegative)
 {
	 accAccel = -4.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_08_NoBrakeWhenAccelPositive)
 {
	 accAccel = +4.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_09_SteerLinearMap)
 {
	 lfaSteer = 135.0f;   /* 25 % */
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 0.25f, 1e-4f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_10_SteerClampWorked)
 {
	 lfaSteer = 9999.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, 1.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_11_ZeroAccelGivesNoPedals)
 {
	 accAccel = 0.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
	 EXPECT_NEAR(cmd.brake,    0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_12_PositiveAccelBrakeZero)
 {
	 accAccel = +2.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_13_NegativeAccelThrottleZero)
 {
	 accAccel = -2.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_14_OutputStructPopulated)
 {
	 accAccel = +5.0f;
	 CallArbitration();
	 EXPECT_FLOAT_EQ(cmd.throttle, 0.5f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_15_NullPointerNoCrash)
 {
	 EXPECT_NO_FATAL_FAILURE(
		 Arbitration(0.0f, 0.0f, 0.0f, AEB_MODE_NORMAL, nullptr));
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_16_YawAndSteerLimitsDoNotAffectThrottle)
 {
	 accAccel = +7.0f;
	 lfaSteer = 540.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.7f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_17_BothPedalsNeverActiveTogether)
 {
	 accAccel = +6.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.brake, 0.0f, kEPS);
 
	 accAccel = -6.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.0f, kEPS);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_18_SignedSteerRange)
 {
	 lfaSteer = -135.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.steer, -0.25f, 1e-4f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_19_OutputRangesAlwaysValid)
 {
	 accAccel = +20.0f;
	 lfaSteer = 10000.0f;
	 CallArbitration();
 
	 EXPECT_GE(cmd.throttle, 0.0f);
	 EXPECT_LE(cmd.throttle, 1.0f);
	 EXPECT_GE(cmd.steer,   -1.0f);
	 EXPECT_LE(cmd.steer,    1.0f);
 }
 
 TEST_F(ArbitrationTest, TC_ARB_RA_20_AllRulesFalseOriginal)
 {
	 accAccel = +1.0f;
	 CallArbitration();
	 EXPECT_NEAR(cmd.throttle, 0.1f, kEPS);
 }
 
