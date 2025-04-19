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
 
 #include "lfa.h"            /* LFA_Mode_t, lfa_output_selection(...) */
 #include "adas_shared.h"    /* Ego_Data_t, Lane_Data_LS_t */
 
 static constexpr float kEps = 1e-3f;
 
 /*==========================================================================*/
 /*  Test Fixture                                                            */
 /*==========================================================================*/
 class LfaOutputSelectionTest : public ::testing::Test
 {
 protected:
	 /* 입력 */
	 LFA_Mode_e        mode;
	 float             steerPID;
	 float             steerStanley;
	 Lane_Data_LS_t    lane;
	 Ego_Data_t        ego;
 
	 /* 출력 */
	 float             steerOut;
 
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
  *  EQ (Test Cases 01 ~ 20)
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_01)
 {
	 mode         = LFA_MODE_LOW_SPEED;
	 steerPID     = 50.0f;
	 steerStanley = 100.0f;
	 EXPECT_NEAR(Call(), 50.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_02)
 {
	 mode         = LFA_MODE_HIGH_SPEED;
	 steerPID     = 50.0f;
	 steerStanley = 60.0f;
	 EXPECT_NEAR(Call(), 60.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_03)
 {
	 steerPID                   = 100.0f;
	 lane.LS_Is_Changing_Lane   = true;
	 EXPECT_NEAR(Call(), 100.0f * 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_04)
 {
	 steerPID                   = 80.0f;
	 lane.LS_Is_Changing_Lane   = false;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_05)
 {
	 steerPID                 = 40.0f;
	 lane.LS_Is_Within_Lane   = false;
	 EXPECT_NEAR(Call(), 40.0f * 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_06)
 {
	 steerStanley               = 70.0f;
	 mode                       = LFA_MODE_HIGH_SPEED;
	 lane.LS_Is_Within_Lane     = true;
	 EXPECT_NEAR(Call(), 70.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_07)
 {
	 steerPID                   = 80.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Yaw_Rate           = 20.0f;
	 EXPECT_NEAR(Call(), 80.0f * 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_08)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = 80.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_09)
 {
	 mode                       = LFA_MODE_HIGH_SPEED;
	 steerStanley               = 100.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Yaw_Rate           = 35.0f;
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_10)
 {
	 mode                       = LFA_MODE_HIGH_SPEED;
	 steerStanley               = 120.0f;
	 lane.LS_Is_Curved_Lane     = true;
	 ego.Ego_Steering_Angle     = 210.0f;
	 EXPECT_NEAR(Call(), 120.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_11)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;
	 EXPECT_NEAR(Call(), 100.0f * 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_12)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
	 EXPECT_NEAR(Call(), 100.0f * 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_13)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 20.0f;
	 EXPECT_NEAR(Call(), 100.0f * 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_14)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_15)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = -210.0f;
	 EXPECT_NEAR(Call(), 100.0f * 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_16)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;      /* ×0.2 → 20 */
	 lane.LS_Is_Within_Lane   = false;     /* ×1.5 → 30 */
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;     /* gain 0.8 → 24 */
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_17)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = 600.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_18)
 {
	 mode             = LFA_MODE_HIGH_SPEED;
	 steerStanley     = -600.0f;
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_19)
 {
	 steerPID = 100.0f;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_EQ_20)
 {
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 /*==========================================================================
  *  BV (Test Cases 21 ~ 40)
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_01)
 {
	 steerPID = 539.9f;
	 EXPECT_NEAR(Call(), 539.9f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_02)
 {
	 steerPID = 540.1f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_03)
 {
	 steerStanley = -540.1f;
	 mode         = LFA_MODE_HIGH_SPEED;
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_04)
 {
	 steerPID = 0.0f;
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_05)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = true;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_06)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
	 EXPECT_NEAR(Call(), 150.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_07)
 {
	 steerPID = 180.0f;
	 EXPECT_NEAR(Call(), 180.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_08)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 29.9f;
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_09)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 30.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_10)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 199.9f;
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_11)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 200.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_12)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_13)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 31.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_14)
 {
	 steerPID                 = 1.0f;
	 lane.LS_Is_Changing_Lane = true;
	 EXPECT_NEAR(Call(), 0.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_15)
 {
	 steerPID               = 1.0f;
	 lane.LS_Is_Within_Lane = false;
	 EXPECT_NEAR(Call(), 1.5f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_16)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 1.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 EXPECT_NEAR(Call(), 1.2f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_17)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 1.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 210.0f;
	 EXPECT_NEAR(Call(), 0.8f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_18)
 {
	 steerPID                 = 1.0f;
	 lane.LS_Is_Changing_Lane = true;   /* ×0.2 → 0.2 */
	 lane.LS_Is_Within_Lane   = false;  /* ×1.5 → 0.3 */
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;  /* ×0.8 → 0.24 */
	 EXPECT_NEAR(Call(), 0.24f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_19)
 {
	 steerPID = 1000.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_BV_20)
 {
	 steerPID = -1000.0f;
	 EXPECT_NEAR(Call(), -540.0f, kEps);
 }
 
 /*==========================================================================
  *  RA (Test Cases 41 ~ 60)
  *==========================================================================*/
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_01)
 {
	 mode      = LFA_MODE_LOW_SPEED;
	 steerPID  = 80.0f;
	 steerStanley = 120.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_02)
 {
	 mode         = LFA_MODE_HIGH_SPEED;
	 steerPID     = 80.0f;
	 steerStanley = 100.0f;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_03)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;
	 EXPECT_NEAR(Call(), 20.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_04)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = true;
	 EXPECT_NEAR(Call(), 100.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_05)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Within_Lane = false;
	 EXPECT_NEAR(Call(), 150.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_06)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 EXPECT_NEAR(Call(), 120.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_07)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_08)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Steering_Angle   = 210.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_09)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;     // ×0.2  =20
	 lane.LS_Is_Within_Lane   = false;    // ×1.5  =30
	 lane.LS_Is_Curved_Lane   = true;     // ×1.2  =36 (yaw <30, steer <200)
	 ego.Ego_Yaw_Rate         = 25.0f;
	 EXPECT_NEAR(Call(), 36.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_10)
 {
	 steerPID = 700.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_11)
 {
	 steerPID     = 0.0f;
	 steerStanley = 0.0f;
	 EXPECT_NEAR(Call(), 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_12)
 {
	 mode                         = LFA_MODE_HIGH_SPEED;
	 steerStanley                 = 100.0f;
	 lane.LS_Is_Changing_Lane     = true;   // ×0.2 =20
	 lane.LS_Is_Within_Lane       = false;  // ×1.5 =30
	 lane.LS_Is_Curved_Lane       = true;
	 ego.Ego_Yaw_Rate             = 35.0f;  // ×0.8 =24
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_13)
 {
	 steerPID = 600.0f;
	 EXPECT_NEAR(Call(), 540.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_14)
 {
	 mode                     = LFA_MODE_HIGH_SPEED;
	 steerStanley             = 200.0f;
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 25.0f;   /* gain 1.2 => 240 */
	 EXPECT_NEAR(Call(), 240.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_15)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;   // ×0.2 =20
	 lane.LS_Is_Within_Lane   = false;  // ×1.5 =30
	 lane.LS_Is_Curved_Lane   = true;   // ×1.2 =36
	 EXPECT_NEAR(Call(), 36.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_16)
 {
	 steerPID               = 100.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
	 EXPECT_NEAR(Call(), 80.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_17)
 {
	 steerPID                 = 100.0f;
	 lane.LS_Is_Changing_Lane = true;   // ×0.2
	 lane.LS_Is_Within_Lane   = false;  // ×1.5
	 lane.LS_Is_Curved_Lane   = true;
	 ego.Ego_Yaw_Rate         = 35.0f;  // ×0.8
	 EXPECT_NEAR(Call(), 24.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_18)
 {
	 float out =
		 lfa_output_selection(LFA_MODE_LOW_SPEED, 100.0f, 50.0f,
							  nullptr,
							  &ego);
	 EXPECT_NEAR(out, 0.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_19)
 {
	 /* (a) gain 1.2 */
	 steerPID               = 50.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 20.0f;
	 float out12 = Call();  /* 50 × 1.2 = 60 */
 
	 /* (b) gain 0.8 */
	 SetUp();               /* fixture reset */
	 steerPID               = 50.0f;
	 lane.LS_Is_Curved_Lane = true;
	 ego.Ego_Yaw_Rate       = 35.0f;
	 float out08 = Call();  /* 50 × 0.8 = 40 */
 
	 EXPECT_NEAR(out12, 60.0f, kEps);
	 EXPECT_NEAR(out08, 40.0f, kEps);
 }
 
 TEST_F(LfaOutputSelectionTest, TC_LFA_OUT_RA_20)
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
 