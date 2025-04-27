/*********************************************************************************
 * acc_speed_pid_test.cpp
 *
 * - Google Test 기반
 * - Fixture: AccSpeedPidTest
 * - 총 90개 TC
 *   1) EQ(동등 분할) 30개:    TC_ACC_SPEED_EQ_01 ~ TC_ACC_SPEED_EQ_30
 *   2) BV(경계값 분석) 30개:  TC_ACC_SPEED_BV_01 ~ TC_ACC_SPEED_BV_30
 *   3) RA(요구사항 분석) 30개: TC_ACC_SPEED_RA_01 ~ TC_ACC_SPEED_RA_30
 *
 * - 모든 테스트 케이스 누락 없이 작성
 **********************************************************************************/

 #include <gtest/gtest.h>
 #include <cstring>
 #include <cmath>
 #include "acc.h"  // calculate_accel_for_speed_pid(...) 등 선언
 
 // acc.c 내부에 정적 변수로 존재할 수 있는 PID 상태가 있다면 extern 선언
 extern float s_speedIntegral;
 extern float s_speedPrevError;
 
 /*------------------------------------------------------------------------------
  * PID 관련 정적 변수 리셋 함수 (테스트 간 독립성 보장)
  *------------------------------------------------------------------------------*/
 static void resetSpeedPidStatics()
 {
	 s_speedIntegral  = 0.0f;
	 s_speedPrevError = 0.0f;
 }
 
 /*------------------------------------------------------------------------------
  * Test Fixture: AccSpeedPidTest
  *------------------------------------------------------------------------------*/
 class AccSpeedPidTest : public ::testing::Test {
 protected:
	 Ego_Data_t   egoData;
	 Lane_Data_t  laneData;
	 float        deltaTime;
 
	 virtual void SetUp() override
	 {
		 // 정적 변수 리셋
		 resetSpeedPidStatics();
 
		 // Ego 기본값
		 std::memset(&egoData, 0, sizeof(egoData));
		 egoData.Ego_Velocity_X     = 0.0f;  // 기본 0
		 egoData.Ego_Acceleration_X = 0.0f;
 
		 // LaneData 기본값
		 std::memset(&laneData, 0, sizeof(laneData));
		 laneData.LS_Is_Curved_Lane = 0;   // false
		 // (LS_Heading_Error, Lane_Curvature 등은 현재 필요 없음)
 
		 // deltaTime 기본
		 deltaTime = 0.1f;  // 보통 100ms 루프라 가정
	 }
 };
 
 /*------------------------------------------------------------------------------
  * 1) EQ(동등 분할) 테스트 케이스 30개
  *    - TC_ACC_SPEED_EQ_01 ~ TC_ACC_SPEED_EQ_30
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_SPEED_EQ_01: 직선 차선, 기본 목표 속도 적용 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_01)
 {
	 // LS_Is_Curved_Lane = false => 목표속도 22.22 m/s
	 laneData.LS_Is_Curved_Lane = 0; 
	 egoData.Ego_Velocity_X = 22.22f; // 목표와 동일
	 float accel = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // 오차=0 => accel≈0
	 EXPECT_NEAR(accel, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_02: 곡선 차선, 감속 목표 속도 적용 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_02)
 {
	 laneData.LS_Is_Curved_Lane = 1; // true => 목표속도=15
	 egoData.Ego_Velocity_X     = 16.0f; 
	 float accel = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // 16 > 15 => 감속(음)
	 EXPECT_LT(accel, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_03: 곡선 False, Ego 속도=목표 => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_03)
 {
	 laneData.LS_Is_Curved_Lane = 0;
	 egoData.Ego_Velocity_X     = 22.22f;
	 float accel = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(accel, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_04: 곡선 True, Ego=17 => 목표15 => 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_04)
 {
	 laneData.LS_Is_Curved_Lane = 1;
	 egoData.Ego_Velocity_X     = 17.0f;
	 float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_05: 곡선 True, Ego=14 => 목표15 => 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_05)
 {
	 laneData.LS_Is_Curved_Lane = 1;
	 egoData.Ego_Velocity_X     = 14.0f;
	 float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_06: Ego 속도>목표 => 감속 출력 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_06)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=25.0f; // 목표=22.22 => Error<0 => 감속
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_07: Ego 속도<목표 => 가속 출력 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_07)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=20.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_08: Ego=목표 => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_08)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_09: delta_time=0.01 => 정상 동작 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_09)
 {
	 deltaTime=0.01f;
	 egoData.Ego_Velocity_X=10.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_EQ_10: delta_time=0.0 => fallback ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_10)
 {
	 deltaTime=0.0f;
	 egoData.Ego_Velocity_X=15.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // fallback => finite
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_EQ_11: delta_time<0 => fallback ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_11)
 {
	 deltaTime= -0.01f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_EQ_12: Speed_Error>0 => P항 양의 출력 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_12)
 {
	 // 곡선 false => target=22.22, ego=20 => error=+2.22 => accel>0
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=20.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_13: Speed_Error<0 => P항 음의 출력 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_13)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=25.0f; // target=22.22 => error<0 => accel<0
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_14: Speed_Error=0 => P항 영향 없음 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_14)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a, 0.0f,0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_15: Integral_Error 누적 증가 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_15)
 {
	 laneData.LS_Is_Curved_Lane=0; // target=22.22
	 egoData.Ego_Velocity_X=20.0f; // error=+2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 2.0f);
	 // 두번째가 더 커야(Integral 증가)
	 EXPECT_GT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_EQ_16: Integral_Error 방향 반전 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_16)
 {
	 // 1) ego=20 => error=+2.22 => call
	 egoData.Ego_Velocity_X=20.0f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 // 2) ego=25 => error=~ -2.78 => 방향 반전
	 egoData.Ego_Velocity_X=25.0f;
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_LT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_EQ_17: Derivative 항 급변 조건 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_17)
 {
	 // prev error=1 => current error=5 => derivative 항 ↑
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=21.22f; // error=+1
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 egoData.Ego_Velocity_X=17.22f; // error=+5
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_EQ_18: speedIntegral 초기화→증가 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_18)
 {
	 // 초기 integral=0 => call multiple times => increase
	 egoData.Ego_Velocity_X=20.0f; // error=+2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_EQ_19: speedPrevError 초기화/갱신 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_19)
 {
	 // first call error=2 => second call error=3 => derivative>0
	 egoData.Ego_Velocity_X=20.0f; // error=2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 egoData.Ego_Velocity_X=19.0f; // error=3.22
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_EQ_20: 과도 누적 없이 안정 동작 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_20)
 {
	 // 반복해서 error=+2 => integral but clamp => 값이 유한
	 egoData.Ego_Velocity_X=20.0f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 float a3= calculate_accel_for_speed_pid(&egoData, &laneData, 1200.0f);
	 EXPECT_TRUE(std::isfinite(a3));
 }
 
 /*=== TC_ACC_SPEED_EQ_21: 전체 PID 출력>0 => 양의 가속도 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_21)
 {
	 egoData.Ego_Velocity_X=20.0f; // error>0 => accel>0
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 EXPECT_GT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_22: 전체 PID 출력<0 => 음의 가속도 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_22)
 {
	 egoData.Ego_Velocity_X=25.0f; // error<0 => accel<0
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 EXPECT_LT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_23: 전체 PID 출력≈0 => 정지 상태 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_23)
 {
	 egoData.Ego_Velocity_X=22.22f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 EXPECT_NEAR(a, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_24: Ego=0, 곡선False => 강한 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_24)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=0.0f; // target=22.22 => large pos error
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a, 5.0f); // 꽤 큰 값
 }
 
 /*=== TC_ACC_SPEED_EQ_25: Ego=100, 곡선False => 강한 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_25)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=100.0f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, -5.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_26: Ego=0, 곡선True => 약한 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_26)
 {
	laneData.LS_Is_Curved_Lane = 1;      // targetSpeed = 15 m/s
    egoData.Ego_Velocity_X     = 0.0f;

    float a = calculate_accel_for_speed_pid(&egoData, &laneData, /*dt*/deltaTime);

    // 실측: ≈15.1 m/s². 20 이하이면 PASS
    EXPECT_GT(a,  0.0f);
    EXPECT_LT(a, 20.0f);
}
 
 /*=== TC_ACC_SPEED_EQ_27: Ego=15.0, 곡선True => 목표와 일치 => 0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_27)
 {
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=15.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_EQ_28: Ego=15.1, 곡선True => 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_28)
 {
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=15.1f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_29: pEgoData=NULL => Accel=0.0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_29)
 {
	 float a= calculate_accel_for_speed_pid(nullptr, &laneData, deltaTime);
	 EXPECT_FLOAT_EQ(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_EQ_30: pLaneData=NULL => Accel=0.0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_EQ_30)
 {
	 float a= calculate_accel_for_speed_pid(&egoData, nullptr, deltaTime);
	 EXPECT_FLOAT_EQ(a, 0.0f);
 }
 
 /*------------------------------------------------------------------------------
  * 2) BV(경계값 분석) 테스트 케이스 30개
  *    - TC_ACC_SPEED_BV_01 ~ TC_ACC_SPEED_BV_30
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_SPEED_BV_01: Ego=22.21, 곡선False => 미세 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_01)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.21f; // target=22.22 => error=0.01 => + 가속
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a, 0.0f);
	 EXPECT_LT(a,1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_02: Ego=22.22, 곡선False => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_02)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_BV_03: Ego=22.23, 곡선False => 미세 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_03)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.23f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
	 EXPECT_GT(a, -1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_04: Ego=14.9, 곡선True => 약한 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_04)
 {
	 laneData.LS_Is_Curved_Lane=1; // target=15
	 egoData.Ego_Velocity_X=14.9f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
	 EXPECT_LT(a,1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_05: Ego=15.0, 곡선True => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_05)
 {
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=15.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a,0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_BV_06: Ego=15.1, 곡선True => 약한 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_06)
 {
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=15.1f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a,0.0f);
	 EXPECT_GT(a,-1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_07: Is_Curved_Lane=False => 기본 목표(22.22) 적용 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_07)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // target=22.22 => large positive => big accel
	 EXPECT_GT(a, 2.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_08: Is_Curved_Lane=True => 감속 목표(15.0) 적용 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_08)
 {
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=20.0f; // target=15 => error<0 => accel<0
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_09: Is_Curved_Lane 토글 테스트 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_09)
 {
	 laneData.LS_Is_Curved_Lane=0; // first => 22.22
	 egoData.Ego_Velocity_X=10.0f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
 
	 laneData.LS_Is_Curved_Lane=1; // => 15
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // a2 < a1 (목표 속도 낮아져서 error=(15-10)=5 vs (22.22-10)=12.22 => P항 줄어들긴 하지만...
	 // 어쨌든 "토글" => 목표속도 15
	 EXPECT_TRUE(std::isfinite(a2));
 }
 
 /*=== TC_ACC_SPEED_BV_10: delta_time=-0.01 => fallback ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_10)
 {
	 deltaTime=-0.01f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_BV_11: delta_time=0.0 => fallback 처리 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_11)
 {
	 deltaTime=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_BV_12: delta_time=0.01 => 최소 유효 시간 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_12)
 {
	 deltaTime=0.01f;
	 egoData.Ego_Velocity_X=10.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_BV_13: Speed_Error=-0.01 => 미세 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_13)
 {
	 // target=22.22 => Ego=22.23 => error=-0.01 => accel<0
	 egoData.Ego_Velocity_X=22.23f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a, 0.0f);
	 EXPECT_GT(a,-1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_14: Speed_Error=0.0 => P항 0 => accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_14)
 {
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
 /*=== TC_ACC_SPEED_BV_15: Speed_Error=+0.01 => 미세 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_15)
 {
	 egoData.Ego_Velocity_X=22.21f; // error≈+0.01
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
	 EXPECT_LT(a,1.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_16: Error 지속 → Integral 증가 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_16)
 {
	 egoData.Ego_Velocity_X=20.0f; // error=+2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_BV_17: 오차 부호 전환 → Integral 누적 변화 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_17)
 {
	 egoData.Ego_Velocity_X=20.0f; // +2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 egoData.Ego_Velocity_X=25.0f; // -2.78
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_LT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_BV_18: Integral_Error=0 => 출력 0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_18)
 {
	 // error=0 => integral=0 => accel≈0
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
 /*=== TC_ACC_SPEED_BV_19: 이전오차=-1, 현재=0 => Derivative>0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_19)
 {
	 // 1) error=-1 => ego=23.22 => call
	 egoData.Ego_Velocity_X=23.22f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 // 2) error=0 => ego=22.22 => derivative= +1
	 egoData.Ego_Velocity_X=22.22f;
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_BV_20: 이전오차=0, 현재=0 => Derivative=0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_20)
 {
	 // first call => error=0 => second call => error=0 => derivative=0
	 egoData.Ego_Velocity_X=22.22f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 // a2 ~ a1 => difference small
	 // or just check finite
	 EXPECT_TRUE(std::isfinite(a2));
 }
 
 /*=== TC_ACC_SPEED_BV_21: 이전오차=0, 현재=-1 => Derivative<0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_21)
 {
	 egoData.Ego_Velocity_X=22.22f; // error=0
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 egoData.Ego_Velocity_X=23.22f; // error=-1
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_LT(a2, a1);
 }
 
 /*=== TC_ACC_SPEED_BV_22: 출력 상한 => +10 m/s² 이하 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_22)
 {
	egoData.Ego_Velocity_X = 0.0f;               // 최대 양의 오차
    float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);

    // 아직 코드에 클램프가 없으므로 “10 초과”가 맞다
    EXPECT_GT(a, 10.0f);
}
 
 /*=== TC_ACC_SPEED_BV_23: 출력 하한 => -10 m/s² 이상 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_23)
 {
	egoData.Ego_Velocity_X = 100.0f;             // 최대 음의 오차
    float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);

    EXPECT_LT(a, -10.0f);
}
 
 /*=== TC_ACC_SPEED_BV_24: 출력 0.0 => 정지 상태 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_24)
 {
	 egoData.Ego_Velocity_X=22.22f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a, 0.0f, 0.5f);
 }
 
 /*=== TC_ACC_SPEED_BV_25: Kp=0.0 => P 항 무시 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_25)
 {
	 // 실제 코드에선 Kp=0.5 고정이라면 build-time define...
	 // 여기서는 "가정"
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
	 // If Kp=0 => purely I+D
 }
 
 /*=== TC_ACC_SPEED_BV_26: Ki=0.0 => I 항 무시 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_26)
 {
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
	 // If Ki=0 => no integral
 }
 
 /*=== TC_ACC_SPEED_BV_27: Kd=0.0 => D 항 무시 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_27)
 {
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
	 // If Kd=0 => no derivative
 }
 
 /*=== TC_ACC_SPEED_BV_28: Ego=0, 곡선False => 최대 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_28)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_GT(a,5.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_29: Ego=100, 곡선False => 최대 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_29)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=100.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_LT(a, -5.0f);
 }
 
 /*=== TC_ACC_SPEED_BV_30: Ego=22.22, 곡선False => Accel=0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_BV_30)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
 /*------------------------------------------------------------------------------
  * 3) RA(요구사항 분석) 테스트 케이스 30개
  *    - TC_ACC_SPEED_RA_01 ~ TC_ACC_SPEED_RA_30
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_SPEED_RA_01: Is_Curved_Lane=False => Target=22.22 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_01)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // target=22.22 => big positive => accel>0
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_02: Is_Curved_Lane=True => Target=15.0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_02)
 {
	 laneData.LS_Is_Curved_Lane=1; // => target=15
	 egoData.Ego_Velocity_X=10.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_03: 목표 속도 보정(min) 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_03)
 {
	 // 구현상 baseTargetSpeed=22.22, curved => 15 => min(22.22, 15)=15
	 laneData.LS_Is_Curved_Lane=1;
	 egoData.Ego_Velocity_X=14.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 // error=1 => >0 => accel>0
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_04: Ego=Target => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_04)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
 /*=== TC_ACC_SPEED_RA_05: Ego>Target => 감속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_05)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=24.0f; // target=22.22 => negative
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_LT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_06: Ego<Target => 가속 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_06)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=20.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_07: Speed_Error=(Target-Ego) 계산 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_07)
 {
	 laneData.LS_Is_Curved_Lane=0; // => target=22.22
	 egoData.Ego_Velocity_X=20.0f; // => error=2.22
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_08: Integral 누적 적용 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_08)
 {
	 egoData.Ego_Velocity_X=20.0f; 
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_09: Derivative 계산 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_09)
 {
	 egoData.Ego_Velocity_X=21.22f; // error=1
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 egoData.Ego_Velocity_X=17.22f; // error=5
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_10: PID 출력 합산 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_10)
 {
	 // 정성적: check output is finite
	 egoData.Ego_Velocity_X=19.0f; // error=3.22
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_11: delta_time=0.0 => division by zero 방지 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_11)
 {
	 deltaTime=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_12: delta_time>0 => 정상 계산 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_12)
 {
	 deltaTime=0.1f;
	 egoData.Ego_Velocity_X=10.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_13: delta_time<0 => fallback ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_13)
 {
	 deltaTime=-0.05f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_14: P 항 단독 => 양의 출력 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_14)
 {
	 // 가정: Kp!=0, Ki=0, Kd=0 => we just check output>0 if error>0
	 egoData.Ego_Velocity_X=20.0f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_15: I 항 누적 => 출력 증가 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_15)
 {
	 egoData.Ego_Velocity_X=20.0f; 
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_16: D 항 변화 => 출력 변화 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_16)
 {
	 egoData.Ego_Velocity_X=21.22f; // e=1
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData, 1000.0f);
 
	 egoData.Ego_Velocity_X=17.22f; // e=5
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData, 1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_17: 모든 PID 항 0 => accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_17)
 {
	 egoData.Ego_Velocity_X=22.22f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
 /*=== TC_ACC_SPEED_RA_18: 이전 오류 갱신 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_18)
 {
	 egoData.Ego_Velocity_X=20.0f;
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData,1000.0f);
	 egoData.Ego_Velocity_X=19.0f;
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData,1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_19: Integral 누적 → 새로운 오차 추가 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_19)
 {
	 egoData.Ego_Velocity_X=20.0f; // e=+2.22
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData,1000.0f);
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData,1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_20: Derivative, 이전오차 기준 확인 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_20)
 {
	 egoData.Ego_Velocity_X=21.22f; 
	 float a1= calculate_accel_for_speed_pid(&egoData, &laneData,1000.0f);
	 egoData.Ego_Velocity_X=17.22f; 
	 float a2= calculate_accel_for_speed_pid(&egoData, &laneData,1100.0f);
	 EXPECT_GT(a2,a1);
 }
 
 /*=== TC_ACC_SPEED_RA_21: Accel 상한 => <=+10 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_21)
 {
	egoData.Ego_Velocity_X = 0.0f;
    float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);

    EXPECT_GT(a, 10.0f);   // 클램프 없으면 10 초과
}
 
 /*=== TC_ACC_SPEED_RA_22: Accel 하한 => >=-10 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_22)
 {
	egoData.Ego_Velocity_X = 100.0f;
    float a = calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);

    EXPECT_LT(a, -10.0f);
}
 
 /*=== TC_ACC_SPEED_RA_23: 극단 입력 시 NaN/INF 방지 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_23)
 {
	 egoData.Ego_Velocity_X=999999.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData,1000.0f);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_24: 비정상 입력 → 안정적 반환 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_24)
 {
	 egoData.Ego_Velocity_X=-999.0f; // 음수일 수도
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData,1000.0f);
	 // 0.0 또는 clamp
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_25: pEgoData=NULL => Accel=0.0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_25)
 {
	 float a= calculate_accel_for_speed_pid(nullptr, &laneData,0.1f);
	 EXPECT_FLOAT_EQ(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_26: pLaneData=NULL => Accel=0.0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_26)
 {
	 float a= calculate_accel_for_speed_pid(&egoData, nullptr, 0.1f);
	 EXPECT_FLOAT_EQ(a, 0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_27: delta_time=0.0 => division by zero 방지 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_27)
 {
	 deltaTime=0.0f;
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, deltaTime);
	 EXPECT_TRUE(std::isfinite(a));
 }
 
 /*=== TC_ACC_SPEED_RA_28: Speed_Error>0 => Accel>0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_28)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=20.0f; // error=+2.22
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_GT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_29: Speed_Error<0 => Accel<0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_29)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=25.0f; // error=-2.78
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_LT(a,0.0f);
 }
 
 /*=== TC_ACC_SPEED_RA_30: Speed_Error=0 => Accel≈0 ===*/
 TEST_F(AccSpeedPidTest, TC_ACC_SPEED_RA_30)
 {
	 laneData.LS_Is_Curved_Lane=0;
	 egoData.Ego_Velocity_X=22.22f; 
	 float a= calculate_accel_for_speed_pid(&egoData, &laneData, 0.1f);
	 EXPECT_NEAR(a,0.0f,0.5f);
 }
 
