/****************************************************************************
 * acc_output_selection_test.cpp
 *
 * - Google Test 기반
 * - Fixture: AccOutputSelectionTest
 * - 총 30개 테스트 케이스:
 *    1) EQ(동등 분할): TC_ACC_OUT_EQ_01 ~ TC_ACC_OUT_EQ_10
 *    2) BV(경계값 분석): TC_ACC_OUT_BV_01 ~ TC_ACC_OUT_BV_10
 *    3) RA(요구사항 분석): TC_ACC_OUT_RA_01 ~ TC_ACC_OUT_RA_10
****************************************************************************/

#include <gtest/gtest.h>
#include <cstring>
#include <cmath>
#include "acc.h"  // acc_output_selection(...), ACC_Mode_e, etc.
 
/*------------------------------------------------------------------------------
 * Test Fixture
 *------------------------------------------------------------------------------*/
 class AccOutputSelectionTest : public ::testing::Test {
 protected:
	 // 입력 값들
	 ACC_Mode_e accMode;
	 float      accelDistanceX;
	 float      accelSpeedX;
 
	 virtual void SetUp() override
	 {
		 // 기본값 (테스트마다 필요한 값만 변경)
		 accMode         = ACC_MODE_SPEED;  // 기본 SPEED
		 accelDistanceX  = 0.0f;
		 accelSpeedX     = 0.0f;
	 }
 };
 
 /*------------------------------------------------------------------------------
  * 1) EQ(동등 분할) 테스트 10개
  *    - TC_ACC_OUT_EQ_01 ~ TC_ACC_OUT_EQ_10
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_OUT_EQ_01: SPEED → Accel_Speed_X 반환 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_01)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     =  2.5f;
	 accelDistanceX  =  1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // SPEED => Accel_Speed_X(=+2.5) 사용
	 EXPECT_FLOAT_EQ(result, 2.5f);
 }
 
 /*=== TC_ACC_OUT_EQ_02: DISTANCE → Accel_Distance_X 반환 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_02)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelSpeedX     =  3.0f;
	 accelDistanceX  = -1.5f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // DISTANCE => Accel_Distance_X(=-1.5) 사용
	 EXPECT_FLOAT_EQ(result, -1.5f);
 }
 
 /*=== TC_ACC_OUT_EQ_03: STOP → 0.0 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_03)
 {
	 accMode         = ACC_MODE_STOP;
	 accelSpeedX     = 123.0f;
	 accelDistanceX  = -999.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // STOP => 0.0
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_04: SPEED 모드, 우선순위 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_04)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 2.0f;
	 accelDistanceX  = 1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // SPEED => +2.0
	 EXPECT_FLOAT_EQ(result, 2.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_05: DISTANCE 모드, 우선순위 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_05)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelSpeedX     = 1.0f;
	 accelDistanceX  = -2.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // DISTANCE => -2.0
	 EXPECT_FLOAT_EQ(result, -2.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_06: 동일 입력 값 => 동일 결과 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_06)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 2.5f;
	 accelDistanceX  = 2.5f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // 둘 다 +2.5, SPEED => +2.5
	 EXPECT_FLOAT_EQ(result, 2.5f);
 }
 
 /*=== TC_ACC_OUT_EQ_07: 잘못된 모드 입력 => 기본값(0.0) 반환 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_07)
 {
	 // 정의되지 않은 모드 99 => else 구문 => 0.0
	 accMode         = (ACC_Mode_e)99;
	 accelSpeedX     = 2.0f;
	 accelDistanceX  = 1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_08: 모두 0.0 입력 => 0.0 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_08)
 {
	 accMode         = ACC_MODE_SPEED; // 무관
	 accelSpeedX     = 0.0f;
	 accelDistanceX  = 0.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_09: 음의 가속도 입력 시 → 정상 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_09)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = -2.0f;  // 감속
	 accelDistanceX  = -1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // SPEED => -2.0
	 EXPECT_FLOAT_EQ(result, -2.0f);
 }
 
 /*=== TC_ACC_OUT_EQ_10: 양의 가속도 입력 시 → 정상 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_EQ_10)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelSpeedX     = 3.0f;
	 accelDistanceX  = 4.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // DISTANCE => +4.0
	 EXPECT_FLOAT_EQ(result, 4.0f);
 }
 
 /*------------------------------------------------------------------------------
  * 2) BV(경계값 분석) 테스트 10개
  *    - TC_ACC_OUT_BV_01 ~ TC_ACC_OUT_BV_10
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_OUT_BV_01: Accel_Speed_X=+10.0 => 최대 가속 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_01)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 10.0f;
	 accelDistanceX  = 5.0f; // 무시
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 10.0f);
 }
 
 /*=== TC_ACC_OUT_BV_02: Accel_Speed_X=-10.0 => 최대 감속 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_02)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = -10.0f;
	 accelDistanceX  = -5.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, -10.0f);
 }
 
 /*=== TC_ACC_OUT_BV_03: Accel_Speed_X=0.0 => 0 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_03)
 {
	 accMode        = ACC_MODE_SPEED;
	 accelSpeedX    = 0.0f;
	 accelDistanceX = 0.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_BV_04: Accel_Distance_X=+10.0 => 최대 가속 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_04)
 {
	 accMode        = ACC_MODE_DISTANCE;
	 accelDistanceX = 10.0f;
	 accelSpeedX    = 5.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 10.0f);
 }
 
 /*=== TC_ACC_OUT_BV_05: Accel_Distance_X=-10.0 => 최대 감속 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_05)
 {
	 accMode        = ACC_MODE_DISTANCE;
	 accelDistanceX = -10.0f;
	 accelSpeedX    = -5.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, -10.0f);
 }
 
 /*=== TC_ACC_OUT_BV_06: Accel_Distance_X=0.0 => 0 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_06)
 {
	 accMode        = ACC_MODE_DISTANCE;
	 accelDistanceX = 0.0f;
	 accelSpeedX    = 5.0f; // 무시
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_BV_07: SPEED 모드, 미세 양의 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_07)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 0.01f;
	 accelDistanceX  = 0.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_NEAR(result, 0.01f, 1e-5f);
 }
 
 /*=== TC_ACC_OUT_BV_08: DISTANCE 모드, 미세 음의 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_08)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelDistanceX  = -0.01f;
	 accelSpeedX     = 0.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_NEAR(result, -0.01f, 1e-5f);
 }
 
 /*=== TC_ACC_OUT_BV_09: STOP 모드 → 무조건 0 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_09)
 {
	 accMode         = ACC_MODE_STOP;
	 accelDistanceX  = 5.0f;
	 accelSpeedX     = -5.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_BV_10: 이상 입력 → 안전 출력 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_BV_10)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelDistanceX  = -3.4e38f; // 매우 작은 값
	 accelSpeedX     =  3.4e38f; // 매우 큰 값
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 // 구현에 따라 0.0 or clamp
	 // 여기서는 그냥 finite 여부 체크 (가정)
	 EXPECT_TRUE(std::isfinite(result));
 }
 
 /*------------------------------------------------------------------------------
  * 3) RA(요구사항 분석) 테스트 10개
  *    - TC_ACC_OUT_RA_01 ~ TC_ACC_OUT_RA_10
  *------------------------------------------------------------------------------*/
 
 /*=== TC_ACC_OUT_RA_01: SPEED 모드 → Accel_Speed_X 선택 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_01)
 {
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 2.5f;
	 accelDistanceX  = 1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 2.5f);
 }
 
 /*=== TC_ACC_OUT_RA_02: DISTANCE 모드 → Accel_Distance_X 선택 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_02)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelSpeedX     = 3.0f;
	 accelDistanceX  = -1.5f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, -1.5f);
 }
 
 /*=== TC_ACC_OUT_RA_03: STOP 모드 → 0.0 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_03)
 {
	 accMode         = ACC_MODE_STOP;
	 accelSpeedX     = -9.0f;
	 accelDistanceX  = 9.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
 /*=== TC_ACC_OUT_RA_04: AEB 판단 후 단순 출력 선택 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_04)
 {
	 // 단순히 모드와 두 값 중 하나를 선택한다는 시나리오
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 2.5f;
	 accelDistanceX  = 1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 2.5f);
 }
 
 /*=== TC_ACC_OUT_RA_05: 출력 값의 전달 정확성 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_05)
 {
	 accMode         = ACC_MODE_DISTANCE;
	 accelDistanceX  = -1.0f;
	 accelSpeedX     = 99.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, -1.0f);
 }
 
 /*=== TC_ACC_OUT_RA_06: 내부 상태 불참여 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_06)
 {
	 // acc_output_selection()이 별도 내부 상태(정적 변수 등)에 의존 않는지
	 accMode        = ACC_MODE_SPEED;
	 accelSpeedX    = 2.0f;
	 float result = acc_output_selection(accMode, 1.0f, 2.0f);
	 EXPECT_FLOAT_EQ(result, 2.0f);
 }
 
 /*=== TC_ACC_OUT_RA_07: 경계 모드 전환 시 출력 안정성 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_07)
 {
	 // 예: 이전 모드= SPEED => 현 모드= DISTANCE
	 // call 1
	 accMode        = ACC_MODE_SPEED;
	 float r1 = acc_output_selection(accMode, 1.0f, 2.0f);
	 EXPECT_FLOAT_EQ(r1, 2.0f);
 
	 // 모드 전환
	 accMode        = ACC_MODE_DISTANCE;
	 float r2 = acc_output_selection(accMode, 1.0f, 2.0f);
	 // DISTANCE => 1.0
	 EXPECT_FLOAT_EQ(r2, 1.0f);
 }
 
 /*=== TC_ACC_OUT_RA_08: 모드 전환 직후 정확 동작 확인 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_08)
 {
	 // 예: DISTANCE => STOP
	 accMode        = ACC_MODE_DISTANCE;
	 float r1 = acc_output_selection(accMode, -3.0f, +1.0f);
	 EXPECT_FLOAT_EQ(r1, -3.0f);
 
	 accMode        = ACC_MODE_STOP;
	 float r2 = acc_output_selection(accMode, -3.0f, +1.0f);
	 EXPECT_FLOAT_EQ(r2, 0.0f);
 }
 
 /*=== TC_ACC_OUT_RA_09: 임의 float 입력 => 안정 동작 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_09)
 {
	 // SPEED => select Accel_Speed_X
	 accMode         = ACC_MODE_SPEED;
	 accelSpeedX     = 1.2345f;
	 accelDistanceX  = 5.6789f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 1.2345f);
 }
 
 /*=== TC_ACC_OUT_RA_10: else 구문 => 0.0 반환 ===*/
 TEST_F(AccOutputSelectionTest, TC_ACC_OUT_RA_10)
 {
	 // invalid mode => 0.0
	 accMode        = (ACC_Mode_e)999;
	 accelSpeedX    = 2.0f;
	 accelDistanceX = 1.0f;
 
	 float result = acc_output_selection(accMode, accelDistanceX, accelSpeedX);
	 EXPECT_FLOAT_EQ(result, 0.0f);
 }
 
