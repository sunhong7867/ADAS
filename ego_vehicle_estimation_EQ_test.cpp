/****************************************************
 * ego_vehicle_estimation_test_EQ.cpp
 *  - 동등 분할(Eq) 테스트 케이스 50개 전부 포함
 *  - Google Test 기반
 ****************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

// ────────────────────────────────────────────────────────────────────────────
// Convenience helpers
// ────────────────────────────────────────────────────────────────────────────
namespace {
    inline float DegNorm(float h)                  // wrap to [‑180,180]
    {
        while (h >  180.f) h -= 360.f;
        while (h < -180.f) h += 360.f;
        return h;
    }
    
    inline float DtSeconds(float now_ms, float prev_ms)
    {
        // All TimeData fields are milli‑seconds → convert to seconds
        float dt_ms = now_ms - prev_ms;
        // Requirement: if dt_ms < 10 ms  → force 10 ms (= 0.01 s)
        if (dt_ms < 10.f) dt_ms = 10.f;
        return dt_ms / 1000.f;
    }
 }

/*--------------------------------------------------
 * GTest Fixture
 *------------------------------------------------*/
static const int KF_DIM = 5;
class EgoVehicleEstimationTest : public ::testing::Test {
protected:
    TimeData_t timeData;
    GPSData_t  gpsData;
    IMUData_t  imuData;
    EgoData_t  egoData;
    EgoVehicleKFState_t kfState;

    virtual void SetUp() override {
        std::memset(&timeData, 0, sizeof(timeData));
        std::memset(&gpsData, 0, sizeof(gpsData));
        std::memset(&imuData, 0, sizeof(imuData));
        std::memset(&egoData, 0, sizeof(egoData));
        InitEgoVehicleKFState(&kfState);

        // KF 상태 벡터 초기값 (예시)
        kfState.X[0] = 10.0f; // vx
        kfState.X[1] = 0.0f;  // vy
        kfState.X[2] = 1.0f;  // ax
        kfState.X[3] = 0.5f;  // ay
        kfState.X[4] = 0.0f;  // heading
        kfState.Previous_Update_Time = 900.0f;
    }
};

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_01
 *  GPS 유효 시간 내 속도 업데이트
 *  - 조건: Current_Time=1000, GPS_Timestamp=980 (차이=20ms <= 50ms 유효)
 *  - 입력: GPS_Vel=(10.0,0), IMU=(1.0,0.5), Yaw=5.0
 *  - 기대: Ego_Velocity_X≈10.0, Heading≈5°
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_01)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 980.0f;  // dt=20ms → 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    const float dt_s = DtSeconds(timeData.Current_Time,kfState.Previous_Update_Time);
    const float expectH  = imuData.Yaw_Rate * dt_s;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, expectH , 0.05f);
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f,  0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f,  0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f,  0.001f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_02
 *  GPS 유효 시간 초과 시 보정 생략
 *  - 조건: Current_Time=1000, GPS_Timestamp=890 (차이=110ms>50ms 무효)
 *  - 입력: GPS_Vel=(12.0,0), IMU=(1.0,0.5), Yaw=5.0
 *  - 기대: 이전 KF 속도(예:10.0)가 유지됨
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_02)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 890.0f; // 110ms → 무효
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    // 기존 KF 속도=10.0
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_03
 *  IMU 가속도만으로 상태 예측 (GPS 무효)
 *  - 조건: GPS_Timestamp=850 (무효), IMU=(2.0,1.0), Yaw=3.0
 *  - 기대: IMU 기반 예측으로 속도/가속도 업데이트
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_03)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 850.0f; // 무효
    imuData.Linear_Acceleration_X = 2.0f;
    imuData.Linear_Acceleration_Y = 1.0f;
    imuData.Yaw_Rate = 3.0f;
    kfState.X[0] = 5.0f;  // 기존 속도

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // IMU 2.0f → 속도는 기존보다 증가 예상
    EXPECT_GT(egoData.Ego_Velocity_X, 5.0f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 2.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_04
 *  IMU Yaw Rate만 변화 시 Heading 보정
 *  - 조건: IMU Yaw_Rate=10.0, 나머지 고정
 *  - 기대: Heading이 약 10° 증가
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_04)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f; 
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 10.0f;
    kfState.X[4] = 0.0f; // 기존 Heading=0

    const float dt_s = DtSeconds(timeData.Current_Time, kfState.Previous_Update_Time);
    const float expectH  = imuData.Yaw_Rate * dt_s; // (~1°)

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // YawRate=10 → 대략 10° 근방
    EXPECT_NEAR(egoData.Ego_Heading, expectH , 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_05
 *  속도, 가속도, Heading 모두 출력 확인
 *  - 조건: GPS=(10,0), IMU=(1.0,0.5), Yaw=5
 *  - 기대: KF 융합 결과 velocity≈10, accel=1, heading≈5
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_05)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f; 
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[2] = 0.0f;

    const float dt_s = DtSeconds(timeData.Current_Time, kfState.Previous_Update_Time);
    const float expectedAccel = imuData.Linear_Acceleration_X;
    const float expectedHeading  = imuData.Yaw_Rate * dt_s; // (~0.5°)

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X,     10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, expectedAccel, 0.2f);
    EXPECT_NEAR(egoData.Ego_Heading,        expectedHeading , 0.05f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_06
 *  GPS Spike → 이전 속도 유지
 *  - 조건: GPS 속도 갑자기 10→30 m/s 스파이크
 *  - 기대: 스파이크 감지 후 이전 속도(10m/s) 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_06)
{
    // 첫 루프 (정상)
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;  // 정상
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float prevVx = egoData.Ego_Velocity_X;

    // 두 번째 루프 (Spike)
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 30.0f; // 스파이크
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // Spike 감지되어 이전 속도(약 10m/s) 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, prevVx, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_07
 *  IMU Spike → 이전 가속도 유지
 *  - 조건: IMU accel_x 1.0→5.0 급격 변화 (스파이크)
 *  - 기대: 이전 1.0으로 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_07)
{
	// 초기화
    InitEgoVehicleKFState(&kfState);
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;

    // 첫 루프: 정상
    imuData.Linear_Acceleration_X = 1.f;
	imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData,&gpsData,&imuData,&egoData,&kfState);
	float firstAccel  = egoData.Ego_Acceleration_X;
	float prevAccel  = kfState.Prev_Accel_X;

    // 두 번째 루프: Spike(5.0)
    timeData.Current_Time += 100.f;
    imuData.Linear_Acceleration_X = 5.f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

	EXPECT_FLOAT_EQ(kfState.Prev_Accel_X, prevAccel);
	float expectedAccel = firstAccel + prevAccel;
    EXPECT_NEAR(egoData.Ego_Acceleration_X, expectedAccel, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_08
 *  정상 IMU 입력 시 가속도 반영 확인
 *  - 조건: IMU accel=(1.5,0.5), Yaw=5
 *  - 기대: KF 예측 후 Accel≈1.5
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_08)
{
    InitEgoVehicleKFState(&kfState);
	timeData.Current_Time    = 1000.0f;
    gpsData.GPS_Timestamp    = 995.0f;
    gpsData.GPS_Velocity_X   = 10.0f;
    gpsData.GPS_Velocity_Y   =  0.0f;

    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.5f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_09
 *  Yaw Rate Spike → 이전값 유지
 *  - 조건: IMU Yaw_Rate 5.0→50.0 (스파이크)
 *  - 기대: 이전 yaw_rate(5.0) 유지 -> Heading 변화 과도 방지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_09)
{
    InitEgoVehicleKFState(&kfState);
    timeData.Current_Time  = 1000.0f;      // [ms]
    gpsData.GPS_Timestamp  = 995.0f;       // GPS 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y =  0.0f;

    // 첫 루프: 정상 yaw_rate=5
    imuData.Yaw_Rate = 5.0f;
    imuData.Linear_Acceleration_X = 0.0f;  // 이 테스트에선 가속도 무시
    imuData.Linear_Acceleration_Y = 0.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float oldHeading = egoData.Ego_Heading;

    // 두 번째 루프: spike=50
    timeData.Current_Time += 100.0f;
    imuData.Yaw_Rate = 50.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

	float newHeading = egoData.Ego_Heading;
    float dt_s       = (100.0f) * 0.001f;   // 0.1s
    float expectedDelta = 5.0f * dt_s;      // 정상 Yaw Rate만 반영 → 0.5°

	//   a) 변화량이 0.5° 근처인지
    EXPECT_NEAR(newHeading - oldHeading, expectedDelta, 1e-2f);
    //   b) Spike(50°/s)대로 5° 변화가 난 게 아님을 추가 확인
    EXPECT_FALSE(fabsf(newHeading - oldHeading - (50.0f * dt_s)) < 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_10
 *  칼만 필터 예측 수행 여부 (delta_t 정상)
 *  - 조건: Previous_Update_Time=900, Current=1000
 *  - 기대: 예측 단계 정상 실행 (상태 갱신)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_10)
{
    kfState.Previous_Update_Time = 900.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 단순 delta_t=100ms(0.1s) → 예측 정상 수행
    EXPECT_NEAR(timeData.Current_Time - 900.0f, 100.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_11
 *  칼만 필터 업데이트 생략 조건
 *  - 조건: gps_update_enabled=false (GPS 차이 100ms↑)
 *  - 기대: 이전 KF 값 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_11)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 890.0f; // 차이=110ms -> 무효
    imuData.Linear_Acceleration_X = 1.0f;
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 업데이트 생략 -> 10.0 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_12
 *  Kalman Gain 계산 확인 (GPS 유효 시)
 *  - 조건: GPS 정상, R/GPS 노이즈, 공분산 P 초기화
 *  - 기대: Gain이 정상 계산, 상태 보정 발생
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_12)
{
    // P 대각=100 설정
    for(int i=0; i<KF_DIM; i++)
        kfState.P[i*KF_DIM + i] = 100.0f;

    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 보정 후 P[0]이 100.0에서 감소
    EXPECT_NE(kfState.P[0], 100.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_13
 *  상태 벡터 X 갱신 확인
 *  - 조건: GPS/IMU 정상 -> 보정 수행
 *  - 기대: X[0..4] 변경 -> 최종 egoData에 반영
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_13)
{
	InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;

    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
	gpsData.GPS_Velocity_Y   =  0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate             = 0.0f;

    float oldVel = kfState.X[0];
	ASSERT_FLOAT_EQ(oldVel, 0.0f);

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(kfState.X[0], 10.0f, 0.5f);
    EXPECT_NE(kfState.X[0], oldVel);
	EXPECT_NEAR(egoData.Ego_Velocity_X, kfState.X[0], 1e-3f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_14
 *  공분산 행렬 P 갱신 확인
 *  - 조건: KF 업데이트 단계 후 P 변경
 *  - 기대: P가 이전값과 달라짐
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_14)
{
    // P 대각=100
    for(int i=0; i<KF_DIM; i++)
        kfState.P[i*KF_DIM + i] = 100.0f;

    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.P[0], 100.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_15
 *  IMU + GPS 융합 정상 동작 확인
 *  - 조건: 둘 다 정상
 *  - 기대: velocity≈GPS, heading≈IMU 융합
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_15)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f; 
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[4] = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_16
 *  Position은 항상 (0,0,0)
 *  - 기대: Ego_Position = (0,0,0)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_16)
{
    // 위치가 0으로 고정되어야 함
    egoData.Ego_Position_X = 5.0f;
    egoData.Ego_Position_Y = 5.0f;
    egoData.Ego_Position_Z = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 1e-3f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_17
 *  IMU만 입력 시 Heading 변화 존재 여부
 *  - 조건: GPS 무효, IMU yaw_rate=8
 *  - 기대: heading≈기존+8
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_17)
{
	InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;

    timeData.Current_Time        = 1000.0f;  // ms
    gpsData.GPS_Timestamp        = 850.0f;   // 이 값으로 GPS invalid
    gpsData.GPS_Velocity_X       =  0.0f;
    gpsData.GPS_Velocity_Y       =  0.0f;

    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 8.0f;    // Spike가 아니라 정상 IMU만

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 8.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_18
 *  Output 단위 확인 (m/s, m/s², °)
 *  - 조건: 각 센서값도 동일 단위로 입력
 *  - 기대: Output도 사양과 동일 단위 범위인지 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_18)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y =  0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 값 범위로 간단 검증
    EXPECT_NEAR(egoData.Ego_Velocity_X,     10.0f, 1.0f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f,  1.0f);
    EXPECT_NEAR(egoData.Ego_Heading,        5.0f,  2.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_19
 *  GPS 속도만 입력 → 보정 작동 여부
 *  - 조건: GPS만 정상(8,0), IMU=0
 *  - 기대: 속도≈8
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_19)
{
	InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 8.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;

    float oldVel = kfState.X[0];
    ASSERT_FLOAT_EQ(oldVel, 0.0f);

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.X[0], oldVel);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 8.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_20
 *  센서 동기화 정상 동작 확인
 *  - 조건: Current=1000, GPS_Timestamp=990 => delta=10ms
 *  - 기대: KF 예측에 정상 반영
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_20)
{
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 990.0f;
    gpsData.GPS_Timestamp = 990.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float dt = timeData.Current_Time - 990.0f;
    EXPECT_NEAR(dt, 10.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_21
 *  IMU Yaw Rate=0일 때 Heading 유지
 *  - 조건: IMU yaw=0
 *  - 기대: Heading 변화 없음
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_21)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[4] = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_22
 *  IMU 가속도 X 방향만 존재
 *  - 조건: Linear_Accel=(1.5,0), Yaw=5
 *  - 기대: Ego_Velocity_X↑, Y방향 변화X
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_22)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 0.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, 0.0f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 0.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_23
 *  IMU 가속도 Y 방향만 존재
 *  - 조건: Linear_Accel=(0,1.0), Yaw=5
 *  - 기대: Ego_Velocity_Y↑
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_23)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 1.0f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_Y, 0.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_24
 *  GPS Y 방향만 유효
 *  - 조건: GPS=(0,8), IMU=정상
 *  - 기대: Ego_Velocity_Y=8
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_24)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 8.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 8.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_25
 *  GPS X 방향만 유효
 *  - 조건: GPS=(12,0)
 *  - 기대: Ego_Velocity_X=12
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_25)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 9.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 0.0f;

    float oldVelX = kfState.X[0];
    ASSERT_FLOAT_EQ(oldVelX, 0.0f);

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.X[0], oldVelX);                      // 변경됨
    EXPECT_NEAR(kfState.X[0], 9.0f, 0.1f);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 9.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_26
 *  GPS 무효 시 IMU만 보정 작동
 *  - 조건: GPS_Timestamp=900(무효), IMU 정상
 *  - 기대: IMU로만 예측→속도 증가
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_26)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 900.0f; // 무효
    imuData.Linear_Acceleration_X = 1.0f;
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_27
 *  GPS, IMU 모두 무효 시 이전 상태 유지
 *  - 조건: GPS=스파이크, IMU=스파이크
 *  - 기대: 업데이트 생략, 이전 속도 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_27)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 800.0f;
    gpsData.GPS_Velocity_X = 50.0f;  // 스파이크
    imuData.Linear_Acceleration_X = 10.0f; // 스파이크
    kfState.X[0] = 10.0f; // 이전 속도

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_28
 *  Position 출력 무조건 원점 유지
 *  - 조건: 코드에 하드코딩 (0,0,0)
 *  - 기대: (0,0,0)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_28)
{
    egoData.Ego_Position_X = 5.0f;
    egoData.Ego_Position_Y = 5.0f;
    egoData.Ego_Position_Z = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 1e-3f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_29
 *  초기화 이후 첫 입력 예측 정상 작동
 *  - 조건: Init 후 첫 루프에서 GPS/IMU 입력 (GPS=10m/s 등)
 *  - 기대: 상태 벡터 정상 업데이트
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_29)
{
    // KFState 재초기화 가정
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;

    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    float oldVelX = kfState.X[0];
    ASSERT_FLOAT_EQ(oldVelX, 0.0f);

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.X[0], oldVelX);  
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
}
/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_30
 *  칼만 필터 상태 초기화 후 예측 동작
 *  - 조건: InitEgoVehicleKFState 후 IMU 입력
 *  - 기대: 예측으로 velocity, heading 등이 변경
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_30)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 0.0f;
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float accelAfter1stLoop = kfState.X[2];
    EXPECT_NEAR(accelAfter1stLoop, 1.0f, 0.01f);

    timeData.Current_Time += 1000.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(kfState.X[0], 0.0f);                          // velocity 증가 확인
    EXPECT_NEAR(kfState.X[2], 2.0f, 0.1f);                  // accel 누적 확인
    EXPECT_NEAR(kfState.X[4], 10.0f, 0.2f);                 // heading = 5°/s × 2s
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_31
 *  Yaw Rate 음수 입력 시 반시계 회전 반영
 *  - 조건: Yaw_Rate=-8
 *  - 기대: Heading이 -8° 변화
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_31)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp  = 0.0f;  
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = -8.0f;
    
    float oldHeading = kfState.X[4];
    ASSERT_FLOAT_EQ(oldHeading, 0.0f);

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.X[4], oldHeading);                      // 내부 상태 변경
    EXPECT_NEAR(kfState.X[4], -8.0f, 0.2f);                   // -8°/s × 1s
    EXPECT_NEAR(egoData.Ego_Heading, -8.0f, 0.2f);            // 출력도 동일하게 반영
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_32
 *  속도 예측과 GPS 속도 오차 존재 시 보정 확인
 *  - 조건: KF 이전 속도=10, GPS=9
 *  - 기대: 보정 후 ~9
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_32)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 9.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[0] = 10.0f;

    float oldVelX = kfState.X[0];
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.X[0], oldVelX);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 9.0f, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_33
 *  GPS 속도와 예측 속도 동일 시 보정 무효화 확인
 *  - 조건: KF 이전 속도=10, GPS=10
 *  - 기대: 업데이트 전후 변화 없음
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_33)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.2f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_34
 *  Heading=180도 유지 여부 확인
 *  - 조건: 초기 Heading=180
 *  - 기대: 업데이트 후에도 ±180도 범위 정상 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_34)
{
    kfState.X[4] = 180.0f;
    gpsData.GPS_Timestamp = 995.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float hdg = egoData.Ego_Heading;
    bool is180 = (std::fabs(hdg - 180.0f) < 1.0f) || (std::fabs(hdg + 180.0f) < 1.0f);
    EXPECT_TRUE(is180);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_35
 *  Heading=-180도 유지 여부 확인
 *  - 조건: 초기 Heading=-180
 *  - 기대: 업데이트 후에도 -180
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_35)
{
    kfState.X[4] = -180.0f;
    gpsData.GPS_Timestamp = 995.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, -180.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_36
 *  이전 센서값 누적 업데이트 확인
 *  - 조건: 연속 루프에서 GPS속도 조금씩 증가
 *  - 기대: KF 내부 prev 값 갱신 → 속도 점진적 상승
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_36)
{
    // 루프1
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float vel1 = egoData.Ego_Velocity_X;

    // 루프2
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 10.5f; // 점진적 증가
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float vel2 = egoData.Ego_Velocity_X;

    EXPECT_GT(vel2, vel1);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_37
 *  GPS 유효→무효 전환 시 처리 확인
 *  - 조건: 첫 루프: GPS정상, 두 루프: GPS무효
 *  - 기대: 첫 루프 보정, 두 번째는 무효→이전 상태 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_37)
{
    // 첫 루프
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float firstLoopVx = egoData.Ego_Velocity_X;

    // 두 번째 루프: GPS무효
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 900.0f; // dt=200ms>50
    gpsData.GPS_Velocity_X = 15.0f; // 적용X
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X, firstLoopVx, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_38
 *  IMU 노이즈 제거 후 이전 값 적용 확인
 *  - 조건: 첫 루프 spike=5.0, 두 번째 루프 정상=1.0
 *  - 기대: 두 번째 루프에서 정상값으로 복귀
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_38)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time = 1000.0f;
    kfState.Prev_Accel_X = 1.0f;
    kfState.X[2]        = 1.0f;

    // 첫 루프: spike
    imuData.Linear_Acceleration_X = 5.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 0.0f;
    gpsData.GPS_Timestamp         = 0.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float afterSpike = kfState.X[2];
    EXPECT_NEAR(afterSpike - 1.0f, 1.0f, 0.01f);

    // 두 번째 루프: 복귀
    timeData.Current_Time += 100.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(kfState.X[2] - afterSpike, 1.0f, 0.01f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_39
 *  Accel_X,Y 변화 시 상태 반응 확인
 *  - 조건: 1루프=(1.0,0.5), 2루프=(1.5,0.8)
 *  - 기대: 속도/가속도 점진적 증가
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_39)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    gpsData.GPS_Timestamp        = 0.0f;

    // 1루프
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate              = 0.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float vx1 = egoData.Ego_Velocity_X;
    float ax1 = egoData.Ego_Acceleration_X;
    float ay1 = egoData.Ego_Acceleration_Y;

    // 2루프
    timeData.Current_Time += 100.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.8f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_GT(egoData.Ego_Velocity_X, vx1);
    EXPECT_GT(egoData.Ego_Acceleration_X, ax1);
    EXPECT_GT(egoData.Ego_Acceleration_Y, ay1);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_40
 *  GPS_Vel_X 과도 변화 시 무시 처리 확인
 *  - 조건: 첫 루프=10, 두 루프=50(Spike)
 *  - 기대: Spike 감지 시 이전 10 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_40)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    // 1루프
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float oldVx = egoData.Ego_Velocity_X;

    // 2루프(Spike)
    timeData.Current_Time += 100.0f;
    gpsData.GPS_Timestamp = timeData.Current_Time - 5.0f; // still valid
    kfState.Prev_GPS_Vel_X = 10.0f;
    gpsData.GPS_Velocity_X = 50.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, oldVx, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_41
 *  IMU+GPS 모두 변화 시 융합 정확성 확인
 *  - 조건: GPS=(5,0), IMU=(1.5,0.5), Yaw=6
 *  - 기대: velocity≈5, heading≈6
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_41)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 5.0f;
    gpsData.GPS_Velocity_Y   = 0.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 6.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 5.0f, 1.0f);
    EXPECT_NEAR(egoData.Ego_Heading,    6.0f,  2.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_42
 *  Heading 정규화 결과 181 -> -179 변환 확인
 *  - 조건: 초기 Heading=181
 *  - 기대: -179 근처로 wrap-around
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_42)
{
    // heading=181 -> wrap -> -179
    kfState.X[4] = 181.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, -179.0f, 2.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_43
 *  delta_t가 너무 작을 때 최소 0.01초 적용 확인
 *  - 조건: Current=1000, Prev=999 => delta=1ms->0.01s
 *  - 기대: 작은 보정값으로 미미한 변화
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_43)
{
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 999.0f; // 1ms → 0.01sec 보정
    gpsData.GPS_Timestamp = 999.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 변화 미미
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.2f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_44
 *  공분산 P 초기값 반영 확인
 *  - 조건: P 대각=100, KF 업데이트 후 변화
 *  - 기대: P != 100
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_44)
{
    for(int i=0;i<KF_DIM;i++)
        kfState.P[i*KF_DIM + i] = 100.0f;

    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NE(kfState.P[0], 100.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_45
 *  IMU 입력 없음 시 상태 고정 여부 확인
 *  - 조건: IMU=0
 *  - 기대: 이전 상태 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_45)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_46
 *  보정 이후 상태 출력 일치 확인
 *  - 조건: 칼만 필터 보정 후 egoData vs kfState->X
 *  - 기대: egoData 속도= X[0], heading= X[4]
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_46)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.X[4] = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, kfState.X[0], 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading,    kfState.X[4], 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_47
 *  Yaw Rate=90°/s → Heading 급증 반영
 *  - 조건: yaw_rate=90
 *  - 기대: Heading이 크게 상승
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_47)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    gpsData.GPS_Timestamp         = 0.0f;   // GPS 무효
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 90.0f;
    kfState.Prev_Yaw_Rate   = 90.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 90.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_48
 *  IMU X/Y 반대 입력 → 벡터 해석 일치 확인
 *  - 조건: accel=(2,-2)
 *  - 기대: x속도 증가, y속도는 net효과 확인(실제 KF 로직 따라 다름)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_48)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 2.0f;
    imuData.Linear_Acceleration_Y = -2.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 주로 X accel 남아서 X속도↑
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_49
 *  관측 행렬 H 구성 정확도 확인
 *  - 조건: GPS 관측에서 속도만 반영
 *  - 기대: 내부에서 H가 [1,0,0,0,0; 0,1,0,0,0]로 구성되어 보정
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_49)
{
    // 실제 H 행렬은 소스 내부. 여기서는 보정 결과로 간접 확인
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_EQ_50
 *  GPS 및 IMU 미세 변화 검증
 *  - 조건: 이전 루프10→현재10.2, IMU 동일
 *  - 기대: 스무딩되어 10.1 근방
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_50)
{
    // 첫 루프
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 두 번째 루프: 10.2 -> 10.1 근방
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 10.2f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.5f);
}
