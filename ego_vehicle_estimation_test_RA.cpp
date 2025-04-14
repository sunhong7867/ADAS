/****************************************************
 * ego_vehicle_estimation_test_RA.cpp
 *  - 동등 분할(Eq) 테스트 케이스 50개 전부 포함
 *  - Google Test 기반
 ****************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

/* 칼만 필터 상태벡터 크기 */
static const int KF_DIM = 5;

/*--------------------------------------------------
 * GTest Fixture
 *------------------------------------------------*/
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

        // KF 상태 초기화
        InitEgoVehicleKFState(&kfState);

        // 예시 초기 상태 벡터 (테스트 편의를 위해 설정)
        kfState.X[0] = 10.0f; // vx
        kfState.X[1] = 0.0f;  // vy
        kfState.X[2] = 1.0f;  // ax
        kfState.X[3] = 0.5f;  // ay
        kfState.X[4] = 0.0f;  // heading

        // 이전 업데이트 시간
        kfState.Previous_Update_Time = 900.0f;
    }
};

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_01 (RA_01)
 *  - Ego_Position (X,Y,Z) 항상 0.0 고정 출력 & GPS 유효 시간 내 속도 보정 확인
 *  - 조건: Current_Time=1000, GPS_Timestamp=980 (차=20ms<=50ms)
 *  - 입력: GPS_Vel=(10.0,0.0), IMU=(1.0,0.5), Yaw=5.0
 *  - 기대: egoData.Ego_Velocity_X가 대략 10 근처로 보정, Ego_Position_X/Y/Z=0.0 고정
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_01)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 980.0f;  // dt=20ms → 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 위치는 항상 0으로 고정 출력
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 1e-5);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 1e-5);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 1e-5);

    // 속도 보정 기대(10 근처)
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    // Heading은 Yaw_Rate 반영 예측 + GPS 보정(대략 5도 근방 예상)
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_02 (RA_02)
 *  - GPS 유효 시간 초과 시 보정 생략 확인
 *  - 조건: Current_Time=1000, GPS_Timestamp=890(차=110ms>50ms)
 *  - 입력: GPS_Vel=(12.0,0), IMU=(1.0,0.5), Yaw=5.0
 *  - 기대: 이전 KF 속도(기본 10.0) 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_02)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 890.0f; // 무효
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    // 기존 KF 속도 = 10.0
    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // GPS 업데이트 생략 → 이전 속도 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_03 (RA_03)
 *  - IMU 가속도만으로 상태 예측 (GPS 무효)
 *  - 조건: GPS_Timestamp=850 (무효), IMU=(2.0,1.0), Yaw=3.0
 *  - 기대: IMU만으로 속도/가속도 상승
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_03)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 850.0f; // 무효
    imuData.Linear_Acceleration_X = 2.0f;
    imuData.Linear_Acceleration_Y = 1.0f;
    imuData.Yaw_Rate = 3.0f;
    kfState.X[0] = 5.0f;  // 기존 속도

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // IMU 2.0f → 속도 증가 예상
    EXPECT_GT(egoData.Ego_Velocity_X, 5.0f);
    // 가속도는 스파이크 등 없으므로 그대로 반영
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 2.0f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_04 (RA_04)
 *  - Current_Time과 GPS_Timestamp 차=40ms → gps_update_enabled=true 확인
 *  - 조건: Current_Time=1000, GPS_Timestamp=960 → dt=40ms
 *  - 입력: GPS_Vel=(10.0,0.0), IMU=(0.5,0.2), Yaw=2.0
 *  - 기대: gps_update_enabled=true → 상태 보정
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_04)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 960.0f; // 40ms → 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.5f;
    imuData.Linear_Acceleration_Y = 0.2f;
    imuData.Yaw_Rate = 2.0f;

    // 호출 전 상태
    float prevVelX = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // GPS 보정이 이루어졌으므로 속도는 10 근처로 업데이트 예상
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
    // prevVelX와 다른 값이어야 함
    EXPECT_NE(egoData.Ego_Velocity_X, prevVelX);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_05 (RA_05)
 *  - GPS 유효성 판단 기준 50ms(동일) → gps_update_enabled=true
 *  - 조건: Current_Time=1000, GPS_Timestamp=950(차=50ms)
 *  - 입력: GPS_Vel=(10.0,0), IMU=정상
 *  - 기대: 경계값(50ms)에서도 유효 처리
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_05)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 950.0f; // 차=50ms → 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 유효하므로 보정 반영되어 10 근처
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_06 (RA_06)
 *  - gps_dt=60ms > 50ms → gps_update_enabled=false
 *  - 조건: Current_Time=1000, GPS_Timestamp=940(차=60ms)
 *  - 기대: GPS 업데이트 생략
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_06)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 940.0f; 
    gpsData.GPS_Velocity_X = 10.0f;

    float prevVelX = kfState.X[0]; // 기존 속도

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 업데이트 생략 → 기존 속도 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, prevVelX, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_07 (RA_07)
 *  - gps_dt ≤ 50ms → gps_update_enabled = true
 *  - 조건: Current_Time=1000, GPS_Timestamp=950(50ms 이하)
 *  - 입력: GPS_Vel=(10.0,0.0)
 *  - 기대: GPS 보정 수행
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_07)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 950.0f; 
    gpsData.GPS_Velocity_X = 10.0f;

    float prevVelX = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // GPS 보정 → 속도 10 근처
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
    EXPECT_NE(egoData.Ego_Velocity_X, prevVelX);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_08 (RA_08)
 *  - delta_t 계산 정상 수행
 *  - 조건: Current_Time=1100, Previous_Update_Time=1000 → delta_t=100ms=0.1s
 *  - 기대: 0.1s로 KF 예측 반영
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_08)
{
    timeData.Current_Time = 1100.0f;
    kfState.Previous_Update_Time = 1000.0f; // delta_t=100ms

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 내부적으로 delta_t=0.1s 계산 후 예측에 반영
    // 직접 확인은 어렵지만, 보정 전후 Heading이나 속도 변화를 근사로 확인
    // 여기서는 예제상 어느 정도 변화만 있는지 확인
    EXPECT_FLOAT_RA(kfState.Previous_Update_Time, 1100.0f); // 업데이트 후 저장
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_09 (RA_09)
 *  - delta_t 음수 → 최소값 0.01 적용
 *  - 조건: Current_Time=900, Previous_Update_Time=1000 → 실제 diff=-100 → 0.01
 *  - 기대: 0.01초로 강제 적용
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_09)
{
    timeData.Current_Time = 900.0f;
    kfState.Previous_Update_Time = 1000.0f; 

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 내부적으로 delta_t가 0보다 작으면 0.01로 적용
    // 이후 Previous_Update_Time은 900으로 변경
    EXPECT_FLOAT_RA(kfState.Previous_Update_Time, 900.0f);
    // 속도나 Heading이 소폭 변화했는지 정도로만 체크
    EXPECT_NEAR(egoData.Ego_Heading, 0.0f, 3.0f); 
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_10 (RA_10)
 *  - Accel_X 스파이크 제거 확인
 *  - 조건: Prev_Accel_X=1.0, 현재 IMU=4.01 (diff=3.01>3.0)
 *  - 기대: 스파이크 처리 → 이전값 유지(1.0)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_10)
{
    kfState.Prev_Accel_X = 1.0f;
    imuData.Linear_Acceleration_X = 4.01f; // 스파이크
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 스파이크로 인해 이전값(1.0) 사용
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f, 1e-5);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_11 (RA_11)
 *  - Accel_Y 스파이크 제거 작동 여부
 *  - 조건: Prev_Accel_Y=0.5, 현재=3.51(diff=3.01>3.0)
 *  - 기대: 이전값(0.5) 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_11)
{
    kfState.Prev_Accel_Y = 0.5f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 3.51f;
    imuData.Yaw_Rate = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 0.5f, 1e-5);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_12 (RA_12)
 *  - Yaw_Rate 스파이크 제거
 *  - 조건: Prev_Yaw_Rate=5.0, 현재=31.0 (diff=26>25 => 임계=30이므로 스파이크)
 *  - 기대: 이전값(5.0) 유지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_12)
{
    kfState.Prev_Yaw_Rate = 5.0f;
    imuData.Yaw_Rate = 31.0f; // 스파이크
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 스파이크 → 이전 값(5.0) 유지
    EXPECT_NEAR(egoData.Ego_Heading, 0.5f, 1.0f); 
    // Heading = oldHeading(0) + 5 * (delta_t/1000) 등 소폭 증가
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_13 (RA_13)
 *  - GPS Velocity_X/Y 스파이크 제거
 *  - 조건: Prev_GPS_Vel_X=0, 현재=10.1 (diff=10.1>10)
 *  - 기대: gps_update_enabled=false 처리
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_13)
{
    kfState.Prev_GPS_Vel_X = 0.0f;
    kfState.Prev_GPS_Vel_Y = 0.0f;

    gpsData.GPS_Velocity_X = 10.1f; // 스파이크
    gpsData.GPS_Velocity_Y = 10.1f; // 스파이크
    gpsData.GPS_Timestamp = 1000.0f; // 유효 시간이라도 스파이크면 false

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 스파이크 → gps_update_enabled=false → 속도 보정 안됨
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f); 
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_14 (RA_14)
 *  - 이전 정상값 유지 여부
 *  - 조건: 스파이크 후에도 Prev_* 유지되는지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_14)
{
    // 이전에 스파이크가 발생해 Accel_X=1.0, Yaw_Rate=5.0, GPS_Vel=0.0 등으로 유지된 상태 가정
    kfState.Prev_Accel_X = 1.0f;
    kfState.Prev_Yaw_Rate = 5.0f;
    kfState.Prev_GPS_Vel_X = 0.0f;

    // 이번에 들어온 센서도 스파이크로 가정
    imuData.Linear_Acceleration_X = 10.0f; 
    imuData.Yaw_Rate = 40.0f; 
    gpsData.GPS_Velocity_X = 15.0f;
    gpsData.GPS_Velocity_Y = 20.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 다시 스파이크 처리되면 이전 값들이 계속 유지됨
    EXPECT_NEAR(kfState.Prev_Accel_X, 1.0f, 1e-5);
    EXPECT_NEAR(kfState.Prev_Yaw_Rate, 5.0f, 1e-5);
    EXPECT_NEAR(kfState.Prev_GPS_Vel_X, 0.0f, 1e-5);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_15 (RA_15)
 *  - gps_update_enabled=false → 업데이트 생략
 *  - 조건: GPS 스파이크 or GPS 비정상 값
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_15)
{
    timeData.Current_Time = 1000.0f;
    // 예: 비정상 GPS_Timestamp=940(60ms, 무효) + 속도 스파이크
    gpsData.GPS_Timestamp = 940.0f;
    gpsData.GPS_Velocity_X = 20.0f; // 스파이크 가정
    float oldVel = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 업데이트 생략 → 속도 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, oldVel, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_16 (RA_16)
 *  - 예측 단계: 상태 벡터 X̂ 계산 확인
 *  - 조건: IMU=(1.0,0.5, Yaw=5.0), delta_t=0.1
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_16)
{
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 900.0f; // delta_t=100ms
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    // 초기 속도=10, heading=0
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 예측 단계만으로도 속도/heading 약간 증가
    // Accel=1.0 → vx=10 + 1.0 = 11(0.1초 반영시 약 10.1, 구현부마다 다름)
    // Heading=0 + 5*(0.1)=0.5도
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_NEAR(egoData.Ego_Heading, 0.5f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_17 (RA_17)
 *  - 예측 단계: 입력 벡터 u=[accelX, accelY, yawRate] 구성
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_17)
{
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    // u=[1.0, 0.5, 5.0]
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 보정 전 IMU 스파이크 없으면 그대로 반영
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 0.5f, 0.5f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_18 (RA_18)
 *  - 예측 단계: 상태 전이 행렬 A
 *  - 조건: delta_t=100ms, A 반영 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_18)
{
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 900.0f; // delta_t=0.1
    imuData.Linear_Acceleration_X = 0.0f; // 단순화

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // A가 적용되어 vx=10 + (delta_t*ax)
    // 여기선 ax=1.0 (SetUp), delta_t=0.1 → 10.1 근처
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.2f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_19 (RA_19)
 *  - 예측 단계: 공분산 행렬 P = A·P·A^T + Q
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_19)
{
    // 초기 P 대각=100, Q=0.01
    float oldP00 = kfState.P[0];
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // P가 갱신되었는지 확인 (대각 성분이 100 이상으로 커지거나 변화)
    float newP00 = kfState.P[0];
    EXPECT_GT(newP00, oldP00);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_20 (RA_20)
 *  - 예측 단계: Process Noise Q 적용 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_20)
{
    // Q=0.01이 P_pred에 더해지는지
    float oldP00 = kfState.P[0];
    timeData.Current_Time = 950.0f; 
    kfState.Previous_Update_Time = 900.0f; // delta=50

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float newP00 = kfState.P[0];
    // 이전값 + 무언가가 반영되어 증가
    EXPECT_GT(newP00, oldP00);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_21 (RA_21)
 *  - Heading = 이전 Heading + YawRate * dt
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_21)
{
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 900.0f; // delta=0.1
    kfState.X[4] = 0.0f; // Heading
    imuData.Yaw_Rate = 5.0f; 

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // Heading=0 + 5*(0.1)=0.5
    EXPECT_NEAR(egoData.Ego_Heading, 0.5f, 0.3f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_22 (RA_22)
 *  - 속도 = 이전 속도 + acc * dt
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_22)
{
    kfState.X[0] = 10.0f; 
    timeData.Current_Time = 1100.0f;
    kfState.Previous_Update_Time = 1000.0f; // 0.1s
    imuData.Linear_Acceleration_X = 1.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 속도=10 + 1*0.1=10.1 근처
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.2f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_23 (RA_23)
 *  - delta_t 다양한 값(50ms,100ms 등)에 따라 예측 결과가 달라지는지
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_23)
{
    // 첫번째 루프: delta_t=0.05
    timeData.Current_Time = 950.0f;
    kfState.Previous_Update_Time = 900.0f; 
    imuData.Linear_Acceleration_X = 2.0f; // 임의
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float vAfterFirst = egoData.Ego_Velocity_X;

    // 두번째 루프: delta_t=0.1
    timeData.Current_Time = 1000.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float vAfterSecond = egoData.Ego_Velocity_X;

    // 두번째가 더 많이 증가해야 함(가속도 동일, dt 더 큼)
    EXPECT_GT(vAfterSecond - vAfterFirst, 0.0f);
    EXPECT_GT(vAfterSecond, vAfterFirst);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_24 (RA_24)
 *  - 업데이트 단계: gps_update_enabled=true → KF 보정
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_24)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 970.0f; // 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // gps_update_enabled=true → 속도 10 근처로 보정
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_25 (RA_25)
 *  - 업데이트 단계: Kalman Gain K 계산
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_25)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f; // 유효
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;

    // 업데이트 전 P[0] 기억
    float oldP00 = kfState.P[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 업데이트 후 P[0] 달라졌다면 Kalman Gain이 적용되었다고 간접 확인 가능
    EXPECT_NE(kfState.P[0], oldP00);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_26 (RA_26)
 *  - 업데이트 단계: x = x_pred + K*(z - H*x_pred)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_26)
{
    // GPS 관측: 11.0, 실제 예측은 10.0 근처라 가정
    gpsData.GPS_Timestamp = 995.0f; 
    gpsData.GPS_Velocity_X = 11.0f; 
    kfState.X[0] = 10.0f; // 예측 속도
    float oldX = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 보정 후 X[0]이 10과 11 사이로 조정
    EXPECT_GT(egoData.Ego_Velocity_X, oldX);
    EXPECT_LT(egoData.Ego_Velocity_X, 11.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_27 (RA_27)
 *  - 업데이트 단계: 관측 오차 y = z - H*x_pred
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_27)
{
    // z=12, x_pred=10 → y=2
    gpsData.GPS_Timestamp = 1000.0f;
    gpsData.GPS_Velocity_X = 12.0f;
    kfState.X[0] = 10.0f; 

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    // 보정 후 속도(10 + K*2) → 10보다 커지고 12보다는 작음
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_LT(egoData.Ego_Velocity_X, 12.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_28 (RA_28)
 *  - 업데이트 단계: P = (I - K·H)*P_pred
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_28)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    float oldP00 = kfState.P[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 보정 후 P[0]이 줄어드는 경향(불확실성 감소)
    EXPECT_LT(kfState.P[0], oldP00);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_29 (RA_29)
 *  - 관측 모델 H 정의 확인(속도만 보정)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_29)
{
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 1.0f; // vy도 관측
    // 보정 후 vx, vy 두 값만 반영되는지 확인

    float oldVy = kfState.X[1];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // vy도 보정 반영
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
    EXPECT_NE(egoData.Ego_Velocity_Y, oldVy);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_30 (RA_30)
 *  - 관측 잔차 y 적용 후 상태 변화
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_30)
{
    kfState.X[0] = 10.0f;
    gpsData.GPS_Timestamp = 990.0f;
    gpsData.GPS_Velocity_X = 15.0f; // 관측 잔차=5

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 10 < X < 15 예상
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_LT(egoData.Ego_Velocity_X, 15.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_31 (RA_31)
 *  - GPS 무효 시 업데이트 생략
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_31)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 890.0f; // 무효
    gpsData.GPS_Velocity_X = 10.0f;
    float oldVel = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 보정 생략 → 속도 유지
    EXPECT_NEAR(egoData.Ego_Velocity_X, oldVel, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_32 (RA_32)
 *  - Ego_Velocity_X = X[0] 매핑 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_32)
{
    kfState.X[0] = 8.8f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Velocity_X, 8.8f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_33 (RA_33)
 *  - Ego_Velocity_Y = X[1] 매핑 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_33)
{
    kfState.X[1] = 2.2f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Velocity_Y, 2.2f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_34 (RA_34)
 *  - Ego_Acceleration_X = X[2] 매핑 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_34)
{
    kfState.X[2] = 0.7f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Acceleration_X, 0.7f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_35 (RA_35)
 *  - Ego_Acceleration_Y = X[3] 매핑 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_35)
{
    kfState.X[3] = -1.1f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Acceleration_Y, -1.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_36 (RA_36)
 *  - Ego_Heading = X[4] 매핑 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_36)
{
    kfState.X[4] = 30.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Heading, 30.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_37 (RA_37)
 *  - Invert2x2 함수 정상 작동
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_37)
{
    // KF 내부에서 S=[P_pred[0]+R, P_pred[1]; P_pred[5], P_pred[6]+R] 형태
    // 여기에 강제로 어느정도 값 셋팅
    float Ptest[25];
    memcpy(Ptest, kfState.P, sizeof(Ptest));
    Ptest[0] = 110.0f; 
    Ptest[6] = 110.0f; 
    float S[4] = { Ptest[0]+0.1f, Ptest[1], Ptest[5], Ptest[6]+0.1f };
    float S_inv[4] = {0};

    bool ret = false;
    // 외부에서 함수 직접 테스트할 수도 있지만, 여기서는 내부 동작 가정
    // 직접 호출
    extern bool Invert2x2(const float S[4], float S_inv[4]);
    ret = Invert2x2(S, S_inv);
    EXPECT_TRUE(ret);
    // 역행렬값이 유한한지 검사
    EXPECT_NEAR(S_inv[0], 1.0f/(110.1f), 1e-3);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_38 (RA_38)
 *  - Kalman Gain 적용 후 상태 개선(오차 감소)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_38)
{
    // 예: x_pred=10, z=11(관측), K~0.5 => 업데이트 후 10.5 근처
    kfState.X[0] = 10.0f;
    gpsData.GPS_Timestamp = 990.0f;
    gpsData.GPS_Velocity_X = 11.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_LT(egoData.Ego_Velocity_X, 11.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_39 (RA_39)
 *  - P 행렬 대각 성분에 Q 반영
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_39)
{
    float oldP11 = kfState.P[6]; // P[1,1]
    timeData.Current_Time = 910.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float newP11 = kfState.P[6];
    EXPECT_GT(newP11, oldP11);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_40 (RA_40)
 *  - P 행렬 업데이트 후 이전 P와 차이 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_40)
{
    float oldP[25];
    memcpy(oldP, kfState.P, sizeof(oldP));
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f; 

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 업데이트 전후 P가 달라졌는지 확인
    bool changed = false;
    for(int i=0; i<25; i++){
        if(fabsf(kfState.P[i] - oldP[i]) > 1e-3f){
            changed = true;
            break;
        }
    }
    EXPECT_TRUE(changed);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_41 (RA_41)
 *  - 상태 전파: IMU 가속도+YawRate만으로 추정(GPS 무효)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_41)
{
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 800.0f; // 무효
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    float oldVelX = kfState.X[0];
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // GPS 무효라도 IMU로 예측되어 속도 증가
    EXPECT_GT(egoData.Ego_Velocity_X, oldVelX);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_42 (RA_42)
 *  - P 역행렬 계산 불가 시 업데이트 생략
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_42)
{
    // S 행렬 식이 0이 되도록 의도
    // P_pred[0], P_pred[6]를 0으로 강제
    for(int i=0; i<25; i++) kfState.P[i] = 0.0f;

    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    float oldVel = kfState.X[0];

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 업데이트 실패 → 속도 보정 없음
    EXPECT_NEAR(egoData.Ego_Velocity_X, oldVel, 0.1f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_43 (RA_43)
 *  - 가속도+GPS 오차 모두 큰 경우 보정 실패/출력 불안정
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_43)
{
    // 큰 오차 예시
    imuData.Linear_Acceleration_X = 20.0f;
    imuData.Linear_Acceleration_Y = 20.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 50.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 매우 큰 값으로 인해 속도 과도 증가 가능
    // 테스트에선 과도하게 큰 결과(예: 30 m/s 이상) 나올 수 있음
    EXPECT_GT(egoData.Ego_Velocity_X, 20.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_44 (RA_44)
 *  - 입력 모두 정상 → 출력 정확성 검증
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_44)
{
    // IMU, GPS 모두 정상
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 대략 10 근처 속도, Heading도 증가
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 2.0f);
    // 위치 고정
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 1e-5);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_45 (RA_45)
 *  - 이전 P, X 값 보존 상태 확인 (센서 변화 없는 경우)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_45)
{
    float oldX[5], oldP[25];
    memcpy(oldX, kfState.X, sizeof(oldX));
    memcpy(oldP, kfState.P, sizeof(oldP));

    // 센서변화 없음
    gpsData.GPS_Timestamp = 890.0f; // 무효
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 예측도 거의 변화 없고 GPS도 무효 → 상태 거의 동일
    bool sameX = true;
    for(int i=0; i<5; i++){
        if(fabsf(kfState.X[i] - oldX[i])>1e-3f){ 
            sameX = false; 
            break; 
        }
    }
    EXPECT_TRUE(sameX);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_46 (RA_46)
 *  - 내부 상태 초기화 후 출력 0 초기화 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_46)
{
    // 다시 InitEgoVehicleKFState
    InitEgoVehicleKFState(&kfState);

    // KFState가 모두 0(또는 설정값)으로 초기화
    // 실제 EgoVehicleEstimation 한번 호출
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X, 0.0f, 1e-5);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 0.0f, 1e-5);
    EXPECT_NEAR(egoData.Ego_Heading, 0.0f, 1e-5);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_47 (RA_47)
 *  - Output 구조체(EgoData) 일관성 확인
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_47)
{
    // X = [10,1,2,1,45]
    kfState.X[0] = 10.0f;
    kfState.X[1] = 1.0f;
    kfState.X[2] = 2.0f;
    kfState.X[3] = 1.0f;
    kfState.X[4] = 45.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_FLOAT_RA(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_FLOAT_RA(egoData.Ego_Velocity_Y, 1.0f);
    EXPECT_FLOAT_RA(egoData.Ego_Acceleration_X, 2.0f);
    EXPECT_FLOAT_RA(egoData.Ego_Acceleration_Y, 1.0f);
    EXPECT_FLOAT_RA(egoData.Ego_Heading, 45.0f);
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_48 (RA_48)
 *  - Target Selection 등 타 모듈 연계: EgoData 출력 포맷 검사
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_48)
{
    // 이 테스트는 구조체 필드가 맞게 존재하는지 확인
    // 실제론 인터페이스 검사. 여기서는 필드 존재와 값 매핑 정도로 확인
    kfState.X[0] = 8.0f; // VelocityX
    kfState.X[4] = 10.0f; // Heading

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_FLOAT_RA(egoData.Ego_Velocity_X, 8.0f);
    EXPECT_FLOAT_RA(egoData.Ego_Heading, 10.0f);
    // 구조체에 Ego_Position, Accel 등 있음 → 문제 없이 참조
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_49 (RA_49)
 *  - 제어 루프 100Hz 기준(10ms 간격) 연속 실행 시 일관성
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_49)
{
    // 예: 5번 루프 돌리며 10ms씩 증가
    float startTime = 1000.0f;
    for(int i=0; i<5; i++){
        timeData.Current_Time = startTime + i*10;
        gpsData.GPS_Timestamp = timeData.Current_Time - 5.0f; // 유효 범위
        gpsData.GPS_Velocity_X = 10.0f + i*0.1f; // 조금씩 변동
        EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
        // 각 루프마다 속도가 서서히 보정되어 큰 이질감 없이 진행
        EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f + i*0.1f, 1.0f);
    }
}

/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_RA_50 (RA_50)
 *  - GPS 및 IMU 미세 변화 시 스무딩 효과 확인
 *  - 조건: 이전 루프 10→현재10.2
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_50)
{
    // 첫 루프
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 두 번째 루프(약간 증가)
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 10.2f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 10 → 10.2 에 대해 중간 정도로 스무딩
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.5f);
}

/*--------------------------------------------------
 * main()
 *------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
