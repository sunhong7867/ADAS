#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

#define KF_DIM 5

// Test Fixture for EgoVehicleEstimation tests.
class EgoVehicleEstimationTest : public ::testing::Test {
protected:
    TimeData_t timeData;
    GPSData_t gpsData;
    IMUData_t imuData;
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;

    virtual void SetUp() override {
        std::memset(&timeData, 0, sizeof(timeData));
        std::memset(&gpsData, 0, sizeof(gpsData));
        std::memset(&imuData, 0, sizeof(imuData));
        std::memset(&egoData, 0, sizeof(egoData));
        InitEgoVehicleKFState(&kfState);
        // KF 상태 벡터 초기화 (예시 값)
        kfState.X[0] = 10.0f; // 속도
        kfState.X[1] = 0.0f;
        kfState.X[2] = 1.0f;  // 가속도 x
        kfState.X[3] = 0.5f;  // 가속도 y
        kfState.X[4] = 0.0f;  // Heading
        kfState.Previous_Update_Time = 900.0f;
    }
};

/* TC_EGO_EQ_01
   GPS 유효 시간 내 속도 업데이트  
   - Precondition: pTimeData: Current_Time = 1000ms; pGPSData: GPS_Timestamp = 980ms; 
     pIMUData: Linear_Accel = (1.0, 0.5) m/s², Yaw_Rate = 5.0 °/s; pKFData: KF 이전 상태 정상.
   - Input Data: Current_Time = 1000ms; GPS_Timestamp = 980ms; GPS_Velocity_X = 10.0, GPS_Velocity_Y = 0.0; 
     IMU: (1.0, 0.5) m/s², Yaw_Rate = 5.0 °/s.
   - Expected: Ego_Velocity_X ≈ 10.0 m/s, Ego_Heading ≈ 5.0°, Ego_Position = (0,0,0)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_01) {
    // Precondition
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 980.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    // Input Data
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    // KF 초기 상태 (Precondition에 포함됨)
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // Expected results (오차 허용 범위는 예시)
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 0.001f);
}

/* TC_EGO_EQ_02
   GPS 유효 시간 초과 시 보정 생략  
   - Precondition: Current_Time = 1000ms; GPS_Timestamp = 890ms;
   - Input Data: GPS_Velocity_X = 12.0 m/s, GPS_Velocity_Y = 0.0 m/s.
   - Expected: gps_update_enabled = false (내부 처리), 이전 KF 값 유지 (예: Ego_Velocity_X ≈ 10.0 m/s)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_02) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 890.0f; // 차이 = 110ms (> 50ms threshold)
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f; // 이전 정상 속도
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // Expected: GPS 업데이트가 생략되어 이전 KF 상태 (속도 ~10.0 m/s) 유지.
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/* TC_EGO_EQ_03
   IMU 가속도만으로 상태 예측  
   - Precondition: Current_Time = 1000ms; GPS_Timestamp = 850ms (GPS 무효 처리)
   - Input Data: IMU: Linear_Accel = (2.0, 1.0) m/s², Yaw_Rate = 3.0 °/s.
   - Expected: KF 예측 단계에 의해 Ego_Velocity 및 가속도가 IMU 입력 기반으로 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_03) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 850.0f; // 무효한 GPS
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 2.0f;
    imuData.Linear_Acceleration_Y = 1.0f;
    imuData.Yaw_Rate = 3.0f;
    kfState.X[0] = 5.0f;  // 초기값 예시
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_GT(egoData.Ego_Velocity_X, 5.0f);
}

/* TC_EGO_EQ_04
   IMU Yaw Rate만 변화 시 Heading 보정  
   - Precondition: Current_Time = 1000ms;
   - Input Data: IMU: Yaw_Rate = 10.0 °/s (나머지 값 정상, GPS_Timestamp = 995ms, GPS 속도 (10,0));
   - Expected: Ego_Heading 약 10° 증가 (예: 10 ± 오차)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_04) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 10.0f;
    kfState.X[4] = 0.0f;  // 초기 Heading = 0°
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Heading, 10.0f, 2.0f);
}

/* TC_EGO_EQ_05
   Heading 오차 – 음수 차이  
   - Precondition: 현재 KF Heading 상태 = 170° (KF X[4]), 입력: Ego_Heading = -170°
   - Input Data: Lane/센서 입력은 정상; 여기서는 KF 초기 Heading 170°; Ego 입력은 -170°로 설정.
   - Expected: 원시 차이 -340° → 정규화 후 20° 산출.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_05) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 0.0f;
    // KF 초기 Heading = 170 deg
    kfState.X[4] = 170.0f;
    kfState.Previous_Update_Time = 900.0f;
    
    // 여기서 pEgoData 출력은 KF 업데이트 결과를 반영하므로,
    // 우리가 가정하는 내부 로직에 따라 결과가 20 deg로 정규화됨.
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    EXPECT_NEAR(egoData.Ego_Heading, 20.0f, 2.0f);
}

/* TC_EGO_EQ_06
   상태 벡터 X 갱신 확인  
   - Precondition: KF 초기 상태가 정상 (예: X[0] = 10.0, X[1]=0, X[2]=1.0, X[3]=0.5, X[4]=0.0)
   - Input Data: 정상 센서 입력 (Current_Time = 1000ms, GPS_Timestamp = 995ms, GPS = (10.0,0), IMU = (1.0,0.5), Yaw_Rate=5.0)
   - Expected: KF 업데이트 후 상태 벡터 X[0..4]가 변경되어 출력에 반영됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_06) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f; // 초기값
    kfState.X[4] = 0.0f;  // 초기 heading
    kfState.Previous_Update_Time = 900.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    // 단순 예시: 상태 벡터 X가 업데이트되어 EgoData에 반영되었는지 확인 (세부 값은 KF 알고리즘에 따라 다름)
    EXPECT_NE(egoData.Ego_Velocity_X, 10.0f); // 변화가 있음을 확인
}

/* TC_EGO_EQ_07
   공분산 행렬 P 갱신 확인  
   - Precondition: KF 초기 공분산 행렬 P의 대각선 값이 100.0으로 설정되어 있음.
   - Input Data: 센서 입력 정상 (Current_Time=1000, GPS, IMU 정상)
   - Expected: KF 업데이트 후 P 행렬의 값에 변화가 발생함.
   (내부 P 행렬은 직접 검증하기 어렵지만, KF 업데이트로 인해 P 값이 변경되었음을 로그나 내부 변수 접근을 통해 확인할 수 있어야 함.)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_07) {
    // 예시: KF 공분산 행렬 P의 대각선 요소를 100.0으로 설정
    for (int i = 0; i < KF_DIM; ++i) {
        kfState.P[i * KF_DIM + i] = 100.0f;
    }
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 첫 대각 원소(0,0)의 값이 100.0f에서 변경되었는지 확인 (P는 1차원 배열로 접근)
    EXPECT_NE(kfState.P[0], 100.0f);
}


/* TC_EGO_EQ_08
   IMU + GPS 융합 정상 동작 확인  
   - Precondition: 모든 센서 입력이 정상 범위에 있음.
   - Input Data: GPS: (10.0, 0.0) m/s, GPS_Timestamp=995ms; IMU: Linear_Accel=(1.0, 0.5) m/s², Yaw_Rate=5.0 °/s.
   - Expected: KF 융합 결과로 Ego_Velocity, Acceleration, Heading 등이 갱신됨 (예: Ego_Velocity_X ≈ 10.0, Ego_Heading ≈ 5.0°)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_08) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
}

/* TC_EGO_EQ_09
   GPS, IMU 모두 무효 시 이전 상태 유지  
   - Precondition: GPS_Timestamp와 IMU 값이 비정상 (스파이크 등)으로 처리되어야 함.
   - Input Data: GPS: Timestamp = 800ms; IMU: Spike 처리된 값 (예: 아주 높은 accel)
   - Expected: KF 업데이트 생략, 이전 KF 상태(예: 속도 10.0 m/s 등)가 유지됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_09) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 800.0f;  // 비정상
    gpsData.GPS_Velocity_X = 50.0f;    // Spike
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 10.0f; // Spike
    imuData.Linear_Acceleration_Y = 5.0f;
    imuData.Yaw_Rate = 20.0f;
    // 이전 KF 상태
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/* TC_EGO_EQ_10
   센서 동기화 정상 동작 확인  
   - Precondition: Current_Time = 1000ms, GPS_Timestamp = 990ms (delta_t = 10ms)
   - Input Data: GPS 정상, IMU 정상
   - Expected: delta_t ≈ 10ms에 따른 KF 예측이 수행됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_10) {
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 990.0f;
    gpsData.GPS_Timestamp = 990.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(timeData.Current_Time - kfState.Previous_Update_Time, 10.0f, 1.0f);
}

/* TC_EGO_EQ_11
   Kalman Gain 계산 확인  
   - Precondition: KF의 상태 및 공분산 행렬 P가 SDS 기준으로 초기화되어 있어야 함.
   - Input Data: 정상 GPS, IMU 입력.
   - Expected: KF 업데이트 단계에서 Kalman Gain에 따른 보정이 이루어져 상태 벡터 X에 변화가 발생.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_11) {
    for (int i = 0; i < KF_DIM; ++i) {
        kfState.P[i * KF_DIM + i] = 100.0f;
    }
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // KF 업데이트 후 첫 대각 원소가 변경되었음을 확인
    EXPECT_NE(kfState.P[0 * KF_DIM + 0], 100.0f);
}

/* TC_EGO_EQ_12
   상태 벡터 X 갱신 확인  
   - Precondition: KF 초기 상태 벡터가 정상 초기화되어 있어야 함.
   - Input Data: GPS: (10.0, 0.0) m/s, IMU: (1.0, 0.5) m/s², Yaw_Rate = 5.0 °/s.
   - Expected: 상태 벡터 X(속도, 가속도, Heading 등)가 갱신됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_12) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(egoData.Ego_Velocity_X, 10.0f);  // 상태 벡터 X 갱신 확인
}

/* TC_EGO_EQ_13
   공분산 행렬 P 갱신 확인  
   - Precondition: KF 초기 공분산 행렬 P의 대각 성분이 100.0으로 초기화됨.
   - Input Data: GPS, IMU 정상.
   - Expected: KF 업데이트 후 P 행렬 값이 변화됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_13) {
    for (int i = 0; i < KF_DIM; ++i) {
        kfState.P[i * KF_DIM + i] = 100.0f;
    }
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // P[0] (대각 원소) 변화 확인
    EXPECT_NE(kfState.P[0 * KF_DIM + 0], 100.0f);
}

/* TC_EGO_EQ_14
   IMU + GPS 융합 정상 동작 확인  
   - Precondition: 모든 센서 입력이 정상 범위
   - Input Data: GPS: (10.0,0) m/s, GPS_Timestamp=995ms; IMU: (1.0,0.5) m/s², Yaw_Rate = 5.0 °/s.
   - Expected: KF 융합 결과로 Ego_Velocity_X ≈ 10.0 m/s, Ego_Heading ≈ 5.0° 등 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_14) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 1.0f);
}

/* TC_EGO_EQ_15
   Position은 항상 (0,0,0) 확인  
   - Precondition: Ego_Position은 하드코딩되어 (0,0,0)으로 유지되어야 함.
   - Input Data: (별도 입력 없음, Ego_Position은 코드에서 고정)
   - Expected: Ego_Position = (0,0,0)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_15) {
    // 아무 입력 없이 SetUp()에서 Ego_Position이 0으로 초기화됨.
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 0.001f);
}

/* TC_EGO_EQ_16
   IMU만 입력 시 Heading 변화 확인  
   - Precondition: GPS_Timestamp 설정값이 과거 (850ms)로 무효 처리되어야 함.
   - Input Data: IMU: Yaw_Rate = 8.0 °/s; (나머지 값 정상)
   - Expected: Ego_Heading가 약 8° 증가하여 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_16) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 850.0f; // 무효
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 8.0f;
    kfState.X[4] = 5.0f; // 초기 Heading
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f + 8.0f, 2.0f);  // 약 13° 예상
}

/* TC_EGO_EQ_17
   Output 단위 확인  
   - Precondition: 센서 및 KF 데이터 입력 시 단위가 SDS 사양과 일치해야 함.
   - Input Data: GPS, IMU 정상 (GPS: (10.0,0)m/s, IMU: (1.0,0.5)m/s², Yaw_Rate=5.0°/s)
   - Expected: 출력 단위가 m/s, m/s², °로 표시됨.
   (단위 검증은 코드 수준에서 직접 확인하기 어렵으므로, 단위에 맞는 값 범위를 EXPECT_NEAR로 확인)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_17) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 단위 확인: 값의 범위가 SDS에 맞게 (예: Ego_Velocity_X in m/s)
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 2.0f);
}

/* TC_EGO_EQ_18
   GPS 속도만 입력 → 보정 작동 여부  
   - Precondition: GPS 입력만 정상 (GPS_Velocity_X = 12.0, GPS_Velocity_Y = 0) 및 IMU 모두 0.
   - Input Data: GPS: (12.0, 0.0) m/s; IMU: (0, 0) m/s², Yaw_Rate=0.
   - Expected: KF 보정에 의해 Ego_Velocity_X ≈ 12.0 m/s.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_18) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[0] = 10.0f; // 이전 속도 10.0
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 12.0f, 0.5f);
}

/* TC_EGO_EQ_19
   GPS 및 IMU 미세 변화 검증  
   - Precondition: 이전 루프 GPS_Velocity_X = 10.0 m/s; 현재 GPS_Velocity_X = 10.2 m/s; IMU 변화 미미.
   - Input Data: GPS: (10.2,0)m/s; IMU: Linear_Accel=(1.0,0.5) (동일);
   - Expected: KF 융합 결과 Ego_Velocity_X ≈ 10.1 m/s 정도로 미세하게 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_19) {
    // 첫 루프: 이전 KF 상태에는 속도 10.0 m/s가 저장되어 있다고 가정.
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.2f; // 소폭 증가
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.5f);
}

/* TC_EGO_EQ_20
   센서 동기화 정상 동작 확인  
   - Precondition: Current_Time = 1000ms, GPS_Timestamp = 990ms.
   - Input Data: GPS: (10.0, 0.0)m/s; IMU: (1.0,0.5)m/s², Yaw_Rate = 5.0°/s.
   - Expected: delta_t ≈ 10ms에 따른 KF 예측이 정상적으로 적용되어, 상태 벡터 업데이트.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_20) {
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 990.0f;
    gpsData.GPS_Timestamp = 990.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(timeData.Current_Time - kfState.Previous_Update_Time, 10.0f, 1.0f);
}

/* TC_EGO_EQ_21
   IMU Yaw Rate = 0일 때 Heading 유지  
   - Precondition: IMU Yaw_Rate = 0.
   - Input Data: IMU: Yaw_Rate = 0 °/s; GPS: (10.0,0) m/s; 다른 입력 정상.
   - Expected: Ego_Heading는 변화 없이 기존 값 유지 (예: 5.0° 그대로).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_21) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 0.0f; // no change
    kfState.X[4] = 5.0f; // 초기 Heading = 5.0°
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f, 0.5f);
}

/* TC_EGO_EQ_22
   IMU 가속도 X 방향만 존재  
   - Precondition: IMU: Linear_Accel_X = 1.5 m/s², Linear_Accel_Y = 0.0 m/s².
   - Input Data: IMU 입력 외 GPS 정상.
   - Expected: Ego_Velocity_X 증가, Ego_Velocity_Y 변화 없음.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_22) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 0.0f, 0.5f);
}

/* TC_EGO_EQ_23
   IMU 가속도 Y 방향만 존재  
   - Precondition: IMU: Linear_Accel_X = 0.0 m/s², Linear_Accel_Y = 1.0 m/s².
   - Input Data: GPS 정상.
   - Expected: Ego_Velocity_Y 증가, Ego_Velocity_X 변화 없음.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_23) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[1] = 0.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 1.0f, 0.5f);
}

/* TC_EGO_EQ_24
   GPS Y 방향만 유효  
   - Precondition: GPS: Velocity_Y = 8.0 m/s; Velocity_X = 0.0 m/s.
   - Input Data: 나머지 센서 정상.
   - Expected: Ego_Velocity_Y ≈ 8.0 m/s.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_24) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 8.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 8.0f, 0.5f);
}

/* TC_EGO_EQ_25
   GPS X 방향만 유효  
   - Precondition: GPS: Velocity_X = 12.0 m/s; Velocity_Y = 0.0 m/s.
   - Input Data: 나머지 센서 정상.
   - Expected: Ego_Velocity_X ≈ 12.0 m/s.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_25) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 12.0f, 0.5f);
}

/* TC_EGO_EQ_26
   GPS 무효 시 IMU만 보정 작동  
   - Precondition: GPS_Timestamp = 900ms (무효 처리)
   - Input Data: IMU: (Linear_Accel = (1.0,0.5), Yaw_Rate = 5.0 °/s)
   - Expected: KF 예측은 IMU 입력만으로 이루어져 상태가 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_26) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 900.0f; // 무효
    gpsData.GPS_Velocity_X = 0.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
}

/* TC_EGO_EQ_27
   GPS, IMU 모두 무효 시 이전 상태 유지  
   - Precondition: GPS_Timestamp = 800ms; IMU에 스파이크 입력 적용.
   - Input Data: GPS: (50.0,0) m/s (비정상), IMU: (Linear_Accel = (10.0,5.0), Yaw_Rate = 20.0)
   - Expected: KF 업데이트가 생략되어 이전 상태 (예: 속도 10.0 m/s 등)이 유지됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_27) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 800.0f;
    gpsData.GPS_Velocity_X = 50.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 10.0f;
    imuData.Linear_Acceleration_Y = 5.0f;
    imuData.Yaw_Rate = 20.0f;
    kfState.X[0] = 10.0f;  // 이전 상태
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/* TC_EGO_EQ_28
   Position 출력 무조건 원점 유지  
   - Precondition: Ego_Position은 코드 내 하드코딩 (0,0,0) 상태.
   - Input Data: 별도 입력 없음.
   - Expected: Ego_Position = (0,0,0).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_28) {
    // Set some non-zero values to egoData before calling function to test if they get reset
    egoData.Ego_Position_X = 5.0f;
    egoData.Ego_Position_Y = 5.0f;
    egoData.Ego_Position_Z = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 0.001f);
}

/* TC_EGO_EQ_29
   초기화 이후 첫 입력 예측 정상 작동  
   - Precondition: KF 상태 초기화 정상 (InitEgoVehicleKFState 후)
   - Input Data: 첫 루프: GPS: (10.0,0)m/s, GPS_Timestamp = 995ms; IMU: (1.0,0.5)m/s², Yaw_Rate=5.0°/s.
   - Expected: KF 예측에 따라 상태 벡터 X가 갱신됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_29) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    // KF 초기 상태는 SetUp()에서 InitEgoVehicleKFState 호출로 이미 설정되어 있음.
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(egoData.Ego_Velocity_X, 10.0f); // 상태 벡터 업데이트 확인
}

/* TC_EGO_EQ_30
   칼만 필터 상태 초기화 후 예측 동작  
   - Precondition: KF 상태가 InitEgoVehicleKFState 호출 후 SDS 기준 정상 초기화됨.
   - Input Data: IMU: (1.0,0.5) m/s², Yaw_Rate=5.0°/s; GPS 정상.
   - Expected: KF 예측에 의해 Ego_Velocity, Heading 등이 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_30) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    // KF 이미 초기화되어 있음.
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1.0f);  // 값 변화 확인
}

/* TC_EGO_EQ_31
   Yaw Rate 음수 입력 시 반시계 회전 반영  
   - Precondition: IMU: Yaw_Rate = -8.0 °/s; 초기 Heading = 5.0 deg.
   - Input Data: IMU Yaw_Rate = -8.0 °/s.
   - Expected: Ego_Heading가 약 -8° 감소하여 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_31) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = -8.0f;
    kfState.X[4] = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 5.0f - 8.0f, 2.0f);
}

/* TC_EGO_EQ_32
   속도 예측과 GPS 속도 오차 존재 시 보정 확인  
   - Precondition: GPS_Velocity_X = 12.0 m/s vs KF 이전 속도 = 10.0 m/s.
   - Input Data: GPS_Vel_X = 12.0 m/s; IMU 정상.
   - Expected: Ego_Velocity_X 보정 결과가 GPS 값에 근접 (약 12.0 m/s).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_32) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 12.0f, 0.5f);
}

/* TC_EGO_EQ_33
   GPS 속도와 예측 속도 동일 시 보정 무효화 확인  
   - Precondition: GPS_Velocity_X = 10.0 m/s, IMU 입력 0.
   - Input Data: GPS: (10.0,0); IMU: all 0.
   - Expected: KF 업데이트 없이 이전 상태 유지 (Ego_Velocity = 10.0 m/s).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_33) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.2f);
}

/* TC_EGO_EQ_34
   Heading = 180도 유지 여부 확인  
   - Precondition: Ego_Heading 초기값 = 180°.
   - Input Data: Ego_Heading 초기 180°; 기타 센서 정상.
   - Expected: KF 업데이트 후도 Ego_Heading 출력이 180° (또는 -180°)로 정규화되어 유지됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_34) {
    timeData.Current_Time = 1000.0f;
    egoData.Ego_Heading = 180.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_TRUE((std::abs(egoData.Ego_Heading - 180.0f) < 1.0f) ||
                (std::abs(egoData.Ego_Heading + 180.0f) < 1.0f));
}

/* TC_EGO_EQ_35
   Heading = -180도 유지 여부 확인  
   - Precondition: Ego_Heading 초기값 = -180°.
   - Input Data: Ego_Heading 초기 -180°; 센서 입력 정상.
   - Expected: 출력 Ego_Heading이 -180°로 유지됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_35) {
    timeData.Current_Time = 1000.0f;
    egoData.Ego_Heading = -180.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 900.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, -180.0f, 1.0f);
}

/* TC_EGO_EQ_36
   이전 센서값 누적 업데이트 확인  
   - Precondition: 연속 루프 입력 시 KF 내부 Prev 값이 정상 업데이트 되어야 함.
   - Input Data: 첫 루프: GPS_Velocity_X = 10.0 m/s; 두 번째 루프: GPS_Velocity_X = 10.5 m/s.
   - Expected: 연속 입력에 따라 Ego_Velocity_X가 점진적으로 증가함.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_36) {
    // 첫 번째 루프
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float firstVel = egoData.Ego_Velocity_X;

    // 두 번째 루프 (GPS_Vel_X 증가)
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 10.5f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, firstVel);
}

/* TC_EGO_EQ_37
   GPS 유효 → 무효 전환 시 처리 확인  
   - Precondition: 첫 루프는 정상 GPS, 두 번째 루프는 GPS_Timestamp가 900ms (무효).
   - Input Data: 첫 루프: GPS 정상; 두 번째 루프: GPS_Timestamp = 900ms.
   - Expected: 첫 루프에서는 보정, 두 번째 루프에서는 GPS 업데이트 생략(이전 KF 상태 유지).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_37) {
    // 첫 번째 루프
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float velAfterFirst = egoData.Ego_Velocity_X;

    // 두 번째 루프 (GPS 무효)
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 900.0f; // 무효로 간주
    gpsData.GPS_Velocity_X = 15.0f;   // 변경되어도 적용되지 않아야 함
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, velAfterFirst, 0.5f);
}

/* TC_EGO_EQ_38
   IMU 노이즈 제거 후 이전 값 적용 확인  
   - Precondition: 첫 루프 IMU에 Spike 발생, 두 번째 루프 정상 값 복귀.
   - Input Data: 첫 루프: IMU: Linear_Accel_X = 5.0 m/s²; 두 번째 루프: 1.0 m/s².
   - Expected: 두 번째 루프에서 이전 정상 값(1.0 m/s²)이 적용되어 KF 예측이 개선됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_38) {
    // 첫 번째 루프: Spike 상황
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 5.0f; // spike
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 두 번째 루프: 정상 복귀
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    imuData.Linear_Acceleration_X = 1.0f; // 복귀
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // Expected: KF 업데이트 시, 이전 정상값이 반영되어 Ego 상태가 개선됨.
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 1.0f, 0.5f);
}

/* TC_EGO_EQ_39
   Accel_X, Accel_Y 변화 시 상태 반응 확인  
   - Precondition: 첫 루프 IMU: (1.0,0.5) m/s²; 두 번째 루프: (1.5,0.8) m/s².
   - Input Data: 두 번째 루프 적용.
   - Expected: Ego_Velocity 및 Acceleration 값이 업데이트되어 변화 반영.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_39) {
    // 첫 루프:
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float velFirst = egoData.Ego_Velocity_X;
    // 두 번째 루프:
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.8f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_GT(egoData.Ego_Velocity_X, velFirst);
}

/* TC_EGO_EQ_40
   GPS_Vel_X 과도 변화 시 무시 처리 확인  
   - Precondition: 첫 루프: GPS_Vel_X = 10.0 m/s; 두 번째 루프: GPS_Vel_X spikes to 50.0 m/s.
   - Input Data: 두 번째 루프 적용.
   - Expected: KF는 Spike를 감지하여 이전 속도(10.0 m/s)를 유지.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_40) {
    // 첫 루프:
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 두 번째 루프: Spike 발생
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 50.0f;  // spike
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.5f);
}

/* TC_EGO_EQ_41
   IMU + GPS 모두 변화 시 융합 정확성 확인  
   - Precondition: GPS: (12.0, 0.0) m/s; IMU: (1.5, 0.5) m/s², Yaw_Rate=6.0 °/s.
   - Input Data: 위 값 사용.
   - Expected: KF 융합 결과 Ego_Velocity_X ≈ 12.0 m/s, Heading 등 정상 업데이트.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_41) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 1.5f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 6.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 12.0f, 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, 6.0f, 2.0f);
}

/* TC_EGO_EQ_42
   Heading 정규화 결과 179 → -181 변환 확인  
   - Precondition: Ego_Heading 초기값 = 181° (범위 초과)
   - Input Data: 초기 Ego_Heading = 181°.
   - Expected: 출력이 -179° (또는 유사 값)로 정규화됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_42) {
    timeData.Current_Time = 1000.0f;
    egoData.Ego_Heading = 181.0f; // 입력값 181°
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 기대: 181° → 정규화하여 -179° (또는 -181°와 차이가 2° 미만)
    EXPECT_NEAR(egoData.Ego_Heading, -179.0f, 2.0f);
}

/* TC_EGO_EQ_43
   delta_t가 너무 작을 때 보정 미미 확인  
   - Precondition: Current_Time = 1000ms, Previous_Update_Time = 999ms.
   - Input Data: 정상 센서 입력.
   - Expected: delta_t가 최소 0.01초로 보정되어 미미한 상태 변화만 적용됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_43) {
    timeData.Current_Time = 1000.0f;
    kfState.Previous_Update_Time = 999.0f;  // delta_t = 1ms
    gpsData.GPS_Timestamp = 999.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 예상: delta_t 최소 보정 → 변화 미미
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.2f);
}

/* TC_EGO_EQ_44
   공분산 P 초기값 반영 확인  
   - Precondition: KF P행렬 대각 값이 100.0으로 설정되어 있음.
   - Input Data: 센서 입력 정상.
   - Expected: KF 업데이트 후 P 행렬 값이 변화됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_44) {
    for (int i = 0; i < KF_DIM; ++i) {
        kfState.P[i * KF_DIM + i] = 100.0f;
    }
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NE(kfState.P[0 * KF_DIM + 0], 100.0f);
}

/* TC_EGO_EQ_45
   IMU 입력 없음 시 상태 고정 확인  
   - Precondition: IMU: Linear_Accel=(0,0), Yaw_Rate = 0.
   - Input Data: GPS 정상.
   - Expected: KF 업데이트 없이 이전 상태 유지.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_45) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 0.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 0.2f);
}

/* TC_EGO_EQ_46
   보정 이후 상태 출력 일치 확인  
   - Precondition: KF 상태 벡터와 최종 EgoData가 동일하게 매핑되어야 함.
   - Input Data: 센서 입력 정상.
   - Expected: 최종 EgoData의 속도, 가속도, Heading이 KF 업데이트 결과와 일치함.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_46) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.X[4] = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, kfState.X[0], 0.5f);
    EXPECT_NEAR(egoData.Ego_Heading, kfState.X[4], 1.0f);
}

/* TC_EGO_EQ_47
   Yaw Rate = 90°/s → Heading 급증 반영  
   - Precondition: IMU: Yaw_Rate = 90°/s.
   - Input Data: IMU Yaw_Rate = 90°/s; 기타 정상.
   - Expected: Ego_Heading가 약 90° 상승하여 업데이트됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_47) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 90.0f;
    kfState.X[4] = 0.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Heading, 90.0f, 5.0f);
}

/* TC_EGO_EQ_48
   IMU X/Y 반대 입력 → 벡터 해석 일치 확인  
   - Precondition: IMU: Linear_Accel_X = 2.0 m/s², Linear_Accel_Y = -2.0 m/s².
   - Input Data: 위 값, 기타 센서 정상.
   - Expected: 가속도 벡터의 합산 효과로 인해 속도 변화가 올바르게 계산됨.
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_48) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 2.0f;
    imuData.Linear_Acceleration_Y = -2.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // 예상: X축 가속도 효과가 남아야 하므로 Ego_Velocity_X 약간 증가.
    EXPECT_GT(egoData.Ego_Velocity_X, 10.0f);
}

/* TC_EGO_EQ_49
   관측 행렬 H 구성 정확도 확인  
   - Precondition: KF 내부 H 행렬이 "속도 항목만" 포함되도록 구성되어 있어야 함.
   - Input Data: KF 업데이트 시, 정상 GPS 입력.
   - Expected: H 행렬 구성 결과가 설계 사양에 따라 적용됨.
   (내부 H 행렬 검증은 보통 디버깅 로그 또는 내부 변수 접근을 통해 수행하므로, 테스트에서는 상태 변화로 간접 확인)
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_49) {
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    // H 행렬은 내부 변수이므로, 상태 벡터의 올바른 업데이트로 간접 확인
    EXPECT_NEAR(egoData.Ego_Velocity_X, kfState.X[0], 0.5f);
}

/* TC_EGO_EQ_50
   GPS 및 IMU 미세 변화 검증  
   - Precondition: 첫 루프와 비교하여 소폭 변화한 센서 값 입력.
   - Input Data: 이전 루프: GPS_Vel_X = 10.0 m/s; 현재: GPS_Vel_X = 10.2 m/s;
   - Expected: KF 융합 결과 Ego_Velocity_X ≈ 약 10.1 m/s (미세 업데이트).
*/
TEST_F(EgoVehicleEstimationTest, TC_EGO_EQ_50) {
    // 첫 번째 루프:
    timeData.Current_Time = 1000.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 10.0f;
    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Yaw_Rate = 5.0f;
    kfState.X[0] = 10.0f;
    kfState.Previous_Update_Time = 990.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    float firstSpeed = egoData.Ego_Velocity_X;

    // 두 번째 루프: 소폭 변화
    timeData.Current_Time = 1100.0f;
    gpsData.GPS_Timestamp = 1095.0f;
    gpsData.GPS_Velocity_X = 10.2f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.1f, 0.5f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
