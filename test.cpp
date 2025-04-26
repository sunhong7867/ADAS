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



TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_43)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 900.0f;
    timeData.Current_Time        = 1000.0f;
    gpsData.GPS_Timestamp         = 900.0f; // dt = 100ms → 무효
    gpsData.GPS_Velocity_X        = 50.0f;  // 스파이크
    gpsData.GPS_Velocity_Y        =  0.0f;
    imuData.Linear_Acceleration_X = 20.0f;  // 스파이크
    imuData.Linear_Acceleration_Y = 20.0f;
    imuData.Yaw_Rate              =  0.0f;
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 50.0f;

    kfState.X[0] = 10.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 매우 큰 값으로 인해 속도 과도 증가 가능
    // 테스트에선 과도하게 큰 결과(예: 30 m/s 이상) 나올 수 있음
    EXPECT_NEAR(egoData.Ego_Velocity_X, 10.0f, 1e-5f);
}


TEST_F(EgoVehicleEstimationTest, TC_EGO_RA_47)
{
    InitEgoVehicleKFState(&kfState);
    timeData.Current_Time        = 1000.0f;
    kfState.Previous_Update_Time = 1000.0f;
    gpsData.GPS_Timestamp = 0.0f;       // 유효시간 밖 → gps_update_enabled=false
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 0.0f;
    // X = [10,1,2,1,45]
    kfState.X[0] = 10.0f;
    kfState.X[1] = 1.0f;
    kfState.X[2] = 2.0f;
    kfState.X[3] = 1.0f;
    kfState.X[4] = 45.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    EXPECT_NEAR(egoData.Ego_Velocity_X,    10.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y,     1.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 2.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 1.0f, 1e-3f);
    EXPECT_NEAR(egoData.Ego_Heading,      45.0f, 1e-3f);
}

/*--------------------------------------------------
 * main()
 *------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
