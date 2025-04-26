#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

/*--------------------------------------------------
 * GTest Fixture
 *------------------------------------------------*/
static const int KF_DIM = 5;
class EgoVehicleEstimationTestBV : public ::testing::Test {
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

        // 예시로 KF 상태 벡터 및 시간을 임의 초기화
        kfState.X[0] = 5.0f;   // vx
        kfState.X[1] = 2.0f;   // vy
        kfState.X[2] = 0.5f;   // ax
        kfState.X[3] = 0.2f;   // ay
        kfState.X[4] = 0.0f;   // heading
        kfState.Previous_Update_Time = 900.0f; // 이전 업데이트 시각 [ms]

        // 스파이크 검출용 이전 IMU / GPS 값을 임의 설정
        kfState.Prev_Accel_X  = 1.0f;
        kfState.Prev_Accel_Y  = 0.5f;
        kfState.Prev_Yaw_Rate = 0.0f;
        kfState.Prev_GPS_Vel_X = 0.0f;
        kfState.Prev_GPS_Vel_Y = 0.0f;
    }
};



/*─────────────────────────────────────────────────────────────────────────────
 * TC_EGO_BV_41
 *  Kalman Gain = 0.99 → 거의 전량 보정
 *  (실제 코드 유도상황 세팅 필요. 여기선 단순 유추로 확인)
 *────────────────────────────────────────────────────────────────────────────*/
TEST_F(EgoVehicleEstimationTestBV, TC_EGO_BV_41)
{
    InitEgoVehicleKFState(&kfState);
    kfState.Previous_Update_Time = 0.0f;
    timeData.Current_Time        = 1000.0f;
    // 칼만게인을 0.99 근방으로 만들려면 관측오차 R 작게, P 크게 등 여러 조건 필요
    // 여기서는 가정만 하고, 보정 결과가 거의 GPS값 쪽으로 끌려가면 통과
    gpsData.GPS_Timestamp = 995.0f;
    gpsData.GPS_Velocity_X = 20.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    kfState.Prev_GPS_Vel_X  = gpsData.GPS_Velocity_X;
    kfState.Prev_GPS_Vel_Y  = gpsData.GPS_Velocity_Y;
    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate              = 0.0f;
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    // 거의 GPS 값으로 끌려옴
    EXPECT_NEAR(egoData.Ego_Velocity_X, 20.0f, 0.5f);
}
/*--------------------------------------------------
 * main()
 *------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
