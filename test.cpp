#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

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

/*--------------------------------------------------
 * main()
 *------------------------------------------------*/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
