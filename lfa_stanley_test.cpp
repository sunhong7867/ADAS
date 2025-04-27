/*********************************************************************
 * lfa_stanley_test.cpp  ―  calculate_steer_in_high_speed_stanley() UT
 * DUT : calculate_steer_in_high_speed_stanley  (lfa.c / lfa.h)
 * TC  : EQ 20  +  BV 20  +  RA 20  = 60 test‑cases
 *
 * 빌드 예)
 *   g++ -std=c++17 -DUNIT_TEST lfa_stanley_test.cpp lfa.c lane_selection.c \
 *       -I/path/gtest/include -L/path/gtest/lib -lgtest -lgtest_main \
 *       -pthread -o lfa_stanley_test
 *********************************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cfloat>
#include <cstring>

#include "lfa.h"              // calculate_steer_in_high_speed_stanley()
#include "lane_selection.h"   // Lane_Data_LS_t
#include "adas_shared.h"      // EgoData_t

#ifdef UNIT_TEST
extern float g_stanleyGain;   // Stanley gain (lfa.c)
#endif

/*──────────────────────────────────────────────────────────────────*/
static Lane_Data_LS_t makeLaneOut(float headingDeg, float offsetM)
{
    Lane_Data_LS_t o{}; std::memset(&o, 0, sizeof(o));
    o.LS_Heading_Error = headingDeg;
    o.LS_Lane_Offset   = offsetM;
    return o;
}
static Ego_Data_t makeEgo(float vel)
{
    Ego_Data_t e{}; std::memset(&e, 0, sizeof(e));
    e.Ego_Velocity_X = vel;
    return e;
}

static constexpr float TOL = 1e-4f;
static constexpr float YAW_CLAMP = 540.0f;
static constexpr float MIN_VEL   = 0.1f;

/* 테스트 픽스처 ---------------------------------------------------*/
class StanleyTest : public ::testing::Test
{
protected:
    Lane_Data_LS_t lane;
    Ego_Data_t     ego;
    void SetUp() override {
        lane = makeLaneOut(0.0f, 0.0f);
        ego  = makeEgo(20.0f);
#ifdef UNIT_TEST
        g_stanleyGain = 1.0f;
#endif
    }
    float call() {
        return calculate_steer_in_high_speed_stanley(&ego, &lane);
    }
};

/*******************************************************************
 * 1) EQ 20
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_EQ_01) {
    lane = makeLaneOut(0,0);
    EXPECT_NEAR(call(), 0.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_02) {
    lane = makeLaneOut(30,0);
    EXPECT_GT(call(), 0.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_03) {
    lane = makeLaneOut(-30,0);
    EXPECT_LT(call(), 0.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_04) {
    lane = makeLaneOut(0,1.0f);
    EXPECT_GT(call(), 0.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_05) {
    lane = makeLaneOut(0,-1.0f);
    EXPECT_LT(call(), 0.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_06) {
    lane = makeLaneOut(180,2.0f);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_07) {
    lane = makeLaneOut(-180,-2.0f);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_08) {
    lane = makeLaneOut(20,1.0f);
    ego  = makeEgo(0.05f);
    float res = call();
    float expect = 20.0f + static_cast<float>(std::atan2(1.0f, MIN_VEL) * 180.0 / M_PI);
    EXPECT_NEAR(res, expect, 1e-2f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_09) {
    lane = makeLaneOut(20,1.0f);
    ego  = makeEgo(NAN);
    EXPECT_NEAR(call(), 0.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_10) {
    lane = makeLaneOut(20, NAN);
    float out = call();
    EXPECT_TRUE(std::isnan(out) || std::fabs(out) < 1e-6f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_11) {
    lane = makeLaneOut(INFINITY,0);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_12) {
    lane = makeLaneOut(500,10);
    float out = call();
    EXPECT_NEAR(out, 539.9f, 5.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_13) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_14) {
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(nullptr, &lane), 0.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_15) {
    Lane_Data_LS_t dummy;
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(&ego, &dummy), 0.0f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_16) {
    lane = makeLaneOut(30,0);
    ego  = makeEgo(100.0f);
    EXPECT_NEAR(call(), 30.0f, 1e-2f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_17) {
    lane = makeLaneOut(0,1.0f);
    ego  = makeEgo(1.0f);
    EXPECT_NEAR(call(), 45.0f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_18) {
    lane = makeLaneOut(30,1.0f);
    ego  = makeEgo(1000.0f);
    EXPECT_NEAR(call(), 30.057f, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_19) {
    lane = makeLaneOut(20,1);
    float expect = 20 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1e-2f);
}
TEST_F(StanleyTest, TC_LFA_STAN_EQ_20) {
    lane = makeLaneOut(90,0);
    EXPECT_NEAR(call(), 90.0f, TOL);
}

/*******************************************************************
 * 2) BV 20
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_BV_01) {
    lane = makeLaneOut(-180,0);
    EXPECT_NEAR(call(), -180.0f, 1e-2f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_02) {
    lane = makeLaneOut(180,0);
    EXPECT_NEAR(call(), 180.0f, 1e-2f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_03) {
    lane = makeLaneOut(0,-2);
    float expect = static_cast<float>(-std::atan2(2.0,20.0)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.5f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_04) {
    lane = makeLaneOut(0,2);
    float expect = static_cast<float>(std::atan2(2.0,20.0)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.5f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_05) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(0);
    float expect = static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_06) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(0.1f);
    float expect = static_cast<float>(std::atan2(1,0.1f)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_07) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(100);
    float expect = static_cast<float>(std::atan2(1,100)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_08) {
    lane = makeLaneOut(0,0.0001f);
    float expect = static_cast<float>(std::atan2(0.0001,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1e-3f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_09) {
    lane = makeLaneOut(0,-0.0001f);
    float expect = static_cast<float>(std::atan2(-0.0001,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1e-3f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_10) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(FLT_MIN);
    float out = call();
    EXPECT_LE(out, YAW_CLAMP);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_11) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(FLT_MAX);
    float out = call();
    EXPECT_GE(out, -YAW_CLAMP);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_12) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_13) {
    lane = makeLaneOut(-600,-10);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_14) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(MIN_VEL);
    float out = call();
    EXPECT_NEAR(out, 84.3f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_15) {
    lane = makeLaneOut(0,-1);
    ego  = makeEgo(MIN_VEL);
    float out = call();
    EXPECT_NEAR(out, -84.3f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_16) {
    lane = makeLaneOut(-1000,-10);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_17) {
    lane = makeLaneOut(1000,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_18) {
    lane = makeLaneOut(0,0);
    EXPECT_NEAR(call(), 0.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_19) {
    lane = makeLaneOut(30,0);
    EXPECT_NEAR(call(), 30.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_BV_20) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(1);
    float expect = 30 + 45.0f;
    EXPECT_NEAR(call(), expect, 1.0f);
}

/*******************************************************************
 * 3) RA 20
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_RA_01) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(0.05f);
    float res = call();
    float expect = 20 + static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(res, expect, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_02) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(20);
    float expect = 30 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_03) {
    lane = makeLaneOut(-10,1);
    ego  = makeEgo(20);
    float expect = -10 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_04) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_05) {
    lane = makeLaneOut(40,0);
    EXPECT_NEAR(call(), 40.0f, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_06) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(20);
    float expect = static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_07) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(20);
    float low = call();
    ego  = makeEgo(100);
    float high = call();
    EXPECT_LT(high, low);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_08) {
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(&ego, nullptr), 0.0f, TOL);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_09) {
    lane = makeLaneOut(20,NAN);
    float res = call();
    EXPECT_TRUE(std::isnan(res) || std::fabs(res) < 1e-6f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_10) {
    lane = makeLaneOut(400,4);
    EXPECT_TRUE(std::isfinite(call()));
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_11) {
    lane = makeLaneOut(0,MIN_VEL);
    ego  = makeEgo(MIN_VEL);
    float out = call();
    EXPECT_NEAR(out, 45.0f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_12) {
    lane = makeLaneOut(20,1);
    float a = call();
    float b = call();
    EXPECT_NEAR(a, b, 1e-4f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_13) {
    lane = makeLaneOut(40,0);
    EXPECT_NEAR(call(), 40.0f, 0.1f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_14) {
    lane = makeLaneOut(INFINITY,NAN);
    EXPECT_NEAR(call(), 0.0f, 1.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_15) {
    lane = makeLaneOut(20,1);
    float first = call();
    float second = call();
    EXPECT_NEAR(first, second, 1e-4f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_16) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(FLT_MAX);
    float out = call();
    EXPECT_LE(out, YAW_CLAMP);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_17) {
    lane = makeLaneOut(10,0.5f);
    float a = call();
    lane.LS_Lane_Offset = 1.5f;
    float b = call();
    EXPECT_LT(std::fabs(b - a), 100.0f);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_18) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(20);
    float slow = call();
    ego  = makeEgo(100);
    float fast = call();
    EXPECT_LT(fast, slow);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_19) {
    lane = makeLaneOut(10,0);
    float a = call();
    lane.LS_Heading_Error = 20;
    float b = call();
    EXPECT_GT(b, a);
}
TEST_F(StanleyTest, TC_LFA_STAN_RA_20) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(0.05f);
    float expect = 20 + static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}

