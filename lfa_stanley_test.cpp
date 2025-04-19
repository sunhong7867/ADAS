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
/* DUT 내부 상수/훅 노출 (gain 고정, clamp 기준 등) */
extern float g_stanleyGain;                    // lfa.c 에 정의된 Stanley gain (기본 1.0)
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
static constexpr float YAW_CLAMP = 540.0f;     // 시스템 조향 한계
static constexpr float MIN_VEL   = 0.1f;       // 내부 최소 속도 클램프

/* 테스트 픽스처 */
class StanleyTest : public ::testing::Test
{
protected:
    Lane_Data_LS_t lane;
    Ego_Data_t          ego;

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
 * 1) EQ 20 TC – 동등 분할
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_EQ_01_ZeroInput) {
    lane = makeLaneOut(0,0);
    EXPECT_NEAR(call(), 0.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_02_PosHeading_PosOutput) {
    lane = makeLaneOut(30,0);
    EXPECT_GT(call(), 0.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_03_NegHeading_NegOutput) {
    lane = makeLaneOut(-30,0);
    EXPECT_LT(call(), 0.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_04_PosOffset_PosOutput) {
    lane = makeLaneOut(0,1.0f);
    EXPECT_GT(call(), 0.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_05_NegOffset_NegOutput) {
    lane = makeLaneOut(0,-1.0f);
    EXPECT_LT(call(), 0.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_06_MaxPosClamp) {
    lane = makeLaneOut(180,2.0f);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_07_MaxNegClamp) {
    lane = makeLaneOut(-180,-2.0f);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_08_VelocityLowClamped) {
    lane = makeLaneOut(20,1.0f);
    ego  = makeEgo(0.05f);
    float res = call();
    float expect = 20.0f + static_cast<float>(std::atan2(1.0f, MIN_VEL) * 180.0 / M_PI);
    EXPECT_NEAR(res, expect, 1e-2f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_09_VelocityNaN_ReturnZero) {
    lane = makeLaneOut(20,1.0f);
    ego  = makeEgo(NAN);
    EXPECT_NEAR(call(), 0.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_10_OffsetNaN_Safe) {
    lane = makeLaneOut(20, NAN);
    EXPECT_TRUE(std::isnan(call()) || std::fabs(call()) < 1e-6f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_11_HeadingInf_Clamp) {
    lane = makeLaneOut(INFINITY,0);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_12_Output5399_NoClamp) {
    lane = makeLaneOut(500,10);
    float out = call();
    EXPECT_NEAR(out, 539.9f, 5.0f);  // ±5 허용
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_13_Output5401_Clamp) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_14_NullPtr_ReturnZero) {
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(nullptr, &lane), 0.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_15_UninitMembers_Fallback) {
    Lane_Data_LS_t dummy; // 미초기화
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(&ego, &dummy), 0.0f, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_16_OffsetZeroHighVel) {
    lane = makeLaneOut(30,0);
    ego  = makeEgo(100.0f);
    EXPECT_NEAR(call(),30.0f,1e-2f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_17_SmallVelOffsetEffect) {
    lane = makeLaneOut(0,1.0f);
    ego  = makeEgo(1.0f);
    EXPECT_NEAR(call(),45.0f,1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_18_VeryHighVel_SmallEffect) {
    lane = makeLaneOut(30,1.0f);
    ego  = makeEgo(1000.0f);
    float out = call();
    EXPECT_NEAR(out,30.057f,0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_19_GainFixedProportional) {
    lane = makeLaneOut(20,1);
    float out = call();
    float expect = 20 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(out, expect, 1e-2f);
}

TEST_F(StanleyTest, TC_LFA_STAN_EQ_20_HeadingOnly90) {
    lane = makeLaneOut(90,0);
    EXPECT_NEAR(call(),90.0f,TOL);
}

/*******************************************************************
 * 2) BV 20 TC – 경계값 분석
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_BV_01_MinHeading) {
    lane = makeLaneOut(-180,0);
    EXPECT_NEAR(call(), -180.0f, 1e-2f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_02_MaxHeading) {
    lane = makeLaneOut(180,0);
    EXPECT_NEAR(call(), 180.0f, 1e-2f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_03_MinOffset) {
    lane = makeLaneOut(0,-2);
    float expect = static_cast<float>(-std::atan2(2.0,20.0)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.5f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_04_MaxOffset) {
    lane = makeLaneOut(0,2);
    float expect = static_cast<float>(std::atan2(2.0,20.0)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.5f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_05_VelZero_MinClamp) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(0);
    float expect = static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_06_VelPointOne) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(0.1f);
    float expect = static_cast<float>(std::atan2(1,0.1f)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_07_Vel100) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(100);
    float expect = static_cast<float>(std::atan2(1,100)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_08_TinyOffsetPos) {
    lane = makeLaneOut(0,0.0001f);
    float expect = static_cast<float>(std::atan2(0.0001,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1e-3f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_09_TinyOffsetNeg) {
    lane = makeLaneOut(0,-0.0001f);
    float expect = static_cast<float>(std::atan2(-0.0001,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1e-3f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_10_VelFLTMIN) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(FLT_MIN);
    float out = call();
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_11_VelFLTMAX) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(FLT_MAX);
    float out = call();
    EXPECT_GE(out, -YAW_CLAMP);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_12_Output54001_Clamp) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_13_OutputNeg54001_Clamp) {
    lane = makeLaneOut(-600,-10);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_14_Atan90Deg) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(MIN_VEL); // atan2(1,0.1)≈84°, not 90 but close
    float out = call();
    EXPECT_NEAR(out, 84.3f, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_15_AtanMinus90) {
    lane = makeLaneOut(0,-1);
    ego  = makeEgo(MIN_VEL);
    float out = call();
    EXPECT_NEAR(out, -84.3f, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_16_ClampExactNeg540) {
    lane = makeLaneOut(-1000,-10);
    EXPECT_NEAR(call(), -YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_17_ClampExact540) {
    lane = makeLaneOut(1000,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_18_OutputZeroStability) {
    lane = makeLaneOut(0,0);
    EXPECT_NEAR(call(), 0.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_19_GainOffsetZeroInfluence) {
    lane = makeLaneOut(30,0);
    EXPECT_NEAR(call(), 30.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_BV_20_GainCTEEqualsVel) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(1);
    float expect = 30 + 45.0f; // atan2(1,1)=45 deg
    EXPECT_NEAR(call(), expect, 1.0f);
}

/*******************************************************************
 * 3) RA 20 TC – 요구사항 분석
 ******************************************************************/
TEST_F(StanleyTest, TC_LFA_STAN_RA_01_VelBelowMinClamped) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(0.05f);
    float res = call();
    float expect = 20 + static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(res, expect, 1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_02_FormulaAccuracy) {
    lane = makeLaneOut(30,1);
    ego  = makeEgo(20);
    float expect = 30 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_03_AtanSign) {
    lane = makeLaneOut(-10,1);
    ego  = makeEgo(20);
    float expect = -10 + static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_04_ClampApplied) {
    lane = makeLaneOut(600,10);
    EXPECT_NEAR(call(), YAW_CLAMP, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_05_HeadingOnly) {
    lane = makeLaneOut(40,0);
    EXPECT_NEAR(call(),40.0f,0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_06_OffsetOnly) {
    lane = makeLaneOut(0,1);
    ego  = makeEgo(20);
    float expect = static_cast<float>(std::atan2(1,20)*180/M_PI);
    EXPECT_NEAR(call(), expect, 0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_07_HighSpeed_Decrease) {
    lane = makeLaneOut(30,1);
    ego = makeEgo(20);
    float low = call();
    ego = makeEgo(100);
    float high = call();
    EXPECT_LT(high, low);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_08_NullPtrSafe) {
    EXPECT_NEAR(calculate_steer_in_high_speed_stanley(&ego, nullptr), 0.0f, TOL);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_09_OffsetNaN_Safe) {
    lane = makeLaneOut(20,NAN);
    float res = call();
    EXPECT_TRUE(std::isnan(res) || std::fabs(res)<1e-6f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_10_OutputFinite) {
    lane = makeLaneOut(400,4);
    EXPECT_TRUE(std::isfinite(call()));
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_11_RadToDegPi) {
    lane = makeLaneOut(0,MIN_VEL); // offset==vel -> atan2(0.1,0.1)=45
    ego  = makeEgo(MIN_VEL);
    float out = call();
    EXPECT_NEAR(out,45.0f,1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_12_GainStable) {
    lane = makeLaneOut(20,1);
    float a = call();
    float b = call();
    EXPECT_NEAR(a,b,1e-4f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_13_SteeringEqualsHeadingWhenOffsetZero) {
    lane = makeLaneOut(40,0);
    EXPECT_NEAR(call(),40.0f,0.1f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_14_ExceptionValueRobust) {
    lane = makeLaneOut(INFINITY,NAN);
    EXPECT_NEAR(call(),0.0f,1.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_15_ReentrantConsistency) {
    lane = makeLaneOut(20,1);
    float first = call();
    float second = call();
    EXPECT_NEAR(first, second, 1e-4f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_16_NumericExtremesNoOverflow) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(FLT_MAX);
    float out = call();
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_17_OffsetRapidChange) {
    lane = makeLaneOut(10,0.5f);
    float a = call();
    lane.LS_Lane_Offset = 1.5f;
    float b = call();
    EXPECT_LT(std::fabs(b-a),100.0f);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_18_SpeedInfluencesSteer) {
    lane = makeLaneOut(30,1);
    ego = makeEgo(20);
    float slow = call();
    ego = makeEgo(100);
    float fast = call();
    EXPECT_LT(fast, slow);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_19_HeadingChangeReflectsSteer) {
    lane = makeLaneOut(10,0);
    float a = call();
    lane.LS_Heading_Error = 20;
    float b = call();
    EXPECT_GT(b, a);
}

TEST_F(StanleyTest, TC_LFA_STAN_RA_20_VelBelowThresholdAutoClamp) {
    lane = makeLaneOut(20,1);
    ego  = makeEgo(0.05f);
    float expect = 20 + static_cast<float>(std::atan2(1,MIN_VEL)*180/M_PI);
    EXPECT_NEAR(call(), expect, 1.0f);
}

/*******************************************************************
 *  main()
 ******************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
