/*********************************************************************
 * lfa_pid_test.cpp  ―  calculate_steer_in_low_speed_pid() 단위시험
 * DUT : calculate_steer_in_low_speed_pid (lfa.c / lfa.h)
 * TC  : EQ 20  +  BV 20  +  RA 20  = 60 test‑cases
 *
 * 빌드 예)
 *   g++ -std=c++17 -DUNIT_TEST lfa_pid_test.cpp lfa.c lane_selection.c \
 *       -I/path/gtest/include -L/path/gtest/lib -lgtest -lgtest_main \
 *       -pthread -o lfa_pid_test
 *********************************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cfloat>
#include <cstring>

#include "lfa.h"              // calculate_steer_in_low_speed_pid prototype
#include "lane_selection.h"   // Lane_Data_LS_t (PID 입력구조)

/*--------------------------------------------------------------------
 * DUT 내부의 적분 / 이전 오차 변수 테스트‑용 외부 노출
 *  (lfa.c 내부에서  #ifdef UNIT_TEST  extern float g_pidIntegral; ...)
 *------------------------------------------------------------------*/
#ifdef UNIT_TEST
extern float g_pidIntegral;
extern float g_pidPrevError;
#endif

/* ───── 헬퍼 ──────────────────────────────────────────────────── */
static Lane_Data_LS_t makeLaneOut(float headingDeg, float offsetM)
{
    Lane_Data_LS_t o{};
    std::memset(&o, 0, sizeof(o));
    o.LS_Heading_Error = headingDeg;
    o.LS_Lane_Offset   = offsetM;
    return o;
}

static constexpr float TOL = 1e-4f;          // 비교 허용 오차
static constexpr float YAW_CLAMP = 540.0f;   // 시스템 최대/최소 조향각

/* 테스트 픽스처 */
class LfaPidTest : public ::testing::Test
{
protected:
    Lane_Data_LS_t lane;
    void SetUp() override
    {
        lane = makeLaneOut(0.0f, 0.0f);
#ifdef UNIT_TEST
        g_pidIntegral   = 0.0f;
        g_pidPrevError  = 0.0f;
#endif
    }
    float call(float dt)
    {
        return calculate_steer_in_low_speed_pid(&lane, dt);
    }
};

/*******************************************************************
 * 1) EQ 20 TC ‑‑ 동등 분할
 ******************************************************************/
TEST_F(LfaPidTest, TC_LFA_PID_EQ_01)
{
    lane = makeLaneOut(0.0f, 0.0f);
    EXPECT_NEAR(call(1.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_02)
{
    lane = makeLaneOut(30.0f, 0.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_03)
{
    lane = makeLaneOut(-30.0f, 0.0f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_04)
{
    lane = makeLaneOut(0.0f, 1.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_05)
{
    lane = makeLaneOut(0.0f, -1.0f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_06)
{
    lane = makeLaneOut(180.0f, 2.0f);
    EXPECT_NEAR(call(1.0f), YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_07)
{
    lane = makeLaneOut(-180.0f, -2.0f);
    EXPECT_NEAR(call(1.0f), -YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_08)
{
    lane = makeLaneOut(30.0f, NAN);
    float out = call(1.0f);
    EXPECT_TRUE(std::isnan(out));
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_09)
{
    lane = makeLaneOut(INFINITY, 0.0f);
    EXPECT_NEAR(call(1.0f), YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_10)
{
    lane = makeLaneOut(30.0f, 1.0f);
    EXPECT_NEAR(call(0.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_11)
{
    lane = makeLaneOut(30.0f, 1.0f);
    EXPECT_NEAR(call(-1.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_12)
{
    lane = makeLaneOut(1.0f, 1.0f);        // Error = 2
    float out = call(1.0f);                // Kp=0.1, Ki=0.01, Kd=0.005
    float expect = (0.1f*2.0f) + (0.01f*2.0f) + (0.005f*2.0f); // 0.23
    EXPECT_NEAR(out, expect, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_13)
{
    lane = makeLaneOut(90.0f, 0.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_14)
{
    lane = makeLaneOut(0.0f, 2.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_15)
{
#ifdef UNIT_TEST
    g_pidIntegral = 1e6f;
#endif
    lane = makeLaneOut(10.0f, 0.5f);
    float out = call(1.0f);
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_16)
{
#ifdef UNIT_TEST
    g_pidIntegral = -1e6f;
#endif
    lane = makeLaneOut(-10.0f, -0.5f);
    float out = call(1.0f);
    EXPECT_GE(out, -YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_17)
{
#ifdef UNIT_TEST
    g_pidPrevError = 0.0f;
#endif
    lane = makeLaneOut(10.0f, 1.0f);   // Error=11
    float out = call(1.0f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_18)
{
#ifdef UNIT_TEST
    g_pidPrevError = 2.0f;
#endif
    lane = makeLaneOut(0.0f, 0.0f);    // Error=0
    float out = call(1.0f);
    EXPECT_LT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_19)
{
    EXPECT_NEAR(calculate_steer_in_low_speed_pid(nullptr, 1.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_EQ_20)
{
    lane = makeLaneOut(500.0f, 5.0f);
    float out = call(1.0f);
    EXPECT_LE(out, YAW_CLAMP);
    EXPECT_GE(out, -YAW_CLAMP);
}

/*******************************************************************
 * 2) BV 20 TC ‑‑ 경계값 분석
 ******************************************************************/
TEST_F(LfaPidTest, TC_LFA_PID_BV_01)
{
    lane = makeLaneOut(-180.0f, 0.0f);
    EXPECT_NEAR(call(1.0f), -20.7f, 1e-1f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_02)
{
    lane = makeLaneOut(180.0f, 0.0f);
    EXPECT_NEAR(call(1.0f), 20.7f, 1e-1f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_03)
{
    lane = makeLaneOut(0.0f, -2.0f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_04)
{
    lane = makeLaneOut(0.0f, 2.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_05)
{
    lane = makeLaneOut(10.0f, 1.0f);
    float out = call(0.00001f);
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_06)
{
    lane = makeLaneOut(30.0f, 1.0f);
    EXPECT_NEAR(call(0.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_07)
{
    lane = makeLaneOut(500.0f, 10.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, 539.9f, 1.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_08)
{
    lane = makeLaneOut(600.0f, 10.0f);
    EXPECT_NEAR(call(1.0f), YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_09)
{
    lane = makeLaneOut(-500.0f, -10.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, -539.9f, 1.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_10)
{
    lane = makeLaneOut(-600.0f, -10.0f);
    EXPECT_NEAR(call(1.0f), -YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_11)
{
#ifdef UNIT_TEST
    g_pidPrevError = 0.0f;
#endif
    lane = makeLaneOut(FLT_MAX, 0.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_12)
{
#ifdef UNIT_TEST
    g_pidIntegral = FLT_MAX;
#endif
    lane = makeLaneOut(0.0f, 0.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_13)
{
    lane = makeLaneOut(0.0001f, 0.0f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_14)
{
    lane = makeLaneOut(-0.0001f, 0.0f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_15)
{
    lane = makeLaneOut(0.0f, 0.0001f);
    EXPECT_GT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_16)
{
    lane = makeLaneOut(0.0f, -0.0001f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_17)
{
    lane = makeLaneOut(10.0f, 1.0f);
    float out = call(FLT_MAX);
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_18)
{
#ifdef UNIT_TEST
    g_pidPrevError = 0.0f;
#endif
    lane = makeLaneOut(10.0f, 1.0f);
    float out1 = call(1.0f);
#ifdef UNIT_TEST
    extern void pid_set_gains(float,float,float);
    pid_set_gains(0.2f,0.02f,0.01f);
#endif
    lane = makeLaneOut(10.0f, 1.0f);
    float out2 = call(1.0f);
    EXPECT_NE(out1, out2);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_19)
{
#ifdef UNIT_TEST
    g_pidPrevError = 100.0f;
#endif
    lane = makeLaneOut(0.0f, 0.0f);
    EXPECT_LT(call(1.0f), 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_BV_20)
{
    lane = makeLaneOut(0.0f, 0.0f);
    EXPECT_NEAR(call(1.0f), 0.0f, TOL);
}

/*******************************************************************
 * 3) RA 20 TC ‑‑ 요구사항 분석
 ******************************************************************/
TEST_F(LfaPidTest, TC_LFA_PID_RA_01)
{
    lane = makeLaneOut(5.0f, 1.0f);            // Error = 6
#ifdef UNIT_TEST
    g_pidIntegral = 0.0f;
    g_pidPrevError = 0.0f;
#endif
    float out = call(1.0f);
    float expect = (0.1f*6.0f) + (0.01f*6.0f) + (0.005f*6.0f);
    EXPECT_NEAR(out, expect, 1e-3f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_02)
{
    lane = makeLaneOut(5.0f, 0.0f);  // Error = 5
    float out1 = call(1.0f);
    float out2 = call(1.0f);
    EXPECT_GT(std::fabs(out2), std::fabs(out1));
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_03)
{
#ifdef UNIT_TEST
    g_pidPrevError = 0.0f;
#endif
    lane = makeLaneOut(10.0f, 1.0f);
    float out = call(0.0001f);
    EXPECT_LE(out, YAW_CLAMP);
    EXPECT_GT(std::fabs(out), 1000.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_04)
{
#ifdef UNIT_TEST
    extern void pid_set_gains(float,float,float);
    pid_set_gains(0.05f,0.0f,0.0f);
#endif
    lane = makeLaneOut(10.0f, 0.0f);
    float lowGain = call(1.0f);
#ifdef UNIT_TEST
    pid_set_gains(0.2f,0.0f,0.0f);
#endif
    lane = makeLaneOut(10.0f, 0.0f);
    float highGain = call(1.0f);
    EXPECT_GT(std::fabs(highGain), std::fabs(lowGain));
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_05)
{
#ifdef UNIT_TEST
    pid_set_gains(0.1f,0.0f,0.0f);
#endif
    lane = makeLaneOut(10.0f, 1.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, 0.1f*11.0f, 1e-3f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_06)
{
#ifdef UNIT_TEST
    pid_set_gains(0.0f,0.01f,0.0f);
    g_pidIntegral = 0.0f;
#endif
    lane = makeLaneOut(0.0f, 1.0f);
    float out1 = call(1.0f);
    float out2 = call(1.0f);
    EXPECT_GT(out2, out1);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_07)
{
#ifdef UNIT_TEST
    pid_set_gains(0.0f,0.0f,0.005f);
    g_pidPrevError = 10.0f;
#endif
    lane = makeLaneOut(20.0f, 0.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, 0.005f*10.0f, 1e-3f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_08)
{
#ifdef UNIT_TEST
    g_pidIntegral = 1e6f;
#endif
    lane = makeLaneOut(30.0f, 1.0f);
    float out = call(1.0f);
    EXPECT_NEAR(out, YAW_CLAMP, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_09)
{
    lane = makeLaneOut(100.0f, 0.0f);
    float out = call(1.0f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_10)
{
    lane = makeLaneOut(0.0f, 2.0f);
    float out = call(1.0f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_11)
{
    EXPECT_NEAR(calculate_steer_in_low_speed_pid(nullptr, 1.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_12)
{
    lane = makeLaneOut(1000.0f, 10.0f);
    float out = call(1.0f);
    EXPECT_LE(out, YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_13)
{
    lane = makeLaneOut(400.0f, 4.0f);
    float out = call(1.0f);
    EXPECT_TRUE(std::isfinite(out));
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_14)
{
    lane = makeLaneOut(10.0f, 1.0f);
    float first  = call(1.0f);
    float second = call(1.0f);
    EXPECT_GT(std::fabs(second), std::fabs(first));
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_15)
{
    lane = makeLaneOut(0.0f, 0.0f);
    EXPECT_NEAR(call(1.0f), 0.0f, TOL);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_16)
{
    lane = makeLaneOut(30.0f, 0.0f);
    float out = call(1.0f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_17)
{
    lane = makeLaneOut(0.0f, 1.0f);
    float out = call(1.0f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_18)
{
#ifdef UNIT_TEST
    g_pidPrevError = 6.0f;
#endif
    lane = makeLaneOut(5.0f, 1.0f);
    float out = call(1.0f);
    float expectP = 0.1f*6.0f;
    EXPECT_NEAR(out, expectP, 0.2f);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_19)
{
    lane = makeLaneOut(10.0f, 1.0f);
    call(1.0f);
    float out = call(1.0f);
    EXPECT_LE(std::fabs(out), YAW_CLAMP);
}

TEST_F(LfaPidTest, TC_LFA_PID_RA_20)
{
    lane = makeLaneOut(NAN, 1.0f);
    EXPECT_NEAR(call(1.0f), 0.0f, TOL);
}

/*******************************************************************
 *  main()
 ******************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
