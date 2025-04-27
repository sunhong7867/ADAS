/*********************************************************************
 * lfa_mode_test.cpp  ―  LFA Mode‑Selection 단위시험 (30 TC)
 * DUT  : lfa_mode_selection()          (lfa.c / lfa.h)
 * 규격 : SDS 2.2.4  “LFA Mode Selection”
 * 빌드 : g++ -std=c++17 ... -lpthread -lgtest -lgtest_main
 *********************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <cfloat>
#include <cmath>
#include "lfa.h"           // DUT prototype + enums
                           // (lfa.h 내부에 adas_shared.h 포함)

/* ────────── 헬퍼/Fixture ─────────────────────────────────────── */
static Ego_Data_t makeEgo(float vx)
{
    Ego_Data_t e{};
    std::memset(&e, 0, sizeof(Ego_Data_t));
    e.Ego_Velocity_X = vx;
    return e;
}

class LfaModeTest : public ::testing::Test
{
protected:
    Ego_Data_t ego;                 /* 기본 0 m/s → LOW_SPEED */
    void SetUp() override           { ego = makeEgo(0.0f); }

    /* 편의 호출 래퍼 */
    LFA_Mode_e call(float speed)            { ego.Ego_Velocity_X = speed; return lfa_mode_selection(&ego); }
    LFA_Mode_e callWithPtr(Ego_Data_t *ptr) { return lfa_mode_selection(ptr); }
};

/*******************************************************************
 * 1) EQ  (동등 분할) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_EQ_01)
{
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_02)
{
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_03)
{
    EXPECT_EQ(call(0.0f),  LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_04)
{
    EXPECT_EQ(call(100.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_05)
{
    EXPECT_EQ(call(-5.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_06)
{
    EXPECT_EQ(call(NAN),   LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_07)
{
    EXPECT_EQ(call(INFINITY), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_08)
{
    EXPECT_EQ(callWithPtr(nullptr), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_09)
{
    Ego_Data_t trash;
    std::memset(&trash, 0xAA, sizeof(trash));
    EXPECT_EQ(lfa_mode_selection(&trash), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_10)
{
    EXPECT_EQ(call(16.66f), LFA_MODE_LOW_SPEED);
}

/*******************************************************************
 * 2) BV  (경계값 분석) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_BV_01)
{
    EXPECT_EQ(call(16.66f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_02)
{
    EXPECT_EQ(call(16.67f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_03)
{
    EXPECT_EQ(call(16.68f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_04)
{
    EXPECT_EQ(call(-0.01f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_05)
{
    EXPECT_EQ(call(FLT_MIN), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_06)
{
    EXPECT_EQ(call(FLT_MAX), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_07)
{
    EXPECT_EQ(call(0.00001f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_08)
{
    EXPECT_EQ(call(99999.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_09)
{
    EXPECT_EQ(call(16.665f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_10)
{
    EXPECT_EQ(call(16.675f), LFA_MODE_HIGH_SPEED);
}

/*******************************************************************
 * 3) RA  (요구사항 분석) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_RA_01)
{
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_02)
{
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_03)
{
    EXPECT_EQ(callWithPtr(nullptr), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_04)
{
    EXPECT_EQ(call(-5.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_05)
{
    EXPECT_EQ(call(1000.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_06)
{
    EXPECT_EQ(call(16.67f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_07)
{
    EXPECT_EQ(call(NAN), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_08)
{
    for(int i=0;i<2;++i)
        EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_09)
{
    EXPECT_EQ(call(16.6699999f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_10)
{
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

