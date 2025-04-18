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
static EgoData_t makeEgo(float vx)
{
    EgoData_t e{};
    std::memset(&e, 0, sizeof(EgoData_t));
    e.Ego_Velocity_X = vx;
    return e;
}

class LfaModeTest : public ::testing::Test
{
protected:
    EgoData_t ego;             /* 기본 0 m/s → LOW_SPEED */
    void SetUp() override
    {
        ego = makeEgo(0.0f);
    }
    /* 편의 호출 래퍼 */
    LFA_Mode_e call(float speed)             { ego.Ego_Velocity_X = speed; return lfa_mode_selection(&ego); }
    LFA_Mode_e callWithPtr(EgoData_t *ptr)   { return lfa_mode_selection(ptr); }
};

/*******************************************************************
 * 1) EQ  (동등 분할) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_EQ_01_LOW_10ms)
{
    /* 10.0 m/s → LOW_SPEED */
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_02_HIGH_20ms)
{
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_03_STOP_0ms)
{
    EXPECT_EQ(call(0.0f),  LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_04_HIGH_100ms)
{
    EXPECT_EQ(call(100.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_05_NEGATIVE_SPEED)
{
    EXPECT_EQ(call(-5.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_06_NaN_SPEED)
{
    EXPECT_EQ(call(NAN),   LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_07_INF_SPEED)
{
    EXPECT_EQ(call(INFINITY), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_08_NULL_PTR_FALLBACK)
{
    EXPECT_EQ(callWithPtr(nullptr), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_09_UNINITIALIZED_FIELD)
{
    /* 쓰레기 값 시나리오 : 초기화하지 않고 memset 0xAA */
    EgoData_t trash;
    std::memset(&trash, 0xAA, sizeof(trash));
    EXPECT_EQ(lfa_mode_selection(&trash), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_EQ_10_THRESHOLD_BELOW_1666)
{
    EXPECT_EQ(call(16.66f), LFA_MODE_LOW_SPEED);
}

/*******************************************************************
 * 2) BV  (경계값 분석) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_BV_01_1666_LOW)
{
    EXPECT_EQ(call(16.66f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_02_1667_HIGH)
{
    EXPECT_EQ(call(16.67f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_03_1668_HIGH)
{
    EXPECT_EQ(call(16.68f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_04_NEG_0001_LOW)
{
    EXPECT_EQ(call(-0.01f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_05_FLT_MIN_LOW)
{
    EXPECT_EQ(call(FLT_MIN), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_06_FLT_MAX_HIGH)
{
    EXPECT_EQ(call(FLT_MAX), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_07_VERY_SMALL_1e5)
{
    EXPECT_EQ(call(0.00001f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_08_VERY_LARGE_99999)
{
    EXPECT_EQ(call(99999.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_09_16665_LOW_DECIMAL)
{
    EXPECT_EQ(call(16.665f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_BV_10_16675_HIGH_DECIMAL)
{
    EXPECT_EQ(call(16.675f), LFA_MODE_HIGH_SPEED);
}

/*******************************************************************
 * 3) RA  (요구사항 분석) 10 TC
 ******************************************************************/
TEST_F(LfaModeTest, TC_LFA_MODE_RA_01_SPEC_LOW_LT1667)
{
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_02_SPEC_HIGH_GE1667)
{
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_03_NULL_PTR_FALLBACK)
{
    EXPECT_EQ(callWithPtr(nullptr), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_04_NEGATIVE_LOW)
{
    EXPECT_EQ(call(-5.0f), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_05_EXTREME_HIGH_1000)
{
    EXPECT_EQ(call(1000.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_06_EXACT_THRESHOLD_1667)
{
    EXPECT_EQ(call(16.67f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_07_NAN_FALLBACK)
{
    EXPECT_EQ(call(NAN), LFA_MODE_LOW_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_08_REPETITIVE_INPUT_CONSISTENCY)
{
    /* 동일 입력 두 번 → 동일 결과 */
    for(int i=0;i<2;++i)
        EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_09_FLOAT_ROUNDING_16669999_HIGH)
{
    EXPECT_EQ(call(16.6699999f), LFA_MODE_HIGH_SPEED);
}

TEST_F(LfaModeTest, TC_LFA_MODE_RA_10_STATE_INDEPENDENCE)
{
    /* 1) HIGH                      */
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
    /* 2) LOW  → 이전 호출 영향 無  */
    EXPECT_EQ(call(10.0f), LFA_MODE_LOW_SPEED);
    /* 3) 다시 HIGH                */
    EXPECT_EQ(call(20.0f), LFA_MODE_HIGH_SPEED);
}

/*******************************************************************
 * main()
 ******************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
