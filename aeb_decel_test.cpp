/*********************************************************************
 * aeb_decel_test.cpp ― AEB 감속 계산 Unit‑Test (60 TC)
 * ---------------------------------------------------------------
 * DUT  : calculate_decel_for_aeb()   (aeb.c / aeb.h)
 * 규격 : 2.2.3.1.3  (사내 SW 설계서)
 *********************************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cfloat>
#include "aeb.h"              // Unit‑Under‑Test
#include "adas_shared.h"

constexpr float EPS_F   = 1e-4f;
constexpr float INF_TTC = 99999.0f;

/* ────────── TTC 구조체 헬퍼 ─────────────────────────────────── */
static TTC_Data_t makeTtc(float ttc, float brake, float alert)
{
    TTC_Data_t d{};
    d.TTC        = ttc;
    d.TTC_Brake  = brake;
    d.TTC_Alert  = alert;
    return d;
}

/* ────────── Fixture ─────────────────────────────────────────── */
class AebDecelTest : public ::testing::Test
{
protected:
    TTC_Data_t ttc;
    void SetUp() override
    {
        /* 기본값 : Brake 모드 정상 구간(TTC<TTC_Brake)   */
        ttc = makeTtc(1.0f, 2.0f, 3.0f);
    }
    float call(AEB_Mode_e m)
    {
        return calculate_decel_for_aeb(m, &ttc);
    }
};

/*******************************************************************
 * 1) EQ  (동등 분할) 20 TC
 ******************************************************************/
TEST_F(AebDecelTest, TC_AEB_DEC_EQ_01)   /* Normal → 0.0 */
{
    EXPECT_FLOAT_EQ(call(AEB_MODE_NORMAL), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_02)   /* Alert → 0.0 */
{
    EXPECT_FLOAT_EQ(call(AEB_MODE_ALERT), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_03)   /* Brake, TTC<TTC_Brake */
{
    ttc = makeTtc(1.0f, 2.0f, 3.0f);      // –5.0 예상
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_04)   /* Brake, TTC>TTC_Brake → -2 */
{
    ttc = makeTtc(2.5f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_05)   /* Brake, TTC=0 → -10 */
{
    ttc = makeTtc(0.0f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_06)   /* TTC>TTC_Brake (3.0>2.0) clamp */
{
    ttc = makeTtc(3.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_07)   /* TTC==TTC_Brake → 0 */
{
    ttc = makeTtc(2.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_08)   /* 재확인: 선형 계산 */
{
    ttc = makeTtc(1.0f, 2.0f, 3.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_09)   /* TTC NaN → 0.0 */
{
    ttc.TTC = NAN;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_10)   /* TTC 음수 → 0.0 */
{
    ttc.TTC = -1.0f;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_11)   /* TTC_Brake 0 → 0.0 */
{
    ttc = makeTtc(1.0f, 0.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_12)   /* TTC_Brake 극소 → clamp -2 */
{
    ttc = makeTtc(0.5f, 1e-6f, 1.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_13)   /* TTC_Brake NaN → 0 */
{
    ttc.TTC_Brake = NAN;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_14)   /* TTC_Brake 음수 → 0 */
{
    ttc.TTC_Brake = -1.0f;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_15)   /* pTtcData == nullptr → 0 */
{
    EXPECT_FLOAT_EQ(calculate_decel_for_aeb(AEB_MODE_BRAKE, nullptr), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_16)   /* 내부 필드 0 → 0 */
{
    ttc = makeTtc(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_17)   /* Max_Brake_Decel 상수 확인 */
{
    ttc = makeTtc(1.0f, 2.0f, 3.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_18)   /* 계산 -11 → clamp -10 */
{
    ttc = makeTtc(-0.1f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_19)   /* 계산 -1 → clamp -2 */
{
    ttc = makeTtc(1.8f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_EQ_20)   /* Min_Brake_Decel 적용 확인 */
{
    ttc = makeTtc(1.8f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

/*******************************************************************
 * 2) BV  (경계값 분석) 20 TC
 ******************************************************************/
TEST_F(AebDecelTest, TC_AEB_DEC_BV_01)   /* TTC 0.0 */
{
    ttc = makeTtc(0.0f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_02)   /* TTC 0.01 → -9.9 */
{
    ttc = makeTtc(0.01f, 1.0f, 2.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -9.9f, 0.01f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_03)   /* TTC = TTC_Brake-0.01 */
{
    ttc = makeTtc(1.99f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_04)   /* TTC == TTC_Brake */
{
    ttc = makeTtc(2.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_05)   /* TTC_Brake+0.01 -> clamp -2 */
{
    ttc = makeTtc(2.01f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_06)   /* TTC 음수 */
{
    ttc = makeTtc(-0.01f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_07)   /* TTC_Brake 1e-6 극소 */
{
    ttc = makeTtc(0.00002f, 1e-6f, 1.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_08)   /* 표준 구간 */
{
    ttc = makeTtc(0.5f, 1.0f, 1.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_09)   /* TTC_Brake 큰 값 10 */
{
    ttc = makeTtc(5.0f, 10.0f, 11.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_10)   /* -10.01 → clamp -10 */
{
    ttc = makeTtc(-0.001f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_11)   /* -9.99 통과 */
{
    ttc = makeTtc(0.001f, 1.0f, 2.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -9.99f, 0.02f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_12)   /* -2.01 → clamp -2 */
{
    ttc = makeTtc(1.598f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_13)   /* -1.99 → clamp -2 */
{
    ttc = makeTtc(1.602f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_14)   /* TTC= TTC_Brake -> 0 */
{
    ttc = makeTtc(2.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_15)   /* ±0.0001 민감도 → clamp -2 */
{
    ttc = makeTtc(2.0001f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_16)   /* TTC = FLT_MAX → clamp -2 */
{
    ttc = makeTtc(FLT_MAX, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_17)   /* TTC = FLT_MIN → -10 근접 */
{
    ttc = makeTtc(FLT_MIN, 1.0f, 2.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -10.0f, 0.01f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_18)   /* -10.01 clamp 상한 */
{
    ttc = makeTtc(-0.001f, 1.0f, 2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_19)   /* -1.99 clamp 하한 */
{
    ttc = makeTtc(1.8f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_BV_20)   /* Min_Brake_Decel 적용 */
{
    ttc = makeTtc(1.8f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

/*******************************************************************
 * 3) RA  (요구사항 분석) 20 TC
 ******************************************************************/
TEST_F(AebDecelTest, TC_AEB_DEC_RA_01)   { EXPECT_FLOAT_EQ(call(AEB_MODE_NORMAL), 0.0f); }
TEST_F(AebDecelTest, TC_AEB_DEC_RA_02)   { EXPECT_FLOAT_EQ(call(AEB_MODE_ALERT ), 0.0f); }

TEST_F(AebDecelTest, TC_AEB_DEC_RA_03)
{
    ttc = makeTtc(1.0f,2.0f,3.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_04)
{
    ttc = makeTtc(2.5f,2.0f,3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_05)
{
    ttc = makeTtc(0.0f,1.0f,2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_06)
{
    ttc = makeTtc(3.0f,2.0f,3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_07)
{
    ttc = makeTtc(2.0f,2.0f,3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_08)
{
    ttc = makeTtc(1.0f,2.0f,3.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_09)
{
    ttc.TTC = NAN;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_10)
{
    ttc.TTC = -1.0f;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_11)
{
    ttc.TTC_Brake = 0.0f;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_12)
{
    ttc = makeTtc(0.5f,1e-6f,1.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_13)
{
    ttc.TTC_Brake = NAN;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_14)
{
    ttc.TTC_Brake = -1.0f;
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_15)
{
    EXPECT_FLOAT_EQ(calculate_decel_for_aeb(AEB_MODE_BRAKE, nullptr), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_16)
{
    ttc = makeTtc(0.0f,0.0f,0.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), 0.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_17)
{
    ttc = makeTtc(1.0f,2.0f,3.0f);
    EXPECT_NEAR(call(AEB_MODE_BRAKE), -5.0f, EPS_F);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_18)
{
    ttc = makeTtc(-0.001f,1.0f,2.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -10.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_19)
{
    ttc = makeTtc(1.8f,2.0f,3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

TEST_F(AebDecelTest, TC_AEB_DEC_RA_20)
{
    ttc = makeTtc(1.8f,2.0f,3.0f);
    EXPECT_FLOAT_EQ(call(AEB_MODE_BRAKE), -2.0f);
}

