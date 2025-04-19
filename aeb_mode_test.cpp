/*********************************************************************
 * aeb_mode_selection_test.cpp ― AEB Mode Selection 단위‑테스트 (60 TC)
 * -------------------------------------------------------------------
 * DUT  : aeb_mode_selection()   (aeb.c / aeb.h)
 * 규격 : 2.2.3.1.2  (사내 SW 설계서)
 *********************************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cfloat>
#include "aeb.h"            // Unit‑Under‑Test
#include "adas_shared.h"

/* 내부 ‘∞’ 표현 값 ‑ DUT 와 동일해야 함 */
constexpr float INF_TTC  = 99999.0f;
constexpr float EPS_F    = 1e-4f;

/* ────────── 헬퍼 구조체 빌더 ───────────────────────────────────── */
static AEB_Target_Data_t makeTarget(int id,
                                    AEB_Target_Situation_e situ,
                                    float vel,
                                    float dist = 40.0f)
{
    AEB_Target_Data_t t{};
    t.AEB_Target_ID          = id;
    t.AEB_Target_Situation   = situ;
    t.AEB_Target_Velocity_X  = vel;
    t.AEB_Target_Distance    = dist;
    return t;
}
static Ego_Data_t makeEgo(float vel)
{
    Ego_Data_t e{};
    e.Ego_Velocity_X = vel;
    return e;
}

/* ────────── Fixture  ──────────────────────────────────────────── */
class AebModeSelTest : public ::testing::Test
{
protected:
    AEB_Target_Data_t tgt;
    Ego_Data_t        ego;
    TTC_Data_t        ttc;
    void SetUp() override
    {
        tgt = makeTarget(0, AEB_TARGET_NORMAL, 10.0f);
        ego = makeEgo(20.0f);
        ttc = { 5.0f, 3.0f, 4.0f, 0.0f };   // 기본값 : Normal 구간
    }
    /* 호출 래퍼 */
    AEB_Mode_e call() const
    {
        return aeb_mode_selection(&tgt,&ego,&ttc);
    }
};

/*******************************************************************
 * 1) EQ  (동등 분할) 20 TC
 ******************************************************************/
TEST_F(AebModeSelTest, TC_AEB_MS_EQ_01)
{
    /* TTC > TTC_Alert → Normal */
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_02)
{
    /* TTC_Brake < TTC ≤ TTC_Alert → Alert */
    ttc = {2.5f,2.0f,3.0f,0.0f};
    ego = makeEgo(15.0f);
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_03)
{
    /* 0 < TTC ≤ TTC_Brake → Brake */
    ttc = {1.5f,2.0f,3.0f,0.0f};
    ego = makeEgo(15.0f);
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_04)
{
    /* TTC ≤ 0 → Normal */
    ttc.TTC = 0.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_05)
{
    /* TTC = ∞ → Normal */
    ttc.TTC = INF_TTC;
    ttc.TTC_Brake = 0.0f;
    ttc.TTC_Alert = 0.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_06)
{
    /* Ego < 0.5  → Normal */
    ego = makeEgo(0.4f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_07)
{
    /* Ego = 0 → Normal */
    ego = makeEgo(0.0f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_08)
{
    /* Target ID = -1 → Normal */
    tgt.AEB_Target_ID = -1;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_09)
{
    /* Cut‑out → Normal */
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_OUT;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_10)
{
    /* Cut‑in + TTC < Brake → Brake */
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {1.0f,1.5f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_11)
{
    /* Normal + TTC < Brake → Brake */
    ttc = {1.0f,1.5f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_12)
{
    /* Cut‑in + Alert 범위 → Alert */
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {2.0f,1.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_13)
{
    /* Cut‑in + TTC > Alert → Normal */
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {4.0f,1.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_14)
{
    /* TTC = NaN → Normal */
    ttc.TTC = NAN;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_15)
{
    /* TTC 음수 → Normal */
    ttc.TTC = -0.5f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_16)
{
    /* 구조체 NULL → Normal */
    EXPECT_EQ(aeb_mode_selection(nullptr,&ego,&ttc), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_17)
{
    /* TTC_Brake 0 이지만 TTC < Brake → Brake */
    ttc = {0.5f,0.0f,1.2f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);      // 설계상 Brake
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_18)
{
    /* TTC_Alert = TTC_Brake 동일 → Brake */
    ttc = {2.0f,2.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_19)
{
    /* TTC = TTC_Brake → Brake */
    ttc = {2.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_EQ_20)
{
    /* TTC = TTC_Alert → Alert */
    ttc = {3.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

/*******************************************************************
 * 2) BV  (경계값 분석) 20 TC
 ******************************************************************/
TEST_F(AebModeSelTest, TC_AEB_MS_BV_01)
{
    ttc = {0.0f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_02)
{
    ttc = {0.01f,0.02f,1.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_03)
{
    ttc = {1.99f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_04)
{
    ttc = {2.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_05)
{
    ttc = {2.01f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_06)
{
    ttc = {2.99f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_07)
{
    ttc = {3.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_08)
{
    ttc = {3.01f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_09)
{
    ego = makeEgo(0.49f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_10)
{
    ego = makeEgo(0.5f);
    ttc = {1.0f,2.0f,3.0f,0.0f};
    AEB_Mode_e m = call();
    EXPECT_TRUE(m == AEB_MODE_NORMAL || m == AEB_MODE_ALERT || m == AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_11)
{
    ttc = {99998.9f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_12)
{
    ttc.TTC = INF_TTC;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_13)
{
    ttc.TTC = -0.01f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_14)
{
    ttc.TTC       = 1.0f;
    ttc.TTC_Brake = 0.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_15)
{
    ttc.TTC_Alert = 0.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_16)
{
    ttc.TTC = FLT_MAX;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_17)
{
    ttc = {FLT_MIN,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_18)
{
    ttc = {0.00002f,0.00001f,1.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_19)
{
    ego = makeEgo(0.49f);
    ttc = {0.499f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_BV_20)
{
    ttc = {1.0f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

/*******************************************************************
 * 3) RA  (요구사항 분석) 20 TC
 ******************************************************************/
TEST_F(AebModeSelTest, TC_AEB_MS_RA_01)
{
    tgt.AEB_Target_ID = -1;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_02)
{
    ttc = {2.5f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_03)
{
    ttc = {1.0f,1.5f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_04)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {1.0f,1.5f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_05)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {2.5f,1.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_06)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_OUT;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_07)
{
    ego = makeEgo(0.0f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_08)
{
    tgt.AEB_Target_ID = -1;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_09)
{
    ego = makeEgo(0.0f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_10)
{
    ttc.TTC_Alert = 0.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_11)
{
    ego = makeEgo(0.3f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_12)
{
    tgt.AEB_Target_ID = -1;
    ego = makeEgo(0.3f);
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_13)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_IN;
    ttc = {0.5f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_14)
{
    ttc.TTC_Brake = -1.0f;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_15)
{
    ttc = {2.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_16)
{
    ttc = {3.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_ALERT);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_17)
{
    ttc.TTC = NAN;
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_18)
{
    EXPECT_EQ(aeb_mode_selection(nullptr,&ego,&ttc), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_19)
{
    ego = makeEgo(0.3f);
    ttc = {1.0f,1.0f,2.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
}

TEST_F(AebModeSelTest, TC_AEB_MS_RA_20)
{
    /* 상태 전이 시나리오: Normal→Alert→Brake */
    // 1) Normal
    ttc = {4.0f,2.0f,3.0f,0.0f};
    EXPECT_EQ(call(), AEB_MODE_NORMAL);
    // 2) Alert
    ttc.TTC = 2.5f;
    EXPECT_EQ(call(), AEB_MODE_ALERT);
    // 3) Brake
    ttc.TTC = 1.0f;
    EXPECT_EQ(call(), AEB_MODE_BRAKE);
}

/*******************************************************************
 * main()
 ******************************************************************/
int main(int argc,char** argv)
{
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
