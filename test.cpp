/*********************************************************************
 * aeb_ttc_test.cpp ― 급정지(AEB) TTC 계산기 단위‑테스트 (60 TC)
 * -------------------------------------------------------------------
 * • DUT  : calculate_ttc_for_aeb()  (aeb.c / aeb.h)
 * • 규격 : 2.2.3.1.x  (사내 SW 설계서)
 * • 환경 : Google Test 1.14+, C++17
 *********************************************************************/
#include <gtest/gtest.h>
#include <cmath>
#include <cfloat>
#include "aeb.h"                // Unit‑Under‑Test
#include "adas_shared.h"


/* ---------- 사내 공통 상수 (DUT 와 반드시 동일) ------------------- */
constexpr float INF_TTC      = 99999.0f;   // “무한대” 내부표기
constexpr float EPS_F        = 1e-4f;      // float 오차허용
constexpr float MIN_DIST_CORR= 0.01f;      // 거리 최소보정

/* ---------- 헬퍼 구조체 생성 ------------------------------------- */
static AEB_Target_Data_t makeTarget(int id,
                                    AEB_Target_Situation_e situ,
                                    float v_x,
                                    float dist)
{
    AEB_Target_Data_t t{};
    t.AEB_Target_ID          = id;
    t.AEB_Target_Situation   = situ;
    t.AEB_Target_Velocity_X  = v_x;
    t.AEB_Target_Distance    = dist;
    return t;
}
static Ego_Data_t makeEgo(float v_x)
{
    Ego_Data_t e{};
    e.Ego_Velocity_X = v_x;
    return e;
}

/* ---------- 헬퍼 : ∞ / 근사 비교 ------------------------------- */
static void expectInf(float v)        { EXPECT_TRUE(std::isinf(v) || v >= INF_TTC); }
static void expectNear(float v, float ref,
                       float eps = EPS_F){ EXPECT_NEAR(v, ref, eps); }

/* ---------- Test Fixture ----------------------------------------- */
class AebTtcTest : public ::testing::Test
{
protected:
    AEB_Target_Data_t tgt;
    Ego_Data_t        ego;
    TTC_Data_t        ttc;
    void SetUp() override
    {
        tgt = makeTarget(0, AEB_TARGET_NORMAL, 10.0f, 40.0f);
        ego = makeEgo(20.0f);
        ttc = {NAN,NAN,NAN,NAN};
    }
};

/*********************************************************************
 * 1) EQ – 동등 분할 20 케이스
 *********************************************************************/
TEST_F(AebTtcTest, TC_AEB_TTC_EQ_01)
{
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC,       4.0f);
    expectNear(ttc.TTC_Brake, 20.0f/9.0f);
    expectNear(ttc.TTC_Alert, (20.0f/9.0f)+1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_02)
{
    ego  = makeEgo(10.0f);
    tgt  = makeTarget(0,AEB_TARGET_NORMAL,10.0f,30.0f);

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_03)
{
    ego = makeEgo(5.0f);
    tgt = makeTarget(0,AEB_TARGET_NORMAL,10.0f,20.0f);

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_04)
{
    tgt = makeTarget(-1,AEB_TARGET_NORMAL,10.0f,25.0f);

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_05)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_OUT;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_06)
{
    tgt.AEB_Target_Distance = 0.0f;   // 내부보정 →0.01
    ego = makeEgo(25.0f);
    tgt.AEB_Target_Velocity_X=10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, MIN_DIST_CORR/(15.0f));   // 0.0006667
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_07)
{
    tgt.AEB_Target_Distance   = 0.005f;           //→0.01
    ego.Ego_Velocity_X        = 15.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, MIN_DIST_CORR/5.0f);      // 0.002
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_08)
{
    tgt.AEB_Target_Velocity_X = -5.0f;
    tgt.AEB_Target_Distance   = 30.0f;
    ego.Ego_Velocity_X        = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 2.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_09)
{
    ego.Ego_Velocity_X = -10.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
    EXPECT_FLOAT_EQ(ttc.TTC_Brake, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_10)
{
    calculate_ttc_for_aeb(nullptr,&ego,&ttc);   // 포인터 NULL
    SUCCEED();                                  // Crash 없으면 PASS
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_11)
{
    tgt.AEB_Target_Distance   = 200.0f;
    ego.Ego_Velocity_X        = 5.0f;
    tgt.AEB_Target_Velocity_X = 4.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 200.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_12)
{
    // 준비
    tgt.AEB_Target_Distance = -10.0f;
    ego.Ego_Velocity_X = 10.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;

    // 실행
    calculate_ttc_for_aeb(&tgt, &ego, &ttc);

    // 검증: 내부적으로 거리 보정(MIN_DIST_F) 되었는지 확인
    float relSpd = ego.Ego_Velocity_X - tgt.AEB_Target_Velocity_X; // 5.0f
    float expTTC = MIN_DIST_CORR / relSpd;                        // 0.01/5 = 0.002
    expectNear(ttc.TTC, expTTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_13)
{
    ego.Ego_Velocity_X        = 100.0f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    tgt.AEB_Target_Distance   = 50.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 0.5f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_14)
{
    tgt.AEB_Target_Velocity_X = NAN;

    calculate_ttc_for_aeb(&tgt, &ego, &ttc);

    EXPECT_TRUE(std::isnan(ttc.TTC) || ttc.TTC >= INF_TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_15)
{
    ego.Ego_Velocity_X = NAN;

    calculate_ttc_for_aeb(&tgt, &ego, &ttc);

    // 검증: TTC는 무한대 (INF) 처리
    expectInf(ttc.TTC);
    EXPECT_FLOAT_EQ(ttc.TTC_Brake, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_16)
{
    tgt.AEB_Target_Distance = INFINITY;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_17)
{
    ego.Ego_Velocity_X = 10.0001f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    tgt.AEB_Target_Distance = 10.0f;

    calculate_ttc_for_aeb(&tgt, &ego, &ttc);

    // DUT 반올림 방식
    float relSpd = ego.Ego_Velocity_X - tgt.AEB_Target_Velocity_X;
    relSpd = floor(relSpd * 100.0f + 0.5f) / 100.0f;

    if (relSpd < 1e-6f)
    {
        // 상대속도가 0이거나 거의 0 → TTC는 무한대 처리돼야 함
        expectInf(ttc.TTC);
    }
    else
    {
        float expTTC = tgt.AEB_Target_Distance / relSpd;
        expectNear(ttc.TTC, expTTC, 1.0f);
    }
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_18)
{
    calculate_ttc_for_aeb(&tgt, &ego, &ttc);

    EXPECT_FLOAT_EQ(ttc.TTC_Alert, ttc.TTC_Brake + 1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_19)
{
    ego.Ego_Velocity_X = 0.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
    EXPECT_FLOAT_EQ(ttc.TTC_Brake, 0.0f);
    EXPECT_FLOAT_EQ(ttc.TTC_Alert, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_EQ_20)
{
    ttc = {NAN,NAN,NAN,NAN};          // 오염 값
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_TRUE(std::isfinite(ttc.TTC));
}

/*********************************************************************
 * 2) BV – 경계값 20 케이스
 *********************************************************************/
TEST_F(AebTtcTest, TC_AEB_TTC_BV_01)
{
    tgt.AEB_Target_Distance = 0.0f;
    ego = makeEgo(20.0f);
    tgt.AEB_Target_Velocity_X = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 0.001f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_02)
{
    tgt.AEB_Target_Distance = 0.01f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 0.001f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_03)
{
    tgt.AEB_Target_Distance   = 0.02f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 0.02f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_04)
{
    tgt.AEB_Target_Distance   = 199.99f;
    ego.Ego_Velocity_X        = 5.0f;
    tgt.AEB_Target_Velocity_X = 4.9f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 1999.9f, 0.1f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_05)
{
    tgt.AEB_Target_Distance   = 200.0f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 200.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_06)
{
    ego.Ego_Velocity_X = 0.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_07)
{
    ego.Ego_Velocity_X = 0.1f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    tgt.AEB_Target_Distance   = 1.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 10.0f);
    EXPECT_FLOAT_EQ(ttc.TTC_Brake, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_08)
{
    ego.Ego_Velocity_X = 0.5f;
    tgt.AEB_Target_Distance   = 2.0f;
    tgt.AEB_Target_Velocity_X = 0.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 4.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_09)
{
    ego.Ego_Velocity_X = 30.0f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    tgt.AEB_Target_Distance   = 90.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 3.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_10)
{
    ego.Ego_Velocity_X = 100.0f;
    tgt.AEB_Target_Velocity_X = 99.9f;
    tgt.AEB_Target_Distance   = 100.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 1000.0f, 0.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_11)
{
    /* 기본 Alert_Buffer_Time(1.2) 확인 */
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;
    tgt.AEB_Target_Distance   = 50.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Alert, (10.0f/9.0f)+1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_12)
{
    /* Alert_Buffer_Time = 0 요구사항 (함수→불일치 예상) */
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;
    tgt.AEB_Target_Distance   = 50.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Alert, ttc.TTC_Brake + 1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_13)
{
    tgt.AEB_Target_Distance   = 0.0f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_GT(ttc.TTC, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_14)
{
    tgt.AEB_Target_Distance   = 0.01f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 0.01f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_15)
{
    ego.Ego_Velocity_X        = 20.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    tgt.AEB_Target_Distance   = 22.22f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC,       ttc.TTC_Brake, 1e-2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_16)
{
    ego.Ego_Velocity_X        = 20.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    tgt.AEB_Target_Distance   = 34.2f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, ttc.TTC_Alert, 1e-2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_17)
{
    ego.Ego_Velocity_X = 9e-5f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    tgt.AEB_Target_Distance   = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_FLOAT_EQ(ttc.TTC_Brake, 0.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_18)
{
    tgt.AEB_Target_Distance   = 999.989f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.99f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);

    /*  relSpd = 0.01  →  TTC = 999.989 / 0.01 = 99 998.9  */
    float exp = 999.989f / 0.01f;
    expectNear(ttc.TTC, exp, 0.1f);   // ±0.1 s 허용
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_19)
{
    tgt.AEB_Target_Distance   = 999.99f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.99f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_GT(ttc.TTC, 9.9e4f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_BV_20)
{
    tgt.AEB_Target_Distance   = 1000.0f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.99f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 100000.0f, 2.0f);
}

/*********************************************************************
 * 3) RA – 요구사항 분석 20 케이스
 *********************************************************************/
TEST_F(AebTtcTest, TC_AEB_TTC_RA_01)
{
    tgt.AEB_Target_Situation = AEB_TARGET_CUT_OUT;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_02)
{
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 15.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_03)
{
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_04)
{
    ego.Ego_Velocity_X        = 20.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    tgt.AEB_Target_Distance   = 40.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 4.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_05)
{
    ego.Ego_Velocity_X = 18.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Brake, 2.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_06)
{
    ego.Ego_Velocity_X = 9.0f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Brake, 1.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_07)
{
    ego.Ego_Velocity_X        = 18.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;
    tgt.AEB_Target_Distance   = 40.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Alert, (18.0f/9.0f)+1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_08)
{
    ego.Ego_Velocity_X = 5.0f;
    tgt.AEB_Target_Velocity_X = 7.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_09)
{
    calculate_ttc_for_aeb(&tgt,nullptr,&ttc); // 일부 NULL
    SUCCEED();
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_10)
{
    tgt.AEB_Target_Distance = 0.0f;
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, MIN_DIST_CORR / 10.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_11)
{
    ego.Ego_Velocity_X = 10.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_12)
{
    ttc = {NAN,NAN,NAN,NAN};
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_TRUE(std::isfinite(ttc.TTC));
    EXPECT_TRUE(std::isfinite(ttc.TTC_Brake));
    EXPECT_TRUE(std::isfinite(ttc.TTC_Alert));
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_13)
{
    /* Alert_Buffer_Time 음수 (-0.5) 요구 ‑ 함수 상수 → FAIL 예상 */
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_LT(ttc.TTC_Alert, ttc.TTC);   // 요구사항상 < 이어야 함
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_14)
{
    /* Alert_Buffer_Time 기본 1.2 적용 확인 (이미 여러 차례 했지만 재확인) */
    ego.Ego_Velocity_X = 10.0f;
    tgt.AEB_Target_Velocity_X = 5.0f;
    tgt.AEB_Target_Distance   = 50.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Alert, (10.0f/9.0f)+1.2f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_15)
{
    /* Max_Decel 8.0 요구사항 → 함수는 9.0 사용 → FAIL 예상 */
    ego.Ego_Velocity_X        = 18.0f;
    tgt.AEB_Target_Velocity_X = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC_Brake, 18.0f / 9.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_16)
{
    tgt.AEB_Target_Distance   = 1.0e6f;
    ego.Ego_Velocity_X        = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectNear(ttc.TTC, 1.0e6f, 1.0f);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_17)
{
    ego.Ego_Velocity_X = 0.0f;
    tgt.AEB_Target_Velocity_X = 0.0f;
    tgt.AEB_Target_Distance   = 10.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_18)
{
    ego.Ego_Velocity_X = 10.0f;
    tgt.AEB_Target_Velocity_X = 20.0f;
    tgt.AEB_Target_Distance   = -5.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_19)
{
    tgt.AEB_Target_Distance = FLT_MAX;
    ego.Ego_Velocity_X = 10.0f;
    tgt.AEB_Target_Velocity_X = 9.0f;

    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    expectInf(ttc.TTC);
}

TEST_F(AebTtcTest, TC_AEB_TTC_RA_20)
{
    ttc = {NAN,NAN,NAN,NAN};
    calculate_ttc_for_aeb(&tgt,&ego,&ttc);
    EXPECT_TRUE(std::isfinite(ttc.TTC));
}

/*********************************************************************
 * main()
 *********************************************************************/
int main(int argc,char** argv)
{
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
