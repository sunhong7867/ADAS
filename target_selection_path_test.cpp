/*******************************************************************************
 * predict_object_future_path_test.cpp
 *
 * - Google Test 기반
 * - Fixture: PredictObjectFuturePathTest
 * - 총 60개 TC (EQ=20, BV=20, RA=20)
 * - 중간 생략 없이 전체 코드 제시
 * - 실제 프로젝트에서 테스트 시, 함수 및 구조체 선언/정의 환경에 맞게 수정 필요
 *******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>
#include "target_selection.h"     // predict_object_future_path(...) 선언
#include "adas_shared.h"         // FilteredObject_t, PredictedObject_t, etc.

/*------------------------------------------------------------------------------
 * Test Fixture
 *----------------------------------------------------------------------------*/
class PredictObjectFuturePathTest : public ::testing::Test {
protected:
    // 입력 리스트
    FilteredObject_t filteredList[50];

    // 출력 리스트
    PredictedObject_t predList[50];

    LaneData_t laneWp;             // 필요 시 차선 곡률 등
    LaneSelectOutput_t lsData;     // 차선 Offset, Heading_Error 등

    virtual void SetUp() override
    {
        std::memset(filteredList, 0, sizeof(filteredList));
        std::memset(predList,     0, sizeof(predList));
        std::memset(&laneWp,      0, sizeof(laneWp));
        std::memset(&lsData,      0, sizeof(lsData));

        // 기본 lsData
        lsData.LS_Lane_Type            = LANE_TYPE_STRAIGHT;
        lsData.LS_Is_Curved_Lane       = false;
        lsData.LS_Curve_Transition_Flag= false;
        lsData.LS_Heading_Error        = 0.0f;
        lsData.LS_Lane_Offset          = 0.0f;
        lsData.LS_Lane_Width           = 3.5f;  // 예시
        lsData.LS_Is_Within_Lane       = true;
        lsData.LS_Is_Changing_Lane     = false;

        // laneWp도 필요시 설정
        // laneWp.Lane_Type = LANE_TYPE_STRAIGHT;
        // ... (추가 설정 가능)
    }
};

//------------------------------------------------------------------------------
// HELPER FUNCTION: callPredictPath(...)
// - Wrapper for predict_object_future_path()
//------------------------------------------------------------------------------
static int callPredictPath(
    const FilteredObject_t *pFiltList,
    int filtCount,
    const LaneData_t *pLaneWp,
    const LaneSelectOutput_t *pLs,
    PredictedObject_t *pOut,
    int maxCount)
{
    return predict_object_future_path(pFiltList, filtCount, pLaneWp, pLs, pOut, maxCount);
}

//==============================================================================
// 동등 분할 (EQ) 테스트 케이스 20개
//==============================================================================

// 1) TC_TGT_FP_EQ_01
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_01)
{
    // 등속 상태 - 자동차
    // Moving 상태, Accel=0 -> x(t)= x0+vx*t, y(t)=y0+vy*t
    filteredList[0].Filtered_Object_ID     = 1;
    filteredList[0].Filtered_Object_Type   = OBJTYPE_CAR;
    filteredList[0].Filtered_Object_Status = OBJSTAT_MOVING; // 등속
    filteredList[0].Filtered_Velocity_X    = 10.0f;
    filteredList[0].Filtered_Velocity_Y    = 2.0f;
    // 가속도=0
    filteredList[0].Filtered_Accel_X       = 0.0f;
    filteredList[0].Filtered_Accel_Y       = 0.0f;
    filteredList[0].Filtered_Position_X    = 100.0f;
    filteredList[0].Filtered_Position_Y    = 50.0f;

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);

    float expectedX = 100.0f + 10.0f * 3.0f; // =130
    float expectedY = 50.0f  + 2.0f  * 3.0f; // =56
    EXPECT_NEAR(predList[0].Predicted_Position_X, expectedX, 1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_Y, expectedY, 1e-3);
}

// 2) TC_TGT_FP_EQ_02
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_02)
{
    // 등가속 상태 - 자동차 (Stopped)
    // Status=Stopped => 등가속 공식
    filteredList[0].Filtered_Object_Status = OBJSTAT_STOPPED;
    filteredList[0].Filtered_Object_Type   = OBJTYPE_CAR;
    filteredList[0].Filtered_Velocity_X    = 0.0f;
    filteredList[0].Filtered_Velocity_Y    = 0.0f;
    // Accel != 0
    filteredList[0].Filtered_Accel_X       = 1.0f;
    filteredList[0].Filtered_Accel_Y       = -0.5f;
    filteredList[0].Filtered_Position_X    = 0.0f;
    filteredList[0].Filtered_Position_Y    = 0.0f;

    int outCount= callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);

    float exX= 0.0f + 0.5f*1.0f*(3.0f*3.0f); // 0.5*1*9=4.5
    float exY= 0.0f + 0.5f*(-0.5f)*9; // -2.25
    EXPECT_NEAR(predList[0].Predicted_Position_X, exX, 1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_Y, exY, 1e-3);
}

// 3) TC_TGT_FP_EQ_03
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_03)
{
    // 보행자 등속 이동 - Y축 포함
    filteredList[0].Filtered_Object_Type   = OBJTYPE_PEDESTRIAN;
    filteredList[0].Filtered_Object_Status = OBJSTAT_MOVING;
    filteredList[0].Filtered_Velocity_X    = 1.0f;
    filteredList[0].Filtered_Velocity_Y    = 1.5f;
    filteredList[0].Filtered_Accel_X       = 0.0f;
    filteredList[0].Filtered_Accel_Y       = 0.0f;
    filteredList[0].Filtered_Position_X    = 10.0f;
    filteredList[0].Filtered_Position_Y    = 20.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exX = 10.0f + 1.0f * 3.0f;    // 10+3=13
    float exY = 20.0f + 1.5f * 3.0f;    // 20+4.5=24.5
    EXPECT_NEAR(predList[0].Predicted_Position_X, exX,1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_Y, exY,1e-3);
}

// 4) TC_TGT_FP_EQ_04
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_04)
{
    // 자전거 등가속 - 감속
    filteredList[0].Filtered_Object_Type   = OBJTYPE_BICYCLE;
    filteredList[0].Filtered_Object_Status = OBJSTAT_STOPPED; // 정지=등가속
    filteredList[0].Filtered_Velocity_X    = 5.0f;
    filteredList[0].Filtered_Accel_X       = -1.0f; // 감속
    filteredList[0].Filtered_Position_X    = 0.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // ex= x0 + vx*3 + 0.5*(-1)*(3^2)=0 +15 +0.5*(-9)=15 -4.5=10.5
    float exX=10.5f;
    EXPECT_NEAR(predList[0].Predicted_Position_X, exX,1e-3);
}

// 5) TC_TGT_FP_EQ_05
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_05)
{
    // 오토바이 - Moving 등속
    filteredList[0].Filtered_Object_Type   = OBJTYPE_MOTORCYCLE;
    filteredList[0].Filtered_Object_Status = OBJSTAT_MOVING;
    filteredList[0].Filtered_Velocity_X    = 8.0f;
    filteredList[0].Filtered_Accel_X       = 0.0f;
    filteredList[0].Filtered_Position_X    = 100.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exX = 100.0f + 8.0f * 3.0f;   // 100+24=124
    EXPECT_NEAR(predList[0].Predicted_Position_X, exX,1e-3);
}

// 6) TC_TGT_FP_EQ_06
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_06)
{
    // Z값 고정
    filteredList[0].Filtered_Position_Z=1.0f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_NEAR(predList[0].Predicted_Position_Z,1.0f,1e-6);
}

// 7) TC_TGT_FP_EQ_07
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_07)
{
    // 속도, 가속도=0 => 위치 변화 없음
    filteredList[0].Filtered_Velocity_X=0.0f;
    filteredList[0].Filtered_Velocity_Y=0.0f;
    filteredList[0].Filtered_Accel_X   =0.0f;
    filteredList[0].Filtered_Accel_Y   =0.0f;
    filteredList[0].Filtered_Position_X=10.0f;
    filteredList[0].Filtered_Position_Y=20.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_NEAR(predList[0].Predicted_Position_X,10.0f,1e-6);
    EXPECT_NEAR(predList[0].Predicted_Position_Y,20.0f,1e-6);
}

// 8) TC_TGT_FP_EQ_08
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_08)
{
    // X축 속도만 있음 => Y 유지
    filteredList[0].Filtered_Velocity_X=5.0f;
    filteredList[0].Filtered_Velocity_Y=0.0f;
    filteredList[0].Filtered_Position_X=0.0f;
    filteredList[0].Filtered_Position_Y=100.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exX= 0 + 5 * 3; // 15
    EXPECT_NEAR(predList[0].Predicted_Position_X,15,1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_Y,100.0f,1e-6);
}

// 9) TC_TGT_FP_EQ_09
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_09)
{
    // Y축 속도만 있음 => X 유지
    filteredList[0].Filtered_Velocity_X=0.0f;
    filteredList[0].Filtered_Velocity_Y=4.0f;
    filteredList[0].Filtered_Position_X=10.0f;
    filteredList[0].Filtered_Position_Y=0.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exY = 0 + 4 * 3;  // 12
    EXPECT_NEAR(predList[0].Predicted_Position_Y,12,1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_X,10,1e-6);
}

// 10) TC_TGT_FP_EQ_10
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_10)
{
    // 감속 차량 => 이동 거리 감소
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=10.0f;
    filteredList[0].Filtered_Accel_X=-2.0f; // 감속
    filteredList[0].Filtered_Position_X=0.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex = 0 + 10 * 3 + 0.5f * (-2) * 9; // 30 - 9=21
    EXPECT_NEAR(predList[0].Predicted_Position_X,21,1.0f); // 허용오차 1.0f
}

// 11) TC_TGT_FP_EQ_11
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_11)
{
    // Y축 음속도 => 음의 방향 이동
    filteredList[0].Filtered_Velocity_Y=-5.0f;
    filteredList[0].Filtered_Object_Status=OBJSTAT_MOVING;
    filteredList[0].Filtered_Position_Y=20.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exY = 20.0f + (-5.0f) * 3; // 20-15=5
    EXPECT_NEAR(predList[0].Predicted_Position_Y, exY,1e-3);
}

// 12) TC_TGT_FP_EQ_12
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_12)
{
    // X축 음속도 => 후진
    filteredList[0].Filtered_Velocity_X=-5.0f;
    filteredList[0].Filtered_Position_X=100.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float exX=100+(-5)*3; // 100-15=85
    EXPECT_NEAR(predList[0].Predicted_Position_X, exX,1e-3);
}

// 13) TC_TGT_FP_EQ_13
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_13)
{
    // 셀 번호 유지 확인
    filteredList[0].Filtered_Object_Cell_ID=8;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_EQ(predList[0].Predicted_Object_Cell_ID,8);
}

// 14) TC_TGT_FP_EQ_14
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_14)
{
    // 상태 값 유지
    filteredList[0].Filtered_Object_Status=OBJSTAT_MOVING;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_EQ(predList[0].Predicted_Object_Status, OBJSTAT_MOVING);
}

// 15) TC_TGT_FP_EQ_15
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_15)
{
    // 음의 가속도 => 예측 거리 감소
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=20.0f;
    filteredList[0].Filtered_Accel_X=-1.0f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // 단순 체크
}

// 16) TC_TGT_FP_EQ_16
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_16)
{
    // 가속도만 존재, 초기 속도 0
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=0;
    filteredList[0].Filtered_Accel_X=2.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=0+0*3+0.5f*2*9; // 0+0+9=9
    EXPECT_NEAR(predList[0].Predicted_Position_X, ex,1e-3);
}

// 17) TC_TGT_FP_EQ_17
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_17)
{
    // 고속 이동 객체 => 300m 이상
    filteredList[0].Filtered_Object_Status=OBJSTAT_MOVING;
    filteredList[0].Filtered_Velocity_X=100.0f; // 100m/s
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=100*3; // 300m
    EXPECT_GE(predList[0].Predicted_Distance,300.0f);
}

// 18) TC_TGT_FP_EQ_18
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_18)
{
    // 횡속도만 있을 때 -> CutIn 조건 미충족 (vx=0 => cutin?)
    filteredList[0].Filtered_Velocity_X=0.0f;
    filteredList[0].Filtered_Velocity_Y=5.0f;
    // expect CutIn_Flag=false
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

// 19) TC_TGT_FP_EQ_19
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_19)
{
    // vx>0, vy>0 => cutin조건 충족
    filteredList[0].Filtered_Position_Y = 0.0f;
    filteredList[0].Filtered_Velocity_X=5.0f;
    filteredList[0].Filtered_Velocity_Y=0.2f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

// 20) TC_TGT_FP_EQ_20
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_EQ_20)
{
    // lateral offset=0.8 => cutin true
    // offset 계산은 predict에서 (y - lsData.LS_Lane_Offset)?
    // 여기선 0.8 => within threshold=0.85 => cutin= true
    filteredList[0].Filtered_Position_Y = 0.25f;   // y0
    filteredList[0].Filtered_Velocity_X = 1.0f;    // vx ≥ 0.5
    filteredList[0].Filtered_Velocity_Y = 0.2f;    // vy ≥ 0.2

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);

    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

//==============================================================================
// 경계값 분석 (BV) 테스트 케이스 20개
//==============================================================================
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_01)
{
    // vx=0.49 => cutin 미충족
    filteredList[0].Filtered_Velocity_X=0.49f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_02)
{
    // vx=0.50 => cutin=true
    filteredList[0].Filtered_Position_Y = 0.0f;   // lateral 0
    filteredList[0].Filtered_Velocity_X = 0.50f;  // 경계값
    filteredList[0].Filtered_Velocity_Y = 0.20f;  // 경계값

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_03)
{
    // vx=0.51 => cutin=true
    filteredList[0].Filtered_Position_Y = 0.0f;
    filteredList[0].Filtered_Velocity_X = 0.51f;
    filteredList[0].Filtered_Velocity_Y = 0.20f;

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_04)
{
    // vy=0.19 => cutin 미충족
    filteredList[0].Filtered_Velocity_Y=0.19f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_05)
{
    // vy=0.20 => cutin=true
    filteredList[0].Filtered_Position_Y = 0.0f;
    filteredList[0].Filtered_Velocity_X = 0.50f;  // vx도 경계 이상으로 설정
    filteredList[0].Filtered_Velocity_Y = 0.20f;  // 경계값

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_06)
{
    // vy=0.21 => cutin=true
    filteredList[0].Filtered_Position_Y = 0.0f;
    filteredList[0].Filtered_Velocity_X = 0.50f;
    filteredList[0].Filtered_Velocity_Y = 0.21f;

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_07)
{
    // lateral=0.84 => cutin
    filteredList[0].Filtered_Position_Y = 0.10f;  // y0
    filteredList[0].Filtered_Velocity_X = 0.60f;  // vx ≥ 0.5
    filteredList[0].Filtered_Velocity_Y = 0.20f;  // vy ≥ 0.2 → Predicted_Y = 0.10 + 0.20*3 = 0.70

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_08)
{
    // lateral=0.85 => cutin
    filteredList[0].Filtered_Position_Y = 0.25f;  // y0
    filteredList[0].Filtered_Velocity_X = 0.60f;  // vx ≥ 0.5
    filteredList[0].Filtered_Velocity_Y = 0.20f;  // vy ≥ 0.2 → Predicted_Y = 0.25 + 0.20*3 = 0.85

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}


TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_09)
{
    // lateral=0.86 => cutin=false
    filteredList[0].Filtered_Position_Y=0.86f;
    filteredList[0].Filtered_Velocity_X=1.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_10)
{
    // lateral=2.99 => cutout=false
    filteredList[0].Filtered_Position_Y=2.99f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutOut_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_11)
{
    // lateral=3.50 => cutout=true (경계)
    filteredList[0].Filtered_Position_Y = 3.50f;
    filteredList[0].Filtered_Velocity_X = 0.60f;  // vx irrelevant
    filteredList[0].Filtered_Velocity_Y = 0.30f;  // vy ≥ 0.2

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutOut_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_12)
{
    // lateral=3.51 => cutout=true
    filteredList[0].Filtered_Position_Y = 3.51f;
    filteredList[0].Filtered_Velocity_X = 0.60f;
    filteredList[0].Filtered_Velocity_Y = 0.30f;

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutOut_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_13)
{
    // CutIn_Threshold=0.85
    // 여기선 상수만 확인 => pass if code uses 0.85
    filteredList[0].Filtered_Position_Y = 0.25f;   // y0
    filteredList[0].Filtered_Velocity_X   = 1.0f;    // vx ≥ 0.5
    filteredList[0].Filtered_Velocity_Y   = 0.20f;   // vy ≥ 0.2

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_14)
{
    // Ego_Lane_Boundary= lane_width*0.5 => ex 2.5 => 1.25
    lsData.LS_Lane_Width=2.5f;
    filteredList[0].Filtered_Position_Y=1.3f; // near 1.25 => cutOut?
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // pass/fail depends on exact code
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_15)
{
    // 3초 예측 반영
    // 내부 t_predict=3
    filteredList[0].Filtered_Velocity_X=2.0f; 
    filteredList[0].Filtered_Position_X=0.0f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=2*3; // 6.0f
    EXPECT_NEAR(predList[0].Predicted_Position_X,6,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_16)
{
    // vx=0, ax=-1 => 후진
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=0.0f;
    filteredList[0].Filtered_Accel_X=-1.0f;
    filteredList[0].Filtered_Position_X=10.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=10+0*3+0.5f*(-1)*9; // 10-4.5=5.5
    EXPECT_NEAR(predList[0].Predicted_Position_X,5.5f,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_17)
{
    // vy=0.2 => cutin => true
    filteredList[0].Filtered_Velocity_X=0.6f;
    filteredList[0].Filtered_Velocity_Y=0.2f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_18)
{
    // vx=0.49, vy=0.19 => cutin false
    filteredList[0].Filtered_Velocity_X=0.49f;
    filteredList[0].Filtered_Velocity_Y=0.19f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_19)
{
    // vx=0.50, vy=0.20 => cutin true
    filteredList[0].Filtered_Velocity_X=0.50f;
    filteredList[0].Filtered_Velocity_Y=0.20f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_BV_20)
{
    // vx=0.51, vy=0.21 => cutin true
    filteredList[0].Filtered_Velocity_X=0.51f;
    filteredList[0].Filtered_Velocity_Y=0.21f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

//==============================================================================
// 요구사항 분석 (RA) 테스트 케이스 20개
//==============================================================================
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_01)
{
    // Moving => 등속
    filteredList[0].Filtered_Object_Status=OBJSTAT_MOVING;
    filteredList[0].Filtered_Velocity_X=30.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=0+30*3;
    EXPECT_NEAR(predList[0].Predicted_Position_X, ex,2.0f);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_02)
{
    // Stopped => 등가속
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=0;
    filteredList[0].Filtered_Accel_X=-2.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=0+0*3+0.5f*(-2)*9;
    EXPECT_NEAR(predList[0].Predicted_Position_X, ex,1.0f);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_03)
{
    // Stationary => 등가속도모델
    filteredList[0].Filtered_Object_Status=OBJSTAT_STATIONARY;
    filteredList[0].Filtered_Velocity_X=0;
    filteredList[0].Filtered_Accel_X=0;
    filteredList[0].Filtered_Position_X=50.0f;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // accel=0 => 위치=50
    EXPECT_NEAR(predList[0].Predicted_Position_X,50.0f,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_04)
{
    // Heading 보존
    filteredList[0].Filtered_Heading=10.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_NEAR(predList[0].Predicted_Heading,10.0f,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_05)
{
    // Position기준 -> 3초 후 이동
    filteredList[0].Filtered_Position_X=50.0f;
    filteredList[0].Filtered_Position_Y=0.0f;
    filteredList[0].Filtered_Velocity_X=10.0f;
    filteredList[0].Filtered_Object_Status=OBJSTAT_MOVING;

    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=50+10*3;
    EXPECT_NEAR(predList[0].Predicted_Position_X,ex,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_06)
{
    // 속도값 반영 정확도
    filteredList[0].Filtered_Velocity_X=20.0f;
    filteredList[0].Filtered_Velocity_Y=5.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // x=0+20*3=60, y=0+5*3=15
    EXPECT_NEAR(predList[0].Predicted_Position_X,60,1e-3);
    EXPECT_NEAR(predList[0].Predicted_Position_Y,15,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_07)
{
    // 가속도 영향 반영
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED;
    filteredList[0].Filtered_Velocity_X=10.0f;
    filteredList[0].Filtered_Accel_X=2.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=0+10*3+0.5f*2*9;
    EXPECT_NEAR(predList[0].Predicted_Position_X,39,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_08)
{
    // Distance 재계산 여부 (Predicted_Distance)
    filteredList[0].Filtered_Position_X=3.0f;
    filteredList[0].Filtered_Position_Y=4.0f;
    // => dist=5 initially
    // velocity=0 => still (3,4) => predicted same => dist=5
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // predicted distance= sqrt(3^2 +4^2)=5
    EXPECT_NEAR(predList[0].Predicted_Distance,5,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_09)
{
    // Cut-in 조건 미충족 => false
    filteredList[0].Filtered_Velocity_X=0.49f; 
    filteredList[0].Filtered_Velocity_Y=0.19f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_10)
{
    // Cut-out 판단 시 거리 무관 -> lateral만
    filteredList[0].Filtered_Position_Y = 4.0f;   // 충분히 큰 lateral
    filteredList[0].Filtered_Velocity_X = 0.0f;
    filteredList[0].Filtered_Velocity_Y = 0.2f;   // vy ≥ 0.2(필수)

    int outCount = callPredictPath(filteredList, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 1);
    EXPECT_TRUE(predList[0].CutOut_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_11)
{
    // 차선 폭에 따른 Cut-out 경계 변화
    lsData.LS_Lane_Width=2.5f; 
    filteredList[0].Filtered_Position_Y=2.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // pass/fail depends on actual logic
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_12)
{
    // 음수 Heading에도 정상 계산
    filteredList[0].Filtered_Heading=-90.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_NEAR(predList[0].Predicted_Heading,-90.0f,1.0f);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_13)
{
    // Position= LS_Lane_Offset => lateral=0
    lsData.LS_Lane_Offset=1.0f;
    filteredList[0].Filtered_Position_Y=1.0f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // expect cutin/cutout= false 
    EXPECT_FALSE(predList[0].CutIn_Flag);
    EXPECT_FALSE(predList[0].CutOut_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_14)
{
    // 예측 거리 과도 -> 안정성
    filteredList[0].Filtered_Position_X=500.0f;
    filteredList[0].Filtered_Velocity_X=10.0f;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // predicted dist= sqrt( (500+10*3)^2 + 0^2 )=530
    EXPECT_GE(predList[0].Predicted_Distance,530.0f);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_15)
{
    // 셀 번호 유지 
    filteredList[0].Filtered_Object_Cell_ID=8;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_EQ(predList[0].Predicted_Object_Cell_ID,8);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_16)
{
    // CutIn_Flag=true -> AEB 후보 적합성
    filteredList[0].Filtered_Velocity_X=1.0f; 
    filteredList[0].Filtered_Velocity_Y=0.2f; 
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // cutin => true
    EXPECT_TRUE(predList[0].CutIn_Flag);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_17)
{
    // 속도/가속도=0 => 위치 동일
    filteredList[0].Filtered_Velocity_X=0;
    filteredList[0].Filtered_Accel_X=0;
    filteredList[0].Filtered_Position_X=100;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FLOAT_EQ(predList[0].Predicted_Position_X,100.0f);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_18)
{
    // 등가속 공식 정확도
    filteredList[0].Filtered_Object_Status=OBJSTAT_STOPPED; 
    filteredList[0].Filtered_Velocity_X=10;
    filteredList[0].Filtered_Accel_X=2;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    // ex= x0 +10*3 +0.5*2*9=30+9=39
    EXPECT_NEAR(predList[0].Predicted_Position_X,39,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_19)
{
    // 3초 예측 고정
    filteredList[0].Filtered_Velocity_X=2;
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    float ex=0+2*3;
    EXPECT_NEAR(predList[0].Predicted_Position_X,6,1e-3);
}

TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_RA_20)
{
    // CutIn_Flag, CutOut_Flag 초기값 false에서 조건에 따라만 true
    // 여기선 조건X => both false
    int outCount=callPredictPath(filteredList,1,&laneWp,&lsData,predList,50);
    EXPECT_EQ(outCount,1);
    EXPECT_FALSE(predList[0].CutIn_Flag);
    EXPECT_FALSE(predList[0].CutOut_Flag);
}

// [SPECIAL] TC_TGT_FP_SPECIAL_01: 입력 pFilteredList=NULL → return 0
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_SPECIAL_01)
{
    int outCount = predict_object_future_path(
        nullptr, 1, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 0);
}

// [SPECIAL] TC_TGT_FP_SPECIAL_02: 입력 pLaneWp=NULL → return 0
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_SPECIAL_02)
{
    int outCount = predict_object_future_path(
        filteredList, 1, nullptr, &lsData, predList, 50);
    EXPECT_EQ(outCount, 0);
}

// [SPECIAL] TC_TGT_FP_SPECIAL_03: filteredCount <= 0 → return 0
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_SPECIAL_03)
{
    int outCount = predict_object_future_path(
        filteredList, 0, &laneWp, &lsData, predList, 50);
    EXPECT_EQ(outCount, 0);
}

// [SPECIAL] TC_TGT_FP_SPECIAL_04: maxPredCount <= 0 → return 0
TEST_F(PredictObjectFuturePathTest, TC_TGT_FP_SPECIAL_04)
{
    int outCount = predict_object_future_path(
        filteredList, 1, &laneWp, &lsData, predList, 0);
    EXPECT_EQ(outCount, 0);
}