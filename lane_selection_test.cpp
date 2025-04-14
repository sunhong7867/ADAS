#include <gtest/gtest.h>
#include <iostream>
#include <cmath>
#include <cstring>

extern "C" {
  #include "lane_selection.h"
  #include "adas_shared.h"
}

// 명확한 결과 출력을 위한 함수
void PrintLaneResult(const LaneSelectOutput_t& out) {
    std::cout << "=== Lane Selection Output ===\n";
    std::cout << "LS_Is_Curved_Lane       = " << (out.LS_Is_Curved_Lane ? "true" : "false") << "\n";
    std::cout << "LS_Lane_Type            = " << (out.LS_Lane_Type == LANE_TYPE_CURVE ? "CURVE" : "STRAIGHT") << "\n";
    std::cout << "LS_Curve_Transition_Flag= " << (out.LS_Curve_Transition_Flag ? "true" : "false") << "\n";
    std::cout << "LS_Heading_Error        = " << out.LS_Heading_Error << "\n";
    std::cout << "LS_Is_Within_Lane       = " << (out.LS_Is_Within_Lane ? "true" : "false") << "\n";
    std::cout << "LS_Is_Changing_Lane     = " << (out.LS_Is_Changing_Lane ? "true" : "false") << "\n";
    std::cout << "=============================\n\n";
}

// (1) Lane Type and Curve Test
TEST(LaneSelectionTest, LaneTypeAndCurveTest) {
    LaneData_t laneData = {
        .Lane_Type = LANE_TYPE_CURVE,
        .Lane_Curvature = 500.0f,
        .Next_Lane_Curvature = 500.0f,
        .Lane_Offset = 0.0f,
        .Lane_Heading = 0.0f,
        .Lane_Width = 3.5f,
        .Lane_Change_Status = LANE_CHANGE_KEEP
    };
    EgoData_t egoData = { .Ego_Heading = 0.0f };
    LaneSelectOutput_t out{};

    int ret = LaneSelection_Update(&laneData, &egoData, &out);
    PrintLaneResult(out);

    EXPECT_EQ(ret, 0);
    EXPECT_EQ(out.LS_Lane_Type, LANE_TYPE_CURVE);
    EXPECT_TRUE(out.LS_Is_Curved_Lane);
    EXPECT_FALSE(out.LS_Curve_Transition_Flag);
}

// (2) Curvature Transition Test
TEST(LaneSelectionTest, CurvatureTransitionTest) {
    LaneData_t laneData = {
        .Lane_Type = LANE_TYPE_CURVE,
        .Lane_Curvature = 600.0f,
        .Next_Lane_Curvature = 1200.0f,
        .Lane_Offset = 0.0f,
        .Lane_Heading = 0.0f,
        .Lane_Width = 3.5f,
        .Lane_Change_Status = LANE_CHANGE_KEEP
    };
    EgoData_t egoData = { .Ego_Heading = 0.0f };
    LaneSelectOutput_t out{};

    int ret = LaneSelection_Update(&laneData, &egoData, &out);
    PrintLaneResult(out);

    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(out.LS_Curve_Transition_Flag);
}

// (3) Heading and Offset Test
TEST(LaneSelectionTest, HeadingAndOffsetTest) {
    LaneData_t laneData = {
        .Lane_Type = LANE_TYPE_STRAIGHT,
        .Lane_Curvature = 900.0f,
        .Next_Lane_Curvature = 900.0f,
        .Lane_Offset = 1.0f,
        .Lane_Heading = 10.0f,
        .Lane_Width = 3.5f,
        .Lane_Change_Status = LANE_CHANGE_KEEP
    };
    EgoData_t egoData = { .Ego_Heading = 200.0f };
    LaneSelectOutput_t out{};

    int ret = LaneSelection_Update(&laneData, &egoData, &out);
    PrintLaneResult(out);

    EXPECT_EQ(ret, 0);
    EXPECT_NEAR(out.LS_Heading_Error, -170.0f, 0.01f);
    EXPECT_TRUE(out.LS_Is_Within_Lane);
}

// (4) Lane Change Status Test
TEST(LaneSelectionTest, LaneChangeStatusTest) {
    LaneData_t laneData = {
        .Lane_Type = LANE_TYPE_STRAIGHT,
        .Lane_Curvature = 1000.0f,
        .Next_Lane_Curvature = 1000.0f,
        .Lane_Offset = 0.0f,
        .Lane_Heading = 0.0f,
        .Lane_Width = 3.5f,
        .Lane_Change_Status = LANE_CHANGE_CHANGING
    };
    EgoData_t egoData = { .Ego_Heading = 0.0f };
    LaneSelectOutput_t out{};

    int ret = LaneSelection_Update(&laneData, &egoData, &out);
    PrintLaneResult(out);

    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(out.LS_Is_Changing_Lane);
}

// (5) Invalid Input Test
TEST(LaneSelectionTest, InvalidInputTest) {
    LaneData_t laneData{};
    EgoData_t egoData{};
    LaneSelectOutput_t out{};

    int ret1 = LaneSelection_Update(nullptr, &egoData, &out);
    int ret2 = LaneSelection_Update(&laneData, nullptr, &out);
    int ret3 = LaneSelection_Update(&laneData, &egoData, nullptr);

    std::cout << "[InvalidInputTest] ret1 (pLaneData=NULL) = " << ret1 << std::endl;
    std::cout << "[InvalidInputTest] ret2 (pEgoData=NULL)  = " << ret2 << std::endl;
    std::cout << "[InvalidInputTest] ret3 (pLaneOut=NULL)  = " << ret3 << std::endl;

    EXPECT_EQ(ret1, -1);
    EXPECT_EQ(ret2, -1);
    EXPECT_EQ(ret3, -1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
