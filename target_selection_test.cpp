#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

extern "C" {
  #include "adas_shared.h"
  #include "target_selection.h"
}

// 출력 함수들
void PrintFilteredObjects(const FilteredObject_t* list, int count) {
    std::cout << "Filtered Count: " << count << std::endl;
    for (int i = 0; i < count; ++i) {
        std::cout << "  [Filtered] ID=" << list[i].Filtered_Object_ID
                  << ", Distance=" << list[i].Filtered_Distance
                  << ", Cell=" << list[i].Filtered_Object_Cell_ID << std::endl;
    }
}

void PrintPredictedObject(const PredictedObject_t& obj) {
    std::cout << "[Predicted] ID=" << obj.Predicted_Object_ID
              << ", Pos_X=" << obj.Predicted_Position_X
              << ", Vel_X=" << obj.Predicted_Velocity_X
              << ", CutIn=" << obj.CutIn_Flag
              << ", CutOut=" << obj.CutOut_Flag << std::endl;
}

void PrintTarget(const ACC_Target_t& acc, const AEB_Target_t& aeb) {
    std::cout << "[ACC] Target_ID = " << acc.ACC_Target_ID << std::endl;
    std::cout << "[AEB] Target_ID = " << aeb.AEB_Target_ID << std::endl;
}

// (1) Object Filtering Test
TEST(TargetSelectionTest, SelectTargetFromObjectList_NormalFiltering) {
    ObjectData_t objList[3] = {
        { 1, OBJTYPE_CAR, 30.0f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, OBJSTAT_MOVING, 0 },
        { 2, OBJTYPE_PEDESTRIAN, 250.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 250.0f, OBJSTAT_MOVING, 0 },
        { 3, OBJTYPE_CAR, 50.0f, 2.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 10.0f, 50.0f, OBJSTAT_MOVING, 0 }
    };

    EgoData_t egoData = { 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0 };

    LaneSelectOutput_t lsData = {
        LANE_TYPE_STRAIGHT, false, false,
        0.0f, 0.0f, 3.5f, true, false
    };

    FilteredObject_t filteredList[5] = {0};
    int filteredCount = select_target_from_object_list(objList, 3, &egoData, &lsData, filteredList, 5);

    PrintFilteredObjects(filteredList, filteredCount);
    EXPECT_EQ(filteredCount, 2);
}

// (2) Path Prediction Test
TEST(TargetSelectionTest, PredictObjectFuturePath_PredictionTest) {
    FilteredObject_t filteredList[1] = {
        { 1, OBJTYPE_CAR, 30.0f, 0.0f, 0.0f,
          8.0f, 0.0f, 0.0f, 0.0f, 0.0f,
          30.0f, OBJSTAT_MOVING, 1 }
    };

    LaneData_t laneWp = {
        LANE_TYPE_STRAIGHT, 1000.0f, 1000.0f,
        0.0f, 0.0f, 3.5f, LANE_CHANGE_KEEP
    };

    LaneSelectOutput_t lsData = {
        LANE_TYPE_STRAIGHT, false, false,
        0.0f, 0.0f, 3.5f, true, false
    };

    PredictedObject_t predList[1] = {0};
    int predCount = predict_object_future_path(filteredList, 1, &laneWp, &lsData, predList, 1);

    PrintPredictedObject(predList[0]);
    EXPECT_EQ(predCount, 1);
    EXPECT_NEAR(predList[0].Predicted_Position_X, 54.0f, 0.01f); // 30 + 8*3
}

// (3) ACC/AEB Target Selection Test
TEST(TargetSelectionTest, SelectTargetsForAccAeb_NormalCase) {
    PredictedObject_t predList[1] = {
        { 1, OBJTYPE_CAR,
          40.0f, 0.0f, 0.0f,   // Pos
          8.0f, 0.0f,          // Vel
          0.0f, 0.0f,          // Accel
          0.0f, 40.0f,         // Heading, Distance
          OBJSTAT_MOVING, 2,   // Status, Cell
          false, false         // CutIn/Out
        }
    };

    EgoData_t egoData = { 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0 };

    LaneSelectOutput_t lsData = {
        LANE_TYPE_STRAIGHT, false, false,
        0.0f, 0.0f, 3.5f, true, false
    };

    ACC_Target_t accTarget = {0};
    AEB_Target_t aebTarget = {0};

    select_targets_for_acc_aeb(&egoData, predList, 1, &lsData, &accTarget, &aebTarget);

    PrintTarget(accTarget, aebTarget);
    EXPECT_EQ(accTarget.ACC_Target_ID, 1);
    EXPECT_EQ(aebTarget.AEB_Target_ID, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
