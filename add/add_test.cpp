#include <gtest/gtest.h>
#include "add.h"

TEST(AdditionTest, HandlesPositiveNumbers) {
	EXPECT_EQ(add(2, 3), 5);
}

TEST(AdditionTest, HandlesNegativeNumbers) {
	EXPECT_EQ(add(-2, -3), -5);
} 

TEST(AdditionTest, HandlesMixedNumbers) {
	EXPECT_EQ(add(-2, 3), 1);
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}