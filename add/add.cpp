#include <iostream>
#include "gtest/gtest.h"

int main() {
    std::cout << "GTest version: "
              << GTEST_VERSION_MAJOR << "."
              << GTEST_VERSION_MINOR << "."
              << GTEST_VERSION_PATCH << std::endl;
    return 0;
}
