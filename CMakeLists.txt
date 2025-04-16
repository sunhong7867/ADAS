cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름
project(ADASProject)

# C++ 표준 설정
set(CMAKE_C_STANDARD 99)          # C99 사용 (필요에 따라 변경)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Google Test 수동 추가
add_subdirectory(googletest)

# 테스트 활성화
enable_testing()

# 대상 라이브러리 추가
add_library(adas
    #ego_vehicle_estimation.c
	lane_selection.c
	#target_selection.c

    # 추가 ADAS 관련 C 소스 파일이 있다면 여기에 나열하세요.
)

# 테스트 실행 파일 추가
add_executable(adas_unit_tests 
    #ego_vehicle_estimation_test_EQ.cpp
	#ego_vehicle_estimation_test_RA.cpp
	#ego_vehicle_estimation_test_BV.cpp
	#lane_selection_test.cpp
	target_selection_test_FP.cpp
	#target_selection_test_SA.cpp
	#target_selection_test_ST.cpp

	# 추가 ADAS 관련 테스트 C++ 소스 파일이 있다면 여기에 나열하세요.
)
target_link_libraries(adas_unit_tests PRIVATE adas gtest gtest_main)

# Google Test를 사용하여 테스트 등록
include(GoogleTest)
gtest_discover_tests(adas_unit_tests)
