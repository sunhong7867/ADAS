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
	#lane_selection.c
	#target_selection.c
	#acc.c
	aeb.c
	#lfa.c
	#arbitration.c
)

# 테스트 실행 파일 추가
add_executable(adas_unit_tests 
    #ego_vehicle_estimation_EQ_test.cpp
	#ego_vehicle_estimation_BV_test.cpp
	#ego_vehicle_estimation_RA_test.cpp
	
	#lane_selection_test.cpp

	#target_selection_object_test.cpp
	#target_selection_path_test.cpp
	#target_selection_select_test.cpp
	
	#acc_mode_test.cpp
	#acc_distance_EQ_test.cpp
	#acc_distance_BV_test.cpp
	#acc_distance_RA_test.cpp
	#acc_speed_pid_test.cpp
	#acc_out_test.cpp

	aeb_ttc_test.cpp
	#aeb_mode_test.cpp
	#aeb_decel_test.cpp

	#lfa_mode_test.cpp
	#lfa_pid_test.cpp
	#lfa_stanley_test.cpp
	#lfa_output_test.cpp

	#arbitration_test.cpp
)
target_link_libraries(adas_unit_tests PRIVATE adas gtest gtest_main)

# Google Test를 사용하여 테스트 등록
include(GoogleTest)
gtest_discover_tests(adas_unit_tests)
