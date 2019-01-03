

cmake_minimum_required(VERSION 2.8.1 FATAL_ERROR)

project(rpii2c)

set (CMAKE_CXX_STANDARD 11)

add_definitions(--std=c++11 )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package( Eigen3 REQUIRED )

include_directories(
	${EIGEN3_INCLUDE_DIR}
	# piutl include file TODO
	# rpihw include file TODO
)

add_subdirectory(
	I2Cdev
)

add_subdirectory(
	LSM303DLHC
)

add_subdirectory(
	LowPassFilter
)

add_executable( rpi_test
#	executable_source_location
	${PROJECT_SOURCE_DIR}/test/test.cpp
)

target_link_libraries( rpi_test
#	executable_referenced_library_location
	LowPassFilter
	LSM303DLHC
	# piutl library TODO
)


