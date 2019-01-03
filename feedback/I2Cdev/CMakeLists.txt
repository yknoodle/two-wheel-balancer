cmake_minimum_required(VERSION 2.8.1 FATAL_ERROR)

project(I2Cdev)

set (CMAKE_CXX_STANDARD 11)

add_definitions(--std=c++11 -lwiringPi)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


include_directories(
	${PROJECT_SOURCE_DIR}/inc
)

add_library( I2Cdev
	${PROJECT_SOURCE_DIR}/src/I2Cdev.cpp
)

target_include_directories( I2Cdev PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries( I2Cdev
#	library_location
	wiringPi
)