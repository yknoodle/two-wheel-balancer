

cmake_minimum_required(VERSION 2.8.1 FATAL_ERROR)

project(LSM303DLHC)

set (CMAKE_CXX_STANDARD 11)

add_definitions(--std=c++11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(I2CDEV_SOURCE_DIRECTORY ${PROJECT_SOURCE_DIR}/../i2cdevlib/RaspberryPi/I2Cdev)

include_directories( 
	${PROJECT_SOURCE_DIR}/inc
)

add_library( LSM303DLHC
	${PROJECT_SOURCE_DIR}/src/LSM303DLHC.cpp
)

target_link_libraries( LSM303DLHC LINK_PUBLIC
	I2Cdev
)

target_include_directories( LSM303DLHC PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)