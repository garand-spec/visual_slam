#CMake minimal version 3.10
cmake_minimum_required(VERSION 3.10)

#define project name
project(slam_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#auto search OpenCV
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: $(OpenCV_VERSION)")
message(STATUS "OpenCV include: $(OpenCV_INCLUDE_DIRS)")

add_executable(
	yxt_visual_slam
	src/main.cpp
)

target_include_directories(
	yxt_visual_slam PRIVATE
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}	
)

target_link_libraries(
	yxt_visual_slam PRIVATE
	${OpenCV_LIBS}
)


