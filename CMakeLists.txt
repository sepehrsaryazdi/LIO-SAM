cmake_minimum_required(VERSION 3.5)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # required for ROS2
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)


# include directories
include_directories(
	include
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_dummyScript src/dummyScript.cpp)
# add_dependencies(${PROJECT_NAME}_dummyScript ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dummyScript Boost::timer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


ament_package()





# ament_package()
