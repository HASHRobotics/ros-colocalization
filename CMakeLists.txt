cmake_minimum_required(VERSION 2.8.3)
project(colocalization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	message_generation
	geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	thread
	system
)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}

)

add_service_files(
	FILES
	addBearingRangeNodes.srv
	optimizeFactorGraph.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES particle_filter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

add_executable(
  colocalization
	main.cpp)

target_link_libraries(colocalization
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
  tbb
  tbbmalloc
  gtsam
)
