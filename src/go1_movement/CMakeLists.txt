cmake_minimum_required(VERSION 3.0.2)
project(go1_movement)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  unitree_legged_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy geometry_msgs unitree_legged_msgs nav_msgs
)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

add_executable(highcmd_publisher src/highcmd_publisher.cpp)
target_link_libraries(highcmd_publisher ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(highcmd_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_motion src/test.cpp)
target_link_libraries(test_motion ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(test_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(go1_teleop src/go1_teleop.cpp)
target_link_libraries(go1_teleop ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(go1_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
