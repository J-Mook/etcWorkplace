cmake_minimum_required(VERSION 2.8.3)
project(unitree_base)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  realtime_tools
  std_msgs
  geometry_msgs
  sensor_msgs
  unitree_legged_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  $ENV{UNITREE_LEGGED_SDK_PATH}/include
  $ENV{ALIENGO_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories($ENV{ALIENGO_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} libaliengo_comm.so lcm)

add_executable(unitree_base_node
  src/unitree_base.cpp
)
add_dependencies(unitree_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(unitree_base_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EXTRA_LIBS})

add_executable(lcm_server_walking_node 
  src/lcm_server_walking.cpp
)
add_dependencies(lcm_server_walking_node ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcm_server_walking_node ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(lcm_server_running_node 
  src/lcm_server_running.cpp
)
add_dependencies(lcm_server_running_node ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcm_server_running_node ${EXTRA_LIBS} ${catkin_LIBRARIES})

install(
  DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS unitree_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)