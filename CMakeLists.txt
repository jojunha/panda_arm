cmake_minimum_required(VERSION 3.0.2)

project(panda_arm)

add_compile_options(-std=c++11 -O)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf
    visualization_msgs 
)

find_package(Eigen3 3.3 REQUIRED)
find_package(Franka REQUIRED)
find_package(RBDL REQUIRED)

if(NOT OMPL_INCLUDE_DIRS)
	find_package(ompl REQUIRED)
endif()


set(SRC_FILES
    src/position_controller.cpp
    src/robotmodel.cpp
    src/trajectory.cpp
)

################ For cmake ###########################

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES panda_arm
  CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Franka_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIRS}
    #usr/local/lib

)

add_library(panda_lib
    ${SRC_FILES}
    ${catkin_LIBRARIES}
)

add_executable(panda_arm
    src/main.cpp
)

target_link_libraries(panda_arm
PUBLIC
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    Franka::Franka
    panda_lib
    ${RBDL_LIBRARIES}
    -pthread
    -lrbdl
    -lrbdl_urdfreader
    -lpthread
    # ${OMPL_LIBRARIES}
)
