cmake_minimum_required(VERSION 3.23)
project(legged_ctrl)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17")

# check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

######################## DEPENDENCIES ########################
include(FetchContent)

# catkin
find_package(catkin REQUIRED
        COMPONENTS roscpp rospy std_msgs geometry_msgs unitree_legged_msgs tf
        )
catkin_package(
        CATKIN_DEPENDS roscpp
)

# add Eigen3
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Enable creation of Eigen documentation." FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Enable creation of Eigen tests." FORCE)
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c  # version 3.4.0
)
FetchContent_MakeAvailable(Eigen)

# add Altro
FetchContent_Declare(
        Altro
        # SOURCE_DIR /home/zixin/Dev/ALTRO
        # GIT_REPOSITORY https://github.com/RoboticExplorationLab/ALTRO.git
        # GIT_TAG 0cb4f9c8ed7d619bfc3c127bb2f7826261bcc5a4
        GIT_REPOSITORY https://github.com/zixinz990/altro.git
        GIT_TAG b47202ffb9e09d5a2013d4661260988810e2eaef
)
FetchContent_MakeAvailable(Altro)

# add JSON
FetchContent_Declare(
        JSON
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(JSON)

# add OSQP
include_directories(
        /usr/local/include/osqp
)
find_package(OsqpEigen REQUIRED)

# include some directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

######################## LIBRARIES ########################
# ctrl_lib
add_library(ctrl_lib
        src/RobotState.cpp
        src/GazeboInterface.cpp
        src/Kinematics.cpp
        src/Utils.cpp
        src/GyrostatModel.cpp
        src/LeggedMpc.cpp
        src/GyrostatQuatMpc.cpp
        src/GyrostatEulerMpc.cpp
)
target_link_libraries(ctrl_lib
        ${catkin_LIBRARIES}
        altro::altro
        Eigen3::Eigen
        fmt::fmt
        nlohmann_json::nlohmann_json        
)

######################## MAIN EXECUTABLE TARGETS ########################
add_executable(main
        src/Main.cpp
)
target_link_libraries(main
        ctrl_lib
        Eigen3::Eigen
)
