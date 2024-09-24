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

# add Unitree hardware library
include_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/include
)
link_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/lib/cpp/amd64
)
set(UNITREE_LIBS -pthread libunitree_legged_sdk.a lcm)

# include some directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

######################## LIBRARIES ########################
# utils_lib
add_library(utils_lib
        src/utils/Utils.cpp
        src/LeggedState.cpp
        src/utils/A1Kinematics.cpp
        src/utils/AltroUtils.cpp
        src/utils/QuaternionUtils.cpp
        src/utils/Go1Dynamics.cpp
        )
target_link_libraries(utils_lib
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        altro::altro
        fmt::fmt
        nlohmann_json::nlohmann_json
        )

# casadi_ekf_lib
find_package(casadi REQUIRED)
find_package(gram_savitzky_golay REQUIRED)
add_subdirectory(src/estimation/casadi_ekf)

add_library(casadi_ekf_lib
        src/estimation/casadi_ekf/A1KF.hpp
        src/estimation/casadi_ekf/A1KFCombineLOWithFoot.cpp
        src/estimation/casadi_ekf/A1KFCombineLOWithFootTerrain.cpp
        )
include_directories(
        src/estimation/
)
set_target_properties(casadi_ekf_lib PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(casadi_ekf_lib casadi gram_savitzky_golay Eigen3::Eigen)

# interfaces_lib
add_library(interfaces_lib
        src/interfaces/BaseInterface.cpp
        src/interfaces/GazeboInterface.cpp
        src/interfaces/HardwareInterface.cpp
        src/LeggedState.cpp
        src/utils/Utils.cpp
        src/estimation/BasicKF.cpp
        src/utils/A1Kinematics.cpp
        )
target_link_libraries(interfaces_lib
        ${catkin_LIBRARIES}
        ${UNITREE_LIBS}
        casadi_ekf_lib
        Eigen3::Eigen
        )

# mpc_lib
add_library(mpc_lib
        src/mpc/LeggedMpc.cpp
        src/mpc/ConvexMpc.cpp
        src/mpc/QuatMpc.cpp
        src/utils/LeggedContactFSM.cpp
        src/utils/AltroUtils.cpp
        src/utils/QuaternionUtils.cpp
        )
target_link_libraries(mpc_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        Eigen3::Eigen
        altro::altro
        fmt::fmt
        nlohmann_json::nlohmann_json
        )

# altro_test_lib
add_library(altro_test_lib
        src/test/test_altro/AltroTestUtils.hpp
        src/test/test_altro/AltroTestUtils.cpp
        )
target_link_libraries(altro_test_lib
        ${catkin_LIBRARIES}
        altro::altro
        Eigen3::Eigen
        fmt::fmt
        nlohmann_json::nlohmann_json
        )

######################## MAIN EXECUTABLE TARGETS ########################
# main
add_executable(main
        src/Main.cpp
        )
target_link_libraries(main
        utils_lib
        interfaces_lib
        mpc_lib
        Eigen3::Eigen
        )

######################## TESTS ########################
# add_executable(test_inv_kin
#         src/test/TestInvKin.cpp
#         )
# target_link_libraries(test_inv_kin
#         utils_lib
#         Eigen3::Eigen
#         )

# add_executable(test_altro_api
#         src/test/test_altro/TestAltroApi.cpp
#         )
# target_link_libraries(test_altro_api
#         altro::altro
#         fmt::fmt
#         Eigen3::Eigen
#         )

# add_executable(test_double_integrator
#         src/test/test_altro/TestDoubleIntegrator.cpp
#         )
# target_link_libraries(test_double_integrator
#         altro::altro
#         fmt::fmt
#         Eigen3::Eigen
#         )

# add_executable(test_bicycle
#         src/test/test_altro/TestBicycle.cpp
#         )
# target_link_libraries(test_bicycle
#         altro_test_lib
#         )

# add_executable(test_altro_convex_mpc
#         src/test/test_altro/TestAltroConvexMpc.cpp
#         )
# target_link_libraries(test_altro_convex_mpc
#         mpc_lib
#         utils_lib
#         )

# add_executable(test_quat_utils
#         src/test/TestQuatUtils.cpp
#         )
# target_link_libraries(test_quat_utils
#         utils_lib
#         )

# add_executable(test_altro_quat_mpc
#         src/test/test_altro/TestAltroQuatMpc.cpp
#         )
# target_link_libraries(test_altro_quat_mpc
#         mpc_lib
#         utils_lib
#         )

# add_executable(test_altro_trot_quat_mpc
#         src/test/test_altro/TestAltroTrotQuatMpc.cpp
#         )
# target_link_libraries(test_altro_trot_quat_mpc
#         mpc_lib
#         utils_lib
#         )

# add_executable(test_clock
#         src/test/TestClock.cpp
#         )
# target_link_libraries(test_clock
#         ${catkin_LIBRARIES}
#         Eigen3::Eigen
#         )

# add_executable(test_inv_dynamics
#         src/test/TestInvDynamics.cpp
#         )
# target_link_libraries(test_inv_dynamics
#         ${catkin_LIBRARIES}
#         utils_lib
#         )
