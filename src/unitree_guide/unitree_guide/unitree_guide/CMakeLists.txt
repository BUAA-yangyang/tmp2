cmake_minimum_required(VERSION 3.14)

project(unitree_guide)


# ==============================
# LibTorch CUDA Configuration
# CUDA 12.8 + Blackwell sm_120
# ==============================

set(Torch_DIR "/opt/libtorch/share/cmake/Torch")

list(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")

set(TORCH_CUDA_ARCH_LIST "12.0" CACHE STRING "CUDA architectures for libtorch")

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_CUDA_FLAGS 
    "${CMAKE_CUDA_FLAGS} -arch=sm_120"
)


find_package(Torch REQUIRED)


# ==============================
# Robot Configuration
# ==============================

set(ROBOT_TYPE A1)
set(PLATFORM amd64)

set(CATKIN_MAKE ON)
set(SIMULATION ON)
set(REAL_ROBOT OFF)
set(DEBUG OFF)
option(MOVE_BASE "Build the Unitree move_base velocity-control FSM state" ON)


if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()


if(${ROBOT_TYPE} STREQUAL "A1")

    add_definitions(-DROBOT_TYPE_A1)

elseif(${ROBOT_TYPE} STREQUAL "Go1")

    add_definitions(-DROBOT_TYPE_Go1)

else()

    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")

endif()



if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))

    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")

endif()



if(SIMULATION OR MOVE_BASE)

    add_definitions(-DRUN_ROS)

    set(CATKIN_MAKE ON)

endif()



# ==============================
# Compiler
# ==============================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(Boost)


# ==============================
# ROS
# ==============================

if(CATKIN_MAKE)

    add_definitions(-DCOMPILE_WITH_ROS)


    if(MOVE_BASE)

        add_definitions(-DCOMPILE_WITH_MOVE_BASE)

    endif()

endif()



if(DEBUG)

    add_definitions(-DCOMPILE_DEBUG)

    find_package(Python2 COMPONENTS Interpreter Development NumPy)

endif()



if(CATKIN_MAKE)

    if(SIMULATION)

        add_definitions(-DCOMPILE_WITH_SIMULATION)


        find_package(catkin REQUIRED COMPONENTS

            controller_manager

            joint_state_controller

            gazebo_ros

        )

    endif()



    find_package(catkin REQUIRED COMPONENTS

        genmsg

        robot_state_publisher

        message_generation

        roscpp

        rospy

        gazebo_msgs

        sensor_msgs

        std_srvs

        std_msgs

        tf

        geometry_msgs

        unitree_legged_msgs

    )



    add_message_files(FILES

        CustomPoint.msg

        CustomMsg.msg

    )


    generate_messages(DEPENDENCIES

        std_msgs

    )


    catkin_package(

        CATKIN_DEPENDS

        unitree_legged_msgs

        gazebo_msgs

        std_srvs

    )

endif()



# ==============================
# Include
# ==============================

include_directories(

    include

    ${Boost_INCLUDE_DIRS}

    ${catkin_INCLUDE_DIRS}

    ${GAZEBO_INCLUDE_DIRS}

)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")



# ==============================
# Real Robot
# ==============================

if(REAL_ROBOT)

    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)


    if(${ROBOT_TYPE} STREQUAL "A1")

        include_directories(

            library/unitree_legged_sdk_3.2/include

        )


        link_directories(

            library/unitree_legged_sdk_3.2/lib

        )


    elseif(${ROBOT_TYPE} STREQUAL "Go1")


        include_directories(

            library/unitree_legged_sdk-3.8.0/include

        )


        if(${PLATFORM} STREQUAL "amd64")


            link_directories(

                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64

            )


        elseif(${PLATFORM} STREQUAL "arm64")


            link_directories(

                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64

            )


        endif()

    endif()

endif()



link_directories(

    ${GAZEBO_LIBRARY_DIRS}

)



# ==============================
# Source
# ==============================

file(GLOB_RECURSE SRC_LIST

    "src/*/*.cpp"

    "src/*/*.cc"

)



# ==============================
# Executables
# ==============================


add_executable(

    state_from_gazebo

    src/state_from_gazebo.cpp

)


target_link_libraries(

    state_from_gazebo

    ${catkin_LIBRARIES}

)



add_executable(

    junior_ctrl

    src/main.cpp

    ${SRC_LIST}

)



target_link_libraries(

    junior_ctrl

    ${TORCH_LIBRARIES}

)



if(CATKIN_MAKE)


    target_link_libraries(

        junior_ctrl

        ${catkin_LIBRARIES}

    )


    add_dependencies(

        junior_ctrl

        ${${PROJECT_NAME}_EXPORTED_TARGETS}

        ${catkin_EXPORTED_TARGETS}

    )


endif()



if(DEBUG)


    target_include_directories(

        junior_ctrl PRIVATE

        ${Python2_INCLUDE_DIRS}

        ${Python2_NumPy_INCLUDE_DIRS}

    )


    target_link_libraries(

        junior_ctrl

        Python2::Python

        Python2::NumPy

    )


endif()



if(REAL_ROBOT)

    if(${ROBOT_TYPE} STREQUAL "A1")


        if(${PLATFORM} STREQUAL "amd64")


            target_link_libraries(

                junior_ctrl

                libunitree_legged_sdk_amd64.so

            )


        elseif(${PLATFORM} STREQUAL "arm64")


            target_link_libraries(

                junior_ctrl

                libunitree_legged_sdk_arm64.so

            )


        endif()



    elseif(${ROBOT_TYPE} STREQUAL "Go1")


        target_link_libraries(

            junior_ctrl

            libunitree_legged_sdk.a

        )


    endif()

endif()



target_link_libraries(

    junior_ctrl

    -pthread

    lcm

)



if(NOT CATKIN_MAKE)

    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

endif()



catkin_install_python(PROGRAMS

    scripts/virtual_joy.py

    scripts/keyboard_teleop.py

    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)



catkin_install_python(PROGRAMS

    scripts/pointcloud2livox.py

    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)


if(CATKIN_ENABLE_TESTING)

    catkin_add_gtest(

        test_safe_stand_gyro_filter

        test/test_safe_stand_gyro_filter.cpp

    )

    catkin_add_gtest(

        test_navigation_gait_profile

        test/test_navigation_gait_profile.cpp
        src/Gait/WaveGenerator.cpp

    )

endif()
