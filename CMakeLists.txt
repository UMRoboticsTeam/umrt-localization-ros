cmake_minimum_required(VERSION 3.16)
project(umrt-localization-ros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -fPIC)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_lifecycle
        ublox_dgnss
        sensor_msgs
        )

# find dependencies
find_package(ublox_ubx_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(gps_node
    src/heading.cpp
    src/heading_node.cpp
)

target_compile_features(gps_node PUBLIC cxx_std_17)
target_include_directories(gps_node PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )
ament_target_dependencies(gps_node
    rclcpp
    sensor_msgs
    ublox_ubx_msgs
)

# Parse version number from the file
file(READ "version" version_input)
string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" _ ${version_input})

# Fill version number into package.xml
configure_file(
        package.in.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        @ONLY
)

# INSTALL
install(TARGETS
        gps_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
