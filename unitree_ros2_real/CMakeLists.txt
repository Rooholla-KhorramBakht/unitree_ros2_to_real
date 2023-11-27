cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
  link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/amd64)
else()
  set(ARCH arm64)
  link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/arm64)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)


set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)




add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs sensor_msgs)

install(TARGETS
    ros2_udp
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()




