cmake_minimum_required(VERSION 3.5)
project(phantomx_arm_hw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED src/lx_serial.cpp src/arbotix.cpp src/phantomx_arm_hw.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface phantomx_arm_hardware.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
        DESTINATION include)

ament_package()
