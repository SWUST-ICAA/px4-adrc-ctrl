cmake_minimum_required(VERSION 3.8)
project(px4_adrc_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(uav_adrc_mbd STATIC
  MBD_code/src/uav_adrc.cpp
  MBD_code/src/uav_adrc_data.cpp
)
target_include_directories(uav_adrc_mbd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MBD_code/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(adrc_controller_node
  src/adrc_controller.cpp
  src/main.cpp
)
target_include_directories(adrc_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(adrc_controller_node uav_adrc_mbd Eigen3::Eigen)
ament_target_dependencies(adrc_controller_node rclcpp px4_msgs)

install(TARGETS
  uav_adrc_mbd
  adrc_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  MBD_code/include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
