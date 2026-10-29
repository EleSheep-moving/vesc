cmake_minimum_required(VERSION 3.5)
project(vesc_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set minimum C++ standard to C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Threads)

###########
## Build ##
###########

# node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_driver::VescDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_executable(
  vesc_device_namer
  src/vesc_device_namer.cpp
  src/vesc_device_uuid_lookup.cpp
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(interface_writes test/test_interface_writes.cpp TIMEOUT 30)
  target_link_libraries(interface_writes ${PROJECT_NAME} util ${CMAKE_THREAD_LIBS_INIT})
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(driver_link test/test_driver_link.py TIMEOUT 60)
  ament_add_pytest_test(longitudinal_tracking test/test_longitudinal_tracking.py TIMEOUT 60)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)
