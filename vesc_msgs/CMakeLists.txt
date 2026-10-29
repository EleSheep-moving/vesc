cmake_minimum_required(VERSION 3.5)
project(vesc_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(vesc_msgs
  "msg/VescCommand.msg"
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  "msg/VescImu.msg"
  "msg/VescImuStamped.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_command_validation test/test_command_validation.cpp)
  target_compile_options(test_command_validation PRIVATE -UNDEBUG)
  target_include_directories(test_command_validation PRIVATE include)
  rosidl_get_typesupport_target(command_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(test_command_validation "${command_typesupport}")
  add_test(NAME command_validation COMMAND test_command_validation)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_auto_package()
