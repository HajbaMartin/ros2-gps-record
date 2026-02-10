cmake_minimum_required(VERSION 3.8)
project(shell_simulation)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)



install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(
  PROGRAMS
    scripts/example_control.py
    scripts/keyboard_teleop.py
    scripts/gps_recorder.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Python modul telepitese
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
