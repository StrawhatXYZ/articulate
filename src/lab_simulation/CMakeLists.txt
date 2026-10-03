cmake_minimum_required(VERSION 3.8)
project(lab_simulation)

find_package(ament_cmake REQUIRED)

# Robot descriptions, worlds, configuration and launch files.
install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Scripts are installed as executables so they can be run with
#   ros2 run lab_simulation <script>
install(PROGRAMS
  scripts/move_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry no per-file copyright headers; the license is declared
  # in package.xml. Remove this line if headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
endif()

ament_package()
