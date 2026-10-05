cmake_minimum_required(VERSION 3.8)
project(lab_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Robot descriptions, worlds, configuration and launch files.
install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# The lab_simulation Python package (kinematics, tasks, benchmark, ...).
ament_python_install_package(${PROJECT_NAME})

# Scripts are installed as executables so they can be run with
#   ros2 run lab_simulation <script>
install(PROGRAMS
  scripts/move_arm.py
  scripts/object_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry no per-file copyright headers; the license is declared
  # in package.xml. Remove this line if headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lab_simulation_tests test TIMEOUT 300)
endif()

ament_package()
