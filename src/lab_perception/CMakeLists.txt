cmake_minimum_required(VERSION 3.8)
project(lab_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The lab_perception Python package (camera model, segmentation, naming).
ament_python_install_package(${PROJECT_NAME})

# Installed as an executable so launch files can start it (with the
# /opt/lab-ai interpreter as prefix).
install(PROGRAMS
  scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry no per-file copyright headers; the license is declared
  # in package.xml. Remove this line if headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lab_perception_tests test TIMEOUT 300)
endif()

ament_package()
