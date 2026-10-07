cmake_minimum_required(VERSION 3.8)
project(so101_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry no per-file copyright headers; the license is declared
  # in package.xml. Remove this line if headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(so101_moveit_config_tests test TIMEOUT 120)
endif()

ament_package()
