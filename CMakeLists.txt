cmake_minimum_required(VERSION 3.8)
project(ros2_robotiqgripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME server_mock_tests
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_*.py
  )
  set_tests_properties(server_mock_tests PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
  )
endif()

# ================================================== #

install(
  DIRECTORY 
    srv
    python
  DESTINATION 
    share/${PROJECT_NAME}
)

install(PROGRAMS
    
    python/server.py
    #python/client.py

  DESTINATION lib/${PROJECT_NAME} 
)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RobotiqGripper.srv"
)

ament_package()
