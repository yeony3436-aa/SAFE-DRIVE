cmake_minimum_required(VERSION 3.8)
project(safe_drive_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install the package's runnable default with the package itself.
install(
  FILES config/controller.yaml config/fast_straight.yaml config/signal_controller.yaml
        config/right_turn.yaml config/left_turn.yaml config/curve.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_controller test/test_controller.py)
  ament_add_pytest_test(test_signal_stop test/test_signal_stop.py)
  ament_add_pytest_test(test_straight_tracking test/test_straight_tracking.py)
  ament_add_pytest_test(test_ackermann test/test_ackermann.py)
endif()

ament_package()
