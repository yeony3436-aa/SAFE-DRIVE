cmake_minimum_required(VERSION 3.8)
project(safe_drive_scenario)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/scenario_manager scripts/traffic_signal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES config/sudden_pedestrian.yaml config/traffic_signal.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sudden_pedestrian test/test_sudden_pedestrian.py)
  ament_add_pytest_test(test_traffic_signal test/test_traffic_signal.py)
endif()

ament_package()
