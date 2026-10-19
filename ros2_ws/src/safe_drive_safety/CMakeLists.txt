cmake_minimum_required(VERSION 3.8)
project(safe_drive_safety)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/safety_monitor scripts/route_monitor scripts/red_stop_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES config/safety.yaml config/red_stop_monitor.yaml
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_metrics test/test_metrics.py)
  ament_add_pytest_test(test_route_geometry test/test_route_geometry.py)
  ament_add_pytest_test(test_red_stop test/test_red_stop.py)
endif()

ament_package()
