cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME "imarker_detector_ros")
project(${PROJECT_NAME} VERSION 1.0.0)

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/singlevision_off_rosbag.py
  src/singlevision_rs.py
  src/dualvision_ids.py
  src/dualvision_usb.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()