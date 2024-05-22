cmake_minimum_required(VERSION 3.8)
project(cx7_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  cx7_config/robot_description_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
