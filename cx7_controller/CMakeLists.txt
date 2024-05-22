cmake_minimum_required(VERSION 3.8)
project(cx7_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(cx7_interface REQUIRED)

# build nodes list
set(node_list
    move_to_pose
)
foreach(node IN LISTS node_list)
    add_executable(${node} src/${node}.cpp)
    ament_target_dependencies(${node}
        rclcpp
        moveit_ros_planning_interface
        cx7_interface
    )
    install(TARGETS
        ${node}
        DESTINATION lib/${PROJECT_NAME}
    )
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
