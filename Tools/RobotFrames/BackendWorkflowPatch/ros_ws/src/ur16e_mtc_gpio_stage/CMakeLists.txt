cmake_minimum_required(VERSION 3.8)
project(ur16e_mtc_gpio_stage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur16e_interfaces REQUIRED)

add_executable(test_task src/test_task.cpp)
target_compile_features(test_task PRIVATE cxx_std_17)

target_include_directories(test_task PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(test_task
  rclcpp
  rcl_interfaces
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
  ur_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  moveit_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  ur16e_interfaces
)

# Optional: only if you want to install and `ros2 run` it
install(TARGETS test_task
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Optional: only if you intend other packages to include your headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)

  ament_add_gtest(test_scene_effects test/test_scene_effects.cpp)
  target_include_directories(test_scene_effects PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_scene_effects PRIVATE cxx_std_17)
  ament_target_dependencies(test_scene_effects
    moveit_core
    moveit_msgs
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    urdf
    srdfdom
  )

  add_executable(test_execution_sequence test/test_execution_sequence.cpp)
  target_include_directories(test_execution_sequence PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_execution_sequence PRIVATE cxx_std_17)
  add_test(NAME execution_sequence COMMAND test_execution_sequence)

  add_executable(test_plan_gate test/test_plan_gate.cpp)
  target_include_directories(test_plan_gate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_plan_gate PRIVATE cxx_std_17)
  add_test(NAME plan_gate COMMAND test_plan_gate)

  ament_add_gtest(test_plan_validation test/test_plan_validation.cpp)
  target_include_directories(test_plan_validation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_plan_validation PRIVATE cxx_std_17)
  ament_target_dependencies(test_plan_validation moveit_msgs sensor_msgs)

  add_executable(test_tool_definition test/test_tool_definition.cpp)
  target_compile_features(test_tool_definition PRIVATE cxx_std_17)
  target_include_directories(test_tool_definition PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME tool_definition COMMAND test_tool_definition)

  ament_add_gtest(test_tool_model_validation test/test_tool_model_validation.cpp)
  target_compile_features(test_tool_model_validation PRIVATE cxx_std_17)
  target_include_directories(test_tool_model_validation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_tool_model_validation moveit_core urdf srdfdom)

  ament_add_gtest(test_tcp_waypoints test/test_tcp_waypoints.cpp)
  target_compile_features(test_tcp_waypoints PRIVATE cxx_std_17)
  target_include_directories(test_tcp_waypoints PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_tcp_waypoints geometry_msgs)

  ament_add_gtest(test_pick_place_targets test/test_pick_place_targets.cpp)
  target_compile_features(test_pick_place_targets PRIVATE cxx_std_17)
  target_include_directories(test_pick_place_targets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_pick_place_targets geometry_msgs ur16e_interfaces)

  ament_add_gtest(test_startup_parameter test/test_startup_parameter.cpp)
  target_compile_features(test_startup_parameter PRIVATE cxx_std_17)
  target_include_directories(test_startup_parameter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_startup_parameter rclcpp rcl_interfaces)
endif()

ament_package()
