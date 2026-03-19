cmake_minimum_required(VERSION 3.8)
project(p9n_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Teleop Twist Node =================================================
set(TWIST_TARGET teleop_twist_joy_node)
ament_auto_add_library(${PROJECT_NAME}_${TWIST_TARGET} SHARED src/${TWIST_TARGET}.cpp)
rclcpp_components_register_node(
  ${PROJECT_NAME}_${TWIST_TARGET}
  PLUGIN "p9n_node::TeleopTwistJoyNode"
  EXECUTABLE ${TWIST_TARGET}_exec)

# Teleop Ackermann Node =============================================
set(ACK_TARGET teleop_ackermann_joy_node)
ament_auto_add_library(${PROJECT_NAME}_${ACK_TARGET} SHARED src/${ACK_TARGET}.cpp)
rclcpp_components_register_node(
  ${PROJECT_NAME}_${ACK_TARGET}
  PLUGIN "p9n_node::TeleopAckermannJoyNode"
  EXECUTABLE ${ACK_TARGET}_exec)

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()