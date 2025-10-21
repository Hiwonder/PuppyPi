cmake_minimum_required(VERSION 3.8)
project(puppy_control_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Gait.msg"
    "msg/Pose.msg"
    "msg/SetServo.msg"
    "msg/Velocity.msg"
    "srv/SetFloat64.srv"
    "srv/SetFloat64List.srv"
    "srv/SetInt64.srv"
    "srv/SetRunActionName.srv"
    DEPENDENCIES std_msgs  
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
