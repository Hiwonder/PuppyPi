cmake_minimum_required(VERSION 3.8)
project(ros_robot_controller_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/BusServoState.msg"
    "msg/ButtonState.msg"
    "msg/BuzzerState.msg"
    "msg/LedState.msg"
    "msg/MotorState.msg"
    "msg/MotorsState.msg"
    "msg/PWMServoState.msg"
    "msg/Sbus.msg"
    "msg/ButtonStatus.msg"
    "msg/OLEDState.msg"
    "msg/RGBState.msg"
    "msg/RGBsState.msg"
    "msg/SetBusServoState.msg"
    "msg/ServoPosition.msg"
    "msg/ServosPosition.msg"
    "msg/SetPWMServoState.msg"
    "msg/GetBusServoCmd.msg"
    "msg/GetPWMServoCmd.msg"
    "srv/GetBusServoState.srv"
    "srv/GetPWMServoState.srv"
    DEPENDENCIES std_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
