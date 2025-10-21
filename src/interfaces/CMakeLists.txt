cmake_minimum_required(VERSION 3.8)
project(interfaces)

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
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ColorDetect.msg"
    "msg/ColorInfo.msg"
    "msg/ColorsInfo.msg"
    "msg/LineROI.msg"
    "msg/ObjectInfo.msg"
    "msg/ObjectsInfo.msg"
    "msg/PixelPosition.msg"
    "msg/Point2D.msg"
    "msg/Points.msg"
    "msg/Pose2D.msg"
    "msg/ROI.msg"
    "srv/GetPose.srv"
    "srv/SetCircleROI.srv"
    "srv/SetColorDetectParam.srv"
    "srv/SetColorRGBA.srv"
    "srv/SetFloat64List.srv"
    "srv/SetFloat64.srv"
    "srv/SetInt64.srv"
    "srv/SetLineROI.srv"
    "srv/SetPoint.srv"
    "srv/SetPose2D.srv"
    "srv/SetPose.srv"
    "srv/SetStringList.srv"
    "srv/SetString.srv"
    "srv/ChangeRange.srv"
    "srv/GetAllColorName.srv"
    "srv/GetRange.srv"
    "srv/StashRange.srv"
    DEPENDENCIES std_msgs std_srvs geometry_msgs builtin_interfaces
 )

install(FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
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
