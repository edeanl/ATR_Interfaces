cmake_minimum_required(VERSION 3.5)
project(atr_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(msg_files
  "msg/AngleWithTolerance.msg"
  "msg/ATRActuatorError.msg"
  "msg/ATRBatteryState.msg"
  "msg/ATRCollisionSensor.msg"
  "msg/ATRCollisionState.msg"
  "msg/ATRData.msg"
  "msg/ATREdgeList.msg"
  "msg/ATREdge.msg"
  "msg/ATRError.msg"
  "msg/ATRFleet.msg"
  "msg/ATRFormationList.msg"
  "msg/ATRFormation.msg"
  "msg/ATRJobData.msg"
  "msg/ATRJobList.msg"
  "msg/ATRJob.msg"
  "msg/ATRList.msg"
  "msg/ATRMotionData.msg"
  "msg/ATRMotionStamped.msg"
  "msg/ATRPathList.msg"
  "msg/ATRPath.msg"
  "msg/ATRPathStamped.msg"
  "msg/ATRPoseData.msg"
  "msg/ATRPoseStamped.msg"
  "msg/ATRPoseType.msg"
  "msg/ATRProcessDescription.msg"
  "msg/ATRSignalActionLevel1.msg"
  "msg/ATRSignalActionLevel2.msg"
  "msg/ATRSignalActionLevel3.msg"
  "msg/ATRSignalActions.msg"
  "msg/ATRSignalActionsStamped.msg"
  "msg/ATRSignalError.msg"
  "msg/ATRStateList.msg"
  "msg/ATRStateListStamped.msg"
  "msg/ATRStateLoad.msg"
  "msg/ATRStateMission.msg"
  "msg/ATRState.msg"
  "msg/ATRStateActuator.msg"
  "msg/ATRStateOverall.msg"
  "msg/ATRStateSignals.msg"
  "msg/ATRStateStamped.msg"
  "msg/ATRTask.msg"
  "msg/ATRTimeNodeList.msg"
  "msg/ATRTimeNode.msg"
  "msg/ATRType.msg"
  "msg/DistanceWithTolerance.msg"
  "msg/Ellipse.msg"
  "msg/EllipseStamped.msg"
  "msg/FormationConstraints.msg"
  "msg/MyString.msg"
  "msg/ObjectClass.msg"
  "msg/ObjectList.msg"
  "msg/ObjectStamped.msg"
  "msg/ObjectType.msg"
  "msg/PathWithDTime.msg"
  "msg/PathWithDTimeStamped.msg"
  "msg/PoseWithDTime.msg"
  "msg/PredictedObjectEllipseList.msg"
  "msg/PredictedObjectEllipseStamped.msg"
  "msg/PredictedObjectList.msg"
  "msg/PredictedObjectStamped.msg"
)

set(srv_files
  "srv/GetObjectList.srv"
  "srv/GetATRFormation.srv"
  "srv/TriggerATRActuator.srv"
  "srv/TriggerATRSignals.srv"
  "srv/UpdateObjectList.srv"
  "srv/UpdatePredictedObjectList.srv"
  "srv/UpdateATRPath.srv"
  "srv/UpdateATRPathList.srv"
  "srv/SetPolygon.srv"
)

# set(action_files
# "action/+.action"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  # ADD_LINTER_TESTS
)

# IMPORTANT NOTE: To build executables that use the interfaces generated in the same package
# add_executable(publish_address_book
#   src/publish_address_book.cpp
# )

# ament_target_dependencies(publish_address_book
#   "rclcpp"
# )

# NOTE: To link agains messages generated in the same package use: 
# rosidl_target_interfaces(publish_address_book
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS publish_address_book
#  DESTINATION lib/${PROJECT_NAME})


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

# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)

# Downstream exports
ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs tf2_geometry_msgs)

ament_package()
