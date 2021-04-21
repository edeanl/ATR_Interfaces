cmake_minimum_required(VERSION 3.5)
project(atr_examples)

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

find_package(rclcpp REQUIRED)
# Standard ros messages
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ATR messages and serivces
find_package(atr_interfaces REQUIRED)

# TF pacakges. Needed to convert Eigen into TF, for example Rotation to Quaternion msg
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})



## BUILD

#Library 
#add_library(${PROJECT_NAME} src/MinimalPublisher.cpp) --> *.a
add_library(
  ${PROJECT_NAME}
  SHARED
  src/AuxTools.cpp
  src/MinimalPublisher.cpp
  src/MinimalSubscriber.cpp
  src/ObjectListPublisher.cpp
  src/ObjectListSubscriber.cpp
  src/ATRFormationListPublisher.cpp
  src/ATRFormationListServer.cpp
  src/ATRStatePublisher.cpp
  src/ATRStateListPublisher.cpp
  src/ATRStateListSubscriber.cpp
  src/DynamicObstaclePredictor.cpp
  src/PredictedObjectListSubscriber.cpp
  src/ATRTrajectoryGenerator.cpp
  src/SemanticSegmentation.cpp
  src/NONAGenerator.cpp
  src/ATRFleetControl.cpp
  src/PathListSubscriber.cpp
) # -> *.so
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# target_include_directories(
#   ${PROJECT_NAME}
#   PUBLIC
#   include
# )
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  atr_interfaces
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
  visualization_msgs
)
# Target's especific compiler flags 
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# Executables
set(APP_TARGETS
publisher_member_client
subscriber_member_client
object_list_publisher
object_list_subscriber
atr_formation_list_publisher
atr_formation_list_server
atr_state_publisher
atr_state_list_publisher
atr_state_list_subscriber
pred_object_list_subscriber
dynamic_obstacle_predictor
semantic_segmentation
nona_generator
atr_trajectory_generator
atr_fleet_control
atr_path_list_subscriber
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} rclcpp atr_interfaces Eigen3)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # target_compile_options(${target} PRIVATE -Wall)
  # Other Libraries
  # Dependencies by lib name
  target_link_libraries(${target} ${PROJECT_NAME})
  # target_include_directories(${target} PRIVATE
  #     ${OTHER__INCLUDES}
  # )
  # NOTE: To link agains messages generated in the same package use: 
  # rosidl_target_interfaces(${target}
  # ${PROJECT_NAME} "rosidl_typesupport_cpp")
endforeach()


# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)

# install(
#   TARGETS
#   talker
#   action_server_function
#   DESTINATION lib/${PROJECT_NAME}
# )


install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
${APP_TARGETS}
DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and config file so ROS2 can find it
install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Downstream exports
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake rclcpp std_msgs atr_interfaces tf2 tf2_ros
tf2_eigen tf2_geometry_msgs std_srvs)

ament_package()
