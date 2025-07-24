cmake_minimum_required(VERSION 3.0.2)
project(exoskeleton_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  EStopTrigger.msg
  StopTrigger.msg
  CalibrationTrigger.msg
  GaitParams.msg
  JointsTrajectory.msg
  ExoskeletonState.msg
  MotorStatus.msg
  Torques.msg
)

add_service_files(
  FILES
  InverseKinematicsClosedChain.srv
  InverseKinematicsLeg.srv
  ForwardKinematicsLeg.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/gait_planner_node.py
  scripts/trajectory_generator_node.py
  scripts/motor_control_node.py
  scripts/emergency_stop_node.py
  scripts/dummy_crutches.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
