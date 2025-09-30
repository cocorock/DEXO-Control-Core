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
  FSMState.msg
  Trigger.msg
  CrutchCommand.msg
  CrutchState.msg
  CrutchSensorData.msg
  GaitParams.msg
  JointsTrajectory.msg
  DualAnkleTrajectory.msg
  ExoskeletonState.msg
  MotorStatus.msg
  Torques.msg
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
  scripts/gmr_trajectory_generator_node.py
  scripts/tpgmm_trajectory_generator_node.py
  scripts/motor_control_node.py
  scripts/orchestrator_node.py
  scripts/dummy_crutches.py
  scripts/bluetooth_crutch_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
