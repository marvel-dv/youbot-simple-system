cmake_minimum_required(VERSION 3.0.2)
project(youbot_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation
)

## Arquivos .srv
add_service_files(
  FILES
  GoToPose.srv
  GoToPoseMoveIt.srv
  ForwardKinematics.srv
  ApproachTag.srv
)

## Geração dos serviços
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Configuração do pacote
catkin_package(
  CATKIN_DEPENDS
    rospy
    message_runtime
    std_msgs
    geometry_msgs
)

## Inclui os diretórios do catkin
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Scripts Python (opcional, mas recomendado)
catkin_install_python(
  PROGRAMS
    scripts/go_to_pose_server.py
    scripts/go_to_pose_moveit_server.py
    scripts/forwardkinematics_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

