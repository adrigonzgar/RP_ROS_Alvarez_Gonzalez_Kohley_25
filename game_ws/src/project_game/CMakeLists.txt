cmake_minimum_required(VERSION 3.0.2)
project(project_game)

## -------------------------------------------------------------
##              FIND PACKAGES
## -------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

## -------------------------------------------------------------
##              DECLARE MESSAGES
## -------------------------------------------------------------
add_message_files(
  FILES
  user_msg.msg
  game_state.msg
)

## -------------------------------------------------------------
##              DECLARE SERVICES
## -------------------------------------------------------------
add_service_files(
  FILES
  GetUserScore.srv
  SetGameDifficulty.srv
)

## -------------------------------------------------------------
##      GENERATE MSG/SRV PYTHON & CPP CODE
## -------------------------------------------------------------
generate_messages(
  DEPENDENCIES
  std_msgs
)

## -------------------------------------------------------------
##              EXPORT PACKAGE
## -------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

## -------------------------------------------------------------
##              INCLUDE DIRECTORIES
## -------------------------------------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## -------------------------------------------------------------
##     FIX: NECESARIO PARA QUE SE GENEREN .srv EN PYTHON
## -------------------------------------------------------------
add_dependencies(${PROJECT_NAME}_generate_messages_py ${catkin_EXPORTED_TARGETS})
