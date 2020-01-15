##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(qt_ground_station)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
message_generation 
roscpp
geometry_msgs
sensor_msgs
mavros
nav_msgs
std_msgs
std_srvs
tf2_ros
tf2_eigen
mavros_msgs
)

find_package(Qt5 COMPONENTS 
Core 
Widgets 
REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  AttitudeReference.msg
  ControlCommand.msg
  ControlOutput.msg
  DroneState.msg
  Mocap.msg
  Topic_for_log.msg
  Trajectory.msg
  TrajectoryPoint.msg
)

add_service_files(
  FILES
  ControlParameter.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
# catkin_package()

##############################################################################
# Qt Environment
##############################################################################

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qt_ground_station/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(qt_ground_station ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(qt_ground_station qt_ground_station_generate_messages_cpp)

target_link_libraries(qt_ground_station ${catkin_LIBRARIES} Qt5::Widgets)
install(TARGETS qt_ground_station RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

