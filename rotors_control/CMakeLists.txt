cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  dynamic_reconfigure
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/cbf_clf.cfg)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lee_position_controller roll_pitch_yawrate_thrust_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  src/qpOASES/include
)




#
# building qpOASES LIBRARY
#
set(QPSOURCE
  src/qpOASES/src/BLASReplacement.cpp
  src/qpOASES/src/Constraints.cpp
  src/qpOASES/src/Indexlist.cpp
  src/qpOASES/src/Matrices.cpp
  src/qpOASES/src/Options.cpp
  src/qpOASES/src/QProblemB.cpp
  src/qpOASES/src/SolutionAnalysis.cpp
  src/qpOASES/src/SubjectTo.cpp
  src/qpOASES/src/Bounds.cpp
  src/qpOASES/src/Flipper.cpp
  src/qpOASES/src/LAPACKReplacement.cpp
  src/qpOASES/src/MessageHandling.cpp
  src/qpOASES/src/OQPinterface.cpp 
  src/qpOASES/src/QProblem.cpp
  src/qpOASES/src/SQProblem.cpp
  src/qpOASES/src/Utils.cpp)
#add_library(${PROJECT_NAME} ${SRCS})


#set(ASCTEC_MAV_MOTION_PLANNING_SRC
#    src/math_function.cpp
#    )



add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)

add_library(roll_pitch_yawrate_thrust_controller
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node ${QPSOURCE} src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_controller_node ${QPSOURCE} 
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

install(TARGETS lee_position_controller roll_pitch_yawrate_thrust_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lee_position_controller_node roll_pitch_yawrate_thrust_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


# make sure configure headers are built before any node using them
add_dependencies(lee_position_controller_node ${PROJECT_NAME}_gencfg)
