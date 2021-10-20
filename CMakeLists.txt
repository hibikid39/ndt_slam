cmake_minimum_required(VERSION 3.0.2)
project(ndt_mapping)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_mapping
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ndt_mapping.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ndt_mapping
src/main.cpp
src/MyUtil.cpp
src/SlamLauncher.cpp
src/FrontEnd.cpp
src/Scan2D.cpp
src/Pose2D.cpp
src/PointCloudMap.cpp
src/PoseEstimator.cpp
src/ScanMatcher.cpp
src/RefScanMaker.cpp
#src/DataAssociator.cpp
#src/CostFunction.cpp
#src/PoseOptimizer.cpp
src/ScanPointResampler.cpp
#src/NNGridTable.cpp
#src/ScanPointAnalyser.cpp
src/PoseFuser.cpp
#src/CovarianceCalculator.cpp
src/TFBroadcaster.cpp
#src/LoopDetector.cpp
#src/BackEnd.cpp
#src/PoseGraph.cpp
#src/P2oDriver2D.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ndt_mapping
  ${catkin_LIBRARIES}
)
target_link_libraries(ndt_mapping
  ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})
