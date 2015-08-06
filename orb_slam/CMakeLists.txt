cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  cv_bridge
  g2o
  dbow2
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Cholmod REQUIRED)
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dbow2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    cv_bridge
    sensor_msgs
    image_transport
    g2o
    dbow2
  DEPENDS
    Cholmod
    Eigen3
  LIBRARIES
    orb_slam
)

# Our includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -O3 -march=native")

# Files that we need to build
add_executable(${PROJECT_NAME}
  src/main.cc
  src/types/Frame.cc
  src/types/KeyFrame.cc
  src/types/KeyFrameDatabase.cc
  src/types/Map.cc
  src/types/MapDatabase.cc
  src/types/MapPoint.cc
  src/threads/LocalMapping.cc
  src/threads/LoopClosing.cc
  src/threads/MapMerging.cc
  src/threads/OrbThread.cc
  src/threads/Relocalization.cc
  src/threads/Tracking.cc
  src/publishers/MapPublisher.cc
  src/publishers/FramePublisher.cc
  src/util/FpsCounter.cc
  src/util/Converter.cc
  src/util/Initializer.cc
  src/util/Optimizer.cc
  src/util/ORBextractor.cc
  src/util/ORBmatcher.cc
  src/util/Sim3Solver.cc
  src/util/PnPsolver.cc
)

# What libraries we need
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
)
