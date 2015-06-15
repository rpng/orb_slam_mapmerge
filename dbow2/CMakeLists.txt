cmake_minimum_required(VERSION 2.8.3)
project(dbow2)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS
    OpenCV
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ")

set(SRCS_DBOW2
  src/BowVector.cpp
  src/FORB.cpp      
  src/FeatureVector.cpp
  src/ScoringObject.cpp)

set(SRCS_DUTILS
  src/dutils/Random.cpp
  src/dutils/Timestamp.cpp)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(
  ${PROJECT_NAME}_lib SHARED 
  ${SRCS_DBOW2} 
  ${SRCS_DUTILS}
)

target_link_libraries(
  ${PROJECT_NAME}_lib 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
