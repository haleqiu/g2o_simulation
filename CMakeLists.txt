cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(dysimulator)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

add_library(${PROJECT_NAME} SHARED
src/matrix_utils.cpp
src/Frame.cpp
src/Point.cpp
src/g2o_vertex_distance.cpp
src/Simulator.cpp
)

find_package(PCL 1.2 REQUIRED)
include_directories(
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

target_link_libraries(${PROJECT_NAME}
${PCL_LIBRARIES}
${OpenCV_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PROJECT_NAME})

add_executable (rigid_body_straight rigid_body_straight.cpp)
target_link_libraries (rigid_body_straight ${PROJECT_NAME})
