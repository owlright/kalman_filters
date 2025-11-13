## Use the variable PROJECT_NAME for changing the target name
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set(PROJECT_NAME ${ProjectId} )

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${PROJECT_NAME})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# include_directories(include)

## Define librariess
add_library(${PROJECT_NAME}_lib
  src/kalman_filters.cpp
  src/ekf.cpp
  src/ukf.cpp
  src/system_simulator.cpp
  src/system_model.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
)

## Define the executable
add_executable(ekf_circle examples/ekf_circle.cpp)
target_include_directories(ekf_circle PUBLIC include)
target_link_libraries(ekf_circle ${PROJECT_NAME}_lib)

add_executable(ukf_test
  test/ukf_test.cpp
)
target_include_directories(ukf_test PUBLIC
 	include
)
target_link_libraries(ukf_test ${PROJECT_NAME}_lib)

add_executable(kf examples/kf.cpp)
target_include_directories(kf PUBLIC include)
target_link_libraries(kf ${PROJECT_NAME}_lib)

set(Pangolin_DIR /Users/sjh/3rdParty/pangolin/build)
find_package(Pangolin REQUIRED)
add_executable(ptj plot_trajectory.cpp)
target_link_libraries(ptj ${Pangolin_LIBRARIES})