cmake_minimum_required(VERSION 3.5)

### project
project(pcl)

# point cloud library
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

### executable
add_executable(run src/main.cpp)

# link libraries
target_link_libraries(run ${PCL_LIBRARIES})
target_link_libraries(run ${EIGEN3_LIBRARIES})