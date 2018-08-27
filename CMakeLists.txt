project(reader)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.2 REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (reader main.cpp)
target_link_libraries (reader ${PCL_LIBRARIES})
