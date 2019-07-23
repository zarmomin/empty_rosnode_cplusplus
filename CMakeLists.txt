cmake_minimum_required(VERSION 2.8.3)
project(empty_rosnode_cplusplus)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
#add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_executable(${PROJECT_NAME} src/empty_rosnode_cplusplus.cpp src/some_dummy_class.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
