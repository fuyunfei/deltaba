cmake_minimum_required(VERSION 2.8)

project(deltaba)
add_compile_options(-std=c++11)


find_package(Ceres REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}  ${root_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

 
add_executable(deltaba deltaba.cpp functions.cpp BALProblem.cpp)
target_link_libraries(deltaba ${CERES_LIBRARIES} ${PCL_LIBRARIES})
 

