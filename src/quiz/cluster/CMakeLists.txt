cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#SET (CMAKE_CXX_COMPILER,  "/usr/bin/clang++")
#add_definitions(-std=c++11)
add_definitions(-std=c++14)

#set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
#set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

#SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
#SET(CMAKE_C_FLAGS_DEBUG "-O0")

set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(quizCluster)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizCluster cluster.cpp ../../render/render.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES})
