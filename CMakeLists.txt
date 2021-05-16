cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)
#add_definitions(-std=c++11)

#SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")
SET (CMAKE_CXX_FLAGS                "-Wall")
#SET (CMAKE_CXX_FLAGS_DEBUG          "-g")
#SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
#SET (CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
#SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

#set(CXX_SDK_FLAGS "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk")
#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)
#SET (CMAKE_TOOLCHAIN_FILE "/Users/kamesh/work/development/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(playback)

find_package(PCL 1.2 REQUIRED)
#find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

file(GLOB source_files
        "*.h"
        "*.cpp"
        "src/*/*.cpp"
        "src/*/*.h"
        "src/*.cpp"
        )
#add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)

add_library(kcommon src/processPointClouds.cpp src/render/render.cpp)

add_executable (environment src/environment.cpp)
#add_executable(ransac_qz src/quiz/ransac/ransac2d.cpp)

#target_link_libraries (environment ${PCL_LIBRARIES})
target_link_libraries (environment ${PCL_LIBRARIES} kcommon)
#target_link_libraries (ransac_qz ${PCL_LIBRARIES} kcommon)




