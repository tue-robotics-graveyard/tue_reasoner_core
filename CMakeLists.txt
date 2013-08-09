cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Must have <SWI-Prolog.h>
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES ("/usr/lib/swi-prolog/include/SWI-Prolog.h" HAVE_SWI_PROLOG_H)
if(HAVE_SWI_PROLOG_H)
  message(STATUS "<SWI-Prolog.h> found")
  INCLUDE_DIRECTORIES("/usr/lib/swi-prolog/include/")
else(HAVE_SWI_PROLOG_H)
  message(FATAL_ERROR "Can't proceed without <SWI-Prolog.h>")
endif(HAVE_SWI_PROLOG_H)

find_library(SWIPL_LIB NAMES libpl.so libswipl.so)

message("${SWIPL_LIB}")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# include swi-cpp (C++ wrapper for Prolog C-functions)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)

# to show include files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE HEADER_FILES_3RD 3rdparty/*.h)

rosbuild_add_executable(reasoner src/ReasonerServer.cpp
                                 ${HEADER_FILES}
                                 ${HEADER_FILES_3RD}
)

target_link_libraries(reasoner ${SWIPL_LIB})

rosbuild_add_executable(test_reasoner src/test_reasoner.cpp)								 

rosbuild_add_executable(test_swi src/test_swi.cpp)
target_link_libraries(test_swi ${SWIPL_LIB})

