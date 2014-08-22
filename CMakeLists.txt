cmake_minimum_required(VERSION 2.8.3)
project(tue_reasoner_core)

find_package(catkin REQUIRED COMPONENTS
  psi
  roscpp
  roslib
  tf
)

################################################
#                                              #
#       CATKIN SPECIFIC CONFIGURATION          #
#                                              #
################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reasoner
  CATKIN_DEPENDS psi roscpp roslib tf
)

################################################
#                                              #
#                    BUILD                     #
#                                              #
################################################

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

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/3rdparty
    ${catkin_INCLUDE_DIRS}
)

add_library(reasoner
    src/ReasonerServer.cpp
    src/Reasoner.cpp
    src/conversions.cpp
)
target_link_libraries(reasoner ${SWIPL_LIB} ${catkin_LIBRARIES})

add_executable(reasoner_node src/reasoner_node.cpp)
target_link_libraries(reasoner_node reasoner ${SWIPL_LIB})

add_executable(test_reasoner src/test_reasoner.cpp)
target_link_libraries(test_reasoner reasoner)

add_executable(repl src/repl.cpp)
target_link_libraries(repl reasoner)

add_executable(test_swi src/test_swi.cpp)
target_link_libraries(test_swi ${SWIPL_LIB})

