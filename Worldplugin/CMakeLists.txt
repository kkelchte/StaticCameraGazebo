cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} "/usr/include/OGRE" "/usr/include/OGRE/Paging" "/usr/include/gazebo-5.1/gazebo")

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(camera_world_spawningobjects SHARED camera_world_spawningobjects.cc)
target_link_libraries(camera_world_spawningobjects ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTUBUF_LIBRARIES})

add_library(camera_world_turningobjects SHARED camera_world_turningobjects.cc)
target_link_libraries(camera_world_turningobjects ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTUBUF_LIBRARIES})
