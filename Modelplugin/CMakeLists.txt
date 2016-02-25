cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} "/usr/include/OGRE" "/usr/include/OGRE/Paging" "/usr/include/gazebo-5.1/gazebo")

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(camera_move SHARED camera_move.cc)
target_link_libraries(camera_move ${GAZEBO_libraries})

add_library(camera_move_stoch SHARED camera_move_stoch.cc )
target_link_libraries(camera_move_stoch ${GAZEBO_LIBRARIES})

add_library(camera_move_stoch_adapt SHARED camera_move_stoch_adapt.cc )
target_link_libraries(camera_move_stoch_adapt ${GAZEBO_LIBRARIES})

add_library(camera_gt SHARED camera_gt.cc )
target_link_libraries(camera_gt ${GAZEBO_LIBRARIES} CameraPlugin)

add_library(camera_move_test SHARED camera_move_test.cc)
target_link_libraries(camera_move_test ${GAZEBO_libraries})
