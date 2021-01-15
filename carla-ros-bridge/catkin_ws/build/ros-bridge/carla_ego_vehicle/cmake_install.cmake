# Install script for directory: /home/alejandrosoumah/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ego_vehicle

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/alejandrosoumah/carla-ros-bridge/catkin_ws/build/ros-bridge/carla_ego_vehicle/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/build/ros-bridge/carla_ego_vehicle/catkin_generated/installspace/carla_ego_vehicle.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ego_vehicle/cmake" TYPE FILE FILES
    "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/build/ros-bridge/carla_ego_vehicle/catkin_generated/installspace/carla_ego_vehicleConfig.cmake"
    "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/build/ros-bridge/carla_ego_vehicle/catkin_generated/installspace/carla_ego_vehicleConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ego_vehicle" TYPE FILE FILES "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ego_vehicle/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/carla_ego_vehicle" TYPE PROGRAM FILES "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/build/ros-bridge/carla_ego_vehicle/catkin_generated/installspace/carla_ego_vehicle.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ego_vehicle/launch" TYPE DIRECTORY FILES "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ego_vehicle/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ego_vehicle/config" TYPE DIRECTORY FILES "/home/alejandrosoumah/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ego_vehicle/config/")
endif()

