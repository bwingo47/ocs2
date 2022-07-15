# Install script for directory: /root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/setup.bash"
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/setup.sh"
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/setup.zsh"
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/ocs2_legged_robot_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ocs2_legged_robot_ros/cmake" TYPE FILE FILES
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/ocs2_legged_robot_rosConfig.cmake"
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/catkin_generated/installspace/ocs2_legged_robot_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ocs2_legged_robot_ros" TYPE FILE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ocs2_legged_robot_ros" TYPE DIRECTORY FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/include/ocs2_legged_robot_ros/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/libocs2_legged_robot_ros.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so"
         OLD_RPATH "/usr/local/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libocs2_legged_robot_ros.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros" TYPE EXECUTABLE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc"
         OLD_RPATH "/usr/local/lib:/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros" TYPE EXECUTABLE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc"
         OLD_RPATH "/usr/local/lib:/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_sqp_mpc")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros" TYPE EXECUTABLE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/ocs2_legged_robot_ros/legged_robot_dummy")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy"
         OLD_RPATH "/usr/local/lib:/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_dummy")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros" TYPE EXECUTABLE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/ocs2_legged_robot_ros/legged_robot_target")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target"
         OLD_RPATH "/usr/local/lib:/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_target")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros" TYPE EXECUTABLE FILES "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib/ocs2_legged_robot_ros/legged_robot_gait_command")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command"
         OLD_RPATH "/usr/local/lib:/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/devel/lib:/opt/ros/noetic/lib:/root/catkin_ws/devel/.private/ocs2_ros_interfaces/lib:/root/catkin_ws/devel/.private/ocs2_legged_robot/lib:/root/catkin_ws/devel/.private/ocs2_sqp/lib:/root/catkin_ws/devel/.private/ocs2_ddp/lib:/root/catkin_ws/devel/.private/ocs2_mpc/lib:/root/catkin_ws/devel/.private/ocs2_qp_solver/lib:/root/catkin_ws/devel/.private/hpipm_catkin/lib:/root/catkin_ws/devel/.private/blasfeo_catkin/lib:/root/catkin_ws/devel/.private/ocs2_centroidal_model/lib:/root/catkin_ws/devel/.private/ocs2_pinocchio_interface/lib:/root/catkin_ws/devel/.private/ocs2_robotic_tools/lib:/root/catkin_ws/devel/.private/ocs2_oc/lib:/root/catkin_ws/devel/.private/ocs2_core/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ocs2_legged_robot_ros/legged_robot_gait_command")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ocs2_legged_robot_ros" TYPE DIRECTORY FILES
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/launch"
    "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/rviz"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/root/docker_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot_ros/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
