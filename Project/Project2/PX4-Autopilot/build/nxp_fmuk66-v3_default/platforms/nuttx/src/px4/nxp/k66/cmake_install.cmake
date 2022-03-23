# Install script for directory: /home/ds/Documents/g16/PX4-Autopilot/platforms/nuttx/src/px4/nxp/k66

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
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/adc/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/board_reset/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/board_critmon/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/led_pwm/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/hrt/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/io_pins/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/tone_alarm/cmake_install.cmake")
  include("/home/ds/Documents/g16/PX4-Autopilot/build/nxp_fmuk66-v3_default/platforms/nuttx/src/px4/nxp/k66/version/cmake_install.cmake")

endif()

