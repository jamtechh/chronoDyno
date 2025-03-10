# Install script for directory: /home/saad/OneDrive/chrono-wisc/src

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_parsers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_pardisomkl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_mumps/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_matlab/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_irrlicht/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_vsg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_fmi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_cascade/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_modal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_postprocess/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_multicore/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_powerelectronics/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_opengl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_fsi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_gpu/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_vehicle/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_sensor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_synchrono/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_ros/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_swig/chrono_python/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/chrono_swig/chrono_csharp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/saad/OneDrive/chrono-wisc/Build/src/demos/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/filesystem" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/cxxopts" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/HACD" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/HACDv2" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/rapidjson" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/rapidxml" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/tinyobjloader" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/chpf" FILES_MATCHING REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/stb" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_thirdparty" TYPE DIRECTORY FILES "/home/saad/OneDrive/chrono-wisc/src/chrono_thirdparty/variant" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.inl$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono" TYPE FILE FILES "/home/saad/OneDrive/chrono-wisc/Build/chrono/ChConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono" TYPE FILE FILES "/home/saad/OneDrive/chrono-wisc/Build/chrono/ChVersion.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Chrono" TYPE FILE RENAME "chrono-config.cmake" FILES "/home/saad/OneDrive/chrono-wisc/Build/cmake/chrono-config.cmake.install")
endif()

