# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(Ublox
    VERSION 4.0.0
    DESCRIPTION "UBLOX sensor driver"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Fetch units
  FetchContent_Declare(
    units
    GIT_REPOSITORY 	https://github.com/bolderflight/units.git
    GIT_TAG v4.0.4
  )
  FetchContent_MakeAvailable(units)
  # Add the library target
  add_library(ublox
    src/ublox.cpp
    src/ublox.h
  )
  # Link libraries
  target_link_libraries(ublox
    PUBLIC
      core
      units
  )
  # Setup include directories 
  target_include_directories(ublox PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )

  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the spi example target
    add_executable(ublox_example examples/cmake/ublox_example.cc)
    # Add the includes
    target_include_directories(ublox_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(ublox_example
      PRIVATE 
        ublox
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(ublox_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
