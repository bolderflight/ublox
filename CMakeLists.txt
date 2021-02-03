cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(Ublox
	VERSION 1.2.4
	DESCRIPTION "UBLOX sensor driver"
	LANGUAGES CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	include(FetchContent)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/core.git
		GIT_TAG v2.0.3
	)
	FetchContent_MakeAvailable(core)
	# Fetch eigen
	FetchContent_Declare(
		eigen
		GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/eigen.git
		GIT_TAG v1.0.0
	)
	FetchContent_MakeAvailable(eigen)
	# Fetch global-defs
	FetchContent_Declare(
		global_defs
		GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/global_defs.git
		GIT_TAG v1.1.2
	)
	FetchContent_MakeAvailable(global_defs)
	# Add the library target
	add_library(ublox
		src/ublox/ublox.cc
		include/ublox/ublox.h
	)
	# Link libraries
	target_link_libraries(ublox
		PUBLIC
			core
			eigen
			global_defs
	)
	# Setup include directories 
	target_include_directories(ublox PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
endif()

# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)
		# Add the spi example target
		add_executable(uart_example examples/uart_example.cc)
		# Add the includes
		target_include_directories(uart_example PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the example target
		target_link_libraries(uart_example
			PRIVATE 
				ublox
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(uart_example ${MCU})
	endif()
endif()
