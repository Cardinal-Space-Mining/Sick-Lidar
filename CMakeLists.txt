cmake_minimum_required(VERSION 3.15)
project(SickPerception)

include(CMakeDependentOption)
include(GNUInstallDirs)

if(POLICY CMP0144)
	# cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0077)
	# cmake_policy(SET CMP0077 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}" CACHE STRING "" FORCE)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}" CACHE STRING "" FORCE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}" CACHE STRING "" FORCE)
set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install" CACHE STRING "" FORCE)

# options
option(USE_WPILIB "Enable the compiling and internal usage of Wpilib (results in a multistage build process)" OFF)
cmake_dependent_option(MULTISTAGE_STEP
	"Which step (0 or 1) of the multistage build should be configured. Default of -1 for autodetect (unused when wpilib is not included)"
	-1 USE_WPILIB 1
)
# if(${USE_WPILIB})		# alternative solution?
# 	option(MULTISTAGE_STEP "Which step (0 or 1) of the multistage build should be configured. Default of -1 for autodetect (unused when wpilib is not included)" -1)
# else()
# 	set(MULTISTAGE_STEP 1 CACHE INTERNAL "" FORCE)
# endif()
if(NOT PROJECT_CXX_STANDARD)
	set(PROJECT_CXX_STANDARD 17)
endif()

# automatically deduce build stage
if(${MULTISTAGE_STEP} LESS 0)
	message(STATUS "Attempting build stage automatic deduction...")
	if(EXISTS "${CMAKE_INSTALL_PREFIX}/cmake/protobuf-config.cmake" OR EXISTS "${CMAKE_INSTALL_PREFIX}/lib/cmake/protobuf/protobuf-config.cmake") # possibly set variables like for wpilib and use find_package() ???
		set(MULTISTAGE_STEP 1)
		message(STATUS "Protobuf install detected: automatically deduced build stage 1 (project).")
	else()
		set(MULTISTAGE_STEP 0)
		message(STATUS "Protobuf install not detected: automatically deduced build stage 0 (prereq).")
	endif()
endif()

# run a build stage based on variables
if(${MULTISTAGE_STEP} EQUAL 0)
	message(STATUS "Build stage 0 (prereq) running!...")

# protobuf
	# set(CMAKE_CXX_STANDARD 20)
	set(protobuf_BUILD_EXAMPLES OFF CACHE BOOl "" FORCE)
	set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
	set(protobuf_BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
	set(protobuf_BUILD_WITH_ZLIB OFF CACHE BOOL "" FORCE)
	add_subdirectory(submodule-deps/protobuf)


elseif(${MULTISTAGE_STEP} GREATER 0)
	message(STATUS "Build stage 1 (project) running!...")

#wpilib
if(${USE_WPILIB})
	if(WIN32)
		set(_PROTOC_EXECUTABLE "protoc.exe")
	else()
		set(_PROTOC_EXECUTABLE "protoc")
	endif()

	set(__BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
	set(BUILD_SHARED_LIBS ON)
	set(WITH_JAVA OFF CACHE BOOL "" FORCE)
	set(WITH_CSCORE OFF CACHE BOOL "" FORCE)
	set(WITH_WPILIB OFF CACHE BOOL "" FORCE)
	set(WITH_EXAMPLES ON CACHE BOOL "" FORCE)
	set(WITH_TESTS OFF CACHE BOOL "" FORCE)
	set(WITH_GUI OFF CACHE BOOL "" FORCE)
	set(WITH_SIMULATION_MODULES OFF CACHE BOOL "" FORCE)
	set(Protobuf_INCLUDE_DIR		"${CMAKE_INSTALL_PREFIX}/include")
	set(Protobuf_PROTOC_EXECUTABLE	"${CMAKE_INSTALL_PREFIX}/bin/${_PROTOC_EXECUTABLE}")	# make sure that this is still compatible with windows!
	list(APPEND CMAKE_LIBRARY_PATH	"${CMAKE_INSTALL_PREFIX}/lib")
	add_subdirectory(submodule-deps/allwpilib)
	set(BUILD_SHARED_LIBS ${__BUILD_SHARED_LIBS})
endif()

#sick_scan_xd
	# set(ROS_VERSION 0 CACHE BOOL	"Force disable ROS" FORCE)
	# set(LDMRS 0 CACHE BOOL			"Force disable LDMRS" FORCE)
	# set(__CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
	# if(NOT ${CMAKE_CXX_STANDARD} OR ${CMAKE_CXX_STANDARD} GREATER 14)
	# 	set(CMAKE_CXX_STANDARD 14)	# sick_scan_xd fails to build with > C++14
	# endif()
	# add_subdirectory(submodule-deps/sick_scan_xd)
	# message(STATUS "Building sick_scan_xd with ROS version ${ROS_VERSION}")
	# set(CMAKE_CXX_STANDARD ${__CMAKE_CXX_STANDARD})

#ldrp shared library
	# set(CMAKE_CXX_STANDARD ${PROJECT_CXX_STANDARD})
	add_subdirectory(src/shared)

#main program
	set(CMAKE_CXX_STANDARD ${PROJECT_CXX_STANDARD})
	add_executable(${PROJECT_NAME}_test "src/main.cpp")
	target_link_libraries(${PROJECT_NAME}_test
		PRIVATE ${PROJECT_NAME}_shared
	)

	file(GLOB _lidar_configs "${CMAKE_CURRENT_LIST_DIR}/src/*.launch")
	install(TARGETS ${PROJECT_NAME}_test
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	install(FILES ${_lidar_configs}
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)


else()
	message(FATAL_ERROR "Internal branch logic error. Please manually set variable MULTISTAGE_STEP to define a build stage.")
endif()
