cmake_minimum_required(VERSION 3.15)
project(SickPerception)

# include(CMakeDependentOption)
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
if(${CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT})	# if an install prefix is defined by not the default, that overrides our default >>
	set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install" CACHE PATH "" FORCE)
	message(STATUS "[SICK PERCEPTION]: Install prefix overridden: ${CMAKE_INSTALL_PREFIX}")
endif()

if(NOT MSVC)
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
		message(STATUS "[SICK PERCEPTION]: Release build type applied by default for non-MSVC build system")
	endif()
endif()

# options
option(sickperception_BUILD_TEST "Build the included test program" ON)
option(sickperception_USE_WPILIB "Enable the compiling and internal usage of Wpilib (results in a multistage build process)" ON)

if(${sickperception_USE_WPILIB})		# alternative solution?
	if(NOT DEFINED sickperception_MULTISTAGE_STEP)
		set(sickperception_MULTISTAGE_STEP -1)
	endif()
else()
	set(sickperception_MULTISTAGE_STEP 1 CACHE INTERNAL "" FORCE)
endif()
if(NOT sickperception_CXX_STANDARD)
	set(sickperception_CXX_STANDARD 20)
endif()

# automatically deduce build stage
if(${sickperception_MULTISTAGE_STEP} LESS 0)
	message(STATUS "[SICK PERCEPTION]: Attempting build stage automatic deduction...")
	if(EXISTS "${CMAKE_INSTALL_PREFIX}/cmake/protobuf-config.cmake" OR EXISTS "${CMAKE_INSTALL_PREFIX}/lib/cmake/protobuf/protobuf-config.cmake") # possibly set variables like for wpilib and use find_package() ???
		set(sickperception_MULTISTAGE_STEP 1)
		message(STATUS "[SICK PERCEPTION]: Protobuf install detected: automatically deduced build stage 1 (project).")
	else()
		set(sickperception_MULTISTAGE_STEP 0)
		message(STATUS "[SICK PERCEPTION]: Protobuf install not detected: automatically deduced build stage 0 (prereq).")
	endif()
endif()

# run a build stage based on variables
if(${sickperception_MULTISTAGE_STEP} EQUAL 0)
	message(STATUS "[SICK PERCEPTION]: Build stage 0 (prereq) running!...")

# protobuf
	set(CMAKE_CXX_STANDARD 20)
	set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
	set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
	set(protobuf_BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
	set(protobuf_BUILD_WITH_ZLIB OFF CACHE BOOL "" FORCE)
	add_subdirectory(submodule-deps/protobuf)


elseif(${sickperception_MULTISTAGE_STEP} GREATER 0)
	message(STATUS "[SICK PERCEPTION]: Build stage 1 (project) running!...")

#wpilib
if(${sickperception_USE_WPILIB})
	if(WIN32)
		set(_PROTOC_EXECUTABLE "protoc.exe")
	else()
		set(_PROTOC_EXECUTABLE "protoc")
	endif()

	add_subdirectory(submodule-deps/eigen)
	# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/submodule-deps/eigen")
	# set(USE_SYSTEM_EIGEN ON)

	set(__BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
	set(BUILD_SHARED_LIBS OFF)
	if(NOT BUILD_SHARED_LIBS AND NOT WIN32)
		add_compile_options(-fPIC)
	endif()
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
	# message(STATUS "[SICK PERCEPTION]: Building sick_scan_xd with ROS version ${ROS_VERSION}")
	# set(CMAKE_CXX_STANDARD ${__CMAKE_CXX_STANDARD})

#ldrp shared library
	# set(CMAKE_CXX_STANDARD ${sickperception_CXX_STANDARD})
	add_subdirectory(src/core)

#main program
	if(${sickperception_BUILD_TEST})
		set(CMAKE_CXX_STANDARD ${sickperception_CXX_STANDARD})
		add_executable(${PROJECT_NAME}_test "src/main.cpp")
		target_link_libraries(${PROJECT_NAME}_test
			PRIVATE ${PROJECT_NAME}_lib
		)
		if(${BUILD_SHARED_LIBS})
			target_compile_definitions(${PROJECT_NAME}_test
				-D_LDRP_DYNAMIC_INTERFACE
			)
		endif()

		# file(GLOB _lidar_configs "${CMAKE_CURRENT_LIST_DIR}/src/*.launch")
		install(TARGETS ${PROJECT_NAME}_test
			DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
		# install(FILES ${_lidar_configs}
		# 	DESTINATION ${CMAKE_INSTALL_BINDIR}
		# )
	endif()


else()
	message(FATAL_ERROR "[SICK PERCEPTION]: Internal branch logic error. Please manually set variable sickperception_MULTISTAGE_STEP to define a build stage.")
endif()
