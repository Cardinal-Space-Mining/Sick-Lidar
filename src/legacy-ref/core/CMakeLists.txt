# NOTE: add logic + defines for disabling wpilib, disabling debug prints, etc.

if(WIN32)
	add_compile_options(/Zc:__cplusplus)
endif()

#sick scan files
set(SICK_SCAN_DIR "${PROJECT_SOURCE_DIR}/submodule-deps/sick_scan_xd")
set(WPILIB_DIR "${PROJECT_SOURCE_DIR}/submodule-deps/allwpilib")

if(${sickperception_USE_WPILIB})	# submodule manually checkout out to the same commit ref as is included with wpilib
	set(EIGEN_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/submodule-deps/eigen" CACHE STRING "" FORCE)
endif()
find_package(PCL REQUIRED COMPONENTS common io filters features kdtree octree segmentation sample_consensus)

# SSXD CMakeLists.txt line 53
# if(NOT WIN32)
# 	add_compile_options(-O3)
# endif()
if (POLICY CMP0022)	# Added CMP0022 and CMP0048 to avoid cmake warnings
	cmake_policy(SET CMP0022 NEW)
endif ()
if (POLICY CMP0048)
	cmake_policy(SET CMP0048 NEW)
endif()
set(CMAKE_CXX_STANDARD ${sickperception_CXX_STANDARD})
# if(NOT ${CMAKE_CXX_STANDARD} OR ${CMAKE_CXX_STANDARD} GREATER 17)
# 	set(CMAKE_CXX_STANDARD 17)	# sick_scan_xd fails to build with > C++14
# endif()
# if(BUILD_LIB_HIDE_FUNCTIONS AND NOT WIN32)
# 	add_compile_options(-fvisibility=hidden)
# 	add_link_options(-fvisibility=hidden)
# endif()
# if(NOT WIN32)	# Switch on, if you use c11-specific commands
# 	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-overflow -fno-var-tracking-assignments")
# endif()
# #SSXD CMakeLists.txt line 97
# add_compile_options(
# 	-D__ROS_VERSION=0
# 	-DROSSIMU)
# #SSXD CMakeLists.txt line 111
# if(WIN32)
# 	add_compile_options(
# 		-D_WINSOCK_DEPRECATED_NO_WARNINGS
# 		-D_CRT_SECURE_NO_WARNINGS
# 		-D_WIN32_WINNT=0x0A00)
# 	link_libraries(ws2_32)
# else()
# 	link_libraries(pthread) # std::thread requires pthread on linux
# 	link_libraries(dl)      # dynamic loading of libraries
# endif()
# if(RASPBERRY EQUAL 1)
# 	add_compile_options(-DRASPBERRY=1)
# else()
# 	add_compile_options(-DRASPBERRY=0)
# endif()
# # SSXD CMakeLists.txt line 370
# add_compile_options(
# 	-DSCANSEGMENT_XD_SUPPORT=1)
include_directories(
	${SICK_SCAN_DIR}/include/sick_scansegment_xd/msgpack11)
file(GLOB SCANSEGMENT_XD_SOURCES
	${SICK_SCAN_DIR}/driver/src/sick_scansegment_xd/*.cpp
	${SICK_SCAN_DIR}/driver/src/sick_scansegment_xd/msgpack11/*.cpp)
# SSXD CMakeLists.txt line 484
include_directories(
	${SICK_SCAN_DIR}/include
	${SICK_SCAN_DIR}/include/sick_scan_xd_api
	${SICK_SCAN_DIR}/include/tinyxml
	${SICK_SCAN_DIR}/include/sick_scan
	${SICK_SCAN_DIR}/roswrap/src/include/launchparser
	${SICK_SCAN_DIR}/roswrap/src/msg_header)
# SSXD CMakeLists.txt line 580
include_directories(
	${SICK_SCAN_DIR}/roswrap/src/include)
if(WIN32)
	include_directories(
		${SICK_SCAN_DIR}/roswrap/helper_win)
	file(GLOB SRC_WIN_FILES
		${SICK_SCAN_DIR}/roswrap/helper_win/usleep/usleep.c)
endif()
include_directories(
	${SICK_SCAN_DIR}/roswrap/src/include
	${SICK_SCAN_DIR}/roswrap/src/rossimu/melodic/include
	${SICK_SCAN_DIR}/roswrap/src/rossimu/kinetic/include
	${SICK_SCAN_DIR}/roswrap/src/cfgsimu
	${SICK_SCAN_DIR}/roswrap/src/toojpeg
	${SICK_SCAN_DIR}/roswrap/src/tools )
file(GLOB SRC_ROSSIMU_FILES
	${SICK_SCAN_DIR}/roswrap/src/cfgsimu/sick_scan/time_modi.cpp
	${SICK_SCAN_DIR}/roswrap/src/rossimu/kinetic/src/rossimu.cpp
	${SICK_SCAN_DIR}/roswrap/src/rossimu/kinetic/src/duration.cpp
	# ${SICK_SCAN_DIR}/roswrap/src/rossimu/kinetic/src/rate.cpp
	# ${SICK_SCAN_DIR}/roswrap/src/tools/sick_scan/pointcloud_utils.cpp
)
set(SICK_SCAN_LIB_SRC
	# ${SICK_SCAN_DIR}/driver/src/abstract_parser.cpp
	# ${SICK_SCAN_DIR}/driver/src/binPrintf.cpp
	# ${SICK_SCAN_DIR}/driver/src/binScanf.cpp
	# ${SICK_SCAN_DIR}/driver/src/dataDumper.cpp
	# ${SICK_SCAN_DIR}/driver/src/helper/angle_compensator.cpp
	${SICK_SCAN_DIR}/driver/src/sick_cloud_transform.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_callback.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_field_mon.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_imu.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_nav_scandata_parser.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_laser.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_monitoring.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_parser.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_generic_radar.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_lmd_scandata_parser.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_common.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_common_nw.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_common_tcp.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_marker.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_messages.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_parse_util.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_services.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_xd_api/api_impl.cpp
	# ${SICK_SCAN_DIR}/driver/src/sick_scan_xd_api/sick_scan_api_converter.cpp
	${SICK_SCAN_DIR}/driver/src/softwarePLL.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/Mutex.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/SickThread.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/Time.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/colaa.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/colab.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/errorhandler.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/tcp.cpp
	# ${SICK_SCAN_DIR}/driver/src/tcp/toolbox.cpp
	${SICK_SCAN_DIR}/driver/src/tcp/wsa_init.cpp
	# ${SICK_SCAN_DIR}/driver/src/tinyxml/tinystr.cpp
	# ${SICK_SCAN_DIR}/driver/src/tinyxml/tinyxml.cpp
	# ${SICK_SCAN_DIR}/driver/src/tinyxml/tinyxmlerror.cpp
	# ${SICK_SCAN_DIR}/driver/src/tinyxml/tinyxmlparser.cpp
	# ${SICK_SCAN_DIR}/roswrap/src/getopt/getopt.c
	# ${SICK_SCAN_DIR}/roswrap/src/launchparser/launchparser.cpp
	# ${SICK_SCAN_DIR}/roswrap/src/toojpeg/toojpeg.cpp
	${SCANSEGMENT_XD_SOURCES}
	${SRC_ROSSIMU_FILES}
	${SRC_WIN_FILES} )
# SSXD CMakeLists.txt line 643 (equivalent)
add_library(SickScanInternal_lib STATIC ${SICK_SCAN_LIB_SRC})

# converted options from above specific to lib target
target_compile_options(SickScanInternal_lib PUBLIC
	-D__ROS_VERSION=0
	-DROSSIMU
	-DSCANSEGMENT_XD_SUPPORT=1
	PRIVATE -DBOOST_NO_AUTO_PTR)
if(WIN32)
	target_compile_options(SickScanInternal_lib PUBLIC
		-D_WINSOCK_DEPRECATED_NO_WARNINGS
		-D_CRT_SECURE_NO_WARNINGS
		-D_WIN32_WINNT=0x0A00)
	target_link_libraries(SickScanInternal_lib ws2_32)
else()
	target_compile_options(SickScanInternal_lib PUBLIC
		-O3
		-fPIC
		-Wno-format-overflow
		-fno-var-tracking-assignments)
	target_link_libraries(SickScanInternal_lib pthread dl)	# std::thread requires pthread on linux, dynamic loading of libraries
	if(BUILD_LIB_HIDE_FUNCTIONS)
		target_compile_options(SickScanInternal_lib PUBLIC
			-fvisibility=hidden)
		target_link_options(SickScanInternal_lib PUBLIC
			-fvisibility=hidden)
	endif()
endif()
if(RASPBERRY EQUAL 1)
	target_compile_options(SickScanInternal_lib
		PUBLIC -DRASPBERRY=1)
else()
	target_compile_options(SickScanInternal_lib
		PUBLIC -DRASPBERRY=0)
endif()




# Expose core library macro options
option(LDRP_ENABLE_LOGGING "Enable logging for the core library" ON)
option(LDRP_ENABLE_DEBUG_LOGGING "Enable DEBUG LEVEL logging for the core library" ON)
option(LDRP_ENABLE_SAFETY_CHECKS "Enable additional safety checks for the core library" ON)
option(LDRP_USE_UE_SIM_POINTS "Enable unreal engine simulation as point source" OFF)
option(LDRP_USE_INTERNAL_SIM_POINTS "Enable internal simulation as point source" OFF)
option(LDRP_USE_ECHO_POINTS "Enable the use of all echo points from the scanner" OFF)
option(LDRP_USE_PRELIM_FILTERING "Enable filtering of point azimuth angle and minimum range" ON)
option(LDRP_ENABLE_TUNING "Enable live tuning via networktables" ON)
option(LDRP_ENABLE_PROFILING "Enable filter pipeline profiling data via networktables" OFF)

if(${LDRP_ENABLE_LOGGING})
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_LOGGING=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_LOGGING=false)
endif()
if(${LDRP_ENABLE_DEBUG_LOGGING})
	list(APPEND _PROJECT_DEFINES -DLDRP_DEBUG_LOGGING=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_DEBUG_LOGGING=false)
endif()
if(${LDRP_ENABLE_SAFETY_CHECKS})
	list(APPEND _PROJECT_DEFINES -DLDRP_SAFETY_CHECKS=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_SAFETY_CHECKS=false)
endif()
if(${LDRP_USE_UE_SIM_POINTS})
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_SIM_MODE=2)
elseif(${LDRP_USE_INTERNAL_SIM_POINTS})
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_SIM_MODE=1)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_SIM_MODE=false)		# default of 0 when both sim options are false (use live scanner)
endif()
if(${LDRP_USE_ECHO_POINTS})
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_ALL_ECHOS=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_ALL_ECHOS=false)
endif()
if(${LDRP_USE_PRELIM_FILTERING})
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_PRELIM_POINT_FILTERING=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_PRELIM_POINT_FILTERING=false)
endif()
if(${LDRP_ENABLE_TUNING})
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_NT_TUNING=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_NT_TUNING=false)
endif()
if(${LDRP_ENABLE_PROFILING})
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_NT_PROFILING=true)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_ENABLE_NT_PROFILING=false)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS OFF)
set(CMAKE_CXX_STANDARD ${sickperception_CXX_STANDARD})

file(GLOB _shared_sources "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
file(GLOB _target_headers "${CMAKE_CURRENT_LIST_DIR}/*.h")
if(${sickperception_USE_WPILIB})	# compile DataLogManager directly into our library
	list(APPEND _shared_sources "${WPILIB_DIR}/ntcoreffi/src/main/native/cpp/DataLogManager.cpp")
	list(APPEND _include_dirs "${WPILIB_DIR}/ntcoreffi/src/main/native/include")
else()
	list(APPEND _include_dirs "${WPILIB_DIR}/wpiutil/src/main/native/thirdparty/fmtlib/include")
endif()
# set(SHARED_TARGET   "${PROJECT_NAME}_lib")
if(${BUILD_SHARED_LIBS})
	message(STATUS "[SICK PERCEPTION]: Project configured to build SHARED library")
	add_library(${PROJECT_NAME}_lib
		SHARED ${_shared_sources}
	)
else()
	message(STATUS "[SICK PERCEPTION]: Project configured to build STATIC library")
	add_library(${PROJECT_NAME}_lib
		STATIC ${_shared_sources}
	)
endif()
target_include_directories(${PROJECT_NAME}_lib
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
	PRIVATE ${_include_dirs}
	PRIVATE ${PCL_INCLUDE_DIRS}
	# PRIVATE "${PROJECT_SOURCE_DIR}/submodule-deps/sick_scan_xd/include"
)
target_link_directories(${PROJECT_NAME}_lib
	PRIVATE ${PCL_LIBRARY_DIRS}
)
# message(STATUS "PCL Libraries: ${PCL_LIBRARIES}")
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	target_link_libraries(${PROJECT_NAME}_lib
		PRIVATE debug ${PCL_LIBRARIES}
		PRIVATE general SickScanInternal_lib
	)
else()
	target_link_libraries(${PROJECT_NAME}_lib
		PRIVATE pcl_common pcl_io pcl_octree #PCL_FILTERS
		PRIVATE general SickScanInternal_lib
	)
endif()
if(${sickperception_USE_WPILIB})
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_WPILIB=true)
	target_link_libraries(${PROJECT_NAME}_lib
		PRIVATE wpiutil
		PRIVATE wpinet
		PRIVATE wpimath
		PRIVATE ntcore
	)
else()
	list(APPEND _PROJECT_DEFINES -DLDRP_USE_WPILIB=false)
endif()
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
	-D_LDRP_SOURCE
	${_PROJECT_DEFINES}
	${PCL_DEFINITIONS}
)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES PUBLIC_HEADER "lidar_api.h")
install(TARGETS ${PROJECT_NAME}_lib
	EXPORT ${PROJECT_NAME}_Export
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/SickPerception"
	# INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# install(FILES ${_target_headers}	# reuse if we want to export all headers
# 	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )
# install(EXPORT ${PROJECT_NAME}_Export
# 	FILE ${PROJECT_NAME}-config.cmake
# 	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
# )
