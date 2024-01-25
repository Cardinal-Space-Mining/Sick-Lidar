cmake_minimum_required(VERSION 3.13)
project(SickLidar)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27.0")
	cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

set(ROS_VERSION 0 CACHE BOOL "Disable ROS!" FORCE)
# set(BUILD_DEBUG_TARGET OFF CACHE BOOL "Build release SickScanXD" FORCE)
add_subdirectory(sick_scan/sick_scan_xd)
message(STATUS "Configured sick_scan_xd with ROS version ${ROS_VERSION}")
add_subdirectory(src/shared)

add_executable(SickLidar_test "src/main.cpp")
target_link_libraries(SickLidar_test
	PRIVATE SickLidar_shared
)





# set(TARGET_DEFINITIONS
# 	${PCL_DEFINITIONS}
# )
# add_definitions(${TARGET_DEFINITIONS})


# #targets
# file(GLOB SHARED_SOURCES src/shared/*.cpp)
# # file(GLOB SOURCES src/*.cpp)
# set(PROGRAM_SOURCES src/main.cpp)

# set(SHARED_TARGET "${PROJECT_NAME}_Shared")
# set(TEST_TARGET "${PROJECT_NAME}_Test")

# add_library(${SHARED_TARGET}   SHARED ${SHARED_SOURCES})
# add_executable(${TEST_TARGET}   ${PROGRAM_SOURCES})	#add shared lib target with selection option
# #end targets

# #includes
# set(INCLUDE_BASE_DIR "${PROJECT_SOURCE_DIR}/include")
# set(INCLUDE_SUBDIRS
# 	"sick_scan_xd-3.1.0"
# )
# set(INCLUDE_SRC_SUBDIRS
# 	"shared"
# )
# set(INCLUDE_EXTERN_DIRS
# 	${PCL_INCLUDE_DIRS}
# )
# foreach(_subdir ${INCLUDE_SUBDIRS})
# 	list(APPEND INCLUDE_INTERNAL_DIRS ${INCLUDE_BASE_DIR}/${_subdir})
# endforeach()
# foreach(_subdir ${INCLUDE_SRC_SUBDIRS})
# 	list(APPEND INCLUDE_INTERNAL_DIRS "src/${_subdir}")
# endforeach()
# list(APPEND INCLUDE_INTERNAL_DIRS "src")

# list(APPEND INCLUDE_DIRS   ${INCLUDE_INTERNAL_DIRS} ${INCLUDE_EXTERN_DIRS})
# #target_include_directories(${PROJECT_NAME}   ${INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})
# #end includes

# #libs
# set(LIBS_BASE_DIR "${PROJECT_SOURCE_DIR}/lib")
# if(WIN32)
# 	if(CMAKE_BUILD_TYPE EQUAL Debug)
# 		set(LIB_INTERN_LINKS
# 			"${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Debug/sick_scan_xd_shared_lib.lib"		#may need to add msgpack11
# 			# "${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Debug/sick_scan_xd_shared_lib.dll"
# 		)
# 	else()
# 		set(LIB_INTERN_LINKS
# 			"${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Release/sick_scan_xd_shared_lib.lib"
# 			# "${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Release/sick_scan_xd_shared_lib.dll"
# 		)
# 	endif()
# elseif(UNIX)
# 	set(LIB_INTERN_LINKS
# 		"${LIBS_BASE_DIR}/ubuntu/sick_scan_xd-3.1.0/build-sickxd/libsick_scan_xd_shared_lib.so"
# 	)
# endif()
# set(LIB_EXTERN_DIRS
# 	${PCL_LIBRARY_DIRS}
# )
# set(LIB_EXTERN_LINKS
# 	${PCL_LIBRARIES}
# )

# # target_link_directories(${PROJECT_NAME}   ${LIB_INTERN_DIRS} ${LIB_EXTERN_DIRS})
# link_directories(${LIB_INTERN_DIRS} ${LIB_EXTERN_DIRS})
# target_link_libraries(${SHARED_TARGET}    PRIVATE ${LIB_INTERN_LINKS} PRIVATE ${LIB_EXTERN_LINKS})
# target_link_libraries(${TEST_TARGET}   PUBLIC ${SHARED_TARGET} ${LIB_INTERN_LINKS} ${LIB_EXTERN_LINKS})
# #end libs
