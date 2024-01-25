# project(LidarProcShared)	# get good

find_package(PCL REQUIRED)

# if(WIN32)
# 	if(CMAKE_BUILD_TYPE EQUAL Debug)
# 		set(SICK_SCAN_LIB_REL "windows/sick_scan_xd-3.1.0/Debug/sick_scan_xd_shared_lib.lib")
# 	else()
# 		set(SICK_SCAN_LIB_REL "windows/sick_scan_xd-3.1.0/Release/sick_scan_xd_shared_lib.lib")
# 	endif()
# elseif(UNIX)
# 	set(SICK_SCAN_LIB_REL "ubuntu/sick_scan_xd-3.1.0/build-sickxd/libsick_scan_xd_shared_lib.so")
# endif()


set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB _shared_sources "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
# set(SHARED_TARGET   "${PROJECT_NAME}_Shared")
add_library(SickLidar_shared
	SHARED ${_shared_sources}
)
add_definitions(
	-D_LIB_SOURCE
	${PCL_DEFINITIONS}
)
target_include_directories(SickLidar_shared
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
	PRIVATE "${PROJECT_SOURCE_DIR}/sick_scan/sick_scan_xd/include"
	PRIVATE ${PCL_INCLUDE_DIRS}
)
target_link_directories(SickLidar_shared
	PRIVATE ${PCL_LIBRARY_DIRS}
)
target_link_libraries(SickLidar_shared
	PRIVATE ${PCL_LIBRARIES}
	PRIVATE sick_scan_xd_shared_lib
	# PRIVATE "${PROJECT_SOURCE_DIR}/lib/${SICK_SCAN_LIB_REL}"
)