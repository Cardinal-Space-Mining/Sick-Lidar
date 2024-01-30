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
file(GLOB _target_headers "${CMAKE_CURRENT_LIST_DIR}/*.h")
# set(SHARED_TARGET   "${PROJECT_NAME}_Shared")
add_library(${PROJECT_NAME}_shared
	SHARED ${_shared_sources}
)
add_definitions(
	-D_LIB_SOURCE
	${PCL_DEFINITIONS}
)
target_include_directories(${PROJECT_NAME}_shared
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
	PRIVATE "${PROJECT_SOURCE_DIR}/sick_scan/sick_scan_xd/include"
	PRIVATE ${PCL_INCLUDE_DIRS}
)
target_link_directories(${PROJECT_NAME}_shared
	PRIVATE ${PCL_LIBRARY_DIRS}
)
target_link_libraries(${PROJECT_NAME}_shared
	PRIVATE ${PCL_LIBRARIES}
	PRIVATE sick_scan_xd_shared_lib
	# PRIVATE "${PROJECT_SOURCE_DIR}/lib/${SICK_SCAN_LIB_REL}"
)

set_target_properties(${PROJECT_NAME}_shared PROPERTIES PUBLIC_HEADER "lidar_api.h")
install(TARGETS ${PROJECT_NAME}_shared
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
# 	NAMESPACE ${PROJECT_NAME}::
# 	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
# )