cmake_minimum_required(VERSION 3.13)
project(SickLidar)

find_package(PCL REQUIRED)

# option(LINK_SHARED_LIBS "Link to shared libs (ON) or static libs (OFF)" ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

set(TARGET_DEFINITIONS
	${PCL_DEFINITIONS}
)
add_definitions(${TARGET_DEFINITIONS})


#sources
file(GLOB SOURCES src/*.cpp)
add_executable(${PROJECT_NAME}   ${SOURCES})	#add shared lib target with selection option
#end sources

#includes
set(INCLUDE_BASE_DIR "${PROJECT_SOURCE_DIR}/include")
set(INCLUDE_SUBDIRS
	"sick_scan_xd-3.1.0"
)
set(INCLUDE_EXTERN_DIRS
	${PCL_INCLUDE_DIRS}
)
foreach(_subdir ${INCLUDE_SUBDIRS})
	list(APPEND INCLUDE_INTERNAL_DIRS ${INCLUDE_BASE_DIR}/${_subdir})
endforeach()

list(APPEND INCLUDE_DIRS   ${INCLUDE_INTERNAL_DIRS} ${INCLUDE_EXTERN_DIRS})
#target_include_directories(${PROJECT_NAME}   ${INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
#end includes

#libs
set(LIBS_BASE_DIR "${PROJECT_SOURCE_DIR}/lib")
if(WIN32)
	if(CMAKE_BUILD_TYPE EQUAL Debug)
		set(LIB_INTERN_LINKS
			"${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Debug/sick_scan_xd_shared_lib.lib"		#may need to add msgpack11
			# "${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Debug/sick_scan_xd_shared_lib.dll"
		)
	else()
		set(LIB_INTERN_LINKS
			"${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Release/sick_scan_xd_shared_lib.lib"
			# "${LIBS_BASE_DIR}/windows/sick_scan_xd-3.1.0/Release/sick_scan_xd_shared_lib.dll"
		)
	endif()
elseif(UNIX)
	set(LIB_INTERN_LINKS
		"${LIBS_BASE_DIR}/ubuntu/sick_scan_xd-3.1.0/build-sickxd/libsick_scan_xd_shared_lib.so"
	)
endif()
set(LIB_EXTERN_DIRS
	${PCL_LIBRARY_DIRS}
)
set(LIB_EXTERN_LINKS
	${PCL_LIBRARIES}
)

# target_link_directories(${PROJECT_NAME}   ${LIB_INTERN_DIRS} ${LIB_EXTERN_DIRS})
link_directories(${LIB_INTERN_DIRS} ${LIB_EXTERN_DIRS})
target_link_libraries(${PROJECT_NAME}   ${LIB_INTERN_LINKS} ${LIB_EXTERN_LINKS})
#end libs
