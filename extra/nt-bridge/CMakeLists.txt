cmake_minimum_required(VERSION 3.12)
project(nt_bridge)

if(NOT MSVC)
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
		message(STATUS "[NT BRIDGE]: Release build type applied by default for non-MSVC build system")
	endif()
endif()

set(CMAKE_CXX_STANDARD 20)

message(STATUS "[NT BRIDGE]: Configuring protobuf...")
execute_process(
	COMMAND cmake
		-S ${PROJECT_SOURCE_DIR}/protobuf
		-B ${PROJECT_SOURCE_DIR}/protobuf/cmake-build
		-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		-DCMAKE_CXX_STANDARD=20
		-DCMAKE_BUILD_TYPE=Release
		-Dprotobuf_BUILD_EXAMPLES=OFF
		-Dprotobuf_BUILD_TESTS=OFF
		-Dprotobuf_BUILD_SHARED_LIBS=ON
		-Dprotobuf_BUILD_WITH_ZLIB=OFF
)
message(STATUS "[NT BRIDGE]: Buildig protobuf...")
execute_process(
	COMMAND cmake
		--build ${PROJECT_SOURCE_DIR}/protobuf/cmake-build
		--target install
		--config Release
		--parallel
)

if(WIN32)
	set(_PROTOC_EXECUTABLE "protoc.exe")
else()
	set(_PROTOC_EXECUTABLE "protoc")
endif()

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
add_subdirectory(allwpilib)
set(BUILD_SHARED_LIBS ${__BUILD_SHARED_LIBS})


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(nt_bridge_node "src/main.cpp")
target_link_libraries(nt_bridge_node
	wpiutil
	wpinet
	wpimath
	ntcore
)

ament_target_dependencies(nt_bridge_node
	"rclcpp"
	"nav_msgs"
	"sensor_msgs"
	"geometry_msgs"
)
install(
	TARGETS nt_bridge_node
	DESTINATION lib/${PROJECT_NAME}/)
ament_package()
