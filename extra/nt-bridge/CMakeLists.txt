cmake_minimum_required(VERSION 3.12)
project(nt-bridge)

if(NOT MSVC)
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
		message(STATUS "[NT BRIDGE]: Release build type applied by default for non-MSVC build system")
	endif()
endif()

message(STATUS "[NT BRIDGE]: Configuring protobuf...")
execute_process(
	COMMAND cmake
		-S ${PROJECT_SOURCE_DIR}/protobuf
		-B ${PROJECT_SOURCE_DIR}/protobuf/cmake-build
		-DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/protobuf/cmake-install
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
set(Protobuf_INCLUDE_DIR		"${PROJECT_SOURCE_DIR}/protobuf/cmake-install/include")
set(Protobuf_PROTOC_EXECUTABLE	"${PROJECT_SOURCE_DIR}/protobuf/cmake-install/bin/${_PROTOC_EXECUTABLE}")	# make sure that this is still compatible with windows!
list(APPEND CMAKE_LIBRARY_PATH	"${PROJECT_SOURCE_DIR}/protobuf/cmake-install/lib")
add_subdirectory(allwpilib)
set(BUILD_SHARED_LIBS ${__BUILD_SHARED_LIBS})
