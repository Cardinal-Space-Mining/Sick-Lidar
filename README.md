[![CI](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml)

# Sick-Perception

## Purpose
This project aims to provide an obstacle detection module utilizing SICK lidar hardware and PCL filtering algorithms for usage in autonomous robotics.

## Build System
The project can be built using CMake, **but requires a nonstandard multisage build**. Dependencies are shown below:

| Dependency | Sourcing Method [for this project] | Version/Ref, Relevance | Other Considerations |
|-|-|-|-|
| [sick_scan_xd](https://github.com/Cardinal-Space-Mining/sick_scan_xd) | Included as submodule (now using the Cardinal-Space-Mining fork), specific sections only built staticly within the shared library. | The `csm-main` branch of the Cardinal-Space-Mining fork of the repo, very important. | **Make sure to clone recursively** so that the submodule is pulled! |
| [PCL](https://github.com/PointCloudLibrary/pcl) *(Point Cloud Library)* | Must be manually installed. On linux, the easiest method is to use `sudo apt-get install libpcl-dev`. For windows, PCL provides an installer for each release from their github repository. | Version 1.12.0 or greater is ideal, fairly important. | Make sure to add the relevant directories to PATH when installing on windows (see the PCL docs guide for installation) so that CMake can properly find the libraries and headers. |
| [wpilib](https://github.com/wpilibsuite/allwpilib) | Included as submodule, built using CMake `add_subdirectory` from top level project. | The latest stable/compatible release, not important. | Only the `ntcore`, `wpimath`, `wpinet`, and `wpiutil` modules are actually built. |
| [protobuf](https://github.com/ProtocolBuffers/protobuf) | Included as submodule, built using CMake `add_subdirectory` from top level project. | Version 3.21.12 (latest 3.21.x), very important as newer versions that depend on Abseil fail to link. | Protobuf is not a direct dependency but is a dependency for the wpilib modules that are. Since the protoc compiler needs to be present for wpilib compilation, the build system is now split into two stages; the first of which solely compiles protobuf. |

Because of wpilib's dependence on the protoc compiler while building, protobuf must be built as a prerequisite to the rest of the project. **For a fresh install/build, this simply means that cmake must be configured and run twice**. Internally a search for protobuf cmake files persists to determine what build stage should occur when cmake is configured, thus after the first stage is built and installed, the second stage is enabled.

For convenience, the CMake variables `USE_WPILIB` and `MULTISTAGE_STEP` may be configured to more selectively build the project's subcomponents. All other CMake standard variables may also be configured, although it should be noted that C++20 or greater is required.

**The standard build commands consist of the following:**
- To configure cmake, use `cmake . -B ./BUILD_DIRECTORY`, where "./BUILD_DIRECTORY" is the directory where build occurs.
- To build with cmake directly, use `cmake --build ./BUILD_DIRECTORY --config Release`, where "--config Release" defines the build type and can be ignored or modifed with any of the standard CMake standard build types.
- To install all programs, libraries, headers, and config files, use `cmake --install ./BUILD_DIRECTORY --config Release`. Note that the build type used in the build command must be mirrored here, otherwise a rebuild may be initiated. Also note that the default install location is in the project directory under "install".

**Below lists an example of the commands used to build with a fresh project:**
```
>> STAGE 1 <<
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --config Release
  cmake --install ./cmake-build --config Release

>> STAGE 2 << (configuration and build results should now be different)
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --config Release
  cmake --install ./cmake-build --config Release
```

*Build artifacts [for ubuntu] can also be obtained from Github Actions since the project is built as part of CI, and artifacts are uploaded.*

## External Usage
During a build, the core of the project is compiled to a shared library which links dynamically with sick_scan_xd, PCL, and wpilib. A test executable is also built that links to this library but not it's dependencies. To use the project, simply link to the shared libary and include it's respective headers (make sure that dependencies are retained if moving build artifacts between systems!)
