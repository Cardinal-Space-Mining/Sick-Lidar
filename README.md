[![CI](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml)

# Sick-Perception

## Purpose
This project aims to provide an obstacle detection module utilizing SICK lidar hardware and PCL filtering algorithms for usage in autonomous robotics.

## Initialization / Updating
When cloning, make sure to include the `--recurse-submodules` option to pull the project's submodules (required for build). When updating or changing branches, use `git submodule update --init --recursive` to update and pull the correct submodule versions needed.

## Build System
The project can be built using CMake, **but requires a nonstandard multisage build**. Dependencies are listed below:

| Dependency | Sourcing Method [for this project] | Version/Ref, Relevance | Other Considerations |
|-|-|-|-|
| [sick_scan_xd](https://github.com/Cardinal-Space-Mining/sick_scan_xd) | Included as submodule, select source files are included and compiled as part of the main project library. | The `csm-main` branch of the Cardinal-Space-Mining fork of the repo, **very important**. | **Make sure to clone recursively** so that the submodule is pulled! |
| [PCL](https://github.com/PointCloudLibrary/pcl) *(Point Cloud Library)* | Must be manually installed. On linux, the easiest method is to use `sudo apt-get install libpcl-dev`. For windows, PCL provides an installer for each release from their github repository. | Version 1.12.0 or greater is ideal, **fairly important**. | Make sure to add the relevant directories to PATH when installing on windows (see the PCL docs installation guide) so that CMake can properly find the libraries and headers. |
| [wpilib](https://github.com/wpilibsuite/allwpilib) | Included as submodule, built using CMake `add_subdirectory` from top level project. | The latest stable/compatible (currently meaning any 2024.*) release, **fairly important**. | Only the `ntcore`, `wpimath`, `wpinet`, and `wpiutil` modules are actually built. |
| [protobuf](https://github.com/ProtocolBuffers/protobuf) | Included as submodule, built using CMake `add_subdirectory` from top level project. | Version 3.21.12 (latest 3.21.x release), **very important** as newer versions that depend on Abseil fail to link. | Protobuf is not a direct dependency but is a dependency for building the wpimath module. Since the protoc compiler needs to be present for wpilib compilation, the build system is now split into two stages; the first of which solely compiles protobuf. |
| [Eigen](https://gitlab.com/libeigen/eigen) | Included as submodule, not "built" but used to keep wpilib and PCL dependencies the same version. | The specific commit ref as included by the submodule (currently commit hash `9ea520fc`), **very important**. | Only included so that PCL and wpilib eigen dependencies can be forced to be the same. |

Because of wpilib's dependence on the protoc compiler while building, protobuf must be built as a prerequisite to the rest of the project. **For a fresh install/build, this simply means that cmake must be configured and run twice**. Internally a search for protobuf cmake files persists to determine what build stage should occur when cmake is configured, thus after the first stage is built and installed, the second stage is enabled.

For the most part, the project is configured to use the optimal build variables by default, however, the provided options and most common default options are listed here:
- `sickperception_BUILD_TEST` -- Whether or not to build the test executable.
- `sickperception_USE_WPILIB` -- Configures whether or not wpilib is built. Mainly a remnant and debug option since the current code will not work without wpilib.
- `sickperception_MULTISTAGE_STEP` -- Manually force a specific build stage (0 or 1, -1 for auto).
- `sickperception_CXX_STANDARD` -- C++ standard to use for building the project and dependencies. A minimum of C++20 is required.
- `LDRP_ENABLE_LOGGING` -- Enable/disable logging altogether
- `LDRP_ENABLE_DEBUG_LOGGING` -- Enable/disable debug logging in the core library.
- `LDRP_ENABLE_SAFETY_CHECKS` -- Enable/disable "extra" safety checks in the core library.
- `LDRP_USE_UE_SIM_POINTS` -- Enable compiling for testing with the unreal engine simulator. Mutually exclusive with `LDRP_USE_INTERNAL_SIM_POINTS` (enforced sliently, this option takes priority -- **if neither is used, then the program is compiled for use with a physical scanner**).
- `LDRP_USE_INTERNAL_SIM_POINTS` -- Enable compiling for use with the internal point generator (see the previous note).
- `LDRP_USE_ECHO_POINTS` -- Enable/disable the use of all echo points from the live scanner.
- `LDRP_USE_PRELIM_FILTERING` -- Enable/disable preliminary range and azimuth angle filtering (greatly increases loop efficiency if these aren't needed).
- `LDRP_ENABLE_TUNING` -- Enable/disable live tuning via published networktable entries.
- `LDRP_ENABLE_PROFILING` -- Enable/disable filter pipeline profiling (per thread) via networktables.
- `BUILD_SHARED_LIBS` -- Whether or not to build shared libraries (CMake default variable).
- `CMAKE_BUILD_TYPE` -- Used on some build systems (non-MSVC) to define the build type (CMake default variable, defaults to `Release`).
- `CMAKE_INSTALL_PREFIX` -- Where the installed files will be copied (CMake default variable).

**The standard build commands consist of the following:**
- To configure:
  - `cmake -S <SOURCE DIRECTORY> -B <BUILD DIRECTORY>` on MSVC or similar (ie. if on windows), and
  - `cmake -S <SOURCE DIRECTORY> -B <BUILD DIRECTORY> -DCMAKE_BUILD_TYPE=<BUILD_TYPE>` on non-MSVC build systems (ie. if on linux).
  - Additional options can be appended using the format `-D<OPTION>=<OPTION VAL>` (see above). Note that `-DCMAKE_BUILD_TYPE=<BUILD_TYPE>` may be dropped if a release build is required since this is the default value.
- To build (and install):
  - `cmake --build <BUILD DIRECTORY> --target install --config Release (--parallel)` on MSVC or similar (ie. if on windows), and
  - `cmake --build <BUILD DIRECTORY> --target install (--parallel <#parallel>)` on non-MSVC build systems (ie. if on linux).

Where `-S <SOURCE DIRECTORY>` can be replaced with `.` when already in the source directory. Also note that when specifying the install prefix, the same value must be used for both build stages since this is where the configuration searches for the protobuf compiler.

**Below lists an example of the commands used to build a fresh project install on windows:**
```
>> STAGE 1 <<
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --target install --config Release --parallel

>> STAGE 2 << (configuration and build results should now be different)
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --target install --config Release --parallel
```
**And similarly on linux (make sure to specify an applicable --parallel # for your machine!):**
```
>> STAGE 1 <<
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --target install --parallel 4

>> STAGE 2 << (configuration and build results should now be different)
  cmake . -B ./cmake-build
  cmake --build ./cmake-build --target install --parallel 4
```

*Build artifacts [for ubuntu] can also be obtained from Github Actions since the project is built as part of CI, and artifacts are uploaded.*

## External Usage
During a build, the core library as well as wpilib and some parts of sick_scan_xd are compiled to static or shared libs (configurable). In the case of a static build, all necessary dependencies are present in the resulting `SickPerception.*` archive, but for a shared build, all other libs are dynamically linked together and must be present during usage (PCL is always dynamically linked). The easiest way to link to either build type is to use CMake and add the project as a `add_subdirectory()` dependency. Currently a CMake configuration is not exported for the project so a manual specification of header includes (only one header) and link dependences may need to be specified for other use cases.
