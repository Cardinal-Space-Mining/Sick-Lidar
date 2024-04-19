[![CI](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml)

# Sick-Perception

## ⚠️ IMPORTANT ⚠️
Previous versions (branches) of this project were architected to be standalone "pure-C++" modules, however, __this branch has been updated to target ROS\[2\], and offloads sensor interfacing and localization pipeline prerequisites to various external ROS packages__, which are discuessed below. To view the old architecture, check out the [perception-v1](/Cardinal-Space-Mining/Sick-Perception/tree/perception-v1) and/or [perception-v2](/Cardinal-Space-Mining/Sick-Perception/tree/perception-v2) branches. These branches are not likely to be maintained, and as a result may not reflect the same functionality as this branch in the future.

## Purpose
This project aims to provide an obstacle detection module utilizing SICK lidar hardware and PCL filtering algorithms for usage in autonomous robotics.

## System Overview / ROS Dependencies

TODO (make diagrams!)

| Dependency | Required Version/Ref | Description |
| - | - | - |
| [sick_scan_xd](https://github.com/sickag/sick_scan_xd) | The latest release | Lidar interfacing - provices point clouds and IMU measurements |
| [direct lidar inertial odometry](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) | The `feature/ros2` branch | Localization - uses SLAM to produce a location and orientation estimate |

## Initialization / Updating
The project includes submodules for all relevant ROS dependencies (and some vestigial dependencies). These are not required to build the project, but may be utilized as an easy way to pull all required external packages if they need to built and installed. Use the `--recurse-submodules` option when cloning fresh and `git submodule update --init --recursive` for an already cloned project or when changing/updating branches.

## Build System
Since the project targets ROS2, it can be built using Colcon (which uses CMake). Dependencies are listed below:

| Dependency | \[Recommended\] Sourcing Method | Required Version/Ref | Other Considerations |
|-|-|-|-|
| [PCL](https://github.com/PointCloudLibrary/pcl) *(Point Cloud Library)* | Must be manually installed. On linux, the easiest method is to use `sudo apt-get install libpcl-dev`. For windows, the PCL GitHub repository provides an installer for each release. | Versions 1.12.0 and up have been verified to work. | Make sure to add the relevant directories to PATH when installing on windows (see the PCL docs installation guide) so that CMake can properly find the libraries and headers. |
| [Eigen](https://gitlab.com/libeigen/eigen) | Eigen is a dependenciy of PCL and is thus almost gaurenteed to always be present by default. In any case, external versions can used as well, and can be sourced from the link provided. | Not super relevant. Any version compatible with PCL will suffice. | n/a |

For the most part, the project is configured to use the optimal build variables by default, however, the provided options and some common CMake options are listed here:
- `sickperception_USE_WPILIB_DEBUG` -- Configures whether or not WPILib will be included (built) and if dependant logging/debug functionality (NetworkTables) will be enabled.
- `sickperception_CXX_STANDARD` -- C++ standard to use for building the project and dependencies. A minimum of C++17 is required by default and C++20 is required when using WPILib.
- `LDRP_ENABLE_LOGGING` -- Enable/disable internal logging altogether.
- `LDRP_ENABLE_DEBUG_LOGGING` -- Enable/disable internal debug logging.
- `LDRP_ENABLE_SAFETY_CHECKS` -- Enable/disable "extra" safety checks.
- `LDRP_USE_UESIM_NT_INTERFACE` -- Alternatively compiles for testing using the [Unreal Engine Simulator](https://github.com/S1ink/UE5-LidarSim) where points are published via NetworkTables. Requires WPILib to be enabled.
- `LDRP_ENABLE_TUNING` -- Enable/disable live tuning via ROS topics and/or NetworkTables.
- `LDRP_ENABLE_PROFILING` -- Enable/disable pipeline profiling via ROS topics and/or NetworkTables.
- `CMAKE_BUILD_TYPE` -- Used on some build systems (non-MSVC) to define the build type (the project sets this to `Release` if not explicitly configured).
- `CMAKE_INSTALL_PREFIX` -- Where the installed files will be copied (CMake default variable).

  *Note that 'sickperception_\*' refers to a project-wide configuration, while 'LDRP_\*' (stands for LiDaR Perception) refers to a code-specific \[internal\] configuration.*

TODO: build commands + examples here

## Usage

TODO

## Helpful Docs

- [ROS2 Docs](https://docs.ros.org/en/iron/Installation.html)
- [ROS2 Message Types](https://github.com/ros2/common_interfaces) (make sure to select a relevant branch!)
- [Colcon `build` reference](https://colcon.readthedocs.io/en/released/reference/verb/build.html)
