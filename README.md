[![CI](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Cardinal-Space-Mining/Sick-Perception/actions/workflows/ci.yml)

# Sick-Perception

## Purpose
This project aims to provide an obstacle detection module utilizing SICK lidar hardware and PCL filtering algorithms for usage in autonomous robotics.

## Build System
The project can be built using CMake. Dependencies are shown below:

| Dependency | Sourcing Method [this project] | Other Considerations |
|-|-|-|
| [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd) | Included as submodule, built using CMake `subdirectory` call from top level CMakeLists.txt | Make sure to clone recursively so that the submodule code is pulled. |
| [Point Cloud Library](https://github.com/PointCloudLibrary/pcl) | Must be manually installed. On linux, the easiest method is to use `sudo apt-get install libpcl-dev`. For windows, PCL provides an installer for each release on their github repo. | Make sure to add the relevant directories to PATH when installing on windows (see the PCL docs guide for installation) so that cmake can properly find the it. |

Currently the only configurations for the project include standard CMake variables. The most useful/notable configurations are accessed using the following commands:
- **To configure CMake with the 'default' presets, use `cmake . -B ./PATH/TO/BUILD/DIRECTORY --preset default`. The default preset applies C++20 as the standard library version and adds the compiler option `-Wall`.**
- **To build the project directly using CMake, use `cmake --build ./PATH/TO/BUILD/DIRECTORY --config Release`, where `Release` can be replaced with any of the 4 standard CMake build types (ex. `Debug`).**

Note the currently there is no standard `install` target setup for the project, and binaries end up getting built in different directories depending on the platform. This will likely be addressed in the future.

*Build artifacts can also be obtained from Github Actions since the project is build as part of CI, and artifacts are uploaded.*

## External Usage
During a build, the core of the project is compiled to a shared library which links dynamically with sick_scan_xd and PCL. A test executable is also built that links to this library but not it's dependencies. To use the project, simply link to the shared libary and include it's respective headers (make sure that dependencies are retained if moving build artifacts between systems!)
