# SLAM3D

## Description
The SLAM3D library is a standalone framework for multimodal graph-based Simultaneous Localization and Mapping. Its main purpose is to hold arbitrary measurements from various sensors (and possibly various agents) in a graph structure. Maps can be created from specific sensor types (e.g., pointclouds with [PCL](http://pointclouds.org/)) using the readings within the graph. Global relaxation is done by an optimization backend, currently [g²o](https://github.com/RainerKuemmerle/g2o).

## Dependencies
The following packages are required in order to build SLAM3D. Some of them are optional and the corresponing module will be excluded if not available.

Required:
 - PCL (libpcl-dev)
 - Suitesparse (libsuitesparse-dev)
 - Eigen3 (libeigen3-dev)
 - Boost (libboost-all-dev)
 - g2o (https://github.com/RainerKuemmerle/g2o)

Optional:
 - Pointmatcher (https://github.com/ethz-asl/libpointmatcher) - for 2D laser scans
 - GDAL (libgdal-dev) - for GPS
 
For g2o there is a Ubuntu-Package package as well as a ROS package available for installation. Because of recent issues with incompatibilities of older versions of this library it is suggested to build and install it from source instead of installing any of these binary packages.

## Compiling
The library can be compiled on Linux with CMake or with the build toolchains from the robotic frameworks [ROS](http://www.ros.org/) and [Rock](http://rock-robotics.org/stable/). For a standalone build, make sure that all dependencies are build and installed before calling "cmake" & "make install".

Run "doxygen Doxyfile" to build the API documentation.

## Content
This package contains the following subdirectories:

- [ci] Description file for a Docker container to test the build process
- [slam3d] Contains all header (.h/.hpp) and source files
- [test] Boost-Test modules to verify basic functionality

## Contributing
Commits into the master-branch should only be done via merge requests. For project-specific development this repository should be forked into the project's gitlab-group.
