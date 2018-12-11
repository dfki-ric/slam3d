# SLAM3D

Description
-----------
The SLAM3D library is a standalone framework for multimodal graph based Simultaneous Localization and Mapping. It's main purpose is to hold arbitrary measurements from various sensors (and possibly various agents) in a graph structure. Maps can be created from specific sensor types (e.g. pointclouds with [PCL](http://pointclouds.org/)) using the readings within the graph. Global relaxation is done by an optimization backend, currently [g²o](https://github.com/RainerKuemmerle/g2o).

Compiling
---------
The library can be compiled on Linux with CMake or with the build toolchains from the robotic frameworks [ROS](http://www.ros.org/) and [Rock](http://rock-robotics.org/stable/). For a standalone build, make sure that all dependencies are build and installed before calling "cmake" & "make install".

Run "doxygen Doxyfile" to build the API documentation.

Content
-------
This package contains the following subdirectories:

- [ci] Description file for a Docker container to test the build process
- [slam3d] Contains all header (.h/.hpp) and source files
- [test] Boost-Test modules to verify basic functionality

Contributing
------------
Commits into the master-branch should only be done via merge requests. For project-specific development this repository should be forked into the project's gitlab-group.