find_package(Eigen3 REQUIRED)
if (NOT TARGET Eigen3::Eigen)
	add_library(Eigen3::Eigen INTERFACE IMPORTED)
	set_property(TARGET Eigen3::Eigen APPEND PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(Boost REQUIRED COMPONENTS graph)
find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(g2o REQUIRED)
find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(flann REQUIRED IMPORTED_TARGET flann)

include("${CMAKE_CURRENT_LIST_DIR}/slam3d-targets.cmake")

