include(CMakeFindDependencyMacro)
find_dependency(Eigen3 NO_MODULE)
if (NOT TARGET Eigen3::Eigen)
	find_dependency(Eigen3 REQUIRED)
	add_library(Eigen3::Eigen INTERFACE IMPORTED)
	set_property(TARGET Eigen3::Eigen APPEND PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
endif ()

find_dependency(Boost COMPONENTS graph)
find_dependency(PCL 1.7 COMPONENTS registration)
find_dependency(g2o)
find_dependency(OpenGL)
include("${CMAKE_CURRENT_LIST_DIR}/slam3d-targets.cmake")

