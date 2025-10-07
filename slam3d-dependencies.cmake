# Find all dependencies
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
if (NOT TARGET Eigen3::Eigen)
	add_library(Eigen3::Eigen INTERFACE IMPORTED)
	set_property(TARGET Eigen3::Eigen APPEND PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
endif ()

IF (NOT TARGET JsonCpp::JsonCpp)
	find_package(jsoncpp)
endif()

find_package(g2o REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread graph unit_test_framework serialization)
find_package(PCL 1.8.1 REQUIRED COMPONENTS registration sample_consensus io)
find_package(PCLOMP 1.0)

if (NOT jsoncpp_FOUND)
  pkg_check_modules(jsoncpp IMPORTED_TARGET jsoncpp)
  if (jsoncpp_FOUND)
    add_library(jsoncpp_lib ALIAS PkgConfig::jsoncpp)
  endif()
endif()

# Optional libraries
find_package(libpointmatcher 1.3.1)
find_package(GDAL)
