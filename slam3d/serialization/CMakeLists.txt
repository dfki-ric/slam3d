
add_library(serialization SHARED
	GraphSerialization.cpp
	MeasurementSerialization.cpp
)

target_include_directories(serialization
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(serialization
	PUBLIC
		sensor-pcl
		Boost::serialization
		yaml-cpp

)

# Install header files
install(
	FILES
	GraphSerialization.hpp
	MeasurementSerialization.hpp
	DESTINATION include/slam3d/serialization/
)

# Install the binaries
install(TARGETS serialization EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_serialization.pc.in slam3d_serialization.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/serialization/slam3d_serialization.pc
	DESTINATION lib/pkgconfig
)
