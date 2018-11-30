find_package(Boost REQUIRED)

# Define graph-boost library
add_library(graph-boost
	BoostGraph.cpp
)

target_link_libraries(graph-boost
	PUBLIC core
)

# Install header files
install(
	FILES BoostGraph.hpp
	DESTINATION include/slam3d
)

# Install the binaries
install(TARGETS graph-boost EXPORT graph-boost-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export the library interface
install(EXPORT graph-boost-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
)

set_target_properties(graph-boost PROPERTIES OUTPUT_NAME slam3d_graph_boost)

