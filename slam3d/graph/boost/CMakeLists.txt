# Define graph-boost library
add_library(graph-boost
	BoostGraph.cpp
)

target_include_directories(graph-boost
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(graph-boost
	PUBLIC
		Boost::graph
		Boost::thread
		core
)

# Install header files
install(
	FILES BoostGraph.hpp
	DESTINATION include/slam3d/graph/boost/
)

# Install the binaries
install(TARGETS graph-boost EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_graph_boost.pc.in slam3d_graph_boost.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/graph/boost/slam3d_graph_boost.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(graph-boost PROPERTIES OUTPUT_NAME slam3d_graph_boost)
add_slam3d_library(slam3d_graph_boost)

# Build test
add_executable(test_boost_graph test.cpp)
target_link_libraries(test_boost_graph Boost::unit_test_framework graph-boost)
target_compile_definitions(test_boost_graph PRIVATE -DBOOST_TEST_DYN_LINK)
add_test(boost_graph test_boost_graph)
