# Define graph-neo4j library

	add_library(graph-neo4j SHARED
		Neo4jGraph.cpp
		Neo4jQuery.cpp
		Neo4jConversion.cpp
	)

	target_include_directories(graph-neo4j
		PUBLIC
			$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
			$<INSTALL_INTERFACE:include>
	)

	target_link_libraries(graph-neo4j
		PUBLIC
			# Boost::graph
			# Boost::thread
			cpprest
			crypto
			core
	)

	# Install header files
	install(
		FILES 	Neo4jGraph.hpp 
				Neo4jQuery.hpp
				Neo4jConversion.hpp
		DESTINATION include/slam3d/graph/neo4j/
	)

	# Install the binaries
	install(TARGETS graph-neo4j EXPORT slam3d-targets
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
	)

	# Install pkg-config file
	configure_file(slam3d_graph_neo4j.pc.in slam3d_graph_neo4j.pc @ONLY)
	install(
		FILES ${PROJECT_BINARY_DIR}/slam3d/graph/neo4j/slam3d_graph_neo4j.pc
		DESTINATION lib/pkgconfig
	)

	set_target_properties(graph-neo4j PROPERTIES
		OUTPUT_NAME slam3d_graph_neo4j
	)

	# Build test
	add_executable(test_neo4j_graph Neo4jGraphTest.cpp)
	add_dependencies(test_neo4j_graph sensor-pcl measurements-redis)
	target_link_libraries(test_neo4j_graph Boost::unit_test_framework graph-neo4j sensor-pcl measurements-redis)
	target_compile_definitions(test_boost_graph PRIVATE BOOST_TEST_DYN_LINK)
	add_test(neo4j_graph test_neo4j_graph)