# Define graph-neo4j library

	add_library(measurements-redis SHARED
		RedisMeasurements.cpp
	)

	target_include_directories(measurements-redis
		PUBLIC
			$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
			$<INSTALL_INTERFACE:include>
	)

	target_link_libraries(measurements-redis
		PUBLIC
			core
			hiredis
	)

	# Install header files
	install(
		FILES RedisMeasurements.hpp
		DESTINATION include/slam3d/measurements/redis/
	)

	# Install the binaries
	install(TARGETS measurements-redis EXPORT slam3d-targets
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
	)

	# Install pkg-config file
	configure_file(slam3d_measurements_redis.pc.in slam3d_measurements_redis.pc @ONLY)
	install(
		FILES ${PROJECT_BINARY_DIR}/slam3d/measurements/redis/slam3d_measurements_redis.pc
		DESTINATION lib/pkgconfig
	)

	set_target_properties(measurements-redis PROPERTIES
		OUTPUT_NAME slam3d_measurements-redis
	)

	# # Build test
	# add_executable(test_neo4j_graph Neo4jGraphTest.cpp)
	# add_dependencies(test_neo4j_graph sensor-pcl)
	# target_link_libraries(test_neo4j_graph Boost::unit_test_framework graph-neo4j sensor-pcl)
	# target_compile_definitions(test_boost_graph PRIVATE BOOST_TEST_DYN_LINK)
	# add_test(neo4j_graph test_neo4j_graph)

	# add_executable(test_neo4j_graph_noboost Main.cpp)
	# add_dependencies(test_neo4j_graph_noboost sensor-pcl)
	# target_link_libraries(test_neo4j_graph_noboost Boost::unit_test_framework graph-neo4j sensor-pcl ${PCL_LIBRARIES})
	# target_compile_definitions(test_neo4j_graph_noboost PRIVATE BOOST_TEST_DYN_LINK)

