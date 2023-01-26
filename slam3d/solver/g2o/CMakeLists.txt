add_library(solver-g2o SHARED
	edge_direction_prior.cpp
	edge_position_prior.cpp
	edge_orientation_prior.cpp
	G2oSolver.cpp
)

target_include_directories(solver-g2o
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(solver-g2o
	PUBLIC
		core
	PRIVATE
		g2o::core
		g2o::stuff
		g2o::solver_cholmod
		g2o::types_slam3d
)

# Install header files
install(
	FILES G2oSolver.hpp
	DESTINATION include/slam3d/solver/g2o
)

# Install the binaries
install(TARGETS solver-g2o EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_solver_g2o.pc.in slam3d_solver_g2o.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/solver/g2o/slam3d_solver_g2o.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(solver-g2o PROPERTIES
	OUTPUT_NAME slam3d_solver_g2o
)

# Build test
add_executable(g2o_solver_test G2oSolverTest.cpp)

target_link_libraries(g2o_solver_test
	Boost::unit_test_framework
	Boost::system
	solver-g2o)

target_compile_definitions(g2o_solver_test PRIVATE -DBOOST_TEST_DYN_LINK)
add_test(g2o_solver g2o_solver_test)
