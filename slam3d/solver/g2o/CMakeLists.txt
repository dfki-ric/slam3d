list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/g2o/cmake_modules)

find_package(Cholmod REQUIRED)

# Define solver-g2o library
add_library(solver-g2o
	g2o/g2o/core/batch_stats.cpp
	g2o/g2o/core/cache.cpp
	g2o/g2o/core/estimate_propagator.cpp
	g2o/g2o/core/factory.cpp
	g2o/g2o/core/hyper_dijkstra.cpp
	g2o/g2o/core/hyper_graph_action.cpp
	g2o/g2o/core/hyper_graph.cpp
	g2o/g2o/core/jacobian_workspace.cpp
	g2o/g2o/core/marginal_covariance_cholesky.cpp
	g2o/g2o/core/matrix_structure.cpp
	g2o/g2o/core/optimizable_graph.cpp
	g2o/g2o/core/optimization_algorithm.cpp
	g2o/g2o/core/optimization_algorithm_dogleg.cpp
	g2o/g2o/core/optimization_algorithm_factory.cpp
	g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
	g2o/g2o/core/optimization_algorithm_levenberg.cpp
	g2o/g2o/core/optimization_algorithm_with_hessian.cpp
	g2o/g2o/core/parameter_container.cpp
	g2o/g2o/core/parameter.cpp
	g2o/g2o/core/robust_kernel.cpp
	g2o/g2o/core/robust_kernel_factory.cpp
	g2o/g2o/core/robust_kernel_impl.cpp
	g2o/g2o/core/solver.cpp
	g2o/g2o/core/sparse_optimizer.cpp
	g2o/g2o/core/sparse_optimizer_terminate_action.cpp
	g2o/g2o/solvers/cholmod/solver_cholmod.cpp
	g2o/g2o/types/slam3d/vertex_se3.cpp
	g2o/g2o/types/slam3d/edge_se3.cpp
	edge_direction_prior.cpp
	G2oSolver.cpp
)

target_include_directories(solver-g2o
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/g2o>
		$<INSTALL_INTERFACE:include>
		${EIGEN3_INCLUDE_DIR}
		${CHOLMOD_INCLUDE_DIR}
)

target_link_libraries(solver-g2o
	PUBLIC ${CHOLMOD_LIBRARIES} core
)

# Install header files
install(
	FILES G2oSolver.hpp
	DESTINATION include/slam3d
)

# Install the binaries
install(TARGETS solver-g2o EXPORT solver-g2o-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export the library interface
install(EXPORT solver-g2o-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
)

set_target_properties(solver-g2o PROPERTIES OUTPUT_NAME slam3d_solver_g2o)
