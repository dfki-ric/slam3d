#pragma once

namespace slam3d
{
	enum RegistrationAlgorithm {ICP, GICP, GICP_OMP, NDT, NDT_OMP};

	/**
	 * @class GICPConfiguration
	 * @brief Parameters for the GICP algorithm.
	 */
	struct RegistrationParameters
	{
	// External parameters (not from pcl)
	// ----------------------------------

		// the applied registration algorithm (ICP, GICP, NDT)
		RegistrationAlgorithm registration_algorithm = GICP;

		// pointclouds will be downsampled to this density before the alignement
		double point_cloud_density = 0.2;

		// point with a coordinage higher than point_cloud_coord_limit or lower than 
		// -point_cloud_coord_limit are ignored, only active when point_cloud_density > 0
		double point_cloud_coord_limit = 80;

		// maximum fitness score (e.g., sum of squared distances from the source to the target)
		// to accept the registration result
		double max_fitness_score = 2.0;
		
		// maximum accepted translation during alignment
		double max_translation = 1.0;
		
		// maximum accepted rotation during alignment
		double max_rotation = 1.0;
		
	// General registration parameters
	// -------------------------------

		// maximum allowed distance error before the algorithm will be considered to have converged
		double euclidean_fitness_epsilon = 1.0;

		// transformation epsilon in order for an optimization to be considered as having converged to the final solution
		double transformation_epsilon = 1e-5;

		// maximum distance threshold between a point and its nearest neighbor
		// correspondent in order to be considered in the alignment process
		double max_correspondence_distance = 2.5;

		// the maximum number of iterations the internal optimization should run for
		int maximum_iterations = 50;

	// GICP parameters
	// ---------------
	
		// maximum allowable difference between two consecutive rotations in order
		// for an optimization to be considered as having converged to the final solution.
		double rotation_epsilon = 2e-3;

		// number of neighbors to use when computing covariances
		int correspondence_randomness = 20;

		// maximum number of iterations for the optimizer
		int maximum_optimizer_iterations = 20;

	// NDT parameters
	// --------------

		// side length of voxels in the grid
		float resolution = 1.0;

		// the newton line search maximum step length
		double step_size = 0.05;

		// point cloud outlier ratio
		double outlier_ratio = 0.35;
	};
}
