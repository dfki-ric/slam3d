// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM3D_REGISTRATION_PARAMETERS_HPP
#define SLAM3D_REGISTRATION_PARAMETERS_HPP

namespace slam3d
{
	enum RegistrationAlgorithm {ICP, GICP, NDT};

	/**
	 * @class GICPConfiguration
	 * @brief Parameters for the GICP algorithm.
	 */
	struct RegistrationParameters
	{
	// External parameters (not from pcl)
	// ----------------------------------

		// the applied registration algorithm (ICP, GICP, NDT)
		RegistrationAlgorithm registration_algorithm;

		// pointclouds will be downsampled to this density before the alignement
		double point_cloud_density;

		// maximum fitness score (e.g., sum of squared distances from the source to the target)
		// to accept the registration result
		double max_fitness_score;
		
	// General registration parameters
	// -------------------------------

		// maximum allowed distance error before the algorithm will be considered to have converged
		double euclidean_fitness_epsilon;

		// transformation epsilon in order for an optimization to be considered as having converged to the final solution
		double transformation_epsilon;

		// maximum distance threshold between a point and its nearest neighbor
		// correspondent in order to be considered in the alignment process
		double max_correspondence_distance;

		// the maximum number of iterations the internal optimization should run for
		int maximum_iterations;

	// GICP parameters
	// ---------------
	
		// maximum allowable difference between two consecutive rotations in order
		// for an optimization to be considered as having converged to the final solution.
		double rotation_epsilon;

		// number of neighbors to use when computing covariances
		int correspondence_randomness;

		// maximum number of iterations for the optimizer
		int maximum_optimizer_iterations;

	// NDT parameters
	// --------------

		// side length of voxels in the grid
		float resolution;

		// the newton line search maximum step length
		double step_size;

		// point cloud outlier ratio
		double outlier_ratio;

		RegistrationParameters() : registration_algorithm(GICP),
		                           point_cloud_density(0.2),
		                           max_fitness_score(2.0),
		                           euclidean_fitness_epsilon(1.0),
		                           transformation_epsilon(1e-5),
		                           max_correspondence_distance(2.5),
		                           maximum_iterations(50), 
		                           rotation_epsilon(2e-3),
		                           correspondence_randomness(20),
		                           maximum_optimizer_iterations(20),
		                           resolution(1.0),
		                           step_size(0.05),
		                           outlier_ratio(0.35){};
	};
}

#endif