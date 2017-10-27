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

#ifndef SLAM_GICPCONFIGURATION_HPP
#define SLAM_GICPCONFIGURATION_HPP

namespace slam3d
{
		/**
	 * @class GICPConfiguration
	 * @brief Parameters for the GICP algorithm.
	 */
	struct GICPConfiguration
	{
		double max_correspondence_distance;
		int maximum_iterations;
		double transformation_epsilon;
		double euclidean_fitness_epsilon;
		int correspondence_randomness;
		int maximum_optimizer_iterations;
		double rotation_epsilon;
		double point_cloud_density;
		double max_fitness_score;
		double position_sigma;
		double orientation_sigma;
		double max_sensor_distance;

		GICPConfiguration() : max_correspondence_distance(2.5),
		                      maximum_iterations(50), transformation_epsilon(1e-5),
		                      euclidean_fitness_epsilon(1.0), correspondence_randomness(20),
		                      maximum_optimizer_iterations(20), rotation_epsilon(2e-3),
		                      point_cloud_density(0.2), max_fitness_score(2.0),
		                      position_sigma(0.001), orientation_sigma(0.0001), max_sensor_distance(2.0) {};
	};

}

#endif