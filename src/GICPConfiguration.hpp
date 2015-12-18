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