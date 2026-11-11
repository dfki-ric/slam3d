#include "Common.hpp"

#include <pcl/filters/crop_box.h>

namespace slam3d
{
	PointCloud::Ptr downsample(PointCloud::Ptr in, double leaf_size, double maxrange)
	{
		if(in->size() > 0 && leaf_size > 0)
		{
			PointCloud::Ptr out(new PointCloud);
			pcl::VoxelGrid<PointType> grid;
			if (maxrange>0) {
				grid.setFilterLimits(-maxrange,maxrange);
			}
			grid.setLeafSize (leaf_size, leaf_size, leaf_size);
			grid.setInputCloud(in);
			grid.filter(*out);
			return out;
		}else
		{
			return in;
		}
	}

	PointCloud::Ptr crop(PointCloud::Ptr in, const Eigen::Vector4f& min, const Eigen::Vector4f& max)
	{
		if(min.allFinite() && max.allFinite())
		{
			PointCloud::Ptr out (new PointCloud);
			pcl::CropBox<PointType> cropBox;
			cropBox.setInputCloud(in);
			cropBox.setMin(min);
			cropBox.setMax(max);
			cropBox.filter(*out);
			return out;
		}else
		{
			return in;
		}
	}

	Transform align(PointCloudMeasurement::Ptr source,
					PointCloudMeasurement::Ptr target,
					const Transform& guess,
					const RegistrationParameters& config)
	{
		// Downsample the scans
		PointCloud::Ptr filtered_source = source->getPointCloud();
		PointCloud::Ptr filtered_target = target->getPointCloud();
		if(config.point_cloud_density > 0)
		{
			filtered_source = downsample(source->getPointCloud(), config.point_cloud_density, config.point_cloud_coord_limit);
			filtered_target = downsample(target->getPointCloud(), config.point_cloud_density, config.point_cloud_coord_limit);
		}
		
		// Make sure that there are enough points left (ICP will crash if not)
		if(filtered_target->size() < 100 || filtered_source->size() < 100)
			throw NoMatch("Too few points after filtering, you may have to decrease 'point_cloud_density'.");
		
		// Configure Generalized-ICP
		Transform result;
		switch(config.registration_algorithm)
		{
		case GICP:
			result = doICP< pcl::GeneralizedIterativeClosestPoint<PointType, PointType> >
				(filtered_source, filtered_target, guess, config);
			break;
		case NDT:
			result = doNDT< pcl::NormalDistributionsTransform<PointType, PointType> >
				(filtered_source, filtered_target, guess, config);
			break;
	#ifdef USE_PCLOMP
		case GICP_OMP:
			result = doICP< pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> >
				(filtered_source, filtered_target, guess, config);
			break;
		case NDT_OMP:
			result = doNDT< pclomp::NormalDistributionsTransform<PointType, PointType> >
				(filtered_source, filtered_target, guess, config);
			break;
	#else
		case GICP_OMP:
		case NDT_OMP:
			throw std::runtime_error("OMP is not available, you need to rebuild SLAM3D with OMP or use another matching algorithm.");
	#endif
		default:
			throw std::runtime_error("Unknown registration algorithm specified.");
		}
		
		const Transform delta = guess.inverse() * result;
		const Eigen::AngleAxisd rot(delta.linear());
		if(delta.translation().norm() > config.max_translation || rot.angle() > config.max_rotation)
		{
			throw NoMatch("ICP result is to far away from guess");
		}
		return result;
	}
}