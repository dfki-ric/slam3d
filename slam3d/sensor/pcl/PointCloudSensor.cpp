#include "PointCloudSensor.hpp"
#include "Common.hpp"

#include <slam3d/core/Mapper.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/pcl_config.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#define PI 3.141592654

using namespace slam3d;

PointCloudSensor::PointCloudSensor(const std::string& n, Logger* l)
 : ScanSensor(n, l)
{
	mScanResolution = 0.1;
}

PointCloudSensor::~PointCloudSensor()
{

}

PointCloud::Ptr PointCloudSensor::downsampleScan(PointCloud::Ptr source)
{
	if(mScanResolution > 0)
		return downsample(source, mScanResolution);
	else
		return source;
}

PointCloud::Ptr PointCloudSensor::removeOutliers(PointCloud::Ptr in, double radius, unsigned min_neighbors) const
{
	PointCloud::Ptr out(new PointCloud);
	if(in->size() > 0 && radius > 0 && min_neighbors > 0)
	{
		pcl::RadiusOutlierRemoval<PointType> out_removal;
		out_removal.setInputCloud(in);
		out_removal.setRadiusSearch(radius);
		out_removal.setMinNeighborsInRadius(min_neighbors);
		out_removal.filter(*out);
		return out;
	}else
	{
		return in;
	}
}

PointCloud::Ptr PointCloudSensor::transform(PointCloud::ConstPtr source, const Transform tf) const
{
	PointCloud::Ptr transformedCloud(new PointCloud);
	pcl::transformPointCloud(*source, *transformedCloud, tf.matrix());
	return transformedCloud;
}

PointCloud::Ptr PointCloudSensor::getAccumulatedCloud(const VertexObjectList& vertices) const
{
	PointCloud::Ptr accu(new PointCloud);

	#pragma omp parallel for
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		Measurement::Ptr m = mMapper->getGraph()->getMeasurement(vertices[i].measurementUuid);
		PointCloudMeasurement::Ptr pcl = boost::dynamic_pointer_cast<PointCloudMeasurement>(m);
		if(!pcl)
		{
			mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a point cloud!");
			throw BadMeasurementType();
		}
		
		PointCloud::Ptr tempCloud = transform(pcl->getPointCloud(), (vertices[i].correctedPose * pcl->getSensorPose()));

		#pragma omp critical 
		*accu += *tempCloud; 
	}
	return accu;
}

Measurement::Ptr PointCloudSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const
{
	PointCloud::Ptr cloud = getAccumulatedCloud(vertices);
	PointCloud::Ptr shifted(new PointCloud);
	pcl::transformPointCloud(*cloud, *shifted, pose.inverse().matrix());
	mLogger->message(DEBUG, (boost::format("Patch pointcloud has %1% points.") % cloud->size()).str());
	Measurement::Ptr m(new PointCloudMeasurement(shifted, "AccumulatedPointcloud", mName, Transform::Identity()));
	return m;
}


Constraint::Ptr PointCloudSensor::createConstraint(const Measurement::Ptr& source,
                                                   const Measurement::Ptr& target,
                                                   const Transform& odometry, bool loop)
{
	// Transform guess in sensor frame
	Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();
	
	// Cast to this sensors measurement type
	PointCloudMeasurement::Ptr sourceCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(source);
	PointCloudMeasurement::Ptr targetCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(target);
	if(!sourceCloud || !targetCloud)
	{
		mLogger->message(ERROR, "Measurement given to createConstraint() is not a PointCloud!");
		throw BadMeasurementType();
	}
	
	// For large loops, refine guess by a coarse ICP
	if(loop)
	{
		guess = align(sourceCloud, targetCloud, guess, mCoarseConfiguration);
	}
	
	// Calculate precise alignement with fine ICP
	Transform icp_result = align(sourceCloud, targetCloud, guess, mFineConfiguration);
	
	// Transform back to robot frame
	Transform transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
	Covariance<6> covariance = Covariance<6>::Identity() * mCovarianceScale;
	
	return Constraint::Ptr(new SE3Constraint(mName, transform, covariance.inverse()));
}

PointCloud::Ptr PointCloudSensor::buildMap(const VertexObjectList& vertices) const
{
	return buildMap(vertices, mMapConfig);
}

PointCloud::Ptr PointCloudSensor::buildMap(const VertexObjectList& vertices, const MapConfig& config) const
{
	Clock c;
	timeval start = c.now();
	PointCloud::Ptr map = getAccumulatedCloud(vertices);
	try
	{
		map = crop(map, config.cropBoxMin, config.cropBoxMax);
		map = removeOutliers(map, config.outlierRadius, config.outlierNeighbors);
		map = downsample(map, config.resolution);
	}catch(std::exception &e)
	{
		mLogger->message(ERROR, e.what());
	}
	timeval finish = c.now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated Pointcloud from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	return map;
}

void PointCloudSensor::setRegistrationParameters(const RegistrationParameters& conf, bool coarse)
{
	if(coarse)
	{
		mLogger->message(INFO, " = RegistrationParameters (Coarse) =");
		mCoarseConfiguration = conf;
	}else
	{
		mLogger->message(INFO, " = RegistrationParameters (Fine) =");
		mFineConfiguration = conf;
	}
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());
}

void PointCloudSensor::setScanResolution(double r)
{
	mLogger->message(INFO, (boost::format("scan_resolution:        %1%") % r).str());
	mScanResolution = r;
}

void PointCloudSensor::setMapResolution(double r)
{
	mLogger->message(INFO, (boost::format("map_resolution:         %1%") % r).str());
	mMapConfig.resolution = r;
}

void PointCloudSensor::setMapOutlierRemoval(double r, unsigned n)
{
	mLogger->message(INFO, (boost::format("map_outlier_radius:     %1%") % r).str());
	mLogger->message(INFO, (boost::format("map_outlier_neighbors:  %1%") % n).str());
	mMapConfig.outlierRadius = r;
	mMapConfig.outlierNeighbors = n;
}

void PointCloudSensor::setMapCropBox(const Eigen::Vector4f& min, const Eigen::Vector4f& max)
{
	mMapConfig.cropBoxMin = min;
	mMapConfig.cropBoxMax = max;
}

void PointCloudSensor::fillGroundPlane(PointCloud::Ptr cloud, ScalarType radius)
{
	pcl::SampleConsensusModelPlane<PointType>::Ptr
		model(new pcl::SampleConsensusModelPlane<PointType>(cloud));
	pcl::RandomSampleConsensus<PointType> ransac(model);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();

	Eigen::VectorXf c;
	ransac.getModelCoefficients(c);
	Direction normal(c[0], c[1], c[2]);
	Eigen::Hyperplane<ScalarType, 3> plane(normal, c[3]);
	double angle_inc = mMapConfig.resolution / radius;
	for(ScalarType r = mMapConfig.resolution; r <= radius; r += mMapConfig.resolution)
	{
		Position sample = plane.projection(Position(r,0,0));
		for(ScalarType angle = 0; angle < 2*PI; angle += angle_inc)
		{
			Position rot = Eigen::AngleAxis<ScalarType>(angle, normal).toRotationMatrix() * sample;
			PointType p;
			p.x = rot[0];
			p.y = rot[1];
			p.z = rot[2];
			cloud->push_back(p);
		}
	}
}

void PointCloudSensor::loadPLY(const std::string& path, const std::string& robot)
{
	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	if(ply_reader.read(path, *pcl_cloud) == 0)
	{
		Transform pc_tr(pcl_cloud->sensor_orientation_.cast<ScalarType>());
		pc_tr.translation() = pcl_cloud->sensor_origin_.block(0,0,3,1).cast<ScalarType>();
		PointCloudMeasurement::Ptr initial_map(
			new PointCloudMeasurement(pcl_cloud, robot, mName, pc_tr));
		try
		{
			IdType id = mMapper->getGraph()->addVertex(initial_map, Transform::Identity());
			Constraint::Ptr prior(new PoseConstraint(mName, Transform::Identity(), Covariance<6>::Identity()));
			mMapper->getGraph()->addConstraint(id, 0, prior);
			mLogger->message(INFO, "Successfully loaded initial map.");
		}
		catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding initial point cloud failed: %1%") % e.what()).str());
		}
	}else
	{
		mLogger->message(ERROR, "Could not load initial map.");
	}
}
