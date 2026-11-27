#include "MultiPointCloudSensor.hpp"
#include "Common.hpp"

#include <slam3d/core/Mapper.hpp>

#include <algorithm>
#include <utility>

#include <pcl/common/transforms.h>



namespace slam3d {


PointCloud::Ptr MultiPointCloudSensor::transform(PointCloud::ConstPtr source, const Transform tf) const {
    PointCloud::Ptr transformedCloud(new PointCloud);
    pcl::transformPointCloud(*source, *transformedCloud, tf.matrix());
    return transformedCloud;
}


namespace {

// Returns true if any of the requested tags is contained in 'have'.
bool hasAnyTag(const std::vector<std::string>& have, const std::vector<std::string>& requested)
{
	for (const auto& tag : requested)
	{
		if (std::find(have.begin(), have.end(), tag) != have.end())
		{
			return true;
		}
	}
	return false;
}

}  // namespace

PointCloud::Ptr MultiPointCloudSensor::getAccumulatedCloud(const VertexObjectList& vertices,
                                                     const std::vector<std::string>& tags) const {
    PointCloud::Ptr accu(new PointCloud);

	// Exceptions must not escape an OpenMP structured block, so record a
	// bad-measurement error in a shared flag and throw after the parallel region.
	bool badMeasurementType = false;

	#pragma omp parallel for
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		// Once a bad measurement was seen, cheaply skip the remaining iterations
		// (we cannot break out of an OpenMP for loop).
		#pragma omp flush(badMeasurementType)
		if (badMeasurementType)
		{
			continue;
		}

		const VertexObject& vertex = vertices[i];

		// Decide which measurements of this vertex to accumulate:
		// - No tag filter, or the vertex itself carries a requested tag:
		//   use the vertex' own (combined) measurement, positioned by correctedPose.
		// - Otherwise the vertex is ignored and the tags are searched in its
		//   subMeasurements; every matching subMeasurement is accumulated,
		//   positioned by correctedPose * its own sensor pose.
		// - Main VertexObject has no measurement and no tags given: use combined subs
		std::vector<std::pair<Measurement::Ptr, Transform>> selected;
		Measurement::Ptr m = mMapper->getGraph()->getMeasurement(vertex.measurementUuid);
		if (m && (tags.empty() || hasAnyTag(vertex.tags, tags)))
		{
			selected.emplace_back(m, vertex.correctedPose);
		}
		else
		{
			for (const auto& sub : vertex.subMeasurements)
			{
				if ((!m && tags.size() == 0)|| hasAnyTag(sub.tags, tags))
				{
					Measurement::Ptr sm = mMapper->getGraph()->getMeasurement(sub.measurementUuid);
					selected.emplace_back(sm, vertex.correctedPose * m->getSensorPose());
				}
			}
		}

		for (const auto& sel : selected)
		{
			PointCloudMeasurement::Ptr pcl = boost::dynamic_pointer_cast<PointCloudMeasurement>(sel.first);
			if(!pcl)
			{
				mLogger->message(ERROR, "Measurement in getAccumulatedCloud() is not a point cloud!");
				#pragma omp atomic write
				badMeasurementType = true;
				break;
			}

			PointCloud::Ptr tempCloud = transform(pcl->getPointCloud(), sel.second);

			#pragma omp critical
			*accu += *tempCloud;
		}
	}

	if (badMeasurementType)
	{
		throw BadMeasurementType();
	}
	return accu;
}

Measurement::Ptr MultiPointCloudSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const {
	PointCloud::Ptr cloud = getAccumulatedCloud(vertices);
	PointCloud::Ptr shifted(new PointCloud);
	pcl::transformPointCloud(*cloud, *shifted, pose.inverse().matrix());
	mLogger->message(DEBUG, (boost::format("Patch MultiScanMeasurement has %1% points.") % cloud->size()).str());
	PointCloudMeasurement::Ptr pcm (new PointCloudMeasurement(shifted, "AccumulatedPointcloud", mName, Transform::Identity()));


	// Measurement::Ptr m(new PointCloudMeasurement(clouds, "AccumulatedPointcloud", mName, Transform::Identity()));
	return pcm;
}


Constraint::Ptr MultiPointCloudSensor::createConstraint(const Measurement::Ptr& source,
                                             const Measurement::Ptr& target,
                                             const Transform& odometry,
                                             bool loop)
{

    // Transform guess in sensor frame
    Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();

    // Cast to this sensors measurement type
    PointCloudMeasurement::Ptr sourceCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(source);
    PointCloudMeasurement::Ptr targetCloud = boost::dynamic_pointer_cast<PointCloudMeasurement>(target);
    if (!sourceCloud || !targetCloud) {
        mLogger->message(ERROR, "Measurement given to createConstraint() is not a PointCloudMeasurement!");
        throw BadMeasurementType();
    }

    // For large loops, refine guess by a coarse ICP
    if (loop) {
        guess = align(sourceCloud, targetCloud, guess, mCoarseConfiguration);
    }

    // Calculate precise alignement with fine ICP
    Transform icp_result = align(sourceCloud, targetCloud, guess, mFineConfiguration);

    // Transform back to robot frame
    Transform transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
    Covariance<6> covariance = Covariance<6>::Identity() * mCovarianceScale;

    return Constraint::Ptr(new SE3Constraint(mName, transform, covariance.inverse()));


}

}  // namespace slam3d





