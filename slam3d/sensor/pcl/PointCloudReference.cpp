#include "PointCloudReference.hpp"
#include "Common.hpp"

using namespace slam3d;

PointCloudReference::PointCloudReference(const std::string& n, Graph* g, Logger* l)
: PoseSensor(n, g, l)
{
	 
}

void PointCloudReference::handleNewVertex(IdType vertex)
{
	PointCloudMeasurement::Ptr m = boost::dynamic_pointer_cast<PointCloudMeasurement>(mGraph->getMeasurement(vertex));
	if(m)
	{
		VertexObject v = mGraph->getVertex(vertex);
		for(auto& ref : mReferences)
		{
			// Check distance between reference and new vertex
			auto dist = v.correctedPose.translation() - mGraph->getVertex(ref.vertex_id).correctedPose.translation();
			if(dist.norm() > ref.link_radius)
				continue;
			
			Measurement::Ptr m_ref = mGraph->getMeasurement(ref.vertex_id);
			Transform guess = relativeSensorPose(ref.vertex_id, vertex, mGraph);
			PointCloud result;
			ref.registration->setInputSource(m->getPointCloud());
			ref.registration->align(result, guess.matrix().cast<float>());

			if(ref.registration->hasConverged())
			{
				// Get estimated transform
				Transform reg_result(Eigen::Isometry3f(ref.registration->getFinalTransformation()));

				// Transform back to robot frame
				Transform transform = m_ref->getSensorPose() * reg_result * m->getInverseSensorPose();
				Covariance<6> covariance = Covariance<6>::Identity() * mCovarianceScale;
				
				// Add a constraint
				mGraph->addConstraint(ref.vertex_id, vertex,
					Constraint::Ptr(new SE3Constraint(mName, transform, covariance.inverse())));

				mGraph->setCorrectedPose(vertex, mGraph->getVertex(ref.vertex_id).correctedPose * transform);
			}
		}
	}
}

void PointCloudReference::addReference(PointCloudMeasurement::Ptr pc, const Transform& pose, double rad)
{
	Reference ref;
	switch(mRegParams.registration_algorithm)
	{
	case GICP:
		ref.registration.reset(new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>());
		break;
	case NDT:
		ref.registration.reset(new pcl::NormalDistributionsTransform<PointType, PointType>());
		break;
#ifdef USE_PCLOMP
	case GICP_OMP:
		ref.registration.reset(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
		break;
	case NDT_OMP:
		ref.registration.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>());
		break;
#else
	case GICP_OMP:
	case NDT_OMP:
		throw std::runtime_error("OMP is not available, you need to rebuild SLAM3D with OMP or use another matching algorithm.");
#endif
	default:
		throw std::runtime_error("Unknown registration algorithm specified.");
	}
	
	ref.registration->setMaxCorrespondenceDistance(mRegParams.max_correspondence_distance);
	ref.registration->setMaximumIterations(mRegParams.maximum_iterations);
	ref.registration->setTransformationEpsilon(mRegParams.transformation_epsilon);
	ref.registration->setEuclideanFitnessEpsilon(mRegParams.euclidean_fitness_epsilon);
	ref.registration->setInputTarget(pc->getPointCloud());
	
	mGraph->fixNext();
	ref.vertex_id = mGraph->addVertex(pc, pose);
	ref.link_radius = rad;
	mReferences.push_back(ref);
}
