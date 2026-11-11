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
		
	}
}

void PointCloudReference::addReference(PointCloudMeasurement::Ptr pc)
{
	Reference ref;
	ref.pointcloud = pc;
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
}
