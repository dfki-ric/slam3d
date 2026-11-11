#pragma once

#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/sensor/pcl/RegistrationParameters.hpp>
#include <slam3d/sensor/pcl/PointCloudMeasurement.hpp>

#include <pcl/registration/registration.h>

namespace slam3d
{
	struct Reference
	{
		PointCloudMeasurement::Ptr pointcloud;
		pcl::Registration<PointType, PointType>::Ptr registration;
	};
	
	class PointCloudReference : public PoseSensor
	{
	public:
		PointCloudReference(const std::string& n, Graph* g, Logger* l);

		void handleNewVertex(IdType vertex) override;

		void addReference(PointCloudMeasurement::Ptr ref);

	protected:
		std::vector<Reference> mReferences;
		RegistrationParameters mRegParams;
	};
}