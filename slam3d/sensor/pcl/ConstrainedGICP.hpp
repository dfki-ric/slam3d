#pragma once

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>
#include <pcl/registration/gicp.h>


namespace slam3d
{
	class ConstrainedGICP : public pcl::GeneralizedIterativeClosestPoint<PointType, PointType>
	{
	public:
		using base = pcl::GeneralizedIterativeClosestPoint<PointType, PointType>;
		using base::PointCloudSource;
		using base::PointCloudTarget;
		using base::Matrix4;
		ConstrainedGICP () {
			rigid_transformation_estimation_ =
					[this](const PointCloudSource& cloud_src,
							const pcl::Indices& indices_src,
							const PointCloudTarget& cloud_tgt,
							const pcl::Indices& indices_tgt,
							Matrix4& transformation_matrix) {
				estimateConstrainedRigidTransformationNewton(
						cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);

			};
		}
		void estimateConstrainedRigidTransformationNewton(const PointCloudSource& cloud_src,
		                                       const pcl::Indices& indices_src,
		                                       const PointCloudTarget& cloud_tgt,
		                                       const pcl::Indices& indices_tgt,
		                                       Matrix4& transformation_matrix);

	};
} // namespace slam3d
