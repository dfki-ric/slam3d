#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/register_point_struct.h>

namespace slam3d
{
	/**
	 * @struct SplatPoint
	 * @brief A 3D gaussian splat as stored in 3D gaussian splatting PLY files.
	 * @details The fields are registered with the names of the PLY properties, so pcl can read
	 * these files directly. Higher order spherical harmonics (f_rest_*) are not stored,
	 * the color is view independent.
	 */
	struct EIGEN_ALIGN16 SplatPoint
	{
		PCL_ADD_POINT4D;

		// log of the standard deviation along the local axes
		union
		{
			float scale[3];
			struct
			{
				float scale_0;
				float scale_1;
				float scale_2;
			};
		};

		// orientation quaternion w, x, y, z (not necessarily normalized)
		union
		{
			float rot[4];
			struct
			{
				float rot_0;
				float rot_1;
				float rot_2;
				float rot_3;
			};
		};

		// logit of the opacity
		float opacity;

		// zero order spherical harmonics (base color) r, g, b
		union
		{
			float f_dc[3];
			struct
			{
				float f_dc_0;
				float f_dc_1;
				float f_dc_2;
			};
		};

		PCL_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef pcl::PointCloud<SplatPoint> SplatCloud;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(slam3d::SplatPoint,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scale_0, scale_0)
	(float, scale_1, scale_1)
	(float, scale_2, scale_2)
	(float, rot_0, rot_0)
	(float, rot_1, rot_1)
	(float, rot_2, rot_2)
	(float, rot_3, rot_3)
	(float, opacity, opacity)
	(float, f_dc_0, f_dc_0)
	(float, f_dc_1, f_dc_1)
	(float, f_dc_2, f_dc_2)
)
