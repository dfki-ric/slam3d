#ifndef SLAM3D_MAPPER_HPP
#define SLAM3D_MAPPER_HPP

#include "PoseGraph.hpp"

namespace slam3d
{
	class Mapper
	{
	public:
		Mapper();
		~Mapper();
		
	private:
		PoseGraph mPoseGraph;
	};
}

#endif