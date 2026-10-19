#include "SplatMeasurement.hpp"

#include <algorithm>

using namespace slam3d;

size_t SplatMeasurement::removeSplatsBySphere(const Eigen::Vector3d& center, double radius, bool inside)
{
	if(!mSplats)
		return 0;

	const size_t count = mSplats->size();
	const Eigen::Vector3f c = center.cast<float>();
	const float radiusSquared = radius * radius;
	mSplats->erase(std::remove_if(mSplats->begin(), mSplats->end(), [&](const SplatPoint& splat)
	{
		const bool isInside = (splat.getVector3fMap() - c).squaredNorm() <= radiusSquared;
		return isInside == inside;
	}), mSplats->end());
	return count - mSplats->size();
}
