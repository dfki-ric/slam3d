#ifndef SLAM3D_LASERODOMETRY_HPP
#define SLAM3D_LASERODOMETRY_HPP

#include "Types.hpp"

namespace slam3d
{
	class LaserOdometry
	{
	public:
		LaserOdometry();
		~LaserOdometry();
		
		void addScan(PointCloud::ConstPtr scan);
		void finishSweep(double timestamp);

		PointCloud mSurfacePoints;
		PointCloud mEdgePoints;
		PointCloud mExtraPoints;

		PointCloud mLastSurfacePoints;
		PointCloud mLastEdgePoints;
		PointCloud mLastSweep;
		
	private:
		void extractFeatures(PointCloud::ConstPtr scan);
		void calculatePose();
		void timeShift(PointCloud& pointcloud, double timestamp);
		bool doNonlinearOptimization(int iteration);
		
		double mLaserAngleDeg;
		double mMaxSurfaceAngleDeg;
		double mDistanceRelation;
		
		SearchTree mEdgeTree;
		SearchTree mSurfaceTree;
		
		int mScanSize;
		
		// Transformations, replace with something appropriate
		float transform[6];
		float transformRec[6];
		float transformSum[6];
		
		double mCurrentSweepStart;
		double mLastSweepStart;
		double mLastScanTime;
		double mRelativeSweepTime;
	};
}

#endif