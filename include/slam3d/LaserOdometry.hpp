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

		Affine getPose();
		Affine getTransform() {return mTransform;}
		Affine getAccTransform() {return mAccTransform;}

		PointCloud mSurfacePoints;
		PointCloud mEdgePoints;
		PointCloud mExtraPoints;

		PointCloud mLastSurfacePoints;
		PointCloud mLastEdgePoints;
		PointCloud mLastSweep;
		
	private:
		void extractFeatures(PointCloud::ConstPtr scan);
		void calculatePose();
		void findCorrespondences();
		
		void transformToEnd(PointCloud& pc);
		
		double mLaserAngleDeg;
		double mMaxSurfaceAngleDeg;
		double mDistanceRelation;
		
		SearchTree mEdgeTree;
		SearchTree mSurfaceTree;
		
		int mScanSize;
		
		Affine mTransform;
		Affine mAccTransform;
		
		// Timestamp stuff from LOAM
		double mInitialTime;
		double mCurrentSweepStart;
		double mLastSweepStart;
		
		double mCurrentScanTime;
		double mLastScanTime;
		double mRelativeSweepTime;
	};
}

#endif