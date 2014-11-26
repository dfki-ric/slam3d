#include <slam3d/LaserOdometry.hpp>

#include <utility>
#include <algorithm>
#include <iostream>

#include <boost/make_shared.hpp>
#include <Eigen/Dense>

#include <pcl/common/transforms.h>

// [LOAM] Zhang, J., & Singh, S. LOAM : Lidar Odometry and Mapping in Real-time.

using namespace slam3d;

typedef std::vector< std::pair<double, unsigned int> > ValueList;

LaserOdometry::LaserOdometry()
{
	mMaxSurfaceAngleDeg = 20;
	mLaserAngleDeg = 0.25;
	
	double sin1 = sin(DEG2RAD(mLaserAngleDeg));
	double sin2 = sin(DEG2RAD(mMaxSurfaceAngleDeg));
	mDistanceRelation = (sin1 * sin1) / (sin2 * sin2);
	
	mScanSize = -1;
	
	mInitialTime = -1;
	mCurrentSweepStart = -1;
	mLastSweepStart = -1;
	
	mTransform = Affine::Identity();
	mAccTransform = Affine::Identity();
}

LaserOdometry::~LaserOdometry()
{
	
}

void LaserOdometry::addScan(PointCloud::ConstPtr scan)
{
	// First scan, do some initializaition
	if(mScanSize < 0)
	{
		mScanSize = scan->size();
		mInitialTime = (double)scan->header.stamp / 1000000;
	}
	mEdgePoints.header = scan->header;
	mSurfacePoints.header = scan->header;
	mExtraPoints.header = scan->header;
	mLastScanTime = mCurrentScanTime;
	mCurrentScanTime = (double)scan->header.stamp / 1000000 - mInitialTime;

	mRelativeSweepTime = (mCurrentScanTime - mCurrentSweepStart) / (mCurrentSweepStart - mLastSweepStart);

	extractFeatures(scan);

	mTransform.translate(Translation(0.1, 0, 0));
}

void LaserOdometry::extractFeatures(PointCloud::ConstPtr scan)
{
	// Points flagged in this array are filtered from being used as features
	unsigned int cloudSize = scan->points.size();
	int filter[cloudSize];
	memset(filter, 0, sizeof(filter));

	for (int i = 5; i < cloudSize - 6; i++)
	{
		ScalarType diffX = scan->points[i + 1].x - scan->points[i].x;
		ScalarType diffY = scan->points[i + 1].y - scan->points[i].y;
		ScalarType diffZ = scan->points[i + 1].z - scan->points[i].z;
		ScalarType diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		ScalarType depth1 = sqrt(scan->points[i].x * scan->points[i].x + 
			scan->points[i].y * scan->points[i].y +
			scan->points[i].z * scan->points[i].z);

		// Filter points on boundaries of occluded regions
		if (diff > 0.05)
		{
			ScalarType depth2 = sqrt(scan->points[i + 1].x * scan->points[i + 1].x + 
				scan->points[i + 1].y * scan->points[i + 1].y +
				scan->points[i + 1].z * scan->points[i + 1].z);

			if (depth1 > depth2)
			{
				diffX = scan->points[i + 1].x - scan->points[i].x * depth2 / depth1;
				diffY = scan->points[i + 1].y - scan->points[i].y * depth2 / depth1;
				diffZ = scan->points[i + 1].z - scan->points[i].z * depth2 / depth1;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
				{
					filter[i - 5] = 1;
					filter[i - 4] = 1;
					filter[i - 3] = 1;
					filter[i - 2] = 1;
					filter[i - 1] = 1;
					filter[i] = 1;
				}
			} else
			{
				diffX = scan->points[i + 1].x * depth1 / depth2 - scan->points[i].x;
				diffY = scan->points[i + 1].y * depth1 / depth2 - scan->points[i].y;
				diffZ = scan->points[i + 1].z * depth1 / depth2 - scan->points[i].z;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
				{
					filter[i + 1] = 1;
					filter[i + 2] = 1;
					filter[i + 3] = 1;
					filter[i + 4] = 1;
					filter[i + 5] = 1;
					filter[i + 6] = 1;
				}
			}
		}

		// Filter points that are raughly parallel using law of sines
		ScalarType diffX2 = scan->points[i].x - scan->points[i - 1].x;
		ScalarType diffY2 = scan->points[i].y - scan->points[i - 1].y;
		ScalarType diffZ2 = scan->points[i].z - scan->points[i - 1].z;
		ScalarType diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		if (diff > mDistanceRelation * depth1 && diff2 > mDistanceRelation * depth1)
		{
			filter[i] = 1;
		}
	}

	// Calculate c-Values [LOAM: V-A (1)]
	ValueList c_values[4];
	unsigned int sectionSize = (cloudSize - 10) / 4.0;
	unsigned int i = 5;
	for(unsigned int section = 0; section < 4; section++)
	{
		for(unsigned int c = 0; c < sectionSize; c++, i++)
		{
			// TODO: Multiply with defined kernel instead of this...
			double diffX = scan->points[i - 5].x + scan->points[i - 4].x 
				+ scan->points[i - 3].x + scan->points[i - 2].x 
				+ scan->points[i - 1].x - 10 * scan->points[i].x 
				+ scan->points[i + 1].x + scan->points[i + 2].x
				+ scan->points[i + 3].x + scan->points[i + 4].x
				+ scan->points[i + 5].x;
			double diffY = scan->points[i - 5].y + scan->points[i - 4].y 
				+ scan->points[i - 3].y + scan->points[i - 2].y 
				+ scan->points[i - 1].y - 10 * scan->points[i].y 
				+ scan->points[i + 1].y + scan->points[i + 2].y
				+ scan->points[i + 3].y + scan->points[i + 4].y
				+ scan->points[i + 5].y;
			double diffZ = scan->points[i - 5].z + scan->points[i - 4].z 
				+ scan->points[i - 3].z + scan->points[i - 2].z 
				+ scan->points[i - 1].z - 10 * scan->points[i].z 
				+ scan->points[i + 1].z + scan->points[i + 2].z
				+ scan->points[i + 3].z + scan->points[i + 4].z
				+ scan->points[i + 5].z;

			c_values[section].push_back(std::make_pair(diffX * diffX + diffY * diffY + diffZ * diffZ, i));
		}

		// Sort the points based on their c-value
		std::sort(c_values[section].begin(), c_values[section].end());
	
		// Select the points with largest c-Value (edges)
		int largestPickedNum = 0;
		for (ValueList::reverse_iterator i = c_values[section].rbegin(); i != c_values[section].rend(); i++)
		{
			if (filter[i->second] == 0 && i->first > 0.1)
			{
				PointType newFeature = scan->points[i->second];
				newFeature.intensity = mCurrentScanTime;
				largestPickedNum++;
				if (largestPickedNum <= 2)
				{
					mEdgePoints.push_back(newFeature);
				} else if (largestPickedNum <= 20)
				{
					mExtraPoints.push_back(newFeature);
				}else
				{
					break;
				}
				
				// Invalidate points nearby
				for (int k = i->second-5; k <= i->second+5; k++)
				{
					ScalarType diffX = scan->points[k].x - scan->points[i->second].x;
					ScalarType diffY = scan->points[k].y - scan->points[i->second].y;
					ScalarType diffZ = scan->points[k].z - scan->points[i->second].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ <= 0.2)
					{
						filter[k] = 1;
					}
				}
			}
		}
	
		// Select the points with smallest c-Value (surfaces)
		int smallestPickedNum = 0;
		for (ValueList::iterator i = c_values[section].begin(); i != c_values[section].end(); i++)
		{
			if (filter[i->second] == 0 && i->first < 0.1)
			{
				PointType newFeature = scan->points[i->second];
				newFeature.intensity = mCurrentScanTime;
				smallestPickedNum++;
				if (smallestPickedNum <= 4)
				{
					mSurfacePoints.push_back(newFeature);
				}else
				{
					mExtraPoints.push_back(newFeature);
				}
					
				// Invalidate points nearby
				for (int k = i->second-5; k <= i->second+5; k++)
				{
					ScalarType diffX = scan->points[k].x - scan->points[i->second].x;
					ScalarType diffY = scan->points[k].y - scan->points[i->second].y;
					ScalarType diffZ = scan->points[k].z - scan->points[i->second].z;
					if (diffX * diffX + diffY * diffY + diffZ * diffZ <= 0.2)
					{
						filter[k] = 1;
					}
				}
			}
		}
	}
}

void LaserOdometry::calculatePose()
{
	if(mLastEdgePoints.size() == 0)
		return;

//	for(int i = 1; i <= 25; i++)
//	{
//		if(doNonlinearOptimization(i))
//		{
//			std::cout << "Optimization succeeded after " << i << " iterations." << std::endl;
//			break;
//		}
//	}
}

void LaserOdometry::findCorrespondences()
{	
	// Some definitions from LOAM
	PointCloud laserCloudExtreOri;
	PointCloud coeffSel;
	
	// Correspondences for edge points
	std::vector<int> pointSearchInd;
	std::vector<ScalarType> pointSearchSqDis;
		
	for(PointCloud::iterator point_i = mEdgePoints.begin(); point_i < mEdgePoints.end(); point_i++)
	{
		// Transform point to ??? time
		PointType point_i_sh = *point_i; // shiftToStart(*point_i);
		
		// Let j be the nearest neighbor of i within the previous sweep
		mEdgeTree.nearestKSearch(point_i_sh, 1, pointSearchInd, pointSearchSqDis);
		int index_j = pointSearchInd[0];
		double time_j = mLastEdgePoints[index_j].intensity;
		
		if (pointSearchSqDis[0] > 1.0)
			continue;
		PointCloud::iterator point_j = mLastEdgePoints.begin() + index_j;

		// Let l be the nearest neighbor of i within an adjacent scan
		int index_l = 1;
		double min_dis_l = 1;
		
		int begin = std::max(0, index_j - (2*mScanSize));
		int end = std::min((int)mLastEdgePoints.size(), index_j + (2*mScanSize));
		for(int l = begin; l < end; l++)
		{
			// Check if it is an adjacent scan via the timestamp (-_-)
			if (!((mLastEdgePoints[l].intensity < time_j - 0.005 && mLastEdgePoints[l].intensity > time_j - 0.07) || 
			      (mLastEdgePoints[l].intensity > time_j + 0.005 && mLastEdgePoints[l].intensity < time_j + 0.07)))
				continue;

			// Calculate distance between Points i and l (why?)
			double sq_dis_i_l = (mLastEdgePoints[l].x - point_i_sh.x) * (mLastEdgePoints[l].x - point_i_sh.x) +
			             (mLastEdgePoints[l].x - point_i_sh.y) * (mLastEdgePoints[l].x - point_i_sh.y) + 
			             (mLastEdgePoints[l].x - point_i_sh.z) * (mLastEdgePoints[l].x - point_i_sh.z);

			if (sq_dis_i_l < min_dis_l)
			{
				min_dis_l = sq_dis_i_l;
				index_l = l;
			}
		}

		// Calculate Jacobian
		if (index_l >= 0)
		{
			PointType tripod1 = mLastEdgePoints[index_j];
			PointType tripod2 = mLastEdgePoints[index_l];

			// Distance to edge correspondence (eq. 2)
			ScalarType x0 = point_i_sh.x;
			ScalarType y0 = point_i_sh.y;
			ScalarType z0 = point_i_sh.z;
			ScalarType x1 = tripod1.x;
			ScalarType y1 = tripod1.y;
			ScalarType z1 = tripod1.z;
			ScalarType x2 = tripod2.x;
			ScalarType y2 = tripod2.y;
			ScalarType z2 = tripod2.z;

			// ||(xi-xj) x (xi-xl)||
			ScalarType a012 = sqrt(
			      ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))   // z: OK
//				+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))   // y: WRONG
				+ ((x0 - x2)*(z0 - z1) - (x0 - x2)*(z0 - z2)) * ((x0 - x2)*(z0 - z1) - (x0 - x2)*(z0 - z2))   // y: OK
				+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))); // x: OK

			// ||(xj-xl)||
			ScalarType l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

			// Distance of i to line (j,l)
			ScalarType ld2 = a012 / l12;

			// Some values needed for calculation of Jacobian
			ScalarType la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
			              +  (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / ld2;

			ScalarType lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
			              -   (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / ld2;

			ScalarType lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
			              +   (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / ld2;

			// What is "s" ???
			if(fabs(ld2) > 0.1)
			{
				ScalarType s = 2 * (1 - 8 * fabs(ld2));
				PointType coeff;
				coeff.x = s * la;
				coeff.y = s * lb;
				coeff.z = s * lc;
				coeff.intensity = s * ld2;

				laserCloudExtreOri.push_back(*point_i);
				coeffSel.push_back(coeff);
			}
		}
	}

	// Correspondences for surface points
	// TODO
	
	
	// laserCloudExtreOri: Feature points from current sweep, that have 
	//                     correspondences in the last sweep
	// coeffSel: ???
	
	// Levenberg-Marquardt-Algorithm
	Eigen::Matrix<ScalarType, Eigen::Dynamic, 6> matA(extrePointSelNum, 6);
	Eigen::Matrix<ScalarType, Eigen::Dynamic, 6> matAt(extrePointSelNum, 6);
	Eigen::Matrix<ScalarType, 6, 6> matAtA;
	Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> matB(extrePointSelNum, 1);
	Eigen::Matrix<ScalarType, 6, 1> matAtB;
	Eigen::Matrix<ScalarType, 6, 1> matX;
	
	for (int i = 0; i < extrePointSelNum; i++)
	{
		PointType extreOri = laserCloudExtreOri[i];
		PointType coeff = coeffSel[i];

		// Scan time / Sweep time
//		ScalarType s = (timeLasted - timeLastedRec) / (startTimeCur - startTimeLast);
		ScalarType s =  (extreOri.intensity - mLastScanTime) / (mCurrentSweepStart- mLastSweepStart);

		ScalarType srx = sin(s * transform[0]);
		ScalarType crx = cos(s * transform[0]);
		ScalarType sry = sin(s * transform[1]);
		ScalarType cry = cos(s * transform[1]);
		ScalarType srz = sin(s * transform[2]);
		ScalarType crz = cos(s * transform[2]);
		ScalarType tx = s * transform[3];
		ScalarType ty = s * transform[4];
		ScalarType tz = s * transform[5];

		ScalarType arx = (-s*crx*sry*srz*extreOri.x + s*crx*crz*sry*extreOri.y + s*srx*sry*extreOri.z 
		+ s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
		+ (s*srx*srz*extreOri.x - s*crz*srx*extreOri.y + s*crx*extreOri.z
		+ s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
		+ (s*crx*cry*srz*extreOri.x - s*crx*cry*crz*extreOri.y - s*cry*srx*extreOri.z
		+ s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

		ScalarType ary = ((-s*crz*sry - s*cry*srx*srz)*extreOri.x 
		+ (s*cry*crz*srx - s*sry*srz)*extreOri.y - s*crx*cry*extreOri.z 
		+ tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
		+ s*tz*crx*cry) * coeff.x
		+ ((s*cry*crz - s*srx*sry*srz)*extreOri.x 
		+ (s*cry*srz + s*crz*srx*sry)*extreOri.y - s*crx*sry*extreOri.z
		+ s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
		- tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

		ScalarType arz = ((-s*cry*srz - s*crz*srx*sry)*extreOri.x + (s*cry*crz - s*srx*sry*srz)*extreOri.y
		+ tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
		+ (-s*crx*crz*extreOri.x - s*crx*srz*extreOri.y
		+ s*ty*crx*srz + s*tx*crx*crz) * coeff.y
		+ ((s*cry*crz*srx - s*sry*srz)*extreOri.x + (s*crz*sry + s*cry*srx*srz)*extreOri.y
		+ tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

		ScalarType atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
		- s*(crz*sry + cry*srx*srz) * coeff.z;

		ScalarType aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
		- s*(sry*srz - cry*crz*srx) * coeff.z;

		ScalarType atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

		ScalarType d2 = coeff.intensity;

		matA(i, 0) = arx;
		matA(i, 1) = ary;
		matA(i, 2) = arz;
		matA(i, 3) = atx;
		matA(i, 4) = aty;
		matA(i, 5) = atz;
		matB(i, 0) = -0.015 * mRelativeSweepTime * d2;
	}
	
	matAt = matA.transpose();
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	matX = matAtA.colPivHouseholderQr().solve(matAtB);

	if (fabs(matX(0, 0)) < 0.005 &&
		fabs(matX(1, 0)) < 0.005 &&
		fabs(matX(2, 0)) < 0.005 &&
		fabs(matX(3, 0)) < 0.01 &&
		fabs(matX(4, 0)) < 0.01 &&
		fabs(matX(5, 0)) < 0.01)
	{
		transform[0] += 0.1 * matX(0, 0);
		transform[1] += 0.1 * matX(1, 0);
		transform[2] += 0.1 * matX(2, 0);
		transform[3] += matX(3, 0);
		transform[4] += matX(4, 0);
		transform[5] += matX(5, 0);
	}else
	{
		std::cout << "Odometry update out of bound!" << std::endl;
	}

	ScalarType deltaR = sqrt(RAD2DEG(matX(0, 0)) * RAD2DEG(matX(0, 0))
	                  + RAD2DEG(matX(1, 0)) * RAD2DEG(matX(1, 0))
	                  + RAD2DEG(matX(2, 0)) * RAD2DEG(matX(2, 0)));
	ScalarType deltaT = sqrt(matX(3, 0) * 100 * matX(3, 0) * 100
	                  + matX(4, 0) * 100 * matX(4, 0) * 100
	                  + matX(5, 0) * 100 * matX(5, 0) * 100);

	if (deltaR < 0.02 && deltaT < 0.02)
	{
		return true;
	}
	return false;
}

void LaserOdometry::finishSweep(double timestamp)
{
	
	mLastEdgePoints = mEdgePoints;
	mLastSurfacePoints = mSurfacePoints;
	mLastSweepStart = mCurrentSweepStart;
	mCurrentSweepStart = timestamp - mInitialTime;
	
	// Transform Feature-Clouds to sweep end time
	transformToEnd(mLastEdgePoints);
	transformToEnd(mLastSurfacePoints);
	transformToEnd(mExtraPoints);
	
	// Add to one pointcloud (this will be input for the mapper)
	mLastSweep = mLastEdgePoints;
	mLastSweep += mLastSurfacePoints;
	mLastSweep += mExtraPoints;
	
	mEdgeTree.setInputCloud(boost::make_shared<PointCloud>(mLastEdgePoints));
	mSurfaceTree.setInputCloud(boost::make_shared<PointCloud>(mLastSurfacePoints));
	
	mEdgePoints.clear();
	mSurfacePoints.clear();
	mExtraPoints.clear();
	
	// Reset Transformations
	mAccTransform = mTransform * mAccTransform;
	mTransform.setIdentity();
}

Affine LaserOdometry::getPose()
{
	return mTransform;
}

void LaserOdometry::transformToEnd(PointCloud& pc)
{
	for(PointCloud::iterator i = pc.begin(); i < pc.end(); i++)
	{
		ScalarType s = (i->intensity - mLastSweepStart) / (mCurrentSweepStart - mLastSweepStart);
		Affine scaledTransform = mTransform;
		scaledTransform.linear() = scaledTransform.linear() * s;
		scaledTransform.translation() = scaledTransform.translation() * s;
		*i = pcl::transformPoint(*i, scaledTransform);
	}
}

PointType LaserOdometry::transformToStart(PointType p)
{
	ScalarType s = (p.intensity - mCurrentSweepStart) / (mCurrentSweepStart - mLastSweepStart);
	Affine scaledTransform = mTransform;
	scaledTransform.linear() = scaledTransform.linear() * s;
	scaledTransform.translation() = scaledTransform.translation() * s;
	return pcl::transformPoint(p, scaledTransform);
}