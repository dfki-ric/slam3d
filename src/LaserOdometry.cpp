#include <slam3d/LaserOdometry.hpp>

#include <utility>
#include <algorithm>
#include <iostream>

#include <boost/make_shared.hpp>

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
}

LaserOdometry::~LaserOdometry()
{
	
}

void LaserOdometry::addScan(PointCloud::ConstPtr scan)
{
	// Set scan size
	if(mScanSize < 0)
		mScanSize = scan->size();
	
	mEdgePoints.header = scan->header;
	mSurfacePoints.header = scan->header;
	mExtraPoints.header = scan->header;

	extractFeatures(scan);
	calculatePose();
}

void LaserOdometry::extractFeatures(PointCloud::ConstPtr scan)
{
	// Points flagged in this array are filtered from being used as features
	unsigned int cloudSize = scan->points.size();
	int filter[cloudSize];
	memset(filter, 0, sizeof(filter));

	for (int i = 5; i < cloudSize - 6; i++)
	{
		float diffX = scan->points[i + 1].x - scan->points[i].x;
		float diffY = scan->points[i + 1].y - scan->points[i].y;
		float diffZ = scan->points[i + 1].z - scan->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		float depth1 = sqrt(scan->points[i].x * scan->points[i].x + 
			scan->points[i].y * scan->points[i].y +
			scan->points[i].z * scan->points[i].z);

		// Filter points on boundaries of occluded regions
		if (diff > 0.05)
		{
			float depth2 = sqrt(scan->points[i + 1].x * scan->points[i + 1].x + 
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
		float diffX2 = scan->points[i].x - scan->points[i - 1].x;
		float diffY2 = scan->points[i].y - scan->points[i - 1].y;
		float diffZ2 = scan->points[i].z - scan->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

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
				newFeature.intensity = scan->header.stamp;
				largestPickedNum++;
				if (largestPickedNum <= 2)
				{
					mEdgePoints.push_back(newFeature);
				} else if (largestPickedNum <= 20)
				{
					mExtraPoints.push_back(scan->points[i->second]);
				}else
				{
					break;
				}
				
				// Invalidate points nearby
				for (int k = i->second-5; k <= i->second+5; k++)
				{
					float diffX = scan->points[k].x - scan->points[i->second].x;
					float diffY = scan->points[k].y - scan->points[i->second].y;
					float diffZ = scan->points[k].z - scan->points[i->second].z;
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
				newFeature.intensity = scan->header.stamp;
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
					float diffX = scan->points[k].x - scan->points[i->second].x;
					float diffY = scan->points[k].y - scan->points[i->second].y;
					float diffZ = scan->points[k].z - scan->points[i->second].z;
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
	
	// Correspondences for edge points
	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;
		
	for(PointCloud::iterator point_i = mEdgePoints.begin(); point_i < mEdgePoints.end(); point_i++)
	{
		// Transform point to ??? time
		PointType point_i_sh = *point_i; // timeShift(*point_i, timestamp);
		
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
/*
		if (minPointInd2 >= 0)
		{
			tripod1 = laserCloudCornerPtr->points[closestPointInd];
			tripod2 = laserCloudCornerPtr->points[minPointInd2];

			float x0 = extreSel.x;
			float y0 = extreSel.y;
			float z0 = extreSel.z;
			float x1 = tripod1.x;
			float y1 = tripod1.y;
			float z1 = tripod1.z;
			float x2 = tripod2.x;
			float y2 = tripod2.y;
			float z2 = tripod2.z;

			float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
				* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
				+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
				* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
				+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
				* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

			float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

			float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
			+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

			float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
			- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

			float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
			+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

			float ld2 = a012 / l12;

			extreProj = extreSel;
			extreProj.x -= la * ld2;
			extreProj.y -= lb * ld2;
			extreProj.z -= lc * ld2;

			float s = 2 * (1 - 8 * fabs(ld2));

			coeff.x = s * la;
			coeff.y = s * lb;
			coeff.z = s * lc;
			coeff.h = s * ld2;

			if (s > 0.4)
			{
				laserCloudExtreOri->push_back(extreOri);
				//laserCloudExtreSel->push_back(extreSel);
				//laserCloudExtreProj->push_back(extreProj);
				//laserCloudSel->push_back(tripod1);
				//laserCloudSel->push_back(tripod2);
				coeffSel->push_back(coeff);

				if (isPointSel)
				{
					pointSelInd[3 * i] = closestPointInd;
					pointSelInd[3 * i + 1] = minPointInd2;
				}
			}
			else
			{
			//	laserCloudExtreUnsel->push_back(extreSel);
			}
		}
*/	}
	// Correspondences for surface points
}

void LaserOdometry::finishSweep(double timestamp)
{
	mLastSweep = mEdgePoints;
	mLastSweep += mSurfacePoints;
	mLastSweep += mExtraPoints;
	
	mLastEdgePoints = mEdgePoints;
	mLastSurfacePoints = mSurfacePoints;
	
	mEdgeTree.setInputCloud(boost::make_shared<PointCloud>(mLastEdgePoints));
	mSurfaceTree.setInputCloud(boost::make_shared<PointCloud>(mLastSurfacePoints));
	
	// Shouldn't this be done before setting the kdTree?
	timeShift(mLastEdgePoints, timestamp);
	timeShift(mLastSurfacePoints, timestamp);
	
	mEdgePoints.clear();
	mSurfacePoints.clear();
	mExtraPoints.clear();
}

void LaserOdometry::timeShift(PointCloud& pointcloud, double timestamp)
{
	
}