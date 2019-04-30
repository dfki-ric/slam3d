#include "ScanSensor.hpp"

using namespace slam3d;

ScanSensor::ScanSensor(const std::string& n, Logger* l)
: Sensor(n, l)
{
	mICP.setDefault();
}

ScanSensor::~ScanSensor()
{
	
}

bool ScanSensor::addMeasurement(const ScanMeasurement::Ptr& scan, const Transform& odom)
{
	// Always add the first received scan 
	if(mLastVertex == 0)
	{
		mLastVertex = mMapper->addMeasurement(scan);
		mLastOdometry = odom;
		return true;
	}
	
	// Add measurement if sufficient movement is reported by the odometry
	mOdometryDelta.transform = mLastOdometry.inverse() * odom;
	if(!checkMinDistance(mOdometryDelta.transform))
	{
		return false;
	}
	
	// Get the measurement from the last added vertex
	ScanMeasurement::Ptr target_m;
	try
	{
		Measurement::Ptr m = mMapper->getGraph()->getVertex(mLastVertex).measurement;
		target_m = boost::dynamic_pointer_cast<ScanMeasurement>(m);
	}catch(std::exception &e)
	{
		mLogger->message(ERROR, "ScanSensor could not get its last vertex from mapper.");
		return false;
	}
	
	TransformWithCovariance icp_result;
	try
	{
		icp_result = calculateTransform(target_m, scan, mOdometryDelta);
	}catch(NoMatch &e)
	{
		mLogger->message(WARNING, (boost::format("Failed to match new vertex to previous, because %1%.")
			% e.what()).str());
		return false;
	}

	// Add the new vertex and the ICP edge to previous one
	IdType newVertex = mMapper->addMeasurement(scan);
	SE3Constraint::Ptr se3(new SE3Constraint(mName, icp_result));
	mMapper->getGraph()->addConstraint(mLastVertex, newVertex, se3);
	Transform pose = mMapper->getGraph()->getVertex(mLastVertex).corrected_pose * icp_result.transform;
	mMapper->getGraph()->setCorrectedPose(newVertex, pose);
	
	mLastVertex = newVertex;
	mLastOdometry = odom;
	return true;
}

Transform ScanSensor::convert2Dto3D(const PM::TransformationParameters& in)
{
	assert(in.rows() == 3);
	assert(in.cols() == 3);
	Transform out = Transform::Identity();
	out.matrix().block<2,2>(0,0) = in.block<2,2>(0,0);
	out.matrix().block<2,1>(0,3) = in.block<2,1>(0,2);
	return out;
}

PM::TransformationParameters ScanSensor::convert3Dto2D(const Transform& in)
{
	PM::TransformationParameters out(3,3);
	out.setIdentity();
	out.block<2,2>(0,0) = in.matrix().block<2,2>(0,0);
	out.block<2,1>(0,2) = in.matrix().block<2,1>(0,3);
	return out;
}

TransformWithCovariance ScanSensor::calculateTransform(ScanMeasurement::Ptr source,
                                                       ScanMeasurement::Ptr target,
                                                       TransformWithCovariance odometry)
{
	// Transform target by odometry transform
	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
	const PM::DataPoints initializedTarget = rigidTrans->compute(target->getDataPoints(), convert3Dto2D(odometry.transform));
	
	// Perform ICP
	PM::TransformationParameters icp_result = mICP(initializedTarget, source->getDataPoints());
	TransformWithCovariance twc;
	twc.transform = odometry.transform * convert2Dto3D(icp_result);
	return twc;
}
