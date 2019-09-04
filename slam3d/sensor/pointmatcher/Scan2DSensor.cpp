#include "Scan2DSensor.hpp"

#include <math.h>
#include <fstream>

using namespace slam3d;

Scan2DSensor::Scan2DSensor(const std::string& n, Logger* l, const std::string& configFile)
: ScanSensor(n, l)
{
	if (configFile.empty())
	{
		mLogger->message(INFO, "No ICP configuration specified, using default.");
		mICP.setDefault();
	}else
	{
		// load YAML config
		std::ifstream ifs(configFile.c_str());
		if (!ifs.good())
		{
			mLogger->message(WARNING, (boost::format("Could not load ICP configuration from: %1%") % configFile).str());
			mICP.setDefault();
		}
		mICP.loadFromYaml(ifs);
		mLogger->message(INFO, (boost::format("Successfully loaded ICP configuration from: %1%") % configFile).str());
	}
	mWriteDebugData = false;
}

Scan2DSensor::~Scan2DSensor()
{
	
}

Transform Scan2DSensor::convert2Dto3D(const PM::TransformationParameters& in) const
{
	assert(in.rows() == 3);
	assert(in.cols() == 3);
	Transform out = Transform::Identity();
	out.matrix().block<2,2>(0,0) = in.block<2,2>(0,0);
	out.matrix().block<2,1>(0,3) = in.block<2,1>(0,2);
	return out;
}

PM::TransformationParameters Scan2DSensor::convert3Dto2D(const Transform& in) const
{
//	ScalarType yaw = in.linear().eulerAngles(2, 0, 2)[2];
	PM::TransformationParameters out(3,3);
	out.setIdentity();
	out.block<2,2>(0,0) = in.matrix().block<2,2>(0,0);
	out.block<2,1>(0,2) = in.matrix().block<2,1>(0,3);
//	out(0,0) = cos(yaw);
//	out(1,0) = -sin(yaw);
//	out(0,1) = -out(1,0);
//	out(1,1) = out(0,0);
	return out;
}

Constraint::Ptr Scan2DSensor::createConstraint(const Measurement::Ptr& source,
		                                       const Measurement::Ptr& target,
		                                       const Transform& odometry,
		                                       bool loop)
{
	// Transform guess in sensor frame
	Transform guess = source->getInverseSensorPose() * odometry * target->getSensorPose();

	// Cast to this sensors measurement type
	Scan2DMeasurement::Ptr sourceScan = boost::dynamic_pointer_cast<Scan2DMeasurement>(source);
	Scan2DMeasurement::Ptr targetScan = boost::dynamic_pointer_cast<Scan2DMeasurement>(target);
	if(!sourceScan || !targetScan)
	{
		mLogger->message(ERROR, "Measurement given to calculateTransform() is not a Scan!");
		throw BadMeasurementType();
	}
	
	// Transform target by odometry transform
	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
	const PM::DataPoints initializedTarget = rigidTrans->compute(targetScan->getDataPoints(), convert3Dto2D(guess));

//	if(debug)
//	{
//		sourceScan->getDataPoints().save("source.vtk");
//		initializedTarget.save("target.vtk");
//	}
	
	// Perform ICP
	PM::TransformationParameters tp = mICP(initializedTarget, sourceScan->getDataPoints());
	Transform icp_result = guess * convert2Dto3D(tp);

	// Transform back to robot frame
	TransformWithCovariance twc;
	twc.transform = source->getSensorPose() * icp_result * target->getInverseSensorPose();
	twc.covariance = Covariance<6>::Identity() * mCovarianceScale;
	
	return Constraint::Ptr(new SE3Constraint(mName, twc));
}

Measurement::Ptr Scan2DSensor::createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const
{
	// Concatenate all scans into one
	PM::DataPoints accu = createDataPoints();
	for(VertexObjectList::const_iterator it = vertices.begin(); it != vertices.end(); it++)
	{
		Scan2DMeasurement::Ptr scan = boost::dynamic_pointer_cast<Scan2DMeasurement>(it->measurement);
		if(!scan)
		{
			mLogger->message(WARNING, "Measurement is not a Scan2D!");
			throw BadMeasurementType();
		}
		accu.concatenate(transformDataPoints(scan->getDataPoints(), it->corrected_pose * scan->getSensorPose()));
	}

	// Transform to target frame
	PM::DataPoints accu_tf = transformDataPoints(accu, pose.inverse());	
	timeval t;
	return Scan2DMeasurement::Ptr(new Scan2DMeasurement(accu_tf, t, "AccumulatedScan", mName, Transform::Identity()));
}

PM::DataPoints Scan2DSensor::createDataPoints() const
{
	PM::DataPoints::Labels feat_labels;
	feat_labels.push_back(PM::DataPoints::Label("x", 1));
	feat_labels.push_back(PM::DataPoints::Label("y", 1));
	feat_labels.push_back(PM::DataPoints::Label("w", 1));
	
	PM::Matrix feat_points(3, 0);	
	return PM::DataPoints(feat_points, feat_labels);
}

PM::DataPoints Scan2DSensor::transformDataPoints(const PM::DataPoints& source, const Transform tf) const
{
	std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
	PM::TransformationParameters tp = convert3Dto2D(tf);
	if (!rigidTrans->checkParameters(tp))
	{
		tp = rigidTrans->correctParameters(tp);
	}
	return rigidTrans->compute(source,tp);
}
