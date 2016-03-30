#include "GraphMapper.hpp"

#include <boost/format.hpp>

#define PI 3.141592654

using namespace slam3d;

// Re-orthogonalize the rotation-matrix
// http://stackoverflow.com/questions/23080791/eigen-re-orthogonalization-of-rotation-matrix
Transform GraphMapper::orthogonalize(const Transform& t)
{
	Vector3 x(t(0,0), t(0,1), t(0,2));
	Vector3 y(t(1,0), t(1,1), t(1,2));
	Vector3 z(t(2,0), t(2,1), t(2,2));
	ScalarType error = x.dot(y);
	
	Vector3 x_ort = x - (error/2.0) * y;
	Vector3 y_ort = y - (error/2.0) * x;
	Vector3 z_ort = x_ort.cross(y_ort);

	Transform res = t;
	ScalarType xdot = 0.5 * (3.0 - x_ort.dot(x_ort));
	res(0,0) = xdot * x_ort(0);
	res(0,1) = xdot * x_ort(1);
	res(0,2) = xdot * x_ort(2);
	
	ScalarType ydot = 0.5 * (3.0 - y_ort.dot(y_ort));
	res(1,0) = ydot * y_ort(0);
	res(1,1) = ydot * y_ort(1);
	res(1,2) = ydot * y_ort(2);
	
	ScalarType zdot = 0.5 * (3.0 - z_ort.dot(z_ort));
	res(2,0) = zdot * z_ort(0);
	res(2,1) = zdot * z_ort(1);
	res(2,2) = zdot * z_ort(2);
	
	return res;
}

GraphMapper::GraphMapper(Logger* log)
{
	mOdometry = NULL;
	mSolver = NULL;
	mLogger = log;
	
	mNeighborRadius = 1.0;
	mMinTranslation = 0.5;
	mMinRotation = 0.1;
	mAddOdometryEdges = false;
	mAddRootEdge = false;

	mCurrentPose = Transform::Identity();
}

GraphMapper::~GraphMapper()
{
}

void GraphMapper::setSolver(Solver* solver)
{
	mSolver = solver;
	mSolver->addNode(0, Transform::Identity());
	mSolver->setFixed(0);
}

void GraphMapper::setOdometry(Odometry* odom, bool add_edges)
{
	mOdometry = odom;
	mAddOdometryEdges = add_edges;
}

void GraphMapper::registerSensor(Sensor* s)
{
	std::pair<SensorList::iterator, bool> result;
	result = mSensors.insert(SensorList::value_type(s->getName(), s));
	if(!result.second)
	{
		mLogger->message(ERROR, (boost::format("Sensor with name %1% already exists!") % s->getName()).str());
		return;
	}
}

Transform GraphMapper::getCurrentPose()
{
	return mCurrentPose;
}

void GraphMapper::writeGraphToFile(const std::string &name)
{
	mLogger->message(ERROR, "Graph writing not implemented!");
}

bool GraphMapper::checkMinDistance(const Transform &t)
{
	ScalarType rot = Eigen::AngleAxis<ScalarType>(t.rotation()).angle();
	if(rot > PI)
		rot = (2*PI) - rot;
	if(rot < -PI)
		rot += 2*PI;
	ScalarType dx = t.translation()(0);
	ScalarType dy = t.translation()(1);
	ScalarType dz = t.translation()(2);
	ScalarType trans = sqrt(dx*dx + dy*dy + dz*dz);
	mLogger->message(DEBUG, (boost::format("Translation: %1% / Rotation: %2%") % trans % rot).str());
	if(trans < mMinTranslation && rot < mMinRotation)
		return false;
	else
		return true;
}
