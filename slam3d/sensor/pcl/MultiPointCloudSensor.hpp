#pragma once

#include <vector>
#include <string>

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <boost/uuid/uuid.hpp>
// #include <cloud_slam_types/cloud_slam_types.pb.h>

#include <boost/uuid/uuid_serialize.hpp>


namespace slam3d {

class MultiPointCloudSensor : public slam3d::ScanSensor {
 public:
    MultiPointCloudSensor(const std::string& n, Logger* l): ScanSensor(n,l) {};
    ~MultiPointCloudSensor() {};

    /**
     * @brief Transform source cloud by given transformation.
     * @param source
     * @param tf
     */
    PointCloud::Ptr transform(PointCloud::ConstPtr source, const Transform tf) const;

    /**
     * @brief Creates a single point cloud that contains all measurements in vertices.
     * @details The individual point clouds are transformed by their current pose in the graph,
     * no additional alignement or optimization is performed during this.
     * @param vertices
     * @param tags optional list of tags to filter by. If non-empty, only vertices
     *        carrying at least one of these tags are accumulated. Each tag is looked
     *        up on the VertexObject itself and, if not found there, in its
     *        subMeasurements.
     * @return accumulated pointcloud
     * @throw BadMeasurementType
     */
    PointCloud::Ptr getAccumulatedCloud(const VertexObjectList& vertices,
                                        const std::vector<std::string>& tags = {}) const;


    /**
    * @brief Create a virtual measurement by accumulating scans from given vertices.
    * @param vertices list of vertices that should contain a Measurement of this sensor
    * @param pose origin of the accumulated scan
    * @throw BadMeasurementType
    */		
    virtual Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const;


    /**
    * @brief Create a constraint between two measurements.
    * @details The odometry transformation and the resulting constraint are
    * with regards to the robot coordinate system. Make sure that sensor_pose
    * is properly set within the measurements.
    * @param source
    * @param target
    * @param odometry
    */
    virtual Constraint::Ptr createConstraint(const Measurement::Ptr& source,
                                                const Measurement::Ptr& target,
                                                const Transform& odometry,
                                                bool loop);

 protected:
    RegistrationParameters mFineConfiguration;
    RegistrationParameters mCoarseConfiguration;

    double   mMapResolution;
    double   mMapOutlierRadius;
    unsigned mMapOutlierNeighbors;


};

}  // namespace slam3d
