#pragma once

#include <vector>
#include <string>

#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <boost/uuid/uuid.hpp>
#include <cloud_slam_types/cloud_slam_types.pb.h>

#include <boost/uuid/uuid_serialize.hpp>


namespace slam3d {

// class MultiScanMeasurement : public slam3d::PointCloudMeasurement {
//  public:
//     typedef boost::shared_ptr<MultiScanMeasurement> Ptr;


//     MultiScanMeasurement(const std::vector<slam3d::PointCloudMeasurement::Ptr>& clouds, const std::string& r, const std::string& s,
//                                 const boost::uuids::uuid id = boost::uuids::nil_uuid());


//     const char* getTypeName() const override { return "slam3d::MultiScanMeasurement"; }

//     /**
//      * @brief returns the combined cloud (makes it compatible with PointCloudSensor functions)
//      * 
//      * @return const PointCloud::Ptr 
//      */
//     // const PointCloud::Ptr getCombinedPointCloud(const std::string& annotation = "");


//     // const std::vector<PointCloud::Ptr> getCloudsByAnnotation(const std::string &annotation);


// //  protected:
//     friend class boost::serialization::access;
//     std::vector<slam3d::PointCloudMeasurement::Ptr> clouds;

//     std::vector<std::string> annotations;

//     std::map<boost::uuids::uuid, PointCloud::Ptr> cloudByUuid;


//  private:
//     friend class boost::serialization::access;
//     template <typename Archive>
//     void serialize(Archive &ar, const unsigned int version)
//     {
//         // Tell boost::serialization that this is derived from Measurement.
//         // It is required because we don't explicitely call Measurement::serialize()
//         // from within PointCloudMeasurement::serialize().

//         boost::serialization::void_cast_register<MultiScanMeasurement, Measurement>(
//             static_cast<MultiScanMeasurement *>(NULL),
//             static_cast<Measurement *>(NULL));
//     }

//     // TODO serialize

//     };

class MultiScanSensor : public slam3d::ScanSensor {
 public:
    MultiScanSensor(const std::string& n, Logger* l): ScanSensor(n,l) {};
    ~MultiScanSensor() {};


    // /**
    //  * @brief Sets parameters for the internal pointcloud registration.
    //  * The standard set is always used to calculate the final transformation.
    //  * A coarse set is used to initialize and verify loop-closures.
    //  * @param param new configuration paramerters
    //  * @param coarse whether to set coarse parameter set
    //  */
    // void setRegistrationParameters(const RegistrationParameters& param, bool coarse);
    		
    // /**
    //  * @brief Set density of points in accumulated map cloud.
    //  * @param r
    //  */
    // void setMapResolution(double r);
    
    // /**
    //  * @brief Set parameters for outlier removal.
    //  * @details An outlier is a point that has less then n neighbors within radius r.
    //  * @param r
    //  * @param n
    //  */
    // void setMapOutlierRemoval(double r, unsigned n);
		

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
     * @return accumulated pointcloud
     * @throw BadMeasurementType
     */
    PointCloud::Ptr getAccumulatedCloud(const VertexObjectList& vertices) const;


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

// namespace boost {
// namespace serialization {

// 		template<class Archive>
// 		inline void save_construct_data(Archive & ar, const slam3d::MultiScanMeasurement * m, const unsigned int file_version)
// 		{
// 			// save data required to construct instance
// 			ar << m->clouds;
//             ar << m->annotations;
//             ar << m->cloudByUuid;
// 			ar << m->getRobotName();
// 			ar << m->getSensorName();
// 			ar << m->getSensorPose();
// 			ar << m->getUniqueId();
// 		}

// 		template<class Archive>
// 		inline void load_construct_data(Archive & ar, slam3d::MultiScanMeasurement * t, const unsigned int file_version)
// 		{
// 			// retrieve data from archive required to construct new instance
			
//             std::vector<slam3d::PointCloudMeasurement::Ptr> clouds;
//             std::vector<std::string> annotations;
//             std::map<boost::uuids::uuid, slam3d::PointCloud::Ptr> cloudByUuid;

// 			std::string robot;
// 			std::string sensor;
// 			slam3d::Transform pose;
// 			boost::uuids::uuid id;
// 			ar >> clouds;
//             ar >> annotations;
//             ar >> cloudByUuid;

// 			ar >> robot;
// 			ar >> sensor;
// 			ar >> pose;
// 			ar >> id;

// 			// invoke inplace constructor to initialize instance of PointCloudMeasurement
// 			::new(t)slam3d::MultiScanMeasurement(clouds, robot, sensor, id);
// 		}

// }  // namespace serialization
// }  // namespace boost

// BOOST_CLASS_EXPORT_KEY(slam3d::MultiScanMeasurement)
