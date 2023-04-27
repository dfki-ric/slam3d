#include <slam3d/core/Measurements.hpp>
#include <slam3d/core/MeasurementSerialization.hpp>
#include <boost/test/unit_test.hpp>


/**
 * @brief test basic measurement storage (and serialization if the implementation of the Measurements class uses serialization
 * 
 * TODO for users, test for specific measuremetn types
 * 
 * @param storage 
 * @param m 
 */
void test_measurement_storage(std::shared_ptr<Measurements> storage, slam3d::Measurement::Ptr m) {
    storage->set(m->getUniqueId(), m);

    slam3d::Measurement::Ptr m_res = storage->get(m->getUniqueId());

    // ceck values of base slam3d::Measurement
    BOOST_CHECK_NE(m_res.get(), nullptr);
    BOOST_CHECK_EQUAL(m->getRobotName(), m_res->getRobotName());
    BOOST_CHECK_EQUAL(m->getTimestamp().tv_sec, m_res->getTimestamp().tv_sec);
    BOOST_CHECK_EQUAL(m->getTimestamp().tv_usec, m_res->getTimestamp().tv_usec);
    BOOST_CHECK_EQUAL(m->getSensorName(), m_res->getSensorName());
    BOOST_CHECK(m->getUniqueId() == m_res->getUniqueId());
    BOOST_CHECK(m->getSensorPose().isApprox(m_res->getSensorPose()));
    BOOST_CHECK(m->getInverseSensorPose().isApprox(m_res->getInverseSensorPose()));

    // debug out (run with --log_level=message)
    BOOST_TEST_MESSAGE(m->getSensorPose().matrix());
    BOOST_TEST_MESSAGE(m_res->getSensorPose().matrix());
    BOOST_TEST_MESSAGE(m->getPointCloud()->sensor_origin_.matrix());
    BOOST_TEST_MESSAGE(m_res->getPointCloud()->sensor_origin_.matrix());
}
