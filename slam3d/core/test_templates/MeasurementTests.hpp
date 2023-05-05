#include <slam3d/core/MeasurementStorage.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


slam3d::Measurement::Ptr test_serialization(slam3d::Measurement::Ptr m)
{
		//create data string with the binary data
		std::stringstream ss;
		boost::archive::text_oarchive oa(ss);
		oa << m;
		std::string data = ss.str();
		BOOST_TEST_MESSAGE("Serialized Pointcloud:\n---\n" + data + "\n---");
		BOOST_CHECK_GE(data.size(), 0);

		// anonymous deserialisation
		slam3d::Measurement::Ptr m2;
		boost::archive::text_iarchive ia(ss);
		ia >> m2;
		BOOST_ASSERT(m2);

		BOOST_CHECK_EQUAL(m->getTypeName(), m2->getTypeName());
		BOOST_CHECK_EQUAL(m->getRobotName(), m2->getRobotName());
		BOOST_CHECK_EQUAL(m->getTimestamp().tv_sec, m2->getTimestamp().tv_sec);
		BOOST_CHECK_EQUAL(m->getTimestamp().tv_usec, m2->getTimestamp().tv_usec);
		BOOST_CHECK_EQUAL(m->getSensorName(), m2->getSensorName());
		BOOST_CHECK(m->getUniqueId() == m2->getUniqueId());
		BOOST_CHECK(m->getSensorPose().isApprox(m2->getSensorPose()));
		BOOST_CHECK(m->getInverseSensorPose().isApprox(m2->getInverseSensorPose()));

        return m2;
}


/**
 * @brief test basic measurement storage (and serialization if the implementation of the Measurements class uses serialization
 * 
 * TODO for users, test for specific measuremetn types
 * 
 * @param storage 
 * @param m 
 */
void test_measurement_storage(std::shared_ptr<slam3d::MeasurementStorage> storage, slam3d::Measurement::Ptr m)
{
	storage->add(m);

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
}
