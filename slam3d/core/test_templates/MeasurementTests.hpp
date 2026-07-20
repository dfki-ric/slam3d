#include <slam3d/core/MeasurementStorage.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/uuid/uuid_generators.hpp>


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
		std::stringstream ss2(data);
		boost::archive::text_iarchive ia(ss2);
		ia >> m2;
		BOOST_ASSERT(m2);
		BOOST_CHECK_EQUAL(m->getTypeName(), m2->getTypeName());
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
	boost::uuids::uuid id = boost::uuids::random_generator()();
	storage->add(m, id);

	slam3d::Measurement::Ptr m_res = storage->get(id);

	// ceck values of base slam3d::Measurement
	BOOST_CHECK_NE(m_res.get(), nullptr);
	BOOST_CHECK_EQUAL(m->getTypeName(), m_res->getTypeName());
}
