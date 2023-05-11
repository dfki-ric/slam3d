#pragma once

#include "MeasurementStorage.hpp"
#include "Logger.hpp"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>

namespace slam3d
{
	class PQXXInternal;
	
	class PostgreStorage : public MeasurementStorage
	{
	public:
		PostgreStorage(Logger* l, const std::string& host, const std::string& schema);
		~PostgreStorage();

		void add(Measurement::Ptr measurement);
		Measurement::Ptr get(const boost::uuids::uuid& uuid);

	protected:
		void writeMeasurement(const Measurement::Ptr m);
		Measurement::Ptr readMeasurement(const boost::uuids::uuid& uuid);

	protected:
		std::string mHost;
		std::string mSchema;

		Logger* mLogger;

		PQXXInternal* mInternal;
	};
}
