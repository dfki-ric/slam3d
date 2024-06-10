# pragma once

#include <slam3d/core/Types.hpp>

namespace slam3d
{
	/**
	 * @class MeasurementStorage
	 * @brief Holds measurements to be organized in a slam3d::Graph.
	 * @details This base implementation simply stores the measurements
	 * in a map. It is used to separate the data storage from the topological
	 * organization in the slam3d::Graph. Specializations might derive from
	 * this class and store data externally, e.g. on disk or inside a database.
	 */
	class MeasurementStorage
	{
	public:

		virtual ~MeasurementStorage() {}

		/**
		 * @brief Add the given measurement to the storage
		 * @param measurement 
		 */
		virtual void add(Measurement::Ptr measurement);

		/**
		 * @brief Get the measurement for a given UUID
		 * @param key
		 * @throws std::out_of_range if no measurement exists for that UUID
		 */
		virtual Measurement::Ptr get(const boost::uuids::uuid& key);

		/**
		 * @brief check if the uuid is available in the database
		 * @param key 
		 * @return true 
		 * @return false 
		 */
		virtual bool contains(const boost::uuids::uuid& key);

		/**
		 * @brief Get the measurement for a given UUID
		 * @param key
		 * @throws std::out_of_range if no measurement exists for that UUID
		 * @throws boost::bad_lexical_cast if the given string is no valid UUID
		 */
		Measurement::Ptr get(const std::string& key);

	private:
		std::map<boost::uuids::uuid, Measurement::Ptr> mMeasurements;
	};
}  // namespace slam3d
