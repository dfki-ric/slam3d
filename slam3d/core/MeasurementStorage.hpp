# pragma once

#include "Types.hpp"

namespace slam3d
{
	/**
	 * @class MeasurementStorage
	 * @brief 
	 */
	class MeasurementStorage
	{
	public:

		virtual ~MeasurementStorage() {}
		/**
		 * @brief Set the measurement for a given UUID
		 * @param key 
		 * @param measurement 
		 */
		virtual void set(const boost::uuids::uuid& key, Measurement::Ptr measurement);

		/**
		 * @brief Get the measurement for a given UUID
		 * @param key
		 * @throws std::out_of_range if no measurement exists for that UUID
		 */
		virtual Measurement::Ptr get(const boost::uuids::uuid& key);

		/**
		 * @brief Set the measurement for a given UUID
		 * @param key
		 * @param measurement
		 * @throws boost::bad_lexical_cast if the given string is no valid UUID
		 */
		void set(const std::string& key, Measurement::Ptr measurement);

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
