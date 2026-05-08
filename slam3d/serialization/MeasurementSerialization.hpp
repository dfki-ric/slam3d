#pragma once

#include <slam3d/core/Types.hpp>
#include <slam3d/core/MeasurementStorage.hpp>

namespace slam3d
{
	/**
	 * @brief Serialization for Measurements
	 * Boost serialization will keep the polymophism of the Measurement::Ptr
	 *
	 * All types have to be announced in the cpp file using BOOST_CLASS_EXPORT_IMPLEMENT macro to be run in your code
	 *
	 * e.g.: BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::PointCloudMeasurement)
	 */
	class MeasurementSerialization
	{
	public:
		/**
		 * @brief write measurement fo file
		 * 
		 * @param filename 
		 * @param binary use boost text achhive or binary acrchive
		 * @return true file could be opened and closed
		 * @return false file could not be opened
		 */
		static bool toFile(Measurement::Ptr measurement, const std::string &filename, bool binary = false);

		/**
		 * @brief read measurement from
		 * 
		 * @param binary use boost text achhive or binary acrchive
		 * @return Measurement::Ptr
		 * @throwstd::length_error may occur if wrong format (text/binary) is used
		 */
		static Measurement::Ptr fromFile(const std::string &filename, bool binary = false);

		/**
		 * @brief write all measurements from a storage to files
		 * @param storage
		 * @param dir
		 * @param binary use boost text achhive or binary acrchive
		 * @return number of files that could not be written
		 */
		static unsigned toDirectory(MeasurementStorage* storage, const std::string &dir, bool binary = false);
		
		/**
		 * @brief write measurement to serialized string
		 * 
		 * @param filename 
		 * @param binary use boost text achhive or binary acrchive
		 * @return true file could be opened and closed
		 * @return false file could not be opened
		 */
		static std::string toString(Measurement::Ptr measurement, bool binary = false);

		/**
		 * @brief 
		 * 
		 * @param binary 
		 * @return std::string 
		 * @throwstd::length_error may occur if wrong format (text/binary) is used
		 */
		static Measurement::Ptr fromString(const std::string &serialized, bool binary = false);


	};
}  // namespace slam3d 
