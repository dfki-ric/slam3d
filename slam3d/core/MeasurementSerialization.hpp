# pragma once

#include <stdexcept>
#include <string>
#include <map>
#include <memory>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "Types.hpp"

namespace slam3d
{
	/**
	 * @class MeasurementToStringBase
	 * @brief Abstract base class for a measurement serializer
	 * @details Use serialize and deserialize to read/write a
	 * slam3d::Measurement to/from an std::string.
	 */
	class MeasurementToStringBase
	{
	public:
		virtual ~MeasurementToStringBase() {}
		virtual std::string serialize(Measurement::Ptr ptr) = 0;
		virtual Measurement::Ptr deserialize(const std::string &data) = 0;
		virtual bool isSameType(Measurement::Ptr ptr) = 0;
		virtual const std::string& getTypeName() = 0;
	};

	/**
	 * @class MeasurementToString
	 * @brief Implementation of a generic measurement serializer
	 * @details To use serialization, a specific Measurement must implement
	 * the serialize() method as defined in boost::serialization and be
	 * registered with the MeasurementRegistry.
	 */
	template <class TYPE>
	class MeasurementToString : public MeasurementToStringBase
	{
	public:
		explicit MeasurementToString(const std::string& typeName)
		{
			mTypeName = typeName;
		}
		virtual ~MeasurementToString() {}

		virtual bool isSameType(Measurement::Ptr ptr)
		{
			boost::shared_ptr<TYPE> newptr = boost::dynamic_pointer_cast<TYPE>(ptr);
			if (newptr.get())
			{
				return true;
			}
			return false;
		}

		virtual std::string serialize(Measurement::Ptr ptr)
		{
			boost::shared_ptr<TYPE> newptr = boost::dynamic_pointer_cast<TYPE>(ptr);
			std::stringstream ss;
			boost::archive::text_oarchive oa(ss);
			oa << *(newptr.get());
			return ss.str();
		}

		virtual Measurement::Ptr deserialize(const std::string &data)
		{
			boost::shared_ptr<TYPE> m = boost::make_shared<TYPE>();
			std::stringstream ss(data);
			boost::archive::text_iarchive ia(ss);
			ia >> *(m.get());
			return m;
		}

		virtual const std::string& getTypeName()
		{
			return mTypeName;
		}

	private:
		std::string mTypeName;
	};

	/**
	 * @class MeasurementSerialization
	 * @brief Registers (de-)serializer's for slam3d::Measurement's by their TypeName.
	 */
	class MeasurementSerialization
	{
	public:
		/**
		* @brief register a (de-)serializer fo a given type
		* @tparam TYPE The measuremetn type that should be (de-)serialized
		* @param measurementTypeName the type name that is returned when TYPE.getTypeName() is called. This is used to match the fitting serializer
		*/
		template <class TYPE>
		static void registerMeasurementType(const std::string& measurementTypeName)
		{
			boost::shared_ptr<MeasurementToStringBase> conv = boost::make_shared< MeasurementToString<TYPE> >(measurementTypeName);
			mConverterMap[measurementTypeName] = conv;
		}

		static std::string serialize(Measurement::Ptr ptr)
		{
			if (!ptr.get())
				throw std::runtime_error("Empty pointer given to serialize().");

			return mConverterMap.at(ptr->getTypeName())->serialize(ptr);
		}

		static Measurement::Ptr deserialize(const std::string &data, const std::string& measurementTypeName)
		{
			return mConverterMap.at(measurementTypeName)->deserialize(data);
		}

	private:
		static std::map<std::string, boost::shared_ptr<MeasurementToStringBase> > mConverterMap;
	};

}  // namespace slam3d
