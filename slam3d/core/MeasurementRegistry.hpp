# pragma once

#include <string>
#include <map>
#include <memory>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "Types.hpp"

namespace slam3d
{
	class MeasurementToStringBase
	{
	public:
		virtual ~MeasurementToStringBase() {}
		virtual std::string serialize(Measurement::Ptr ptr) = 0;
		virtual Measurement::Ptr deserialize(const std::string &data) = 0;
		virtual bool isSameType(Measurement::Ptr ptr) = 0;
		virtual const std::string& getTypeName() = 0;
	};

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

	class MeasurementRegistry
	{
	public:
		/**
		* @brief register a (de-)serializer fo a given type
		* @tparam TYPE The measuremetn type that should be (de-)serialized
		* @param measurementTypeName the type name that is returned when TYPE.getMeasurementTypeName() is called. This si used to match the fitting serializer
		*/
		template <class TYPE>
		static void registerMeasurementType(const std::string& measurementTypeName)
		{
			std::shared_ptr<MeasurementToStringBase> conv = std::make_shared< MeasurementToString<TYPE> >(measurementTypeName);
			mConverterMap[measurementTypeName] = conv;
		}

		static std::string serialize(Measurement::Ptr ptr)
		{
			if (ptr.get())
			{
				std::shared_ptr<MeasurementToStringBase> conv = mConverterMap[ptr->getTypeName()];
				if (conv.get())
				{
					return conv->serialize(ptr);
				}else
				{
					printf("no registered converter for %s\n", ptr->getTypeName());
				}
			}
			return "";
		}

		static Measurement::Ptr deserialize(const std::string &data, const std::string& key)
		{
			if (data.size())
			{
				std::shared_ptr<MeasurementToStringBase> conv = mConverterMap[key];
				if (conv.get())
				{
					return mConverterMap[key]->deserialize(data);
				} else
				{
					printf("no registered converter for %s\n", key.c_str());
				}
				return Measurement::Ptr();
			}
			return Measurement::Ptr();
		}

	private:
		static std::map<std::string, std::shared_ptr<MeasurementToStringBase> > mConverterMap;
	};

}  // namespace slam3d
