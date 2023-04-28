#include "MeasurementSerialization.hpp"

using namespace slam3d;

std::map< std::string, boost::shared_ptr<MeasurementToStringBase> > MeasurementSerialization::mConverterMap;
