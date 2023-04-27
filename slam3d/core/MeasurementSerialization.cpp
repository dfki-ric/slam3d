#include "MeasurementSerialization.hpp"

using namespace slam3d;

std::map< std::string, std::shared_ptr<MeasurementToStringBase> > MeasurementSerialization::mConverterMap;
