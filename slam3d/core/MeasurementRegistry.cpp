#include "MeasurementRegistry.hpp"

namespace slam3d {

std::map< std::string, std::shared_ptr<MeasurementToStringBase> > MeasurementRegistry::converters;


}  // namespace slam3d
