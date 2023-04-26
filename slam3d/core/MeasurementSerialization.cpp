#include "MeasurementSerialization.hpp"

namespace slam3d {

    std::map< std::string, std::shared_ptr<MeasurementToStringBase> > MeasurementSerialization::converters;

}  // namespace slam3d
