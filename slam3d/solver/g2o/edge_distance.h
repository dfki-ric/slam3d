#ifndef G2O_EDGE_DISTANCE_H_
#define G2O_EDGE_DISTANCE_H_

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>

namespace g2o {

  // first two args are the measurement type, second two the connection classes
  class G2O_TYPES_SLAM3D_API EdgeDistance : public BaseBinaryEdge<1, number_t, VertexSE3, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeDistance(number_t distance);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a scalar
    void computeError();
 
   // jacobian
// virtual void linearizeOplus();
  };
}
#endif
