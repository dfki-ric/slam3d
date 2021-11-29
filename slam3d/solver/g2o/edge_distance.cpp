#include "edge_distance.h"

#include <iostream>

namespace g2o {
  using namespace std;

  EdgeDistance::EdgeDistance(number_t distance)
  : BaseBinaryEdge<1, number_t, VertexSE3, VertexPointXYZ>() {
    _measurement = distance;
    information().setIdentity();
  }

  void EdgeDistance::computeError() {
    VertexSE3 *v0 = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *v1 = static_cast<VertexPointXYZ*>(_vertices[1]);

    Vector3 diff = v0->estimate().translation() - v1->estimate();
    _error(0) = diff.norm() - _measurement;
  }
  
  bool EdgeDistance::read(std::istream& is) {
    return false;
  }

  bool EdgeDistance::write(std::ostream& os) const {
    return false;
  }
}
