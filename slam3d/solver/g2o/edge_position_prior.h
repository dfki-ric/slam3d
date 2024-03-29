// g2o - General Graph Optimization
// Copyright (C) 2019 S. Kasperski
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_EDGE_POSITION_PRIOR_H_
#define G2O_EDGE_POSITION_PRIOR_H_

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
namespace g2o {
	
  class G2O_TYPES_SLAM3D_API EdgePositionPrior : public BaseUnaryEdge<3, Vector3, VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePositionPrior(const Vector3& measurement, const Isometry3& sensor_pose);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();
    
    // jacobian
//  virtual void linearizeOplus();

  private:
    Isometry3 _sensor_pose;
  };

}
#endif
