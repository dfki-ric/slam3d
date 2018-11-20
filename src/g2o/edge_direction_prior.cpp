// g2o - General Graph Optimization
// Copyright (C) 2018 S. Arnold, S. Kasperski
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

#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <iostream>

#include "g2o/edge_direction_prior.h"

namespace g2o {
  using namespace std;

  EdgeDirectionPrior::EdgeDirectionPrior(const Vector3& m, const Vector3& r)
  : BaseUnaryEdge<2, Vector3, VertexSE3>() {
    _measurement = m / m.norm();
	_reference = r / r.norm();
    information().setIdentity();
  }

  bool EdgeDirectionPrior::read(std::istream& is) {
    int pid;
    is >> pid;
    if (!setParameterId(0, pid))
      return false;
    // measured keypoint
    Vector3 meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
    // don't need this if we don't use it in error calculation (???)
    // information matrix is the identity for features, could be changed to allow arbitrary covariances    
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
    } 
    return true;
  }

  bool EdgeDirectionPrior::write(std::ostream& os) const {
    for (int i=0; i<3; i++)
      os  << _measurement[i] << " ";
    os << information()(0,0);
    return os.good();
}

  void EdgeDirectionPrior::computeError() {
    VertexSE3 *vertex = static_cast<VertexSE3*>(_vertices[0]);
    Eigen::Quaterniond state(vertex->estimate().rotation());
    Vector3 expect = state.inverse() * _reference;
    _error(0) = expect(0) - _measurement(0);
    _error(1) = expect(1) - _measurement(1);
  }
}
