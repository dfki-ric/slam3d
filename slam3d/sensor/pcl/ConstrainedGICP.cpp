#include "ConstrainedGICP.hpp"
#include <pcl/registration/exceptions.h>


using namespace slam3d;
using namespace pcl;

void ConstrainedGICP::estimateConstrainedRigidTransformationNewton(const PointCloudSource& cloud_src,
                                       const pcl::Indices& indices_src,
                                       const PointCloudTarget& cloud_tgt,
                                       const pcl::Indices& indices_tgt,
                                       Matrix4& transformation_matrix)
{
  //  need at least min_number_correspondences_ samples
  if (indices_src.size() < min_number_correspondences_) {
    PCL_THROW_EXCEPTION(NotEnoughPointsException,
                        "[pcl::GeneralizedIterativeClosestPoint::"
                        "estimateRigidTransformationNewton] Need "
                        "at least "
                            << min_number_correspondences_
                            << " points to estimate a transform! "
                               "Source and target have "
                            << indices_src.size() << " points!");
    return;
  }
  // Set the initial solution
  Vector6d x = Vector6d::Zero();
  // translation part
  x[0] = transformation_matrix(0, 3);
  x[1] = transformation_matrix(1, 3);
  x[2] = transformation_matrix(2, 3);
  // rotation part (Z Y X euler angles convention)
  // see: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  x[3] = std::atan2(transformation_matrix(2, 1), transformation_matrix(2, 2));
  x[4] = std::asin(
      std::min<double>(1.0, std::max<double>(-1.0, -transformation_matrix(2, 0))));
  x[5] = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Optimize using Newton
  OptimizationFunctorWithIndices functor(this);
  Eigen::Matrix<double, 6, 6> hessian;
  Eigen::Matrix<double, 6, 1> gradient;
  double current_x_value = functor(x);
  functor.dfddf(x, gradient, hessian);
  Eigen::Matrix<double, 6, 1> delta;
  int inner_iterations_ = 0;
  do {
    ++inner_iterations_;
    // compute descent direction from hessian and gradient. Take special measures if
    // hessian is not positive-definite (positive Eigenvalues)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(hessian);
    Eigen::Matrix<double, 6, 6> inverted_eigenvalues =
        Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
      const double ev = eigensolver.eigenvalues()[i];
      if (ev < 0)
        inverted_eigenvalues(i, i) = 1.0 / eigensolver.eigenvalues()[5];
      else
        inverted_eigenvalues(i, i) = 1.0 / ev;
    }

    // *******************************************************************
    // TODO At this point insert constraint from initial odometry guess
    // *******************************************************************

    delta = eigensolver.eigenvectors() * inverted_eigenvalues *
            eigensolver.eigenvectors().transpose() * gradient;

    // simple line search to guarantee a decrease in the function value
    double alpha = 1.0;
    double candidate_x_value;
    bool improvement_found = false;
    for (int i = 0; i < 10; ++i, alpha /= 2) {
      Vector6d candidate_x = x - alpha * delta;
      candidate_x_value = functor(candidate_x);
      if (candidate_x_value < current_x_value) {
        PCL_DEBUG("[estimateRigidTransformationNewton] Using stepsize=%g, function "
                  "value previously: %g, now: %g, "
                  "improvement: %g\n",
                  alpha,
                  current_x_value,
                  candidate_x_value,
                  current_x_value - candidate_x_value);
        x = candidate_x;
        current_x_value = candidate_x_value;
        improvement_found = true;
        break;
      }
    }
    if (!improvement_found) {
      PCL_DEBUG("[estimateRigidTransformationNewton] finishing because no progress\n");
      break;
    }
    functor.dfddf(x, gradient, hessian);
    if (gradient.head<3>().norm() < translation_gradient_tolerance_ &&
        gradient.tail<3>().norm() < rotation_gradient_tolerance_) {
      PCL_DEBUG("[estimateRigidTransformationNewton] finishing because gradient below "
                "threshold: translation: %g<%g, rotation: %g<%g\n",
                gradient.head<3>().norm(),
                translation_gradient_tolerance_,
                gradient.tail<3>().norm(),
                rotation_gradient_tolerance_);
      break;
    }
  } while (inner_iterations_ < max_inner_iterations_);
  PCL_DEBUG("[estimateRigidTransformationNewton] solver finished after %i iterations "
            "(of max %i)\n",
            inner_iterations_,
            max_inner_iterations_);
  transformation_matrix.setIdentity();
  applyState(transformation_matrix, x);
}
