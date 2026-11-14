#ifndef _FRANKA_KINEMATICS_H_
#define _FRANKA_KINEMATICS_H_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <vector>
#include "../System_Param.h"

using namespace Eigen;
using namespace std;
using namespace SysParams;

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace FRANKA_KINEMATICS_ {
  class FRANKA_Kinematics_ {
  private:
    /// basic functions

    static Matrix3d Rotx(double x);
    static Matrix3d Roty(double y);
    static Matrix3d Rotz(double z);
    static Matrix3d skew(Vector3d r);
    static Vector3d inverseskew(Matrix3d R);
    static Matrix6d Ad(Matrix4d SE3_T);
    static Matrix6d adj(Vector6d V);

  public:
    // static MatrixXd Stiffness_matrix(const int n_);

    /// kinematic functions manipulator

    static void Fkine_franka(
      const Matrix<double, 7, 1>& q, Vector3d& p, Matrix3d& R);
    static Matrix<double, 6, 7> Jacobian_franka(const Matrix<double, 7, 1>& q);

    /// kinematics for each link & jacobian

    static void franka_pose_link(
      const Matrix<double, 7, 1> q, Matrix<double, 3 * 7, 1>& p,
      Matrix<double, 3 * 7, 3>& R);
    static Matrix<double, 6, 7> Jacobian_link_franka(
      const Matrix<double, 7, 1>& q, const int& link_index);
  };

}  // namespace FRANKA_KINEMATICS_

#endif