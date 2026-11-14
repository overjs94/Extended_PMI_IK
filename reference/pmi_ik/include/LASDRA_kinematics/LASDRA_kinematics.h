#ifndef _LASDRA_KINEMATICS_H_
#define _LASDRA_KINEMATICS_H_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <vector>
#include "../System_Param.h"

using namespace Eigen;
using namespace std;
using namespace SysParams;

// using Eigen::Matrix4d;
// using Eigen::Matrix3d;
// using Eigen::Vector3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrixnd = Eigen::Matrix<double, DoF, DoF>;
using Vectornd = Eigen::Matrix<double, DoF, 1>;
using MatrixJaco = Eigen::Matrix<double, 6, DoF>;
using MatrixJaco2 = Eigen::Matrix<double, 3, DoF>;

namespace LASDRA_KINEMATICS_ {

  class LASDRA_Kinematics_ {
  private:
    static Matrix3d Rotx(double x);
    static Matrix3d Roty(double y);
    static Matrix3d Rotz(double z);
    static Matrix3d skew(Vector3d r);
    static Vector3d inverseskew(Matrix3d R);
    static Matrix6d Ad(Matrix4d SE3_T);
    static Matrix6d adj(Vector6d V);

  public:
    static void Fkine_ee(Vectornd q, Vector3d& p_ee, Matrix3d& R_ee);
    static MatrixJaco Jaco_ee(Vectornd q);
    static MatrixJaco Jaco_link(const Vectornd q, const int& link_index);
    static void LASDRA_Pose_link(
      Vectornd q, Matrix<double, 3 * n_link, 1>& p,
      Matrix<double, 3 * n_link, 3>& R);
  };

}  // namespace LASDRA_KINEMATICS_

#endif