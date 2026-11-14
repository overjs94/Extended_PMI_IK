#include "Kinematics.h"
#include "System_Param.h"

namespace KINEMATICS_ {
  ///////Basic matrix functions /////////////
  Matrix3d Kinematics_::Rotx(double x) {
    Matrix3d rotx;
    rotx << 1.0, 0.0, 0.0, 0.0, cos(x), -sin(x), 0.0, sin(x), cos(x);
    return rotx;
  }

  Matrix3d Kinematics_::Roty(double y) {
    Matrix3d roty;
    roty << cos(y), 0.0, sin(y), 0.0, 1.0, 0.0, -sin(y), 0.0, cos(y);
    return roty;
  }

  Matrix3d Kinematics_::Rotz(double z) {
    Matrix3d rotz;
    rotz << cos(z), -sin(z), 0.0, sin(z), cos(z), 0.0, 0.0, 0.0, 1.0;
    return rotz;
  }

  Matrix3d Kinematics_::skew(Vector3d r) {
    Matrix3d skewmat;
    skewmat << 0.0, -r(2), r(1), r(2), 0.0, -r(0), -r(1), r(0), 0.0;
    return skewmat;
  }

  Vector3d Kinematics_::inverseskew(Matrix3d R) {
    Vector3d a = Vector3d::Zero();
    a(0) = R(2, 1);
    a(1) = R(0, 2);
    a(2) = R(1, 0);
    return a;
  }

  Matrix6d Kinematics_::Ad(Matrix4d SE3_T) {
    Matrix3d O = Matrix3d::Zero();
    Matrix3d R = SE3_T.block<3, 3>(0, 0);
    Vector3d p = SE3_T.block<3, 1>(0, 3);

    Matrix6d result = Matrix6d::Zero();
    result.block<3, 3>(0, 0) = R;
    result.block<3, 3>(3, 3) = R;
    result.block<3, 3>(3, 0) = skew(p) * R;
    return result;
  }

  Matrix6d Kinematics_::adj(Vector6d V) {
    // MatrixXd adjoint;
    Matrix6d adjoint = Matrix6d::Zero();
    adjoint.block<3, 3>(0, 0) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 3) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 0) = skew(V.block<3, 1>(3, 0));
    return adjoint;
  }

  Vector6d Kinematics_::error_car(
    const Vector3d& p, const Matrix3d& R, const Vector3d& pd,
    const Matrix3d& Rd) {
    Vector6d error;
    error.setZero();
    error.block<3, 1>(3, 0) = p - pd;
    Matrix3d error_R = Rd.transpose() * R - R.transpose() * Rd;
    error.block<3, 1>(0, 0) = 0.5 * inverseskew(error_R);

    return error;
  }

  Eigen::Vector3d Kinematics_::Rot_to_ZYX(const Matrix3d& R) {
    double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
      x = std::atan2(R(2, 1), R(2, 2));  // roll
      y = std::atan2(-R(2, 0), sy);  // pitch
      z = std::atan2(R(1, 0), R(0, 0));  // yaw
    } else {
      x = std::atan2(-R(1, 2), R(1, 1));  // roll
      y = std::atan2(-R(2, 0), sy);  // pitch
      z = 0;  // yaw undefined in gimbal lock
    }

    return Eigen::Vector3d(z, y, x);  // return (yaw, pitch, roll)
  }

  Eigen::Matrix3d Kinematics_::ZYX_to_Rot(const Vector3d& zyx) {
    double x, y, z;
    z = zyx(0);
    y = zyx(1);
    x = zyx(2);

    Matrix3d Rot = Rotz(z) * Roty(y) * Rotx(x);

    return Rot;  // return (yaw, pitch, roll)
  }

  Eigen::Matrix3d Kinematics_::Jaco_zyx_to_omega(const Vector3d& zyx) {
    double x, y, z;
    z = zyx(0);
    y = zyx(1);
    x = zyx(2);

    Matrix3d Rot;

    Rot.block<3, 1>(0, 0) = (Roty(y) * Rotx(x)).transpose() * e_3;
    Rot.block<3, 1>(1, 0) = (Rotx(x)).transpose() * e_2;
    Rot.block<3, 1>(2, 0) = e_1;

    return Rot;  // return (yaw, pitch, roll)
  }

}  // namespace KINEMATICS_