#include "Franka_kinematics.h"

namespace FRANKA_KINEMATICS_ {
  ///////Basic matrix functions /////////////
  Matrix3d FRANKA_Kinematics_::Rotx(double x) {
    Matrix3d rotx;
    rotx << 1.0, 0.0, 0.0, 0.0, cos(x), -sin(x), 0.0, sin(x), cos(x);
    return rotx;
  }

  Matrix3d FRANKA_Kinematics_::Roty(double y) {
    Matrix3d roty;
    roty << cos(y), 0.0, sin(y), 0.0, 1.0, 0.0, -sin(y), 0.0, cos(y);
    return roty;
  }

  Matrix3d FRANKA_Kinematics_::Rotz(double z) {
    Matrix3d rotz;
    rotz << cos(z), -sin(z), 0.0, sin(z), cos(z), 0.0, 0.0, 0.0, 1.0;
    return rotz;
  }

  Matrix3d FRANKA_Kinematics_::skew(Vector3d r) {
    Matrix3d skewmat;
    skewmat << 0.0, -r(2), r(1), r(2), 0.0, -r(0), -r(1), r(0), 0.0;
    return skewmat;
  }

  Vector3d FRANKA_Kinematics_::inverseskew(Matrix3d R) {
    Vector3d a = Vector3d::Zero();
    a(0) = R(2, 1);
    a(1) = R(0, 2);
    a(2) = R(1, 0);
    return a;
  }

  Matrix6d FRANKA_Kinematics_::Ad(Matrix4d SE3_T) {
    Matrix3d O = Matrix3d::Zero();
    Matrix3d R = SE3_T.block<3, 3>(0, 0);
    Vector3d p = SE3_T.block<3, 1>(0, 3);

    Matrix6d result = Matrix6d::Zero();
    result.block<3, 3>(0, 0) = R;
    result.block<3, 3>(3, 3) = R;
    result.block<3, 3>(3, 0) = skew(p) * R;
    return result;
  }

  Matrix6d FRANKA_Kinematics_::adj(Vector6d V) {
    // MatrixXd adjoint;
    Matrix6d adjoint = Matrix6d::Zero();
    adjoint.block<3, 3>(0, 0) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 3) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 0) = skew(V.block<3, 1>(3, 0));
    return adjoint;
  }

  // Vector6d KUKA_Kinematics_::error_car(const Vector3d &p, const Matrix3d &R,
  // const Vector3d &pd, const Matrix3d &Rd)
  // {
  //     Vector6d error;
  //     error.setZero();
  //     error.block<3,1>(3,0) = p-pd;
  //     Matrix3d error_R = Rd.transpose()*R - R.transpose()*Rd;
  //     error.block<3,1>(0,0) = 0.5*inverseskew(error_R);

  //     return error;

  // }

  void FRANKA_Kinematics_::Fkine_franka(
    const Matrix<double, 7, 1>& q, Vector3d& p, Matrix3d& R) {
    vector<Matrix3d> Rot(7);
    Matrix<double, 3, 7> p_body_pre;
    Matrix<double, 3, 7> p_body_aft;
    p_body_pre.setZero();
    p_body_aft.setZero();
    Matrix3d rotz_m_pi_2 =
      (Matrix3d() << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    Matrix3d rotz_p_pi_2 =
      (Matrix3d() << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    Matrix3d rotx_m_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0).finished();
    Matrix3d rotx_p_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0).finished();

    std::vector<int> index_m = {1, 4, 200};
    std::vector<int> index_p = {2, 3, 5, 6, 200};
    int count_m = 0;
    int count_p = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));  // Rotz(q(i) - PI/2)?;

      } else if (i == index_m[count_m]) {
        Rot[i] = rotx_m_pi_2 * Rotz(q(i));
        count_m++;
      } else if (i == index_p[count_p]) {
        Rot[i] = rotx_p_pi_2 * Rotz(q(i));
        count_p++;
      }
    }

    int count_franka = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        p_body_pre.block<3, 1>(0, i) = L_franka0;
      } else {
        p_body_pre.block<3, 1>(0, i) = L_franka[count_franka];
        count_franka++;
      }
    }

    /// SE(3)

    vector<Matrix4d> Trans(7);
    Matrix<double, 4, 4> T_pre;
    T_pre.setZero();
    for (int i = 0; i < 7; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    /// Forward kinematics

    Matrix4d T_ee = Matrix4d::Identity();
    T_ee.block<3, 1>(0, 3) =
      L_franka7;  // FT는 고려 안되어있는 p^7_7 to ee 정보, system_param
                  // 헤더에서 수정하면 될듯.
    Matrix4d T_tmp = Matrix4d::Identity();

    for (int i = 0; i < 7; i++) {
      T_tmp = T_tmp * Trans[i];
    }

    Matrix4d T_fin = T_tmp * T_ee;

    p = T_fin.block<3, 1>(0, 3);
    R = T_fin.block<3, 3>(0, 0);
  }

  Matrix<double, 6, 7> FRANKA_Kinematics_::Jacobian_franka(
    const Matrix<double, 7, 1>& q) {
    Vector6d A_z = (Vector6d() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();

    vector<Matrix3d> Rot(7);
    Matrix<double, 3, 7> p_body_pre;
    Matrix<double, 3, 7> p_body_aft;
    p_body_pre.setZero();
    p_body_aft.setZero();
    Matrix3d rotx_m_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0).finished();
    Matrix3d rotx_p_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0).finished();

    std::vector<int> index_m = {1, 4, 200};
    std::vector<int> index_p = {2, 3, 5, 6, 200};
    int count_m = 0;
    int count_p = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));  // Rotz(q(i) - PI/2)?;

      } else if (i == index_m[count_m]) {
        Rot[i] = rotx_m_pi_2 * Rotz(q(i));
        count_m++;
      } else if (i == index_p[count_p]) {
        Rot[i] = rotx_p_pi_2 * Rotz(q(i));
        count_p++;
      }
    }

    int count_franka = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        p_body_pre.block<3, 1>(0, i) = L_franka0;
      } else {
        p_body_pre.block<3, 1>(0, i) = L_franka[count_franka];
        count_franka++;
      }
    }

    /// SE(3)

    vector<Matrix4d> Trans(7);
    // vector<Matrix4d> Trans_
    Matrix<double, 4, 4> T_pre;
    T_pre.setZero();
    for (int i = 0; i < 7; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    /// Forward kinematics

    Matrix4d T_ee = Matrix4d::Identity();
    T_ee.block<3, 1>(0, 3) = L_franka7;
    Matrix4d T_tmp = Matrix4d::Identity();

    for (int i = 0; i < 7; i++) {
      T_tmp = T_tmp * Trans[i];
    }

    Matrix4d T_fin = T_tmp * T_ee;

    Vector3d p_ee = T_fin.block<3, 1>(0, 3);
    Matrix3d R_ee = T_fin.block<3, 3>(0, 0);

    Matrix4d T_body = Matrix4d::Identity();
    T_body.block<3, 3>(0, 0) = R_ee.transpose();
    T_body.block<3, 1>(0, 3) = -R_ee.transpose() * p_ee;

    // Jacobian T matrix

    vector<Matrix4d> Trans_stack(7);
    Matrix4d II = Matrix4d::Identity();
    for (int i = 0; i < 7; i++) {
      II = II * Trans[i];
      Trans_stack[i] = II;
    }

    Matrix<double, 6, 7> Jaco = Matrix<double, 6, 7>::Zero();

    for (int i = 0; i < 7; i++) {
      Jaco.block<6, 1>(0, i) = Ad(Trans_stack[i]) * A_z;
    }

    Matrix6d TransformMatrix = Matrix6d::Identity();
    TransformMatrix.block<3, 3>(3, 3) = R_ee;
    Matrix<double, 6, 7> Jaco_body = TransformMatrix * Ad(T_body) * Jaco;

    return Jaco_body;
  }

  void FRANKA_Kinematics_::franka_pose_link(
    const Matrix<double, 7, 1> q, Matrix<double, 3 * 7, 1>& p,
    Matrix<double, 3 * 7, 3>& R) {
    vector<Matrix3d> Rot(7);
    Matrix<double, 3, 7> p_body_pre;
    Matrix<double, 3, 7> p_body_aft;
    p_body_pre.setZero();
    p_body_aft.setZero();
    Matrix3d rotz_m_pi_2 =
      (Matrix3d() << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    Matrix3d rotz_p_pi_2 =
      (Matrix3d() << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    Matrix3d rotx_m_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0).finished();
    Matrix3d rotx_p_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0).finished();

    std::vector<int> index_m = {1, 4, 200};
    std::vector<int> index_p = {2, 3, 5, 6, 200};
    int count_m = 0;
    int count_p = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));  // Rotz(q(i) - PI/2)?;

      } else if (i == index_m[count_m]) {
        Rot[i] = rotx_m_pi_2 * Rotz(q(i));
        count_m++;
      } else if (i == index_p[count_p]) {
        Rot[i] = rotx_p_pi_2 * Rotz(q(i));
        count_p++;
      }
    }

    int count_franka = 0;

    for (int i = 0; i < 7; i++) {
      if (i == 0) {
        p_body_pre.block<3, 1>(0, i) = L_franka0;
      } else {
        p_body_pre.block<3, 1>(0, i) = L_franka[count_franka];
        count_franka++;
      }
    }

    /// SE(3)

    vector<Matrix4d> Trans(7);
    Matrix<double, 4, 4> T_pre;
    T_pre.setZero();
    for (int i = 0; i < 7; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    /// Forward kinematics

    Matrix4d T_cent = Matrix4d::Identity();
    // FT는 고려 안되어있는 p^7_7 to ee 정보, system_param 헤더에서 수정하면
    // 될듯.
    Matrix4d T_tmp = Matrix4d::Identity();
    int count = 0;

    for (int i = 0; i < 7; i++) {
      T_tmp = T_tmp * Trans[i];
      // T_cent.block<3, 1>(0, 3) = Lc_franka[i];
      Matrix<double, 4, 4> T_link_tmp = Matrix<double, 4, 4>::Identity();

      T_link_tmp = T_tmp * T_cent;
      p.block<3, 1>(3 * count, 0) = T_link_tmp.block<3, 1>(0, 3);
      R.block<3, 3>(3 * count, 0) = T_link_tmp.block<3, 3>(0, 0);

      count++;
    }

    // Matrix4d T_fin = T_tmp*T_ee;

    // p = T_fin.block<3,1>(0,3);
    // R = T_fin.block<3,3>(0,0);
  }

  Matrix<double, 6, 7> FRANKA_Kinematics_::Jacobian_link_franka(
    const Matrix<double, 7, 1>& q, const int& link_index) {
    const int DoF = link_index;

    Vector6d A_z = (Vector6d() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();

    vector<Matrix3d> Rot(DoF);
    MatrixXd p_body_pre;
    p_body_pre.setZero(3, DoF);
    MatrixXd p_body_aft;
    p_body_aft.setZero(3, DoF);
    Matrix3d rotx_m_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0).finished();
    Matrix3d rotx_p_pi_2 =
      (Matrix3d() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0).finished();

    std::vector<int> index_m = {1, 4, 200};
    std::vector<int> index_p = {2, 3, 5, 6, 200};
    int count_m = 0;
    int count_p = 0;

    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));  // Rotz(q(i) - PI/2)?;

      } else if (i == index_m[count_m]) {
        Rot[i] = rotx_m_pi_2 * Rotz(q(i));
        count_m++;
      } else if (i == index_p[count_p]) {
        Rot[i] = rotx_p_pi_2 * Rotz(q(i));
        count_p++;
      }
    }

    int count_franka = 0;

    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        p_body_pre.block<3, 1>(0, i) = L_franka0;
      } else {
        p_body_pre.block<3, 1>(0, i) = L_franka[count_franka];
        count_franka++;
      }
    }

    /// SE(3)

    vector<Matrix4d> Trans(DoF);
    // vector<Matrix4d> Trans_
    Matrix<double, 4, 4> T_pre;
    T_pre.setZero();
    for (int i = 0; i < DoF; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    /// Forward kinematics

    Matrix4d T_ee = Matrix4d::Identity();
    // T_ee.block<3,1>(0,3) = Lc_franka[DoF];
    Matrix4d T_tmp = Matrix4d::Identity();

    for (int i = 0; i < DoF; i++) {
      T_tmp = T_tmp * Trans[i];
    }

    Matrix4d T_fin = T_tmp * T_ee;

    Vector3d p_ee = T_fin.block<3, 1>(0, 3);
    Matrix3d R_ee = T_fin.block<3, 3>(0, 0);

    Matrix4d T_body = Matrix4d::Identity();
    T_body.block<3, 3>(0, 0) = R_ee.transpose();
    T_body.block<3, 1>(0, 3) = -R_ee.transpose() * p_ee;

    // Jacobian T matrix

    vector<Matrix4d> Trans_stack(DoF);
    Matrix4d II = Matrix4d::Identity();
    for (int i = 0; i < DoF; i++) {
      II = II * Trans[i];
      Trans_stack[i] = II;
    }

    Matrix<double, 6, 7> Jaco = Matrix<double, 6, 7>::Zero();

    for (int i = 0; i < DoF; i++) {
      Jaco.block<6, 1>(0, i) = Ad(Trans_stack[i]) * A_z;
    }

    Matrix6d TransformMatrix = Matrix6d::Identity();
    TransformMatrix.block<3, 3>(3, 3) = R_ee;
    Matrix<double, 6, 7> Jaco_body = TransformMatrix * Ad(T_body) * Jaco;

    return Jaco_body;
  }

}  // namespace FRANKA_KINEMATICS_