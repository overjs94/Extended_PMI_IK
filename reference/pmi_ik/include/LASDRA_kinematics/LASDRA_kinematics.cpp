#include "LASDRA_kinematics.h"

namespace LASDRA_KINEMATICS_ {
  ///////Basic matrix functions /////////////
  Matrix3d LASDRA_Kinematics_::Rotx(double x) {
    Matrix3d rotx;
    rotx << 1.0, 0.0, 0.0, 0.0, cos(x), -sin(x), 0.0, sin(x), cos(x);
    return rotx;
  }

  Matrix3d LASDRA_Kinematics_::Roty(double y) {
    Matrix3d roty;
    roty << cos(y), 0.0, sin(y), 0.0, 1.0, 0.0, -sin(y), 0.0, cos(y);
    return roty;
  }

  Matrix3d LASDRA_Kinematics_::Rotz(double z) {
    Matrix3d rotz;
    rotz << cos(z), -sin(z), 0.0, sin(z), cos(z), 0.0, 0.0, 0.0, 1.0;
    return rotz;
  }

  Matrix3d LASDRA_Kinematics_::skew(Vector3d r) {
    Matrix3d skewmat;
    skewmat << 0.0, -r(2), r(1), r(2), 0.0, -r(0), -r(1), r(0), 0.0;
    return skewmat;
  }

  Vector3d LASDRA_Kinematics_::inverseskew(Matrix3d R) {
    Vector3d a = Vector3d::Zero();
    a(0) = R(2, 1);
    a(1) = R(0, 2);
    a(2) = R(1, 0);
    return a;
  }

  Matrix6d LASDRA_Kinematics_::Ad(Matrix4d SE3_T) {
    Matrix3d O = Matrix3d::Zero();
    Matrix3d R = SE3_T.block<3, 3>(0, 0);
    Vector3d p = SE3_T.block<3, 1>(0, 3);

    Matrix6d result = Matrix6d::Zero();
    result.block<3, 3>(0, 0) = R;
    result.block<3, 3>(3, 3) = R;
    result.block<3, 3>(3, 0) = skew(p) * R;
    return result;
  }

  Matrix6d LASDRA_Kinematics_::adj(Vector6d V) {
    // MatrixXd adjoint;
    Matrix6d adjoint = Matrix6d::Zero();
    adjoint.block<3, 3>(0, 0) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 3) = skew(V.block<3, 1>(0, 0));
    adjoint.block<3, 3>(3, 0) = skew(V.block<3, 1>(3, 0));
    return adjoint;
  }

  void LASDRA_Kinematics_::LASDRA_Pose_link(
    Vectornd q, Matrix<double, 3 * n_link, 1>& p,
    Matrix<double, 3 * n_link, 3>& R) {
    vector<Matrix3d> Rot(DoF);
    Matrix<double, 3, DoF> p_body_pre;
    p_body_pre.setZero();
    int cnt1 = 0;
    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));

      } else if (i == 1) {
        Rot[i] = Roty(q(i));
      } else if (i == 2) {
        Rot[i] = Rotx(q(i));

      } else if (i % 2 == 1) {
        Rot[i] = Roty(q(i));
        p_body_pre.block<3, 1>(0, i) = L_odar;
        cnt1++;

      } else {
        Rot[i] = Rotx(q(i));
      }
    }

    vector<Matrix4d> Trans(DoF);
    Matrix4d T_pre = Matrix4d::Identity();

    for (int i = 0; i < DoF; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    Matrix4d T_cent = Matrix4d::Identity();
    T_cent.block<3, 1>(0, 3) = Lc_odar;
    int count = 0;

    Matrix4d T_tmp = Matrix4d::Identity();

    for (int i = 0; i < DoF; i++) {
      T_tmp = T_tmp * Trans[i];
      Matrix<double, 4, 4> T_link_tmp = Matrix<double, 4, 4>::Identity();
      if (i % 2 == 0 && i != 0) {
        T_link_tmp = T_tmp * T_cent;
        p.block<3, 1>(3 * count, 0) = T_link_tmp.block<3, 1>(0, 3);
        R.block<3, 3>(3 * count, 0) = T_link_tmp.block<3, 3>(0, 0);

        count++;
      }
    }
  }

  void LASDRA_Kinematics_::Fkine_ee(
    Vectornd q, Vector3d& p_ee, Matrix3d& R_ee) {
    vector<Matrix3d> Rot(DoF);
    Matrix<double, 3, DoF> p_body_pre;
    p_body_pre.setZero();
    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));

      } else if (i == 1) {
        Rot[i] = Roty(q(i));
      } else if (i == 2) {
        Rot[i] = Rotx(q(i));

      } else if (i % 2 == 1) {
        Rot[i] = Roty(q(i));
        p_body_pre.block<3, 1>(0, i) = L_odar;

      } else {
        Rot[i] = Rotx(q(i));
      }
    }

    vector<Matrix4d> Trans(DoF);
    Matrix4d T_pre = Matrix4d::Identity();

    for (int i = 0; i < DoF; i++) {
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      T_pre(3, 3) = 1.0;
      Trans[i] = T_pre;
    }

    Matrix4d T_ee = Matrix4d::Identity();
    T_ee.block<3, 1>(0, 3) = L_odar;
    int count = 0;

    Matrix4d T_tmp = Matrix4d::Identity();

    for (int i = 0; i < DoF; i++) {
      T_tmp = T_tmp * Trans[i];
    }

    Matrix4d T_end = Matrix4d::Identity();

    T_end = T_tmp * T_ee;

    p_ee = T_end.block<3, 1>(0, 3);
    R_ee = T_end.block<3, 3>(0, 0);
  }

  MatrixJaco LASDRA_Kinematics_::Jaco_ee(Vectornd q) {
    vector<Matrix3d> Rot(DoF);
    Matrix<double, 3, DoF> p_body_pre;
    p_body_pre.setZero();
    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));

      } else if (i == 1) {
        Rot[i] = Roty(q(i));
      } else if (i == 2) {
        Rot[i] = Rotx(q(i));

      } else if (i % 2 == 1) {
        Rot[i] = Roty(q(i));
        p_body_pre.block<3, 1>(0, i) = L_odar;

      } else {
        Rot[i] = Rotx(q(i));
      }
    }

    vector<Vector3d> p_body_aft(DoF);

    for (int i = 0; i < DoF; i++) {
      p_body_aft[i] = -Rot[i].transpose() * p_body_pre.block<3, 1>(0, i);
    }

    vector<Matrix4d> Trans(DoF);
    vector<Matrix4d> Trans_inv(DoF);

    for (int i = 0; i < DoF; i++) {
      Matrix4d T_pre = Matrix4d::Identity();
      Matrix4d T_pre_inv = Matrix4d::Identity();
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      Trans[i] = T_pre;
      T_pre_inv.block<3, 3>(0, 0) = Rot[i].transpose();
      T_pre_inv.block<3, 1>(0, 3) = p_body_aft[i];
      Trans_inv[i] = T_pre_inv;
    }

    MatrixXd T_forward = MatrixXd::Identity(4, 4);

    for (int i = 0; i < DoF; i++) {
      T_forward = T_forward * Trans[i];
    }

    Matrix4d T_end = Matrix4d::Identity();
    T_end.block<3, 1>(0, 3) = L_odar;

    Matrix4d T_ee = Matrix4d::Identity();

    T_ee = T_forward * T_end;

    vector<Matrix4d> T_stack_inv(DoF);

    Matrix4d T_back_pre = Matrix4d::Identity();
    T_back_pre.block<3, 1>(0, 3) = -L_odar;

    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        T_stack_inv[DoF - 1 - i] = T_back_pre;
      } else {
        T_back_pre = T_back_pre * Trans_inv[DoF - i];
        T_stack_inv[DoF - 1 - i] = T_back_pre;
      }
    }
    Matrix<double, 6, DoF> Jacobian_pre = Matrix<double, 6, DoF>::Zero();

    Matrix<double, 6, 1> A_x;
    A_x.setZero();
    A_x(0) = 1.0;
    Matrix<double, 6, 1> A_y;
    A_y.setZero();
    A_y(1) = 1.0;
    Matrix<double, 6, 1> A_z;
    A_z.setZero();
    A_z(2) = 1.0;

    for (int i = 0; i < DoF; i++) {
      if (i == 0) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_z;

      } else if (i == 1) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_y;
      } else if (i == 2) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_x;

      } else if (i % 2 == 1) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_y;

      } else {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_x;
      }
    }

    Matrix<double, 6, DoF> Jacobian = Matrix<double, 6, DoF>::Zero();

    Matrix<double, 6, 6> RRR = Matrix<double, 6, 6>::Identity();
    RRR.block<3, 3>(3, 3) = T_ee.block<3, 3>(0, 0);

    Jacobian = RRR * Jacobian_pre;
    return Jacobian;
  }

  MatrixJaco LASDRA_Kinematics_::Jaco_link(
    const Vectornd q, const int& link_index) {
    const int DoF2 = link_index * 2 + 3;
    vector<Matrix3d> Rot(DoF2);
    MatrixXd p_body_pre;
    p_body_pre.setZero(3, DoF2);
    for (int i = 0; i < DoF2; i++) {
      if (i == 0) {
        Rot[i] = Rotz(q(i));

      } else if (i == 1) {
        Rot[i] = Roty(q(i));
      } else if (i == 2) {
        Rot[i] = Rotx(q(i));

      } else if (i % 2 == 1) {
        Rot[i] = Roty(q(i));
        p_body_pre.block<3, 1>(0, i) = L_odar;

      } else {
        Rot[i] = Rotx(q(i));
      }
    }

    vector<Vector3d> p_body_aft(DoF2);

    for (int i = 0; i < DoF2; i++) {
      p_body_aft[i] = -Rot[i].transpose() * p_body_pre.block<3, 1>(0, i);
    }

    vector<Matrix4d> Trans(DoF2);
    vector<Matrix4d> Trans_inv(DoF2);

    for (int i = 0; i < DoF2; i++) {
      Matrix4d T_pre = Matrix4d::Identity();
      Matrix4d T_pre_inv = Matrix4d::Identity();
      T_pre.block<3, 3>(0, 0) = Rot[i];
      T_pre.block<3, 1>(0, 3) = p_body_pre.block<3, 1>(0, i);
      Trans[i] = T_pre;
      T_pre_inv.block<3, 3>(0, 0) = Rot[i].transpose();
      T_pre_inv.block<3, 1>(0, 3) = p_body_aft[i];
      Trans_inv[i] = T_pre_inv;
    }

    MatrixXd T_forward = MatrixXd::Identity(4, 4);

    for (int i = 0; i < DoF2; i++) {
      T_forward = T_forward * Trans[i];
    }

    Matrix4d T_end = Matrix4d::Identity();
    T_end.block<3, 1>(0, 3) = Lc_odar;

    Matrix4d T_ee = Matrix4d::Identity();

    T_ee = T_forward * T_end;

    vector<Matrix4d> T_stack_inv(DoF2);

    Matrix4d T_back_pre = Matrix4d::Identity();
    T_back_pre.block<3, 1>(0, 3) = -Lc_odar;

    for (int i = 0; i < DoF2; i++) {
      if (i == 0) {
        T_stack_inv[DoF2 - 1 - i] = T_back_pre;
      } else {
        T_back_pre = T_back_pre * Trans_inv[DoF2 - i];
        T_stack_inv[DoF2 - 1 - i] = T_back_pre;
      }
    }
    Matrix<double, 6, DoF> Jacobian_pre = Matrix<double, 6, DoF>::Zero();

    Matrix<double, 6, 1> A_x;
    A_x.setZero();
    A_x(0) = 1.0;
    Matrix<double, 6, 1> A_y;
    A_y.setZero();
    A_y(1) = 1.0;
    Matrix<double, 6, 1> A_z;
    A_z.setZero();
    A_z(2) = 1.0;

    for (int i = 0; i < DoF2; i++) {
      if (i == 0) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_z;

      } else if (i == 1) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_y;
      } else if (i == 2) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_x;

      } else if (i % 2 == 1) {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_y;

      } else {
        Jacobian_pre.block<6, 1>(0, i) = Ad(T_stack_inv[i]) * A_x;
      }
    }

    Matrix<double, 6, DoF> Jacobian = Matrix<double, 6, DoF>::Zero();

    Matrix<double, 6, 6> RRR = Matrix<double, 6, 6>::Identity();
    RRR.block<3, 3>(3, 3) = T_ee.block<3, 3>(0, 0);

    Jacobian = RRR * Jacobian_pre;
    return Jacobian;  //.block<3,DoF>(3,0);
  }

}  // namespace LASDRA_KINEMATICS_