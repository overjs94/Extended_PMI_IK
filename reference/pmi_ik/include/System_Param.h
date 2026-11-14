#ifndef _SYSTEM_PARAM_H
#define _SYSTEM_PARAM_H
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

///// IK gains ///////////

///// no obstacle ////////

///// Franka /////////////

// const double kr__ = 1e3;
// const Matrix3d K_p = 1e3 * Matrix3d::Identity();

// PMI-IK
const double kr__ = 0 * 1e4;
const Matrix3d K_p = 1e4 * Matrix3d::Identity();

const double kr__hotstart = 1e4;
const Matrix3d K_p_hotstart = 1e4 * Matrix3d::Identity();

// CLIK

const double kr__clik = 0 * 1e2;
const Matrix3d K_p_clik = 1e2 * Matrix3d::Identity();

// SQP
const double kr__sqp = 0 * 1e2;
const Matrix3d K_p_sqp = 1e2 * Matrix3d::Identity();

// TSGD
const double kr__tsgd = 0 * 1e2;
const Matrix3d K_p_tsgd = 1e2 * Matrix3d::Identity();

///// lasdra /////////////

const double kr__lasdra = 1e4;
const Matrix3d K_p_lasdra = 1e4 * Matrix3d::Identity();

const double kr__lasdra_clik = 1e0;
const Matrix3d K_p_lasdra_clik = 1e0 * Matrix3d::Identity();
// SQP
const double kr_lasdra_sqp = 1e2;
const Matrix3d K_p_lasdra_sqp = 1e2 * Matrix3d::Identity();

// TSGD
const double kr_lasdra_tsgd = 1;
const Matrix3d K_p_lasdra_tsgd = 1 * Matrix3d::Identity();

/////// dual arm ////////////

const Matrix3d K_p_dual = 1e4 * Matrix3d::Identity();

///// obstacle ///////////

const int n_obs_lasdra = 7;

const int n_obs_franka = 3;

////// 3-axis vectors /////

const Vector3d e_1 = (Vector3d() << 1.0, 0.0, 0.0).finished();
const Vector3d e_2 = (Vector3d() << 0.0, 1.0, 0.0).finished();
const Vector3d e_3 = (Vector3d() << 0.0, 0.0, 1.0).finished();

///// Dual arm rod parameters //////

const double d_rod = 0.3;
const double r_rod = 0.015;
const double p_base = 0.9;

const double d_base = 1.0;

const int DoF_dual_arm = 20;

const Vector3d d_vec = (Vector3d() << d_rod, 0.0, 0.0).finished();

const Vector3d p_r_base = (Vector3d() << p_base, 0.0, 0.0).finished();
const Vector3d p_l_base = (Vector3d() << -p_base, 0.0, 0.0).finished();

const Matrix3d R_l_base = Matrix3d::Identity();

const Matrix3d R_r_base =
  (Matrix3d() << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0).finished();

///// LASDRA parameters ////////////

#define PI_IK 3.141592
#define gravity 9.8

const int n_link = 10;
const int DoF = 2 * n_link + 1;
// const int n_obs = 3;

const Vector3d L_odar = (Vector3d() << 1.0, 0.0, 0.0).finished();
const Vector3d Lc_odar = (Vector3d() << 0.5, 0.0, 0.0).finished();

////// KUKA parameters //////////////

// KUKA parameter

const double m1_kuka = 3.5253;
const double m2_kuka = 3.5989;
const double m3_kuka = 3.4970;
const double m4_kuka = 3.3780;
const double m5_kuka = 2.5336;
const double m6_kuka = 2.2953;
const double m7_kuka = 0.4610;

const Eigen::Vector3d Lc1_kuka =
  (Eigen::Vector3d() << 0.0018, 0.0483, 0.3105).finished();
const Eigen::Vector3d Lc2_kuka =
  (Eigen::Vector3d() << -0.0029, -0.1952, -0.0653).finished();
const Eigen::Vector3d Lc3_kuka =
  (Eigen::Vector3d() << -0.00013531, -0.0023, 0.1264).finished();
const Eigen::Vector3d Lc4_kuka =
  (Eigen::Vector3d() << -0.00054979, 0.1303, 0.0004943).finished();
const Eigen::Vector3d Lc5_kuka =
  (Eigen::Vector3d() << 0.00089466, -0.0056, 0.0902).finished();
const Eigen::Vector3d Lc6_kuka =
  (Eigen::Vector3d() << -0.00057069, -0.0325, -0.0109).finished();
const Eigen::Vector3d Lc7_kuka =
  (Eigen::Vector3d() << 0.00015304, -0.00022505, 0.0302).finished();

const Eigen::Vector3d L_kuka1 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.36).finished();
const Eigen::Vector3d L_kuka2 =
  (Eigen::Vector3d() << 0.0, -0.21, 0.0).finished();
const Eigen::Vector3d L_kuka3 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.21).finished();
const Eigen::Vector3d L_kuka4 = (Eigen::Vector3d() << 0.0, 0.2, 0.0).finished();
const Eigen::Vector3d L_kuka5 = (Eigen::Vector3d() << 0.0, 0.0, 0.2).finished();
// const Eigen::Vector3d L_kuka6 = (Eigen::Vector3d() <<
// 0.0,-0.081,0.0).finished(); const Eigen::Vector3d L_kuka7 =
// (Eigen::Vector3d() << 0.0, 0.0, 0.045).finished();
const Eigen::Vector3d L_kuka6 =
  (Eigen::Vector3d() << 0.0, -0.126, 0.0).finished();
const Eigen::Vector3d L_kuka7 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.026).finished();

const Eigen::Matrix3d J1_kuka =
  (Eigen::Matrix3d() << 0.1053, 0, 0, 0, 0.1249, 0, 0, 0, 0.0018).finished();
const Eigen::Matrix3d J2_kuka =
  (Eigen::Matrix3d() << 0.0175, 0, 0, 0, 0.0163, 0, 0, 0, 0.0974).finished();
const Eigen::Matrix3d J3_kuka =
  (Eigen::Matrix3d() << 0.0372, 0, 0, 0, 0.0392, 0, 0, 0, 0.0061).finished();
const Eigen::Matrix3d J4_kuka =
  (Eigen::Matrix3d() << 0.0445, 0, 0, 0, 0.0060, 0, 0, 0, 0.0458).finished();
const Eigen::Matrix3d J5_kuka =
  (Eigen::Matrix3d() << 0.0488, 0, 0, 0, 0.0426, 0, 0, 0, 0.0052).finished();
const Eigen::Matrix3d J6_kuka =
  (Eigen::Matrix3d() << 0.0091, 0, 0, 0, 0.0073, 0, 0, 0, 0.0072).finished();
const Eigen::Matrix3d J7_kuka =
  (Eigen::Matrix3d() << 0.0134, 0, 0, 0, 0.0049, 0, 0, 0, 0.0048).finished();
const Eigen::VectorXd d_kuka =
  (Eigen::VectorXd(7) << 0.36, 0.21, 0.21, 0.2, 0.2, 0.126, 0.026).finished();

// joint limit

// Eigen::Matrix<double,DoF_u,1> joint_limit_mast;// =
// Eigen::Vector<double,DoF_u,1>::Zeros(); joint_limit_mast.Eigen::setZero();

const Eigen::VectorXd joint_limit_kuka =
  (Eigen::VectorXd(7) << 170 * PI_IK / 180, 120 * PI_IK / 180,
   170 * PI_IK / 180, 120 * PI_IK / 180, 170 * PI_IK / 180, 120 * PI_IK / 180,
   175 * PI_IK / 180)
    .finished();

// torque limit

const Eigen::VectorXd torque_limit_kuka =
  (Eigen::VectorXd(7) << 320, 320, 176, 176, 110, 40, 40).finished();

//////// Franka parameters //////////

const Eigen::Vector3d Lc1_franka = (Eigen::Vector3d() << 0.000010000000000,
                                    -0.015910000000000, -0.096720000000000)
                                     .finished();  // p^1_1toCOM1
const Eigen::Vector3d Lc2_franka = (Eigen::Vector3d() << 0.001517925144522,
                                    -0.108681643749627, 0.017250000000000)
                                     .finished();
const Eigen::Vector3d Lc3_franka = (Eigen::Vector3d() << -0.081786771931379,
                                    0.039414325132752, -0.119421188440668)
                                     .finished();
const Eigen::Vector3d Lc4_franka = (Eigen::Vector3d() << -0.054208809995766,
                                    0.136998176182161, 0.000778271059664)
                                     .finished();
const Eigen::Vector3d Lc5_franka = (Eigen::Vector3d() << 0.017290683146545,
                                    0.153198229803176, -0.071807358462768)
                                     .finished();
const Eigen::Vector3d Lc6_franka = (Eigen::Vector3d() << 0.091872348829188,
                                    -0.034154430268144, -0.005382473139147)
                                     .finished();
// const Eigen::Vector3d Lc7 = (Eigen::Vector3d() << 0.033740879468453,
// 0.023292400746012,   0.017931814289065).finished();//p^7_7toCOM7, FT 센서
// 미부착시
const Eigen::Vector3d Lc7_franka =
  (Eigen::Vector3d() << 0.003617894300024, 0.006008203447476, 0.107382415132024)
    .finished();  // p^7_7toCOM7, Ft센서 부착시
// const Eigen::Vector3d Lc7_l = (Eigen::Vector3d() << 0.003617894300024,
// 0.006008203447476, 0.1333660479).finished();//FT + left ball const
// Eigen::Vector3d Lc7_l = (Eigen::Vector3d() << 0.0019, 0.0032,
// 0.1359).finished();//FT + left friction const Eigen::Vector3d Lc7_r =
// (Eigen::Vector3d() << 0.003617894300024, 0.006008203447476,
// 0.133475379).finished();//FT + right grip const Eigen::Vector3d Lc7_r =
// (Eigen::Vector3d() << 0.001466, 0.002436, 0.1698707).finished();//FT + right
// drill

const Eigen::Vector3d L_franka0 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.333).finished();
const Eigen::Vector3d L_franka1 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();  // p^1_12
const Eigen::Vector3d L_franka2 =
  (Eigen::Vector3d() << 0.0, -0.316, 0.0).finished();  // p^2_23
const Eigen::Vector3d L_franka3 =
  (Eigen::Vector3d() << 0.0825, 0.0, 0.0).finished();  // p^3_34
const Eigen::Vector3d L_franka4 =
  (Eigen::Vector3d() << -0.0825, 0.384, 0.0).finished();  // p^4_45
const Eigen::Vector3d L_franka5 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();  // p^5_56
const Eigen::Vector3d L_franka6 =
  (Eigen::Vector3d() << 0.088, 0.0, 0.0).finished();  // p^6_67
const Eigen::Vector3d L_franka7 =
  (Eigen::Vector3d() << 0.0, 0.0, 0.107).finished();  // p^7_7ee
// const Eigen::Vector3d L_franka7 = (Eigen::Vector3d() << 0.0, 0.0,
// 0.144).finished();

const Eigen::VectorXd joint_limit_ub_franka =
  (Eigen::VectorXd(7) << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525,
   2.8973)
    .finished();
const Eigen::VectorXd joint_limit_lb_franka =
  (Eigen::VectorXd(7) << -2.8973, -1.7328, -2.8973, -3.0718, -2.8973, -0.0175,
   -2.8973)
    .finished();

const Eigen::VectorXd torque_limit_franka =
  (Eigen::VectorXd(7) << 87, 87, 87, 87, 12, 12, 12).finished();

namespace SysParams {

  // Vector

  extern std::vector<double> M_kuka;
  extern std::vector<Eigen::Matrix3d> J_kuka;
  extern std::vector<Eigen::Vector3d> L_kuka;
  extern std::vector<Eigen::Vector3d> Lc_kuka;
  extern std::vector<Eigen::Vector3d> L_franka;
  extern std::vector<Eigen::Vector3d> Lc_franka;

  // dual arm object end-effector Jacobian

  extern Eigen::Matrix<double, 6, 20> J_ee_dual;
  extern Eigen::Matrix<double, 3, 20> J_ee_dual_pos;

}  // namespace SysParams

#endif