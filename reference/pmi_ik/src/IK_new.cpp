#include "Kinematics.h"
#include "IK_new.h"
#include "Franka_kinematics/Franka_kinematics.h"
#include "LASDRA_kinematics/LASDRA_kinematics.h"
#include "dsf-mtd/mtd.hpp"
#include "qp_solvers/qp_solvers.hpp"
#include "Franka_kinematics/Franka_Dual_kinematics.h"

#include <qpOASES.hpp>

using namespace qpsolver;

FRANKA_KINEMATICS_::FRANKA_Kinematics_ franka_ik;
LASDRA_KINEMATICS_::LASDRA_Kinematics_ lasdra_ik;
FRANKA_DUAL_KINEMATICS_::FRANKA_DUAL_Kinematics_ franka_dual_ik;
KINEMATICS_::Kinematics_ kine_ik;

namespace pmi_ik {
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  ////// no obstacle version //////////////////////////////////

  Matrix<double, 7, 1> PMI_IK_new_franka_no_obs(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd) {
    int N_itr = 1000;
    double dT = 0.05;

    Eigen::MatrixXd Mk = 1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    double H_tot_prev = 0.0;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP
    Vkhat.setZero();
    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.useInitGuess = true;
    qpoasesSolver.maxWorkingSetRecalculation = 100000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      //   std::cout << "iter: " << i << std::endl;

      if (i == 0) {
        q = q_init;
        dq.setZero();
        // qpoasesSolver.hotstart = false;
      } else {
        // qpoasesSolver.hotstart = true;
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka_ik.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (1.0 * dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr;

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dH < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;
      //   prob.boxLowerValue = lb_joint;
      //   prob.boxUpperValue = ub_joint;

      prob.ineqOperator = Matrix<double, 7, 7>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;

      Vkhat = Ak.inverse() * Bk;
      qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      //   std::cout << "c\n";
    }
    // std::cout << "complete\n";

    return q;
  }

  Matrix<double, 7, 1> PMI_IK_new_franka_hotstart_no_obs(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd, double Time_step, Eigen::Matrix<double, 7, 1> v_pre) {
    int N_itr = 200;
    double dT = 0.005;

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = 1.0 * K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    double scale = 1.0 / (Time_step * Time_step);
    Matrix<double, 7, 7> K_init =
      1.0 * scale * Matrix<double, 7, 7>::Identity();

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    double H_tot_prev = 0.0;

    // velocity constraint

    Eigen::VectorXd delta = Eigen::VectorXd::Constant(q_init.size(), 0.5);
    Eigen::VectorXd v_lower = -delta * Time_step + q_init;
    Eigen::VectorXd v_upper = delta * Time_step + q_init;
    Eigen::VectorXd lb_vel;
    Eigen::VectorXd ub_vel;

    // acceleration constraint
    Eigen::VectorXd a_delta = Eigen::VectorXd::Constant(q_init.size(), 7);
    Eigen::VectorXd a_lower =
      (-a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd a_upper =
      (a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd lb_acc;
    Eigen::VectorXd ub_acc;

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 10;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka_ik.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> A_init = dT * K_init;
      Matrix<double, 7, 1> B_init = -K_init * (q - q_init);

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr + A_init;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr + B_init;

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * ((q - q_init).transpose() * K_init * (q - q_init)).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;
      lb_vel = (v_lower - q) / dT;
      ub_vel = (v_upper - q) / dT;
      lb_acc = (a_lower - q) / dT;
      ub_acc = (a_upper - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      prob.ineqOperator = Matrix<double, 7, 7>::Identity();
      prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
      prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

      Vkhat.resize(nV);
      qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra_no_obs(
    const Matrix<double, DoF, 1>& q_init, const Vector3d& p_ed,
    const Matrix3d& R_ed) {
    int N_itr = 1000;
    double dT = 0.05;

    Matrix<double, DoF, DoF> Mk = 0.001 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);
    // q_lim(0) = PI_IK * 0.8;

    Matrix<double, DoF, DoF> B_ = 0.1 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p_lasdra;

    // PMI matrices

    Vector6d error_e;  //= kine_ik.error_car(p_e, R_e, p_ed, R_ed);
    Matrix<double, 6, DoF> J_e;  // = lasdra_ik.Jaco_ee(q);
    Matrix<double, DoF, DoF>
      A_pose;  // = J_e.transpose() * (dT * K_pose) * J_e;
    Matrix<double, DoF, 1> B_pose;  // = -J_e.transpose() * K_pose * error_e;

    // PMI equations
    Matrix<double, DoF, DoF> Ak;  // = 2 * Mk / dT + B_ + A_pose;
    Matrix<double, DoF, 1> Bk;  // = 2 * Mk / dT * dq + B_pose;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.05;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, DoF> Jaco_link_col;
    std::vector<Vector3d> pos_info_;  // = data->pos_info_; TODO: fix this

    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP
    Vkhat.setZero();

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      J_e = lasdra_ik.Jaco_ee(q);
      A_pose = J_e.transpose() * (2.0 * dT * K_pose) * J_e;
      B_pose = -J_e.transpose() * K_pose * error_e;

      // PMI equations
      Ak = 2 * Mk / dT + B_ + A_pose;
      Bk = 2 * Mk / dT * dq + B_pose;
      // collision avoidance

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dH < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;

      // Vkhat = Ak.inverse() * Bk;
      qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra_hotstart_no_obs(
    const Eigen::Matrix<double, DoF, 1>& q_init, const Eigen::Vector3d& p_ed,
    const Eigen::Matrix3d& R_ed, double Time_step,
    Eigen::Matrix<double, DoF, 1> v_pre) {
    int N_itr = 2000;
    double dT = 0.005;

    Matrix<double, DoF, DoF> Mk = 0.1 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);

    Matrix<double, DoF, DoF> B_ = 0.0001 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    double scale = 1.0 / (Time_step * Time_step);

    Matrix<double, DoF, DoF> K_init =
      1 * scale * Matrix<double, DoF, DoF>::Identity();

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // velocity constraint

    Eigen::VectorXd delta = Eigen::VectorXd::Constant(q_init.size(), 0.5);
    Eigen::VectorXd v_lower = -delta * Time_step + q_init;
    Eigen::VectorXd v_upper = delta * Time_step + q_init;
    Eigen::VectorXd lb_vel;
    Eigen::VectorXd ub_vel;

    // acceleration constraint
    Eigen::VectorXd a_delta = Eigen::VectorXd::Constant(q_init.size(), 7);
    Eigen::VectorXd a_lower =
      (-a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd a_upper =
      (a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd lb_acc;
    Eigen::VectorXd ub_acc;
    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 10;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra_ik.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose = J_e.transpose() * (dT * K_pose) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      Matrix<double, DoF, DoF> A_init = dT * K_init;
      Matrix<double, DoF, 1> B_init = -K_init * (q - q_init);

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose + A_init;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose + B_init;

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * ((q - q_init).transpose() * K_init * (q - q_init)).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-5) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;
      lb_vel = (v_lower - q) / dT;
      ub_vel = (v_upper - q) / dT;
      lb_acc = (a_lower - q) / dT;
      ub_acc = (a_upper - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
      prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
      prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

      Vkhat.resize(nV);
      qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  /////////////////////////////////////////////////////////////

  Matrix<double, 7, 1> PMI_IK_new_franka(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd,
    std::vector<std::unique_ptr<col::Object>>& franka_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles) {
    int N_itr = 2000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = 1e4 * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    //// matrices

    Vector6d error_r;  // = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
    Matrix<double, 6, 7> J_r;  // = franka_ik.Jacobian_franka(q);
    Matrix<double, 7, 7> A_posr;  // = J_r.transpose() * (dT * K_pose) * J_r;
    Matrix<double, 7, 1> B_posr;  // = -J_r.transpose() * K_pose * error_r;

    Matrix<double, 7, 7> Ak;  // = 2 * Mk / dT + B_ + A_posr;
    Matrix<double, 7, 1> Bk;  // = 2 * Mk / dT * dq + B_posr;

    // collision avoidance

    Matrix<double, 3 * 7, 1> p_link;
    Matrix<double, 3 * 7, 3> R_link;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 7> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP
    Vkhat.setZero();

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.useInitGuess = true;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      //   std::cout << "iter: " << i << std::endl;

      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      J_r = franka_ik.Jacobian_franka(q);
      A_posr = J_r.transpose() * (dT * K_pose * 2.0) * J_r;
      B_posr = -J_r.transpose() * K_pose * error_r;

      Ak = 2 * Mk / dT + B_ + A_posr;
      Bk = 2 * Mk / dT * dq + B_posr;

      // collision avoidance

      franka_ik.franka_pose_link(q, p_link, R_link);
      col::apply_franka_pose(franka_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      // if (i == 0) {
      // } else {
      //   if (dH < 1e-4) {
      //     break;
      //   } else {
      //     H_prev = H_tot;
      //   }
      // }
      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;
      //   prob.boxLowerValue = lb_joint;
      //   prob.boxUpperValue = ub_joint;

      collision_result_ = col::broad_phase_collision_detection(
        franka_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, 7, 7>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 5e-1 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + 7, 7);
        A_ineq.setZero();
        Eigen::MatrixXd constraint_(nC, 1);
        constraint_.setZero();

        int i_col = 0;

        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = franka_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part

          Jaco_link_col = franka_ik.Jacobian_link_franka(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - 1.0 * gap;
          i_col++;
        }

        VectorXd lb(nC + 7);
        lb.head(nC) = lb_col;
        lb.tail(7) = lb_joint;

        VectorXd ub(nC + 7);
        ub.head(nC) = ub_col;
        ub.tail(7) = ub_joint;

        A_ineq.block(0, 0, nC, 7) = Jaco_col;
        A_ineq.block(nC, 0, 7, 7) = Matrix<double, 7, 7>::Identity();

        prob.ineqOperator = A_ineq;
        prob.ineqUpperValue = ub;
        prob.ineqLowerValue = lb;
      }

      auto flag = qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      if (i == 0) {
      } else {
        if (dH < 1e-4 && flag == qpsolver::QPExitCode::SOLVER_SUCCESS) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      //   std::cout << "c\n";
    }
    // std::cout << "complete\n";

    return q;
  }

  Matrix<double, 7, 1> PMI_IK_new_franka_hotstart(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd,
    std::vector<std::unique_ptr<col::Object>>& franka_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles,
    double Time_step, Eigen::Matrix<double, 7, 1> v_pre) {
    int N_itr = 200;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.01 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = kr__hotstart * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = 1.0 * K_r;
    K_pose.block<3, 3>(3, 3) = K_p_hotstart;

    double scale = 1.0 / (Time_step * Time_step);
    Matrix<double, 7, 7> K_init =
      0.01 * scale * Matrix<double, 7, 7>::Identity();

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 7> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // velocity constraint

    Eigen::VectorXd delta = Eigen::VectorXd::Constant(q_init.size(), 5);  // 0.5
    Eigen::VectorXd v_lower = -delta * Time_step + q_init;
    Eigen::VectorXd v_upper = delta * Time_step + q_init;
    Eigen::VectorXd lb_vel;
    Eigen::VectorXd ub_vel;

    // acceleration constraint
    Eigen::VectorXd a_delta =
      Eigen::VectorXd::Constant(q_init.size(), 20);  // 7
    Eigen::VectorXd a_lower =
      (-a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd a_upper =
      (a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd lb_acc;
    Eigen::VectorXd ub_acc;

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    Vkhat.setZero();
    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka_ik.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose * 2.0) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> A_init = dT * K_init;
      Matrix<double, 7, 1> B_init = -K_init * (q - q_init);

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr + A_init;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr + B_init;

      // collision avoidance

      Matrix<double, 3 * 7, 1> p_link;
      Matrix<double, 3 * 7, 3> R_link;
      franka_ik.franka_pose_link(q, p_link, R_link);
      col::apply_franka_pose(franka_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * ((q - q_init).transpose() * K_init * (q - q_init)).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p_hotstart *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__hotstart *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      // if (i == 0) {
      // } else {
      //   if (dH < 1e-4) {
      //     break;
      //   } else {
      //     H_prev = H_tot;
      //   }
      // }

      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;
      lb_vel = (v_lower - q) / dT;
      ub_vel = (v_upper - q) / dT;
      lb_acc = (a_lower - q) / dT;
      ub_acc = (a_upper - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      //   if (i == 0) {
      collision_result_ = col::broad_phase_collision_detection(
        franka_object, obstacles, geom_index);
      //   } else {
      //     qpoasesSolver.hotstart = true;
      //   }

      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, 7, 7>::Identity();
        prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
        prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 5e1 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + 7, 7);
        A_ineq.setZero();
        Eigen::MatrixXd constraint_(nC, 1);
        constraint_.setZero();

        int i_col = 0;

        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = franka_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part

          Jaco_link_col = franka_ik.Jacobian_link_franka(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = (dist_thr - 0.2 * gap);  // / dT;
          i_col++;
        }

        VectorXd lb(nC + 7);
        lb.head(nC) = lb_col;
        lb.tail(7) = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

        VectorXd ub(nC + 7);
        ub.head(nC) = ub_col;
        ub.tail(7) = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);

        A_ineq.block(0, 0, nC, 7) = Jaco_col;
        A_ineq.block(nC, 0, 7, 7) = Matrix<double, 7, 7>::Identity();

        // prob.ineqOperator = A_ineq;
        // prob.ineqUpperValue = ub;
        // prob.ineqLowerValue = lb;
        prob.eqOperator = Jaco_col;
        prob.eqValue = constraint_;
        prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
        prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
        prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);
      }

      // Vkhat.resize(nV);
      auto flag = qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      if (i == 0) {
      } else {
        if (dH < 1e-4 && flag == qpsolver::QPExitCode::SOLVER_SUCCESS) {
          break;
        } else {
          H_prev = H_tot;
        }
      }
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra(
    const Matrix<double, DoF, 1>& q_init, const Vector3d& p_ed,
    const Matrix3d& R_ed,
    std::vector<std::unique_ptr<col::Object>>& lasdra_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles) {
    int N_itr = 10000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Matrix<double, DoF, DoF> Mk = 0.1 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);

    Matrix<double, DoF, DoF> B_ = 0.1 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p_lasdra;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.05;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, DoF> Jaco_link_col;
    std::vector<Vector3d> pos_info_;  // = data->pos_info_; TODO: fix this

    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 100000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra_ik.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose =
        J_e.transpose() * (dT * K_pose * 2.0) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose;
      // collision avoidance

      Matrix<double, 3 * n_link, 1> p_link;
      Matrix<double, 3 * n_link, 3> R_link;
      lasdra_ik.LASDRA_Pose_link(q, p_link, R_link);
      col::apply_lasdra_pose(lasdra_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dH < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      collision_result_ = col::broad_phase_collision_detection(
        lasdra_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 1e4 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + nV, nV);
        A_ineq.setZero();

        int i_col = 0;
        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = lasdra_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          // std::cout << gap << std::endl << std::endl << s_1 << std::endl;

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part
          Jaco_link_col = lasdra_ik.Jaco_link(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - gap;
          i_col++;
        }

        VectorXd lb(nC + nV);
        lb.head(nC) = lb_col;
        lb.tail(nV) = lb_joint;

        VectorXd ub(nC + nV);
        ub.head(nC) = ub_col;
        ub.tail(nV) = ub_joint;

        A_ineq.block(0, 0, nC, nV) = Jaco_col;
        A_ineq.block(nC, 0, nV, nV) = Matrix<double, DoF, DoF>::Identity();

        prob.ineqOperator = A_ineq;
        prob.ineqUpperValue = ub;
        prob.ineqLowerValue = lb;
      }

      Vkhat.resize(nV);
      auto flag = qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      if (i == 0) {
      } else {
        if (dH < 1e-4 && flag == qpsolver::QPExitCode::SOLVER_SUCCESS) {
          break;
        } else {
          H_prev = H_tot;
        }
      }
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra_hotstart(
    const Eigen::Matrix<double, DoF, 1>& q_init, const Eigen::Vector3d& p_ed,
    const Eigen::Matrix3d& R_ed,
    std::vector<std::unique_ptr<col::Object>>& lasdra_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles,
    double Time_step, Eigen::Matrix<double, DoF, 1> v_pre) {
    int N_itr = 2000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Matrix<double, DoF, DoF> Mk = 0.1 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);

    Matrix<double, DoF, DoF> B_ = 0.0001 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p_lasdra;

    double scale = 1.0 / (Time_step * Time_step);

    Matrix<double, DoF, DoF> K_init =
      1 * scale * Matrix<double, DoF, DoF>::Identity();

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, DoF> Jaco_link_col;
    std::vector<Vector3d> pos_info_;  // = data->pos_info_; TODO: fix this

    // velocity constraint

    Eigen::VectorXd delta =
      Eigen::VectorXd::Constant(q_init.size(), 100);  // 0.5
    Eigen::VectorXd v_lower = -delta * Time_step + q_init;
    Eigen::VectorXd v_upper = delta * Time_step + q_init;
    Eigen::VectorXd lb_vel;
    Eigen::VectorXd ub_vel;

    // acceleration constraint
    Eigen::VectorXd a_delta =
      Eigen::VectorXd::Constant(q_init.size(), 200);  // 7
    Eigen::VectorXd a_lower =
      (-a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd a_upper =
      (a_delta * Time_step + v_pre) * Time_step + q_init;
    Eigen::VectorXd lb_acc;
    Eigen::VectorXd ub_acc;
    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra_ik.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose = J_e.transpose() * (dT * K_pose) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      Matrix<double, DoF, DoF> A_init = dT * K_init;
      Matrix<double, DoF, 1> B_init = -K_init * (q - q_init);

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose + A_init;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose + B_init;
      // collision avoidance

      Matrix<double, 3 * n_link, 1> p_link;
      Matrix<double, 3 * n_link, 3> R_link;
      lasdra_ik.LASDRA_Pose_link(q, p_link, R_link);
      col::apply_lasdra_pose(lasdra_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * ((q - q_init).transpose() * K_init * (q - q_init)).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      // if (i == 0) {
      // } else {
      //   if (dH < 1e-4) {
      //     break;
      //   } else {
      //     H_prev = H_tot;
      //   }
      // }

      // QP formulation

      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;
      lb_vel = (v_lower - q) / dT;
      ub_vel = (v_upper - q) / dT;
      lb_acc = (a_lower - q) / dT;
      ub_acc = (a_upper - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      collision_result_ = col::broad_phase_collision_detection(
        lasdra_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
        prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
        prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 1e4 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + nV, nV);
        A_ineq.setZero();
        Eigen::MatrixXd constraint_(nC, 1);
        constraint_.setZero();

        int i_col = 0;
        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = lasdra_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          // std::cout << gap << std::endl << std::endl << s_1 << std::endl;

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part
          Jaco_link_col = lasdra_ik.Jaco_link(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = 0.2 * (dist_thr - gap);
          i_col++;
        }

        VectorXd lb(nC + nV);
        lb.head(nC) = lb_col;
        lb.tail(nV) = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);

        VectorXd ub(nC + nV);
        ub.head(nC) = ub_col;
        ub.tail(nV) = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);

        // A_ineq.block(0, 0, nC, nV) = Jaco_col;
        // A_ineq.block(nC, 0, nV, nV) = Matrix<double, DoF, DoF>::Identity();

        // prob.ineqOperator = A_ineq;
        // prob.ineqUpperValue = ub;
        // prob.ineqLowerValue = lb;

        prob.eqOperator = Jaco_col;
        prob.eqValue = constraint_;
        prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
        prob.ineqUpperValue = ub_joint.cwiseMin(ub_vel).cwiseMin(ub_acc);
        prob.ineqLowerValue = lb_joint.cwiseMax(lb_vel).cwiseMax(lb_acc);
      }

      Vkhat.resize(nV);
      auto flag = qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      if (i == 0) {
      } else {
        if (dH < 1e-4 && flag == qpsolver::QPExitCode::SOLVER_SUCCESS) {
          break;
        } else {
          H_prev = H_tot;
        }
      }
    }

    return q;
  }

  ///// qpswift solvers ////////

  Matrix<double, 7, 1> PMI_IK_new_franka_qpswift(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd,
    std::vector<std::unique_ptr<col::Object>>& franka_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles) {
    int N_itr = 1000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 7> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    QpswiftSolver qpswiftSolver;
    qpswiftSolver.maxIteration = 10;
    qpswiftSolver.verbose = false;

    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      //   std::cout << "iter: " << i << std::endl;

      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka_ik.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr;

      // collision avoidance

      Matrix<double, 3 * 7, 1> p_link;
      Matrix<double, 3 * 7, 3> R_link;
      franka_ik.franka_pose_link(q, p_link, R_link);
      col::apply_franka_pose(franka_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;
      // prob.boxLowerValue = lb_joint;
      // prob.boxUpperValue = ub_joint;

      collision_result_ = col::broad_phase_collision_detection(
        franka_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, 7, 7>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 5e-1 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC, 7);
        A_ineq.setZero();

        int i_col = 0;

        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = franka_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part

          Jaco_link_col = franka_ik.Jacobian_link_franka(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - 1.0 * gap;
          i_col++;
        }

        VectorXd lb(nC + 7);
        lb.head(nC) = lb_col;
        lb.tail(7) = lb_joint;

        VectorXd ub(nC + 7);
        ub.head(nC) = ub_col;
        ub.tail(7) = ub_joint;

        A_ineq.block(0, 0, nC, 7) = Jaco_col;
        A_ineq.block(nC, 0, 7, 7) = Matrix<double, 7, 7>::Identity();

        prob.ineqOperator = A_ineq;
        prob.ineqUpperValue = ub;
        prob.ineqLowerValue = lb;
      }

      Vkhat.resize(nV);
      qpswiftSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      //   std::cout << "c\n";
    }
    // std::cout << "complete\n";

    return q;
  }

  Matrix<double, 7, 1> PMI_IK_new_franka_qpswift_no_obs(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd) {
    int N_itr = 1000;
    double dT = 0.005;

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    double H_tot_prev = 0.0;

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    QpswiftSolver qpswiftSolver;
    qpswiftSolver.maxIteration = 100;
    qpswiftSolver.verbose = false;

    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      //   std::cout << "iter: " << i << std::endl;

      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      R_r.setZero();
      p_r.setZero();
      franka_ik.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka_ik.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr;

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }
      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint = (joint_limit_lb_franka - q) / dT;
      ub_joint = (joint_limit_ub_franka - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;
      // prob.boxLowerValue = lb_joint;
      // prob.boxUpperValue = ub_joint;

      prob.ineqOperator = Matrix<double, 7, 7>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;

      Vkhat.resize(nV);
      qpswiftSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;

      //   std::cout << "c\n";
    }
    // std::cout << "complete\n";

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra_qpswift(
    const Matrix<double, DoF, 1>& q_init, const Vector3d& p_ed,
    const Matrix3d& R_ed,
    std::vector<std::unique_ptr<col::Object>>& lasdra_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles) {
    int N_itr = 2000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Matrix<double, DoF, DoF> Mk = 0.1 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);

    Matrix<double, DoF, DoF> B_ = 0.0001 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p_lasdra;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.05;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, DoF> Jaco_link_col;
    std::vector<Vector3d> pos_info_;  // = data->pos_info_; TODO: fix this

    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    QpswiftSolver qpswiftSolver;
    qpswiftSolver.maxIteration = 10;
    qpswiftSolver.verbose = false;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra_ik.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose = J_e.transpose() * (dT * K_pose) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose;
      // collision avoidance

      Matrix<double, 3 * n_link, 1> p_link;
      Matrix<double, 3 * n_link, 3> R_link;
      lasdra_ik.LASDRA_Pose_link(q, p_link, R_link);
      col::apply_lasdra_pose(lasdra_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      collision_result_ = col::broad_phase_collision_detection(
        lasdra_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 1e4 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + nV, nV);
        A_ineq.setZero();

        int i_col = 0;
        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = lasdra_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          // std::cout << gap << std::endl << std::endl << s_1 << std::endl;

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part
          Jaco_link_col = lasdra_ik.Jaco_link(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - gap;
          i_col++;
        }

        VectorXd lb(nC + nV);
        lb.head(nC) = lb_col;
        lb.tail(nV) = lb_joint;

        VectorXd ub(nC + nV);
        ub.head(nC) = ub_col;
        ub.tail(nV) = ub_joint;

        A_ineq.block(0, 0, nC, nV) = Jaco_col;
        A_ineq.block(nC, 0, nV, nV) = Matrix<double, DoF, DoF>::Identity();

        prob.ineqOperator = A_ineq;
        prob.ineqUpperValue = ub;
        prob.ineqLowerValue = lb;
      }

      Vkhat.resize(nV);
      qpswiftSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_new_lasdra_qpswift_no_obs(
    const Matrix<double, DoF, 1>& q_init, const Vector3d& p_ed,
    const Matrix3d& R_ed) {
    int N_itr = 2000;
    double dT = 0.005;

    Matrix<double, DoF, DoF> Mk = 0.1 * Matrix<double, DoF, DoF>::Identity();

    Matrix<double, DoF, 1> q_lim = MatrixXd::Constant(DoF, 1, PI_IK / 3);

    Matrix<double, DoF, DoF> B_ = 0.1 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> q;
    Matrix<double, DoF, 1> dq;
    Matrix<double, DoF, 1> lb_joint;
    Matrix<double, DoF, 1> ub_joint;

    Matrix3d K_r = kr__lasdra * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p_lasdra;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // int nV = DoF;
    int nV = static_cast<int>(DoF);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

    QpswiftSolver qpswiftSolver;
    qpswiftSolver.maxIteration = 10;
    qpswiftSolver.verbose = false;
    qpsolver::QPProblem prob;

    Matrix3d R_e;
    Vector3d p_e;
    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      if (i == 0) {
        q = q_init;
        dq.setZero();
      }

      // end-effector

      R_e.setZero();
      p_e.setZero();
      lasdra_ik.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine_ik.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra_ik.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose = J_e.transpose() * (dT * K_pose) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose;

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dq_norm < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }

      // QP formulation

      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;
      lb_joint = (-q_lim - q) / dT;
      ub_joint = (q_lim - q) / dT;

      prob.costQuadratic = H;
      prob.costLinear = g;

      prob.ineqOperator = Matrix<double, DoF, DoF>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;

      Vkhat.resize(nV);
      qpswiftSolver.solve(prob, Vkhat);

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  ////// Dual-arm scenario //////////

  void PMI_IK_dual_arm(
    const Eigen::Matrix<double, 14, 1>& q_init, const Eigen::Vector3d& p_o_init,
    const Eigen::Matrix3d& R_o_init, const Eigen::Vector3d& p_ed,
    std::vector<std::unique_ptr<col::Object>>& dual_arm_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles,
    Eigen::Matrix<double, 14, 1>& q_sol, Eigen::Vector3d& p_o_sol,
    Eigen::Matrix3d& R_o_sol) {
    //// parameter setting /////

    int N_itr = 2000;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.1 * Eigen::MatrixXd::Identity(20, 20);

    Matrix<double, 20, 20> B_ = 0.1 * Matrix<double, 20, 20>::Identity();
    Matrix<double, 14, 1> q;
    Matrix<double, 20, 1> dq;

    Vector3d p_o_tmp;
    Matrix3d R_o_tmp;

    Matrix<double, 20, 1> lb_joint;
    Matrix<double, 20, 1> ub_joint;

    Matrix<double, 6, 20> Jaco_c;
    Matrix<double, 6, 1> constraint_;
    constraint_.setZero();

    //// matrices

    Vector3d error_op;
    Matrix<double, 20, 20> A_posr;
    Matrix<double, 20, 1> B_posr;

    Matrix<double, 20, 20> A_constr;
    Matrix<double, 20, 1> B_constr;

    Matrix<double, 7, 1> q_l;
    Matrix<double, 7, 1> q_r;
    Matrix3d R_l;
    Vector3d p_l;
    Matrix3d R_r;
    Vector3d p_r;
    Vector6d error_constraint;

    Matrix<double, 20, 20> Ak;  // = 2 * Mk / dT + B_ + A_posr;
    Matrix<double, 20, 1> Bk;  // = 2 * Mk / dT * dq + B_posr;

    // collision avoidance

    Matrix<double, 3 * 15, 1> p_link;
    Matrix<double, 3 * 15, 3> R_link;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    Matrix<double, DoF, DoF> K_init =
      4e1 * Matrix<double, DoF, DoF>::Identity();
    Matrix<double, DoF, 1> error_init = Matrix<double, DoF, 1>::Zero();
    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;
    double H_tot_prev = 0.0;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 20> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    constexpr int nV = 20;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP
    Vkhat.setZero();

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.useInitGuess = true;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    q.setZero();
    for (int i = 0; i < N_itr; i++) {
      //   std::cout << "iter: " << i << std::endl;

      if (i == 0) {
        q = q_init;
        dq.setZero();
        p_o_tmp = p_o_init;
        R_o_tmp = R_o_init;
      }

      error_op = p_o_tmp - p_ed;
      A_posr =
        J_ee_dual_pos.transpose() * (2.0 * dT * K_p_dual) * J_ee_dual_pos;
      B_posr = -J_ee_dual_pos.transpose() * K_p_dual * error_op;

      Vector6d error_init_obj =
        kine_ik.error_car(p_o_tmp, R_o_tmp, p_o_init, R_o_init);
      error_init.head(14) = q - q_init;
      error_init.tail(6) = error_init_obj;
      Matrix<double, DoF, DoF> A_init = dT * K_init;
      Matrix<double, DoF, 1> B_init = -K_init * error_init;
      //// constraint error /////////

      Jaco_c = franka_dual_ik.Jacobian_constraint(q, R_o_tmp);

      q_l = q.head(7);
      q_r = q.tail(7);
      franka_ik.Fkine_franka(q_l, p_l, R_l);
      franka_ik.Fkine_franka(q_r, p_r, R_r);
      error_constraint.head(3) =
        p_l_base + R_l_base * p_l - (p_o_tmp - R_o_tmp * d_vec);
      error_constraint.tail(3) =
        p_r_base + R_r_base * p_r - (p_o_tmp + R_o_tmp * d_vec);

      A_constr = Jaco_c.transpose() * (dT * 1e2) * Jaco_c;
      B_constr = -Jaco_c.transpose() * 1e2 * error_constraint;

      Ak = 2 * Mk / dT + B_ + A_posr + A_constr + A_init;
      Bk = 2 * Mk / dT * dq + B_posr + B_constr + B_init;

      // collision avoidance

      franka_dual_ik.Dual_arm_pose_link(q, p_o_tmp, R_o_tmp, p_link, R_link);
      col::apply_dual_arm_pose(dual_arm_object, p_link, R_link);

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * (error_op.transpose() * K_p_dual * error_op).value();

      double dH = std::abs(H_prev - H_tot);
      double dq_norm = dq.norm();

      if (i == 0) {
      } else {
        if (dH < 1e-4) {
          break;
        } else {
          H_prev = H_tot;
        }
      }
      // QP formulation

      Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
      Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
      lb_joint.block<7, 1>(0, 0) = (joint_limit_lb_franka - q) / dT;
      lb_joint.block<7, 1>(7, 0) = (joint_limit_lb_franka - q) / dT;
      lb_joint.block<6, 1>(14, 0) = MatrixXd::Constant(6, 1, -0.02);

      ub_joint.block<7, 1>(0, 0) = (joint_limit_ub_franka - q) / dT;
      ub_joint.block<7, 1>(7, 0) = (joint_limit_ub_franka - q) / dT;
      ub_joint.block<6, 1>(14, 0) = MatrixXd::Constant(6, 1, 0.02);

      ///// equality constraint
      // prob.eqOperator = Jaco_c;
      // prob.eqValue = constraint_;
      prob.costQuadratic = H;
      prob.costLinear = g;

      collision_result_ = col::broad_phase_collision_detection(
        dual_arm_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
        prob.ineqOperator = Matrix<double, 20, 20>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;

        prob.eqOperator = Jaco_c;
        prob.eqValue = constraint_;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 5e-1 * Eigen::VectorXd::Ones(nC);
        Eigen::MatrixXd A_ineq(nC + 20, 20);
        A_ineq.setZero();

        Eigen::MatrixXd A_eq(nC + 6, 20);
        A_eq.setZero();
        Eigen::MatrixXd constraint_eq(nC + 6, 1);
        constraint_eq.setZero();
        int i_col = 0;

        for (const auto& pair : geom_index) {
          // how to calculate minimum distance between link & obstacle

          auto& link = dual_arm_object[pair.first];
          auto& obstacle = obstacles[pair.second];

          link_index_col = link->link_index;

          auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
            mtd->feature(
              *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
              obstacle->R);

          dgap_jaco.block<1, 3>(0, 0) =
            dgap.block<1, 3>(0, 3);  // rotation part
          dgap_jaco.block<1, 3>(0, 3) =
            dgap.block<1, 3>(0, 0);  // position part

          Jaco_link_col =
            franka_dual_ik.Jacobian_link_Dual_arm(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - 1.0 * gap;
          i_col++;
        }

        VectorXd lb(nC + 20);
        lb.head(nC) = lb_col;
        lb.tail(20) = lb_joint;

        VectorXd ub(nC + 20);
        ub.head(nC) = ub_col;
        ub.tail(20) = ub_joint;

        // A_ineq.block(0, 0, nC, 20) = Jaco_col;
        // A_ineq.block(nC, 0, 20, 20) = Matrix<double, 20, 20>::Identity();

        // prob.ineqOperator = A_ineq;
        // prob.ineqUpperValue = ub;
        // prob.ineqLowerValue = lb;

        A_eq.block(0, 0, nC, 20) = Jaco_col;
        A_eq.block(nC, 0, 6, 20) = Jaco_c;

        prob.eqOperator = A_eq;
        prob.eqValue = constraint_eq;

        prob.ineqOperator = Matrix<double, 20, 20>::Identity();
        prob.ineqUpperValue = ub_joint;
        prob.ineqLowerValue = lb_joint;
      }

      qpoasesSolver.solve(prob, Vkhat);

      q = q + Vkhat.block<14, 1>(0, 0) * dT;
      p_o_tmp = p_o_tmp + Vkhat.block<3, 1>(17, 0) * dT;
      Matrix3d skew_omega = kine_ik.skew(Vkhat.block<3, 1>(14, 0) * dT);
      R_o_tmp = R_o_tmp * skew_omega.exp();
      dq = 2 * Vkhat - dq;

      //   std::cout << "c\n";
    }

    q_sol = q;
    p_o_sol = p_o_tmp;
    R_o_sol = R_o_tmp;
  }

  void PMI_IK_franka_iteration(
    const Matrix<double, 7, 1>& q_init,
    const Eigen::Matrix<double, 7, 1>& dq_init, double& V_e_pre,
    const Vector3d& p_rd, const Matrix3d& R_rd,
    std::vector<std::unique_ptr<col::Object>>& franka_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles,
    Eigen::Matrix<double, 7, 1>& q_sol, Eigen::Matrix<double, 7, 1>& dq_sol,
    double& V_e) {
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.2 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 2 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> lb_joint;
    Matrix<double, 7, 1> ub_joint;

    double kr__itr = 1e5;
    Matrix3d K_p_itr = 1e5 * Matrix3d::Identity();

    Matrix3d K_r = 1 * kr__itr * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = 1 * K_p_itr;

    //// matrices

    Vector6d error_r;  // = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
    Matrix<double, 6, 7> J_r;  // = franka_ik.Jacobian_franka(q);
    Matrix<double, 7, 7> A_posr;  // = J_r.transpose() * (dT * K_pose) * J_r;
    Matrix<double, 7, 1> B_posr;  // = -J_r.transpose() * K_pose * error_r;

    Matrix<double, 7, 7> Ak;  // = 2 * Mk / dT + B_ + A_posr;
    Matrix<double, 7, 1> Bk;  // = 2 * Mk / dT * dq + B_posr;

    // collision avoidance

    Matrix<double, 3 * 7, 1> p_link;
    Matrix<double, 3 * 7, 3> R_link;

    // obstacle avoidance

    std::vector<std::pair<int, int>> geom_index;
    bool collision_result_;

    int link_index_col = -1;

    Eigen::VectorXd lb_col;
    Eigen::VectorXd ub_col;

    Eigen::MatrixXd Jaco_col;

    double dist_thr = 0.01;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 7> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    constexpr int nV = 7;
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP
    Vkhat.setZero();

    qpsolver::QpoasesSolver qpoasesSolver;
    qpoasesSolver.hotstart = false;
    qpoasesSolver.useInitGuess = true;
    qpoasesSolver.maxWorkingSetRecalculation = 10000;
    qpoasesSolver.preset = QpoasesSolver::Preset::MPC;
    qpoasesSolver.printLevel = QpoasesSolver::PrintLevel::NONE;
    // qpsolver::QPSolution qpoasesSolution;
    qpsolver::QPProblem prob;

    Matrix3d R_r;
    Vector3d p_r;

    R_r.setZero();
    p_r.setZero();
    franka_ik.Fkine_franka(q_init, p_r, R_r);
    error_r = kine_ik.error_car(p_r, R_r, p_rd, R_rd);
    J_r = franka_ik.Jacobian_franka(q_init);
    A_posr = J_r.transpose() * (dT * K_pose * 2.0) * J_r;
    B_posr = -J_r.transpose() * K_pose * error_r;

    Ak = 2 * Mk / dT + B_ + A_posr;
    Bk = 2 * Mk / dT * dq_init + B_posr;

    // collision avoidance

    franka_ik.franka_pose_link(q_init, p_link, R_link);
    col::apply_franka_pose(franka_object, p_link, R_link);

    // potential function calculation

    V_e = 0.5 * (dq_init.transpose() * Mk * dq_init).value() +
      0.5 *
        (error_r.block<3, 1>(3, 0).transpose() * K_p_itr *
         error_r.block<3, 1>(3, 0))
          .value() +
      0.5 * kr__itr *
        double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

    double dH = std::abs(V_e_pre - V_e);

    if (dH < 1e-4) {
      q_sol = q_init;
      dq_sol = dq_init;
      // V_e = V_e_pre;
      return;
    } else {
    }

    // QP formulation

    Eigen::MatrixXd H = Ak;  // Ak.transpose() * Ak;
    Eigen::VectorXd g = -Bk;  //-Ak.transpose() * Bk;
    lb_joint = (joint_limit_lb_franka - q_init) / dT;
    ub_joint = (joint_limit_ub_franka - q_init) / dT;

    prob.costQuadratic = H;
    prob.costLinear = g;
    //   prob.boxLowerValue = lb_joint;
    //   prob.boxUpperValue = ub_joint;

    collision_result_ = col::broad_phase_collision_detection(
      franka_object, obstacles, geom_index);
    if (!collision_result_) {
      nC = 0;
      prob.ineqOperator = Matrix<double, 7, 7>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;

    } else {
      nC = static_cast<int>(geom_index.size());
      Jaco_col.resize(nC, nV);
      lb_col = Eigen::VectorXd::Zero(nC);
      ub_col = 5e-1 * Eigen::VectorXd::Ones(nC);
      Eigen::MatrixXd A_ineq(nC + 7, 7);
      A_ineq.setZero();
      Eigen::MatrixXd constraint_(nC, 1);
      constraint_.setZero();

      int i_col = 0;

      for (const auto& pair : geom_index) {
        // how to calculate minimum distance between link & obstacle

        auto& link = franka_object[pair.first];
        auto& obstacle = obstacles[pair.second];

        link_index_col = link->link_index;

        auto [gap, s_1, s_2, normal, dgap, ds_1_dq, ds_2_dq, dnormal] =
          mtd->feature(
            *link->dsf, link->p, link->R, *obstacle->dsf, obstacle->p,
            obstacle->R);

        dgap_jaco.block<1, 3>(0, 0) = dgap.block<1, 3>(0, 3);  // rotation part
        dgap_jaco.block<1, 3>(0, 3) = dgap.block<1, 3>(0, 0);  // position part

        Jaco_link_col = franka_ik.Jacobian_link_franka(q_init, link_index_col);

        Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
        lb_col(i_col) = dist_thr - 1.0 * gap;
        i_col++;
      }

      VectorXd lb(nC + 7);
      lb.head(nC) = lb_col;
      lb.tail(7) = lb_joint;

      VectorXd ub(nC + 7);
      ub.head(nC) = ub_col;
      ub.tail(7) = ub_joint;

      // A_ineq.block(0, 0, nC, 7) = Jaco_col;
      // A_ineq.block(nC, 0, 7, 7) = Matrix<double, 7, 7>::Identity();

      // prob.ineqOperator = A_ineq;
      // prob.ineqUpperValue = ub;
      // prob.ineqLowerValue = lb;
      prob.eqOperator = Jaco_col;
      prob.eqValue = constraint_;
      prob.ineqOperator = Matrix<double, 7, 7>::Identity();
      prob.ineqUpperValue = ub_joint;
      prob.ineqLowerValue = lb_joint;
    }

    auto flag = qpoasesSolver.solve(prob, Vkhat);

    q_sol = q_init + Vkhat * dT;
    dq_sol = 2 * Vkhat - dq_init;
  }
}  // namespace pmi_ik