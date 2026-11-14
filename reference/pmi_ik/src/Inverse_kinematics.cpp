#include "Kinematics.h"
#include "Inverse_kinematics.h"
#include "KUKA_kinematics/KUKA_kinematics.h"
#include "Franka_kinematics/Franka_kinematics.h"
#include "LASDRA_kinematics/LASDRA_kinematics.h"
#include "dsf-mtd/mtd.hpp"

#include <qpOASES.hpp>

KUKA_KINEMATICS_::KUKA_Kinematics_ kuka;
FRANKA_KINEMATICS_::FRANKA_Kinematics_ franka;
LASDRA_KINEMATICS_::LASDRA_Kinematics_ lasdra;
KINEMATICS_::Kinematics_ kine;

namespace pmi_ik {
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  Matrix<double, 7, 1> PMI_IK_kuka(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd) {
    int N_itr = 200;
    double dT = 0.005;

    Eigen::MatrixXd Mk = 1e-2 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 1e-1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb;
    Matrix<double, 7, 1> ub;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    // potential

    double H_tot = 0.0;
    double H_prev = 0.0;

    // int nV = DoF;
    int nV = static_cast<int>(7);
    int nC = 0;

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
      kuka.Fkine_kuka(q, p_r, R_r);
      Vector6d error_r = kine.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = kuka.Jacobian_kuka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr;

      // potential function calculation

      // H_tot = 0.5*dq.transpose()*Mk*dq +
      // 0.5*error_r.block<3,1>(3,0).transpose()*K_p*error_r.block<3,1>(3,0) +
      // 0.5 * 1e2 * double((Matrix3d::Identity() - R_rd.transpose() *
      // R_r).trace());

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      if (H_prev - H_tot > -5e-3) {
        break;
      } else {
        H_prev = H_tot;
      }

      // Instantiate a qpOASES solver object
      qpOASES::SQProblem solver(nV, nC);

      // Set qpOASES options
      qpOASES::Options options;
      // options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;  // suppress output
      solver.setOptions(options);
      // Define H and g from the given code
      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;

      lb = (-joint_limit_kuka - q) / dT;
      ub = (joint_limit_kuka - q) / dT;

      // Convert Eigen matrices and vectors to qpOASES data types
      std::vector<qpOASES::real_t> H_qpOASES(nV * nV);
      std::vector<qpOASES::real_t> g_qpOASES(nV);
      std::vector<qpOASES::real_t> lb_qpOASES(nV);
      std::vector<qpOASES::real_t> ub_qpOASES(nV);

      for (int j = 0; j < nV; j++) {
        g_qpOASES[j] = static_cast<qpOASES::real_t>(g(j));
        lb_qpOASES[j] = static_cast<qpOASES::real_t>(lb(j));
        ub_qpOASES[j] = static_cast<qpOASES::real_t>(ub(j));
        for (int k = 0; k < nV; k++) {
          H_qpOASES[j * nV + k] = static_cast<qpOASES::real_t>(H(j, k));
        }
      }

      // Solve the QP problem
      qpOASES::returnValue status;
      Eigen::VectorXd Vkhat = Eigen::VectorXd::Zero(nV);
      int nWSR = 1000;
      status = solver.init(
        H_qpOASES.data(), g_qpOASES.data(), nullptr, lb_qpOASES.data(),
        ub_qpOASES.data(), nullptr, nullptr, nWSR);

      // Retrieve the optimal solution
      std::vector<qpOASES::real_t> Vkhat_qpOASES(nV);
      solver.getPrimalSolution(Vkhat_qpOASES.data());

      // Convert the solution back to Eigen
      for (int j = 0; j < nV; j++) {
        Vkhat(j) = static_cast<double>(Vkhat_qpOASES[j]);
      }

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  Matrix<double, 7, 1> PMI_IK_franka(
    const Matrix<double, 7, 1>& q_init, const Vector3d& p_rd,
    const Matrix3d& R_rd,
    std::vector<std::unique_ptr<col::Object>>& franka_object,
    const std::vector<std::unique_ptr<col::Object>>& obstacles) {
    int N_itr = 200;
    double dT = 0.005;

    auto mtd = std::make_unique<dsf_mtd::MTD>(dsf_mtd::MTD::config_t {});

    Eigen::MatrixXd Mk = 0.5 * Eigen::MatrixXd::Identity(7, 7);

    Matrix<double, 7, 7> B_ = 0.1 * Matrix<double, 7, 7>::Identity();
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 7, 1> lb;
    Matrix<double, 7, 1> ub;

    Matrix3d K_r = kr__ * Matrix3d::Identity();

    Matrix6d K_pose = Matrix6d::Zero();

    K_pose.block<3, 3>(0, 0) = K_r;
    K_pose.block<3, 3>(3, 3) = K_p;

    Matrix<double, 7, 7> K_init = 1 * 1e3 * Matrix<double, 7, 7>::Identity();

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

    double dist_thr = 1 * 1e-3;
    Matrix<double, 1, 6> dgap_jaco;
    Matrix<double, 6, 7> Jaco_link_col;

    std::vector<Vector3d> pos_info_;  //= data->pos_info_; TODO: fix this

    // int nV = DoF;
    int nV = static_cast<int>(7);
    int nC = 0;

    Eigen::VectorXd Vkhat(nV);  // Needed after QP

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
      franka.Fkine_franka(q, p_r, R_r);
      Vector6d error_r = kine.error_car(p_r, R_r, p_rd, R_rd);
      Matrix<double, 6, 7> J_r = franka.Jacobian_franka(q);
      Matrix<double, 7, 7> A_posr = J_r.transpose() * (dT * K_pose) * J_r;
      Matrix<double, 7, 1> B_posr = -J_r.transpose() * K_pose * error_r;

      Matrix<double, 7, 7> A_init = dT * K_init;
      Matrix<double, 7, 1> B_init = -K_init * (q - q_init);

      Matrix<double, 7, 7> Ak = 2 * Mk / dT + B_ + A_posr + A_init;
      Matrix<double, 7, 1> Bk = 2 * Mk / dT * dq + B_posr + B_init;

      // collision avoidance

      Matrix<double, 3 * 7, 1> p_link;
      Matrix<double, 3 * 7, 3> R_link;
      franka.franka_pose_link(q, p_link, R_link);
      col::apply_franka_pose(franka_object, p_link, R_link);

      // nC = 0;

      collision_result_ = col::broad_phase_collision_detection(
        franka_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;

      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 5e-1 * Eigen::VectorXd::Ones(nC);

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

          Jaco_link_col = franka.Jacobian_link_franka(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - 0.2 * gap;
          i_col++;
        }
      }

      // potential function calculation

      // H_tot = 0.5*dq.transpose()*Mk*dq +
      // 0.5*error_r.block<3,1>(3,0).transpose()*K_p*error_r.block<3,1>(3,0) +
      // 0.5*1e2*(Matrix3d::Identity() - R_rd.transpose()*R_r).trace();

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 * ((q - q_init).transpose() * K_init * (q - q_init)).value() +
        0.5 *
          (error_r.block<3, 1>(3, 0).transpose() * K_p *
           error_r.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__ *
          double((Matrix3d::Identity() - R_rd.transpose() * R_r).trace());

      double dH = std::abs(H_prev - H_tot);
      if (dH < 1e-4) {
        break;
      } else {
        H_prev = H_tot;
      }

      // Instantiate a qpOASES solver object
      qpOASES::SQProblem solver(nV, nC);

      // Set qpOASES options
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;  // suppress output
      solver.setOptions(options);
      // Define H and g from the given code
      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;

      lb = (joint_limit_lb_franka - q) / dT;
      ub = (joint_limit_ub_franka - q) / dT;

      // Convert Eigen matrices and vectors to qpOASES data types
      std::vector<qpOASES::real_t> H_qpOASES(nV * nV);
      std::vector<qpOASES::real_t> g_qpOASES(nV);
      std::vector<qpOASES::real_t> lb_qpOASES(nV);
      std::vector<qpOASES::real_t> ub_qpOASES(nV);

      for (int j = 0; j < nV; j++) {
        g_qpOASES[j] = static_cast<qpOASES::real_t>(g(j));
        lb_qpOASES[j] = static_cast<qpOASES::real_t>(lb(j));
        ub_qpOASES[j] = static_cast<qpOASES::real_t>(ub(j));
        for (int k = 0; k < nV; k++) {
          H_qpOASES[j * nV + k] = static_cast<qpOASES::real_t>(H(j, k));
        }
      }

      if (nC == 0) {
        // Solve the QP problem
        qpOASES::returnValue status;
        Eigen::VectorXd Vkhat = Eigen::VectorXd::Zero(nV);
        int nWSR = 1000;
        status = solver.init(
          H_qpOASES.data(), g_qpOASES.data(), nullptr, lb_qpOASES.data(),
          ub_qpOASES.data(), nullptr, nullptr, nWSR);
      } else {
        std::vector<qpOASES::real_t> A_qpOASES(nC * nV);
        std::vector<qpOASES::real_t> lbA_qpOASES(nC);
        std::vector<qpOASES::real_t> ubA_qpOASES(nC);

        for (int j = 0; j < nC; j++) {
          lbA_qpOASES[j] = static_cast<qpOASES::real_t>(lb_col(j));
          ubA_qpOASES[j] = static_cast<qpOASES::real_t>(ub_col(j));
          for (int k = 0; k < nV; k++) {
            A_qpOASES[j * nV + k] =
              static_cast<qpOASES::real_t>(Jaco_col(j, k));
          }
        }

        // Solve the QP problem
        qpOASES::returnValue status;
        Eigen::VectorXd Vkhat = Eigen::VectorXd::Zero(nV);
        int nWSR = 1000;
        status = solver.init(
          H_qpOASES.data(), g_qpOASES.data(),
          A_qpOASES.data(),  // A matrix (row-major)
          lb_qpOASES.data(), ub_qpOASES.data(), lbA_qpOASES.data(),
          ubA_qpOASES.data(), nWSR);
      }

      // Retrieve the optimal solution
      std::vector<qpOASES::real_t> Vkhat_qpOASES(nV);
      solver.getPrimalSolution(Vkhat_qpOASES.data());

      // Convert the solution back to Eigen
      for (int j = 0; j < nV; j++) {
        Vkhat(j) = static_cast<double>(Vkhat_qpOASES[j]);
      }

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

  Matrix<double, DoF, 1> PMI_IK_lasdra(
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
    Matrix<double, DoF, 1> lb;
    Matrix<double, DoF, 1> ub;

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
      lasdra.Fkine_ee(q, p_e, R_e);
      Vector6d error_e = kine.error_car(p_e, R_e, p_ed, R_ed);
      Matrix<double, 6, DoF> J_e = lasdra.Jaco_ee(q);
      Matrix<double, DoF, DoF> A_pose = J_e.transpose() * (dT * K_pose) * J_e;
      Matrix<double, DoF, 1> B_pose = -J_e.transpose() * K_pose * error_e;

      // PMI equations
      Matrix<double, DoF, DoF> Ak = 2 * Mk / dT + B_ + A_pose;
      Matrix<double, DoF, 1> Bk = 2 * Mk / dT * dq + B_pose;
      // collision avoidance

      Matrix<double, 3 * n_link, 1> p_link;
      Matrix<double, 3 * n_link, 3> R_link;
      lasdra.LASDRA_Pose_link(q, p_link, R_link);
      col::apply_lasdra_pose(lasdra_object, p_link, R_link);

      collision_result_ = col::broad_phase_collision_detection(
        lasdra_object, obstacles, geom_index);
      if (!collision_result_) {
        nC = 0;
      } else {
        nC = static_cast<int>(geom_index.size());
        Jaco_col.resize(nC, nV);
        lb_col = Eigen::VectorXd::Zero(nC);
        ub_col = 1e4 * Eigen::VectorXd::Ones(nC);

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
          Jaco_link_col = lasdra.Jaco_link(q, link_index_col);

          Jaco_col.block(i_col, 0, 1, nV) = dgap_jaco * Jaco_link_col * dT;
          lb_col(i_col) = dist_thr - gap;
          i_col++;
        }
      }

      // potential function calculation

      H_tot = 0.5 * (dq.transpose() * Mk * dq).value() +
        0.5 *
          (error_e.block<3, 1>(3, 0).transpose() * K_p_lasdra *
           error_e.block<3, 1>(3, 0))
            .value() +
        0.5 * kr__lasdra *
          double((Matrix3d::Identity() - R_ed.transpose() * R_e).trace());

      double dH = std::abs(H_prev - H_tot);
      if (dH < 1e-4) {
        break;
      } else {
        H_prev = H_tot;
      }

      // Instantiate a qpOASES solver object
      qpOASES::SQProblem solver(nV, nC);

      // Set qpOASES options
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_NONE;  // suppress output
      solver.setOptions(options);
      // Define H and g from the given code
      Eigen::MatrixXd H = Ak.transpose() * Ak;
      Eigen::VectorXd g = -Ak.transpose() * Bk;

      lb = (-q_lim - q) / dT;
      ub = (q_lim - q) / dT;

      // Convert Eigen matrices and vectors to qpOASES data types
      std::vector<qpOASES::real_t> H_qpOASES(nV * nV);
      std::vector<qpOASES::real_t> g_qpOASES(nV);
      std::vector<qpOASES::real_t> lb_qpOASES(nV);
      std::vector<qpOASES::real_t> ub_qpOASES(nV);

      for (int j = 0; j < nV; j++) {
        g_qpOASES[j] = static_cast<qpOASES::real_t>(g(j));
        lb_qpOASES[j] = static_cast<qpOASES::real_t>(lb(j));
        ub_qpOASES[j] = static_cast<qpOASES::real_t>(ub(j));
        for (int k = 0; k < nV; k++) {
          H_qpOASES[j * nV + k] = static_cast<qpOASES::real_t>(H(j, k));
        }
      }

      if (nC == 0) {
        // Solve the QP problem
        qpOASES::returnValue status;
        Eigen::VectorXd Vkhat = Eigen::VectorXd::Zero(nV);
        int nWSR = 1000;
        status = solver.init(
          H_qpOASES.data(), g_qpOASES.data(), nullptr, lb_qpOASES.data(),
          ub_qpOASES.data(), nullptr, nullptr, nWSR);
      } else {
        std::vector<qpOASES::real_t> A_qpOASES(nC * nV);
        std::vector<qpOASES::real_t> lbA_qpOASES(nC);
        std::vector<qpOASES::real_t> ubA_qpOASES(nC);

        for (int j = 0; j < nC; j++) {
          lbA_qpOASES[j] = static_cast<qpOASES::real_t>(lb_col(j));
          ubA_qpOASES[j] = static_cast<qpOASES::real_t>(ub_col(j));
          for (int k = 0; k < nV; k++) {
            A_qpOASES[j * nV + k] =
              static_cast<qpOASES::real_t>(Jaco_col(j, k));
          }
        }

        // Solve the QP problem
        qpOASES::returnValue status;
        Eigen::VectorXd Vkhat = Eigen::VectorXd::Zero(nV);
        int nWSR = 1000;
        status = solver.init(
          H_qpOASES.data(), g_qpOASES.data(),
          A_qpOASES.data(),  // A matrix (row-major)
          lb_qpOASES.data(), ub_qpOASES.data(), lbA_qpOASES.data(),
          ubA_qpOASES.data(), nWSR);
      }

      // Retrieve the optimal solution
      std::vector<qpOASES::real_t> Vkhat_qpOASES(nV);
      solver.getPrimalSolution(Vkhat_qpOASES.data());

      // Convert the solution back to Eigen
      for (int j = 0; j < nV; j++) {
        Vkhat(j) = static_cast<double>(Vkhat_qpOASES[j]);
      }

      q = q + Vkhat * dT;
      dq = 2 * Vkhat - dq;
    }

    return q;
  }

}  // namespace pmi_ik