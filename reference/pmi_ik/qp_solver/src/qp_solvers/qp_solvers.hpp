#pragma once

#include <Eigen/Core>
#include <memory>

namespace qpsolver
{
  using int_t = int;
  using real_t = double;
  using matrix_t = Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<real_t, Eigen::Dynamic, 1>;

  struct QPProblem
  {
    matrix_t costQuadratic;
    vector_t costLinear;

    vector_t boxLowerValue;
    vector_t boxUpperValue;

    matrix_t eqOperator;
    vector_t eqValue;

    matrix_t ineqOperator;
    vector_t ineqLowerValue;
    vector_t ineqUpperValue;
  };

  bool
  checkProblemSize(QPProblem const& problem);

  enum QPExitCode
  {
    SOLVER_SUCCESS,
    SOLVER_FAILURE,
    UNSUPPORTED_OPTION,
    INVALID_ARGUMENT_SIZE,
  };

  class QPSolver
  {
  public:
    QPSolver() = default;
    virtual ~QPSolver() = default;

    virtual QPExitCode
    solve(QPProblem const& problem, vector_t& solution) = 0;
  };

  class QpoasesSolver : public QPSolver
  {
  protected:
    struct Impl;
    std::unique_ptr<Impl> _impl;

  public:
    QpoasesSolver();
    ~QpoasesSolver();

    QPExitCode
    solve(QPProblem const& problem, vector_t& solution) override;

    enum class Preset
    {
      DEFAULT,
      FAST,
      MPC,
      RELIABLE,
    };

    enum class PrintLevel
    {
      NONE,
      LOW,
      MEDIUM,
      HIGH,
      TABULAR,
    };

    bool hotstart{false};
    bool useInitGuess{false};
    unsigned int maxWorkingSetRecalculation{0};
    std::optional<Preset> preset;
    std::optional<PrintLevel> printLevel;
  };

  class QpswiftSolver : public QPSolver
  {
  public:
    QpswiftSolver();
    ~QpswiftSolver();

    QPExitCode
    solve(QPProblem const& problem, vector_t& solution) override;

    std::optional<int_t> maxIteration;
    std::optional<bool> verbose;
  };
}
