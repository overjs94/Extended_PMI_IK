#include "qpOASES.hpp"
#include "qp_solvers.hpp"

namespace qpsolver
{
  using solver_real_t = qpOASES::real_t;
  using solver_matrix_t = Eigen::Matrix<solver_real_t, -1, -1, Eigen::RowMajor>;
  using solver_vector_t = Eigen::Matrix<solver_real_t, -1, +1>;

  struct QpoasesSolver::Impl
  {
    std::unique_ptr<qpOASES::SQProblem> solver;
    std::unique_ptr<qpOASES::Options> options;
  };

  QpoasesSolver::QpoasesSolver()
  {
    _impl = std::make_unique<Impl>();
  }

  QpoasesSolver::~QpoasesSolver() = default;

  QPExitCode
  QpoasesSolver::solve(QPProblem const& problem, vector_t& solution)
  {
    size_t const nVar = problem.costQuadratic.rows();
    size_t const nBoxL = problem.boxLowerValue.size();
    size_t const nBoxU = problem.boxUpperValue.size();
    size_t const nEq = problem.eqOperator.rows();
    size_t const nIneq = problem.ineqOperator.rows();
    size_t const nIneqL = problem.ineqLowerValue.size();
    size_t const nIneqU = problem.ineqUpperValue.size();

    if (!checkProblemSize(problem) || nVar != solution.size())
      return QPExitCode::INVALID_ARGUMENT_SIZE;

    bool const useBoxL = nBoxL > 0;
    bool const useBoxU = nBoxU > 0;
    bool const useEq = nEq > 0;
    bool const useIneq = nIneq > 0;

    qpOASES::int_t const nV = nVar;
    qpOASES::int_t const nC = nEq + nIneq;

    solver_matrix_t H = problem.costQuadratic;
    solver_vector_t g = problem.costLinear;
    solver_vector_t lb = problem.boxLowerValue;
    solver_vector_t ub = problem.boxUpperValue;
    solver_matrix_t A(nC, nV);
    solver_vector_t lbA(nC);
    solver_vector_t ubA(nC);

    auto const& eqA = useEq ? problem.eqOperator : matrix_t(0, nVar);
    auto const& eqB = useEq ? problem.eqValue : vector_t(0);
    auto const& ineqA = useIneq ? problem.ineqOperator : matrix_t(0, nVar);
    auto const& ineqL = useIneq ? problem.ineqLowerValue : vector_t(0);
    auto const& ineqU = useIneq ? problem.ineqUpperValue : vector_t(0);

    A << eqA, ineqA;
    lbA << eqB, ineqL, vector_t::Constant(nIneq - nIneqL, -qpOASES::INFTY);
    ubA << eqB, ineqU, vector_t::Constant(nIneq - nIneqU, +qpOASES::INFTY);

    if (!hotstart)
    {
      _impl->solver.reset();
      _impl->solver = std::make_unique<qpOASES::SQProblem>(nV, nC);
      _impl->options.reset();
      _impl->options = std::make_unique<qpOASES::Options>();
    }
    else
    {
      if (!_impl->solver || !_impl->options || useInitGuess)
        return QPExitCode::UNSUPPORTED_OPTION;

      if (_impl->solver->getNV() != nV || _impl->solver->getNC() != nC)
        return QPExitCode::INVALID_ARGUMENT_SIZE;
    }

    if (preset.has_value())
    {
      switch (preset.value())
      {
      case Preset::DEFAULT:
        _impl->options->setToDefault();
        break;
      case Preset::FAST:
        _impl->options->setToFast();
        break;
      case Preset::MPC:
        _impl->options->setToMPC();
        break;
      case Preset::RELIABLE:
        _impl->options->setToReliable();
        break;
      }
    }

    if (printLevel.has_value())
    {
      switch (printLevel.value())
      {
      case PrintLevel::NONE:
        _impl->options->printLevel = qpOASES::PL_NONE;
        break;
      case PrintLevel::LOW:
        _impl->options->printLevel = qpOASES::PL_LOW;
        break;
      case PrintLevel::MEDIUM:
        _impl->options->printLevel = qpOASES::PL_MEDIUM;
        break;
      case PrintLevel::HIGH:
        _impl->options->printLevel = qpOASES::PL_HIGH;
        break;
      case PrintLevel::TABULAR:
        _impl->options->printLevel = qpOASES::PL_TABULAR;
        break;
      }
    }

    _impl->solver->setOptions(*_impl->options.get());

    qpOASES::int_t nWsr = maxWorkingSetRecalculation;
    qpOASES::returnValue flag;

    if (!hotstart)
    {
      flag = _impl->solver->init(
        H.data(),
        g.data(),
        A.data(),
        useBoxL ? lb.data() : nullptr,
        useBoxU ? ub.data() : nullptr,
        lbA.data(),
        ubA.data(),
        nWsr,
        0,
        useInitGuess ? solution.data() : nullptr);
    }
    else
    {
      flag = _impl->solver->hotstart(
        H.data(),
        g.data(),
        A.data(),
        useBoxL ? lb.data() : nullptr,
        useBoxU ? ub.data() : nullptr,
        lbA.data(),
        ubA.data(),
        nWsr);
    }

    _impl->solver->getPrimalSolution(solution.data());

    return flag == qpOASES::returnValue::SUCCESSFUL_RETURN
      ? QPExitCode::SOLVER_SUCCESS
      : QPExitCode::SOLVER_FAILURE;
  }
}
