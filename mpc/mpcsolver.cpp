//
// Created by wqx on 25-6-25.
//
#include "mpcsolver.h"

MpcSolver::MpcSolver(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b, const Eigen::MatrixXd &matrix_q,
    const Eigen::MatrixXd &matrix_r, const Eigen::MatrixXd &matrix_initial_x, const Eigen::MatrixXd &matrix_u_lower,
    const Eigen::MatrixXd &matrix_u_upper, const Eigen::MatrixXd &matrix_x_lower, const Eigen::MatrixXd &matrix_x_upper,
    const Eigen::MatrixXd &matrix_x_ref, const int max_iter, const int horizon, const double eps_abs)
     :matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs){

    state_dim_ = matrix_b.rows();
    control_dim_ = matrix_b.cols();
    // std::cout << "state_dim  = " << state_dim_ << std::endl;
    // std::cout << "control_dim_ = " << control_dim_<< std::endl;
    numVariables = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    numConstraints = 2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;

    solver_ = std::unique_ptr<OsqpEigen::Solver>(new OsqpEigen::Solver());
    // 初始化
    P_.resize(numVariables, numVariables);
    P_.setZero();
    q_ = Eigen::VectorXd::Zero(numVariables);
    A_.resize(numConstraints, numVariables);
    A_.setZero();
    lower_bound_ = Eigen::VectorXd::Zero(numConstraints);
    upper_bound_ = Eigen::VectorXd::Zero(numConstraints);

}

MpcSolver::~MpcSolver() {
}

bool MpcSolver::Solve(Eigen::VectorXd &controlcmd) {
    //二次规划求解 0.5 * x^T * P * x + q^T * x
    //step1： 计算P矩阵
    computeHessianMatrix();
    //step2:  计算q矩阵
    computeHessianMatrix();
    // step3: 计算上下边界
    computeLowerAndUpperBound();
    //step4:  计算约束矩阵 A_constrant
    computeConstraintMatrix();

    //求解器设置
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setMaxIteration(max_iteration_);
    solver_->settings()->setAbsoluteTolerance(eps_abs_);
    solver_->settings()->setPolish(true);
    solver_->settings()->setScaledTerimination(true);

    // 数据设置
    solver_->data()->setNumberOfConstraints(numConstraints);
    solver_->data()->setNumberOfVariables(numVariables);

    if (!solver_->data()->setHessianMatrix(P_)) {
        std::cerr << "Failed to set Hessian matrix." << std::endl;
        return false;
    }
    if (!solver_->data()->setGradient(q_)) {
        std::cerr << "Failed to set gradient vector." << std::endl;
        return false;
    }
    if (!solver_->data()->setLinearConstraintsMatrix(A_)) {
        std::cerr << "Failed to set linear constraints matrix." << std::endl;
        return false;
    }
    if (!solver_->data()->setLowerBound(lower_bound_)) {
        std::cerr << "Failed to set lower bound." << std::endl;
        return false;
    }
    if (!solver_->data()->setUpperBound(upper_bound_)) {
        std::cerr << "Failed to set upper bound." << std::endl;
        return false;
    }

    // 初始化求解器
    if (!solver_->initSolver()) {
        std::cerr << "Failed to initialize solver." << std::endl;
        return false;
    }

    // 求解问题
    OsqpEigen::ErrorExitFlag exitFlag = solver_->solveProblem();
    if (exitFlag != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Failed to solve the problem." << std::endl;
        return false;
    }
    // 获取求解结果
    Eigen::VectorXd solution = solver_->getSolution();


    controlcmd = solution.segment(state_dim_ * (horizon_ + 1), control_dim_);

    return  true;


}

void MpcSolver::computeHessianMatrix() {

    // 定义非零元素向量
    std::vector<Eigen::Triplet<double>> triplets;

    // 填充状态权重矩阵Q
    for (size_t i = 0; i <= horizon_; ++i) {
        const size_t start_idx = i * state_dim_;
        for (size_t row = 0; row < state_dim_; ++row) {
            for (size_t col = 0; col < state_dim_; ++col) {
                triplets.emplace_back(start_idx + row, start_idx + col, matrix_q_(row, col));
            }
        }
    }

    // 填充控制权重矩阵R
    const size_t state_block_size = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < horizon_; ++i) {
        const size_t start_idx = state_block_size + i * control_dim_;
        for (size_t row = 0; row < control_dim_; ++row) {
            for (size_t col = 0; col < control_dim_; ++col) {
                triplets.emplace_back(start_idx + row, start_idx + col, matrix_r_(row, col));
            }
        }
    }

    // 设置稀疏矩阵的非零元素
    P_.setFromTriplets(triplets.begin(), triplets.end());

}

void MpcSolver::computeGradientMatrix() {
    // q矩阵应该是[0,0,0,……,0]
}

void MpcSolver::computeConstraintMatrix() {
    std::vector<Eigen::Triplet<double>> triplets;

    const size_t total_state_dim = state_dim_ * (horizon_ + 1);
    const size_t total_control_dim = control_dim_ * horizon_;
    const size_t u_offset = total_state_dim;

    // 1. 初始状态约束 x0 = x_init  ->  A(row, col) = I at x0
    for (size_t row = 0; row < state_dim_; ++row) {
        triplets.emplace_back(row, row, 1.0);
    }

    // 2. 动力学约束 x_{k+1} - A x_k - B u_k = 0
    //    添加到第 row = state_dim_ + ... 后续行
    for (size_t k = 0; k < horizon_; ++k) {
        size_t row_offset = state_dim_ + k * state_dim_;

        // -I * x_k
        for (size_t i = 0; i < state_dim_; ++i) {
            for (size_t j = 0; j < state_dim_; ++j) {
                triplets.emplace_back(row_offset + i, k * state_dim_ + j, -matrix_a_(i, j));
            }
        }

        // I * x_{k+1}
        for (size_t i = 0; i < state_dim_; ++i) {
            triplets.emplace_back(row_offset + i, (k + 1) * state_dim_ + i, 1.0);
        }

        // B * u_k
        for (size_t i = 0; i < state_dim_; ++i) {
            for (size_t j = 0; j < control_dim_; ++j) {
                triplets.emplace_back(row_offset + i, u_offset + k * control_dim_ + j, -matrix_b_(i, j));
            }
        }
    }

    // 3. 状态不等式约束 x_k ∈ [x_lower, x_upper]
    size_t state_ineq_offset = state_dim_ + horizon_ * state_dim_;
    for (size_t k = 0; k <= horizon_; ++k) {
        size_t row_offset = state_ineq_offset + k * state_dim_;
        for (size_t i = 0; i < state_dim_; ++i) {
            triplets.emplace_back(row_offset + i, k * state_dim_ + i, 1.0);
        }
    }

    // 4. 控制不等式约束 u_k ∈ [u_lower, u_upper]
    size_t control_ineq_offset = state_ineq_offset + (horizon_ + 1) * state_dim_;
    for (size_t k = 0; k < horizon_; ++k) {
        size_t row_offset = control_ineq_offset + k * control_dim_;
        for (size_t i = 0; i < control_dim_; ++i) {
            triplets.emplace_back(row_offset + i, u_offset + k * control_dim_ + i, 1.0);
        }
    }

    A_.setFromTriplets(triplets.begin(), triplets.end());
}


void MpcSolver::computeLowerAndUpperBound() {
    const size_t total_state_dim = state_dim_ * (horizon_ + 1);
    const size_t total_control_dim = control_dim_ * horizon_;

    lower_bound_ = Eigen::VectorXd::Zero(numConstraints);
    upper_bound_ = Eigen::VectorXd::Zero(numConstraints);

    // 1. 初始状态 x0 = x_init
    lower_bound_.segment(0, state_dim_) = matrix_initial_x_;
    upper_bound_.segment(0, state_dim_) = matrix_initial_x_;

    // 2. 动力学等式 x_{k+1} = Ax_k + Bu_k
    size_t eq_offset = state_dim_;
    for (size_t i = 0; i < horizon_; ++i) {
        lower_bound_.segment(eq_offset + i * state_dim_, state_dim_) = Eigen::VectorXd::Zero(state_dim_);
        upper_bound_.segment(eq_offset + i * state_dim_, state_dim_) = Eigen::VectorXd::Zero(state_dim_);
    }

    // 3. 状态变量不等式约束
    size_t state_ineq_offset = eq_offset + horizon_ * state_dim_;
    for (size_t i = 0; i <= horizon_; ++i) {
        lower_bound_.segment(state_ineq_offset + i * state_dim_, state_dim_) = matrix_x_lower_;
        upper_bound_.segment(state_ineq_offset + i * state_dim_, state_dim_) = matrix_x_upper_;
    }

    // 4. 控制变量不等式约束
    size_t control_ineq_offset = state_ineq_offset + (horizon_ + 1) * state_dim_;
    for (size_t i = 0; i < horizon_; ++i) {
        lower_bound_.segment(control_ineq_offset + i * control_dim_, control_dim_) = matrix_u_lower_;
        upper_bound_.segment(control_ineq_offset + i * control_dim_, control_dim_) = matrix_u_upper_;
    }
}

