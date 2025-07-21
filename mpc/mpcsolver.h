//
// Created by wqx on 25-6-25.
//
#include <eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>


#ifndef MPCSOLVER_H
#define MPCSOLVER_H
class MpcSolver {
public:
    MpcSolver(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
            const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
            const Eigen::MatrixXd &matrix_initial_x,
            const Eigen::MatrixXd &matrix_u_lower,
            const Eigen::MatrixXd &matrix_u_upper,
            const Eigen::MatrixXd &matrix_x_lower,
            const Eigen::MatrixXd &matrix_x_upper,
            const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
            const int horizon, const double eps_abs);
    ~MpcSolver();

    bool Solve(Eigen::VectorXd &controlcmd);
private:
    /**
     x = [x_k, x_k+1 …… , x_k+N, u_k+1, u_k+2, ……, u_k+N-1]
         QP: 0.5 * x^T * P * x + q^T * x
            s.t.  l <= A * x <= u
                   lx <= x <= ux
    */
    // 输入：
    //      P: 二次项系数矩阵
    void computeHessianMatrix();  // 计算P矩阵
    //      q: 一次项系数矩阵
    void computeGradientMatrix();   // 计算q矩阵  偏置项
    //      A: 约束矩阵
    void computeConstraintMatrix(); // 计算A_constrant矩阵    约束矩阵
    void computeLowerAndUpperBound();       // 计算约束下界
    //      l: 约束下界 u: 约束上界
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;

    //! Hessian and Gradient matrices:
    Eigen::SparseMatrix<double> P_;
    Eigen::VectorXd q_;

    //! Constraint matrix:
    Eigen::SparseMatrix<double> A_;

    //=========== 输入参数 =========
    Eigen::MatrixXd matrix_a_;     // x_k+1 = A * x_k + B * u_k 中的 A
    Eigen::MatrixXd matrix_b_;     // x_k+1 = A * x_k + B * u_k 中的 B
    Eigen::MatrixXd matrix_q_;
    Eigen::MatrixXd matrix_r_;
    Eigen::MatrixXd matrix_initial_x_;
    const Eigen::MatrixXd matrix_u_lower_;
    const Eigen::MatrixXd matrix_u_upper_;
    const Eigen::MatrixXd matrix_x_lower_;
    const Eigen::MatrixXd matrix_x_upper_;
    const Eigen::MatrixXd matrix_x_ref_;
    int max_iteration_;
    size_t horizon_;
    double eps_abs_;
    size_t state_dim_;     //  状态量维度 --> 运动学建模 4 / 动力学建模 6
    size_t control_dim_;   //  控制量维度 --> 运动学建模 [a, delta] /  动力学建模[delta, a]
    size_t num_param_;
    int numVariables;     //  变量个数
    int numConstraints;   //  约束个数


    //! OSQP solver:
    std::unique_ptr<OsqpEigen::Solver> solver_;



};


#endif
