//
// Created by wqx on 25-6-25.
//
#ifndef MPCCONTROL_H
#define MPCCONTROL_H
#include "data_struct.h"
#include "../param.h"
#include "mpcsolver.h"
#include <eigen3/Eigen/Dense>

using Matrix = Eigen::MatrixXd;
class MpcControl {
public:
    MpcControl(const VehicleState &vehiclestate, const std::vector<Trajponits> &refer_path);
    ~MpcControl();
    void MPCControl(double &accel, double &delta_f);
private:
    void Init();
    void LoadParam();
    void UpdateMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B, const Trajponits &ref_point);
    void UpdateState(const VehicleState vehicle_state);
    Trajponits QueryNearestPointByPosition(const std::vector<Trajponits> &point, const double x, const double y);

    void ComputeLateralErrors(const double x, const double y, const double theta,
                          const double linear_v, const double angular_v,
                          const double linear_a);

    void ComputeLongitudinalErrors(const std::vector<Trajponits> &trajectory_, const VehicleState &vehicleState);



private:
    // x_dot = A * x + B * u
    // vehicle state matrix
    Eigen::MatrixXd matrix_a_;  // A矩阵
    // control matrix
    Eigen::MatrixXd matrix_b_;  // B矩阵
    // state matrix
    Eigen::MatrixXd matrix_state_;

    Eigen::MatrixXd matrix_r_;   // R矩阵 控制量权重系数
    Eigen::MatrixXd matrix_q_;   // Q矩阵 状态量权重系数

    double ts_ = 0.0;
    double wheelbase_ = 0.0;     // 轴距

    double wheel_single_direction_max_degree_ = 0.0;  // 最大前轮转角

    double max_acceleration_ = 0.0;

    double max_deceleration_ = 0.0;

    // parameters for mpc solver; number of iterations
    int mpc_max_iteration_ = 0;
    // parameters for mpc solver; threshold for computation
    double mpc_eps_ = 0.0;

    const int basic_state_size_ = 4;  // 状态量维度 横向位置误差、纵向位置误差、heading 误差、 速度误差
    const int controls_ = 2;          // 控制两维度 [a delta]   加速度和前轮转角
    // 预测步长
    const int horizon_ = 10;


    LongitudinalControlError lon_con_err;  //纵向误差
    LateralControlError lat_con_err;       //横向误差

    double acceleration_reference;


    VehicleState localization;
    std::vector<Trajponits> trajectory_points_;

    Trajponits match_point_;
};

#endif //MPCCONTROL_H
