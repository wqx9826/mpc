//
// Created by wqx on 25-6-25.
//

#ifndef PARAM_H
#define PARAM_H

static const double dt = 0.01; // 时间间隔
namespace MpcParam {
    static const int matrix_r_size = 2;                    // 优化系数R矩阵参数的个数 (控制量系数)
    static const std::vector<double> matrix_r = {3, 1}; // R 对角矩阵参数 加速度权重系数、转角权重系数
    static const int matrix_q_size = 4;                 // 优化系数Q矩阵参数的个数 (状态量系数)
    static const std::vector<double> matrix_q = {2,2,8,1};  //Q 对角矩阵参数   横向位置误差、纵向位置误差、heading 误差、 速度误差
    static const double mpcSolutionAccuracy = 0.01; // MPC 迭代求解精度   0.01
    static const int mpcMaxIteration = 1500;        // MPC的迭代次数 1500
}

namespace VehicleParam {
    static const double car_cf = 155494.663;        // 前轮侧偏刚度,左右轮之和     155494.663
    static const double car_cr = 155494.663;        // 后轮侧偏刚度, 左右轮之和    155494.663
    static const double car_mass = 1845.0;          //车重
    static const double car_mass_fl = 1845.0 / 4;   //左前悬重量
    static const double car_mass_fr = 1845.0 / 4;   //右前悬重量
    static const double car_mass_rl = 1845.0 / 4;   //左后悬重量
    static const double car_mass_rr = 1845.0 / 4;   //右后悬重量
    static const double car_wheelbase = 2.852;      //轴距    2.852
    static const double max_steer_angle = 35 * M_PI / 180;      // 最大前轮转角  弧度
    static const double max_acceleration = 2.0;     // 最大加速度
    static const double max_deceleration = -6.0;    // 最大减速度
}


#endif //PARAM_H
