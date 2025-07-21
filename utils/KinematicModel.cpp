#include "KinematicModel.h"
/**
 * 机器人运动学模型构造
 * @param x 位置x
 * @param y 位置y
 * @param psi 偏航角
 * @param v 速度
 * @param dt 采样时间
 */
KinematicModel::KinematicModel(VehicleState robot_state) :
    vehicle(robot_state) {
    ts_ = dt;
    WHEEL_BASE_ = VehicleParam::car_wheelbase;

}
/**
 * 控制量为转向角delta_f和加速度a
 * @param accel 加速度
 * @param delta_f 转向角控制量
 */
void KinematicModel::updateState(double accel, double delta_f) {
    vehicle.x = vehicle.x + vehicle.v* cos(vehicle.yaw)*ts_;
    vehicle.y = vehicle.y + vehicle.v*sin(vehicle.yaw)*ts_;
    vehicle.yaw = vehicle.yaw + vehicle.v / WHEEL_BASE_ * tan(delta_f)*ts_;
    vehicle.v = vehicle.v + accel*ts_;
}

/**
 * 状态获取
 * @return
 */
VehicleState KinematicModel::getState() {
    return vehicle;
}
