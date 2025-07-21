//
// Created by wqx on 25-6-25.
//

#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <unistd.h>


class VehicleState{
public:
    double x;
    double y;
    double z;
    double yaw; //rad
    double v;   //m/s
    double w;   // rad/s
    double a;   // m/s^2
};


struct Trajponits {
    double x;
    double y;
    double heading;
    double v;
    double a;
    double kappa;
};

struct LateralControlError {
    double lateral_error; // 横向误差
    double heading_error; // 转向误差
    double lateral_error_rate;  // 横向误差速率
    double heading_error_rate;  // 转向误差速度
};

struct LongitudinalControlError{
    double station_error;   //纵向误差
    double speed_error;     //速度误差
};

/**
 * 角度归一化
 * @param angle
 * @return
 */
inline double normalizeAngle(double angle) {
    while(angle>M_PI){
        angle-=2.0*M_PI;
    }
    while(angle<-M_PI){
        angle+=2.0*M_PI;
    }
    return angle;
}

inline double PointDistanceSquare(const Trajponits &point, const double x, const double y)
{
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

#endif //DATA_STRUCT_H
