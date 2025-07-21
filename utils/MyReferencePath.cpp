//
// Created by chh3213 on 2022/11/24.
//

#include "MyReferencePath.h"

/**
 * 构造函数，求解出参考轨迹点上的曲率等信息
 */
MyReferencePath::MyReferencePath() {
    setReferencePath();
}

void MyReferencePath::setReferencePath() {
    // 初始化参考轨迹点容器
    refer_path = std::vector<Trajponits>(1000);

    // 生成参考轨迹坐标(x, y)
    for (int i = 0; i < 1000; i++) {
        refer_path[i].x = 0.1 * i;
        refer_path[i].y = 2 * sin(refer_path[i].x / 3.0) + 2.5 * cos(refer_path[i].x / 2.0);

    }

    double dx, dy, ddx, ddy;
    for (int i = 0; i < refer_path.size(); i++) {
        if (i == 0) {
            // 处理第一个点
            dx = refer_path[i+1].x - refer_path[i].x;
            dy = refer_path[i+1].y - refer_path[i].y;
            ddx = refer_path[2].x + refer_path[0].x - 2 * refer_path[1].x;
            ddy = refer_path[2].y + refer_path[0].y - 2 * refer_path[1].y;
        } else if (i == refer_path.size() - 1) {
            // 处理最后一个点
            dx = refer_path[i].x - refer_path[i-1].x;
            dy = refer_path[i].y - refer_path[i-1].y;
            ddx = refer_path[i].x + refer_path[i-2].x - 2 * refer_path[i-1].x;
            ddy = refer_path[i].y + refer_path[i-2].y - 2 * refer_path[i-1].y;
        } else {
            // 处理中间点
            dx = refer_path[i+1].x - refer_path[i].x;
            dy = refer_path[i+1].y - refer_path[i].y;
            ddx = refer_path[i+1].x + refer_path[i-1].x - 2 * refer_path[i].x;
            ddy = refer_path[i+1].y + refer_path[i-1].y - 2 * refer_path[i].y;
        }

        // 计算航向角yaw
        refer_path[i].heading = atan2(dy, dx);

        // 计算曲率kappa（修正浮点数除法）
        double denominator = pow(dx * dx + dy * dy, 3.0 / 2.0);
        refer_path[i].kappa = (denominator > 1e-10) ? (ddy * dx - ddx * dy) / denominator : 0.0;

        // 计算速度v（假设为恒定速度，实际应用中可根据需求修改）
        refer_path[i].v = 3.0;

        // 计算加速度a（假设为零加速度，实际应用中可根据需求修改）
        refer_path[i].a = 0.0;

        // // 根据曲率设置速度（曲率越大，速度越小）
        // refer_path[i].v = 5.0 / (1.0 + abs(refer_path[i].kappa) * 2.0);
        //
        // if (i > 0) {
        //     // 根据相邻点速度差计算加速度
        //     refer_path[i].a = (refer_path[i].v - refer_path[i-1].v) / 0.1;  // 假设时间间隔为0.1s
        // } else {
        //     refer_path[i].a = 0.0;
        // }
    }
}

// /**
//  * 计算跟踪误差
//  * @param robot_state  机器人状态
//  * @return
//  */
// vector<double> MyReferencePath::calcTrackError(vector<double> robot_state) {
//     double x = robot_state[0],y=robot_state[1];
//     vector<double>d_x(refer_path.size()),d_y(refer_path.size()),d(refer_path.size());
//     for(int i=0;i<refer_path.size();i++) {
//         d_x[i]=refer_path[i][0]-x;
//         d_y[i]=refer_path[i][1]-y;
//         d[i] = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);
//
//     }
//     double min_index = min_element(d.begin(),d.end())-d.begin();
//     //cout<<min_index<<endl;
//     double yaw = refer_path[min_index][2];
//     double k = refer_path[min_index][3];
//     double angle = normalizeAngle(yaw- atan2(d_y[min_index],d_x[min_index]));
//     double error = d[min_index];//误差
//     if(angle<0)error*=-1;
//     return {error,k,yaw,min_index};
// }





