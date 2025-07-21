#include "data_struct.h"
#include "KinematicModel.h"
#include "MpcControl.h"
#include "MyReferencePath.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void plotpath(std::vector<Trajponits> path,std::string s)
{
    std::vector<double> x,y;
    for (auto p : path) {
        x.push_back(p.x);
        y.push_back(p.y);
    }

    plt::plot(x,y,s);
}
// 计算两点之间的距离
double dis_to_point(Trajponits p1, double x, double y) {
    return sqrt(pow(p1.x - x, 2) + pow(p1.y - y, 2));
}


int main() {
    // 初始化车辆状态
    VehicleState robot_state;
    robot_state.x = 0.0;
    robot_state.y = 0.0;
    robot_state.yaw = 0.0;
    robot_state.v = 0.0;

    KinematicModel Vehicle(robot_state); //初始状态
    MyReferencePath referencePath; // 参考轨迹
    std::vector<Trajponits> refer_path = referencePath.refer_path; // 获取参考轨迹

    //小车运动过程中的轨迹
    std::vector<double> x_history;
    std::vector<double> y_history;
    std::unique_ptr<MpcControl> mpcControl;


    while (1) {
        plt::clf();
        plotpath(refer_path,"b--");
        double accel, delta_f;
        // 状态更新
        mpcControl = std::make_unique<MpcControl>(Vehicle.getState(), refer_path);
        // 计算控制量
        mpcControl->MPCControl(accel, delta_f);
        // 状态更新
        Vehicle.updateState(accel, delta_f);
        robot_state = Vehicle.getState();
        x_history.push_back(robot_state.x);
        y_history.push_back(robot_state.y);
        plt::plot(x_history,y_history,"r-");
        plt::pause(0.01);

        if(dis_to_point(refer_path.back(),robot_state.x,robot_state.y)<0.5) {
            break;
        }
        // usleep(10 * 1000); // 10ms


    }
    plt::show();

    return 0;
}