//
// Created by wqx on 25-6-25.
//

#include "MpcControl.h"



MpcControl::MpcControl(const VehicleState &vehiclestate, const std::vector<Trajponits> &refer_path)
                    :localization(vehiclestate), trajectory_points_(refer_path){
    Init();
    LoadParam();
}

MpcControl::~MpcControl() {
}

void MpcControl::MPCControl(double &accel, double &delta_f) {
    match_point_ = QueryNearestPointByPosition(trajectory_points_, localization.x, localization.y);

    ComputeLongitudinalErrors(trajectory_points_, localization);
    UpdateState(localization);
    // 确定A B矩阵
    UpdateMatrix(matrix_a_, matrix_b_,match_point_);


    Matrix control_matrix = Matrix::Zero(controls_, 1);  //[2 x 1]
    // 10个 [2 x 1]的矩阵
    std::vector<Matrix> control(horizon_, control_matrix);

    Matrix reference_state = Matrix::Zero(basic_state_size_, 1);
    Matrix lower_bound(controls_, 1);
    lower_bound << max_deceleration_, -wheel_single_direction_max_degree_;

    Matrix upper_bound(controls_, 1);
    upper_bound << max_acceleration_, wheel_single_direction_max_degree_;

    const double max_ = std::numeric_limits<double>::max();
    Matrix lower_state_bound(basic_state_size_, 1);
    Matrix upper_state_bound(basic_state_size_, 1);

    // lateral_error  station_error  heading_error  speed_error

    lower_state_bound << -1.0 * max_, -1.0 * max_, -1.0 * M_PI, -1.0 * max_;
    upper_state_bound << max_, max_, M_PI, max_;

    MpcSolver mpcSolver(matrix_a_, matrix_b_, matrix_q_, matrix_r_, matrix_state_, lower_bound, upper_bound,
                        lower_state_bound, upper_state_bound, reference_state, mpc_max_iteration_, horizon_, mpc_eps_);

    Eigen::VectorXd controlcmd_;
    mpcSolver.Solve(controlcmd_);
    accel = controlcmd_(0);
    delta_f =  - controlcmd_(1);
    std::cout << "accel: " << accel << std::endl;
    std::cout << "delta_f: " << delta_f << std::endl;
    std::cout << " ================== "<< std::endl;


}

void MpcControl::Init() {

    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);

    matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);

    // QP优化 权重参数，矩阵R 作用是调整控制量的权重，矩阵Q 作用是调整状态量的权重
    matrix_r_ = Matrix::Zero(controls_, controls_);
    matrix_r_ = Matrix::Identity(controls_, controls_);
    //矩阵Q 作用是调整状态量的权重
    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);



    int r_param_size = MpcParam::matrix_r_size;
    if (controls_ != r_param_size)
    {
        printf("MPC controller error: matrix_r size: %d in parameter file not equal to controls_size: %d\n",
               r_param_size, controls_);
    }
    assert(controls_ == r_param_size);

    for (int i = 0; i < r_param_size; ++i) {
        matrix_r_(i, i) = MpcParam::matrix_r[static_cast<size_t>(i)];
    }

    int q_param_size = MpcParam::matrix_q_size;

    if (basic_state_size_ != q_param_size)
    {
        printf("MPC controller error: matrix_q size: %d in parameter file not equal to basic_state_size_: %d\n",
               q_param_size, basic_state_size_);
    }
    assert(basic_state_size_ == q_param_size);
    for (int i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = MpcParam::matrix_q[static_cast<size_t>(i)];
    }
}

void MpcControl::LoadParam() {
    wheelbase_ = VehicleParam::car_wheelbase;

    mpc_eps_ = MpcParam::mpcSolutionAccuracy;
    mpc_max_iteration_ = MpcParam::mpcMaxIteration;

    ts_ = dt;   // 控制周期

    max_acceleration_ = VehicleParam::max_acceleration;
    max_deceleration_ = VehicleParam::max_deceleration;

    wheel_single_direction_max_degree_ = VehicleParam::max_steer_angle;

}

void MpcControl::UpdateMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B, const Trajponits &ref_point) {

    // Check for correct size of the matrices:
    /**
    A(0, 0) = 0;     A(0, 1) = 0;     A(0,2) = v*(-1)*std::sin(theta);    A(0,3) = std::cos(theta);

    A(1, 0) = 0;     A(1, 1) = 0;     A(1,2) = v*std::cos(theta);         A(1,3) = std::sin(theta);

    A(2, 0) = 0;     A(2, 1) = 0;     A(2,2) = 0;                           A(2,3) = std::tan(theta) / L;

    A(3, 0) = 0;     A(3, 1) = 0;     A(3,2) = 0;                           A(3,3) = 0;

    // Input to state mapping matrix (B)
    B(0,0) = 0;            B(0,1) = 0;

    B(1,0) = 0;            B(1,1) = 0;

    B(2,0) = 0;            B(2,1) = (v/L)*(1/ (cos(theta))^2);

    B(3,0) = 1;            B(3,1) = 0;
    */

    const double v_ref = ref_point.v;
    const double theta_ref = ref_point.heading;
    const double kappa_ref = ref_point.kappa;
    const double L = wheelbase_;

    double ref_delta = std::atan(L * kappa_ref);
    double tan_delta = std::tan(ref_delta);
    double cos_delta = std::cos(ref_delta);
    double cos_delta_squared = cos_delta * cos_delta;

    // basic_state_size_ = 4, controls_ = 2
    Eigen::MatrixXd Ac(basic_state_size_, basic_state_size_);
    Eigen::MatrixXd Bc(basic_state_size_, controls_);
    Ac.setZero();
    Bc.setZero();


    // 状态矩阵 Ac
    Ac(0, 2) = -v_ref * std::sin(theta_ref);
    Ac(0, 3) = std::cos(theta_ref);

    Ac(1, 2) = v_ref * std::cos(theta_ref);
    Ac(1, 3) = std::sin(theta_ref);

    Ac(2, 3) = tan_delta / L;

    // 控制矩阵 Bc
    Bc(2, 1) = v_ref / (L * cos_delta_squared);  // 对 delta 求偏导
    Bc(3, 0) = 1.0;  // dv/dt = a

    // === 离散化处理 ===
    A = Eigen::MatrixXd::Identity(basic_state_size_, basic_state_size_) + Ac * ts_;
    B = Bc * ts_;
}

void MpcControl::UpdateState(const VehicleState vehicle_state) {
    // 计算横向误差
    ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.yaw,
                         vehicle_state.v, vehicle_state.w,
                         vehicle_state.a);
    matrix_state_(0, 0) = lat_con_err.lateral_error;  // e_d
    matrix_state_(1, 0) = lon_con_err.station_error;  // e_s
    matrix_state_(2, 0) = lat_con_err.heading_error;  // e_theta
    matrix_state_(3, 0) = lon_con_err.speed_error;    // e_v
}

Trajponits MpcControl::QueryNearestPointByPosition(const std::vector<Trajponits> &point, const double x, const double y) {
    double d_min = PointDistanceSquare(point.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < point.size(); ++i) {
        double d_temp = PointDistanceSquare(point[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return point[index_min];
}

void MpcControl::ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a) {

    //使用match_point 来计算横向误差
    auto trajectory_points = match_point_;
    double dx = trajectory_points.x-x;
    double dy = trajectory_points.y-y;
    //            std::cout<<"参考点角度:"<<trajectory_points.heading<<std::endl;
    //            std::cout<<"主车角度:"<<theta<<std::endl;

    double delta = trajectory_points.heading-atan2(dy,dx);
    double e_y = -sin(delta)*sqrt(PointDistanceSquare(trajectory_points, x, y));
    //            std::cout << "横向误差："<< e_y <<std::endl;
    double e_theta = trajectory_points.heading-theta;
    e_theta = normalizeAngle(e_theta);
    //    if (e_theta > M_PI) {
    //        e_theta = e_theta - M_PI * 2;
    //    }
    //    if (e_theta < -M_PI) {
    //        e_theta = e_theta + M_PI * 2;
    //    }

    lat_con_err.lateral_error=e_y;
    lat_con_err.heading_error =e_theta;
    lat_con_err.lateral_error_rate =  linear_v*std::sin(e_theta);
    lat_con_err.heading_error_rate =  angular_v - trajectory_points.v*trajectory_points.kappa;
}

void MpcControl::ComputeLongitudinalErrors(const std::vector<Trajponits> &trajectory_,
    const VehicleState &vehicleState) {
    /**
  使用匹配点，来计算纵向误差
  */
    //    match_point_ = Tracker::QueryNearestPointByPosition(trajectory_,vehicleState.x, vehicleState.y);

    //    auto trajectory_points = match_point_;
    double dx = match_point_.x-vehicleState.x;
    double dy = match_point_.y-vehicleState.y;

    const double cos_theta = std::cos(match_point_.heading);
    const double sin_theta = std::sin(match_point_.heading);
    // 向量投用法
    lon_con_err.station_error = dx * cos_theta + dy * sin_theta;

    double e_theta = match_point_.heading-vehicleState.yaw;
    e_theta = normalizeAngle(e_theta);
    lon_con_err.speed_error = vehicleState.v * std::cos(e_theta) -  match_point_.v;

    acceleration_reference = match_point_.a;
}
