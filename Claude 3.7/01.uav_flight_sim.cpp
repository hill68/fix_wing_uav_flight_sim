#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <memory>

/**
 * @brief 无人机状态结构体
 */
struct UAVState {
    // 位置 [x, y, z] (m)
    std::array<double, 3> position = {0.0, 0.0, 0.0};
    
    // 机体坐标系下速度 [u, v, w] (m/s)
    std::array<double, 3> velocity = {0.0, 0.0, 0.0};
    
    // 姿态角 [roll, pitch, yaw] (rad)
    std::array<double, 3> attitude = {0.0, 0.0, 0.0};
    
    // 角速度 [p, q, r] (rad/s)
    std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0};
    
    // 辅助变量
    double airspeed = 0.0;      // 空速 (m/s)
    double altitude = 0.0;      // 高度 (m)
    double heading = 0.0;       // 航向 (rad)
};

/**
 * @brief 控制输入结构体
 */
struct ControlInputs {
    double throttle = 0.0;      // 油门 [0, 1]
    double aileron = 0.0;       // 副翼 (rad)
    double elevator = 0.0;      // 升降舵 (rad)
    double rudder = 0.0;        // 方向舵 (rad)
};

/**
 * @brief 目标指令结构体
 */
struct Commands {
    double desired_heading = 0.0;   // 期望航向 (rad)
    double desired_altitude = 0.0;  // 期望高度 (m)
    double desired_airspeed = 0.0;  // 期望空速 (m/s)
};

/**
 * @brief 气动参数结构体
 */
struct AeroParameters {
    // 升力系数
    double CL0 = 0.2;           // 零攻角升力系数
    double CL_alpha = 5.7;      // 升力曲线斜率 (1/rad)
    double CL_delta_e = 0.8;    // 升力-升降舵偏导 (1/rad)
    
    // 阻力系数
    double CD0 = 0.02;          // 零升阻力系数
    double k = 0.066;           // 诱导阻力因子
    
    // 力矩系数
    double Cm0 = 0.05;          // 零攻角俯仰力矩系数
    double Cm_alpha = -0.38;    // 俯仰力矩斜率 (1/rad)
    double Cm_delta_e = -1.1;   // 俯仰力矩-升降舵偏导 (1/rad)
    
    // 侧向系数
    double Cy_beta = -0.3;      // 侧力-侧滑角偏导 (1/rad)
    double Cl_beta = -0.05;     // 滚转力矩-侧滑角偏导 (1/rad)
    double Cn_beta = 0.08;      // 偏航力矩-侧滑角偏导 (1/rad)
    
    // 控制面效率
    double Cl_delta_a = 0.15;   // 滚转力矩-副翼偏导 (1/rad)
    double Cn_delta_r = -0.12;  // 偏航力矩-方向舵偏导 (1/rad)
    
    // 阻尼系数
    double Cl_p = -0.5;         // 滚转阻尼
    double Cm_q = -12.0;        // 俯仰阻尼
    double Cn_r = -0.15;        // 偏航阻尼
};

/**
 * @brief 无人机物理参数结构体
 */
struct UAVParameters {
    double mass = 2.0;          // 质量 (kg)
    double wing_area = 0.3;     // 机翼面积 (m^2)
    double wing_span = 1.5;     // 翼展 (m)
    double chord = 0.2;         // 平均气动弦长 (m)
    
    // 转动惯量 (kg⋅m^2)
    std::array<double, 3> inertia = {0.1, 0.15, 0.2}; // [Ixx, Iyy, Izz]
    
    // 重力加速度
    double gravity = 9.81;      // (m/s^2)
    
    // 空气密度
    double air_density = 1.225; // (kg/m^3)
    
    AeroParameters aero;
};

/**
 * @brief PID控制器类
 */
class PIDController {
private:
    double kp_, ki_, kd_;
    double integral_;
    double previous_error_;
    double dt_;
    
public:
    PIDController(double kp, double ki, double kd, double dt) 
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0), previous_error_(0.0) {}
    
    /**
     * @brief 计算PID控制输出
     * @param error 误差
     * @return 控制输出
     */
    double compute(double error) {
        integral_ += error * dt_;
        double derivative = (error - previous_error_) / dt_;
        previous_error_ = error;
        
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
    
    /**
     * @brief 重置PID控制器
     */
    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
    }
};

/**
 * @brief 飞行动力学模型类
 */
class FlightDynamics {
private:
    UAVParameters params_;
    
    /**
     * @brief 计算气动力和力矩
     * @param state 当前状态
     * @param inputs 控制输入
     * @return [力向量, 力矩向量]
     */
    std::pair<std::array<double, 3>, std::array<double, 3>> 
    computeAeroForces(const UAVState& state, const ControlInputs& inputs);
    
    /**
     * @brief 计算推力
     * @param throttle 油门设置
     * @return 推力 (N)
     */
    double computeThrust(double throttle);
    
public:
    FlightDynamics(const UAVParameters& params) : params_(params) {}
    
    /**
     * @brief 更新飞行动力学状态（RK4积分）
     * @param state 当前状态
     * @param inputs 控制输入
     * @param dt 时间步长
     * @return 更新后的状态
     */
    UAVState update(const UAVState& state, const ControlInputs& inputs, double dt);
    
    /**
     * @brief 计算状态导数
     * @param state 当前状态
     * @param inputs 控制输入
     * @return 状态导数
     */
    std::array<double, 12> computeDerivatives(const UAVState& state, const ControlInputs& inputs);
};

/**
 * @brief 级联控制器类
 */
class CascadeController {
private:
    // 外环控制器（20Hz）
    std::unique_ptr<PIDController> heading_controller_;
    std::unique_ptr<PIDController> altitude_controller_;
    std::unique_ptr<PIDController> airspeed_controller_;
    
    // 中环控制器（50Hz）
    std::unique_ptr<PIDController> roll_controller_;
    std::unique_ptr<PIDController> pitch_controller_;
    
    // 内环控制器（100Hz）
    std::unique_ptr<PIDController> roll_rate_controller_;
    std::unique_ptr<PIDController> pitch_rate_controller_;
    std::unique_ptr<PIDController> yaw_rate_controller_;
    
    // 中间变量
    double desired_roll_ = 0.0;
    double desired_pitch_ = 0.0;
    double desired_roll_rate_ = 0.0;
    double desired_pitch_rate_ = 0.0;
    double desired_yaw_rate_ = 0.0;
    
public:
    /**
     * @brief 构造函数，初始化所有PID控制器
     */
    CascadeController();
    
    /**
     * @brief 外环控制（20Hz调用）
     * @param commands 目标指令
     * @param state 当前状态
     */
    void outerLoop(const Commands& commands, const UAVState& state);
    
    /**
     * @brief 中环控制（50Hz调用）
     * @param state 当前状态
     */
    void middleLoop(const UAVState& state);
    
    /**
     * @brief 内环控制（100Hz调用）
     * @param state 当前状态
     * @return 控制输入
     */
    ControlInputs innerLoop(const UAVState& state);
    
    /**
     * @brief 重置所有控制器
     */
    void reset();
};

/**
 * @brief 无人机仿真系统主类
 */
class UAVSimulation {
private:
    FlightDynamics dynamics_;
    CascadeController controller_;
    UAVState current_state_;
    double simulation_time_;
    double dt_;
    
    // 控制频率计数器
    int outer_loop_counter_;
    int middle_loop_counter_;
    
public:
    /**
     * @brief 构造函数
     * @param params 无人机参数
     * @param dt 仿真时间步长
     */
    UAVSimulation(const UAVParameters& params, double dt = 0.01) 
        : dynamics_(params), dt_(dt), simulation_time_(0.0),
          outer_loop_counter_(0), middle_loop_counter_(0) {}
    
    /**
     * @brief 设置初始状态
     * @param initial_state 初始状态
     */
    void setInitialState(const UAVState& initial_state);
    
    /**
     * @brief 单步仿真更新
     * @param commands 目标指令
     */
    void step(const Commands& commands);
    
    /**
     * @brief 获取当前状态
     * @return 当前状态
     */
    const UAVState& getCurrentState() const { return current_state_; }
    
    /**
     * @brief 获取仿真时间
     * @return 仿真时间 (s)
     */
    double getSimulationTime() const { return simulation_time_; }
    
    /**
     * @brief 重置仿真
     */
    void reset();
};

// ================ 实现部分 ================

/**
 * @brief 角度归一化到[-π, π]
 */
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 旋转矩阵计算
 */
std::array<std::array<double, 3>, 3> rotationMatrix(double roll, double pitch, double yaw) {
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    
    return {{
        {{cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr}},
        {{sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr}},
        {{-sp, cp*sr, cp*cr}}
    }};
}

// CascadeController实现
CascadeController::CascadeController() {
    // 外环控制器参数（基于文档典型增益）
    heading_controller_ = std::make_unique<PIDController>(1.2, 0.01, 0.0, 0.05);
    altitude_controller_ = std::make_unique<PIDController>(0.8, 0.005, 0.0, 0.05);
    airspeed_controller_ = std::make_unique<PIDController>(0.5, 0.02, 0.0, 0.05);
    
    // 中环控制器参数
    roll_controller_ = std::make_unique<PIDController>(5.5, 0.0, 1.2, 0.02);
    pitch_controller_ = std::make_unique<PIDController>(6.0, 0.2, 1.0, 0.02);
    
    // 内环控制器参数
    roll_rate_controller_ = std::make_unique<PIDController>(8.0, 0.0, 1.5, 0.01);
    pitch_rate_controller_ = std::make_unique<PIDController>(9.0, 0.0, 1.8, 0.01);
    yaw_rate_controller_ = std::make_unique<PIDController>(4.0, 0.0, 0.5, 0.01);
}

void CascadeController::outerLoop(const Commands& commands, const UAVState& state) {
    // 航向控制 -> 滚转角指令
    double heading_error = normalizeAngle(commands.desired_heading - state.heading);
    desired_roll_ = heading_controller_->compute(heading_error);
    desired_roll_ = std::max(-0.5, std::min(0.5, desired_roll_)); // 限制滚转角
    
    // 高度控制 -> 俯仰角指令
    double altitude_error = commands.desired_altitude - state.altitude;
    desired_pitch_ = altitude_controller_->compute(altitude_error);
    desired_pitch_ = std::max(-0.3, std::min(0.3, desired_pitch_)); // 限制俯仰角
    
    // 空速控制通过油门在innerLoop中处理
}

void CascadeController::middleLoop(const UAVState& state) {
    // 滚转角控制 -> 滚转角速度指令
    double roll_error = normalizeAngle(desired_roll_ - state.attitude[0]);
    desired_roll_rate_ = roll_controller_->compute(roll_error);
    
    // 俯仰角控制 -> 俯仰角速度指令
    double pitch_error = normalizeAngle(desired_pitch_ - state.attitude[1]);
    desired_pitch_rate_ = pitch_controller_->compute(pitch_error);
    
    // 偏航速率通常设为0（协调转弯）
    desired_yaw_rate_ = 0.0;
}

ControlInputs CascadeController::innerLoop(const UAVState& state) {
    ControlInputs inputs;
    
    // 角速度控制
    double roll_rate_error = desired_roll_rate_ - state.angular_velocity[0];
    inputs.aileron = roll_rate_controller_->compute(roll_rate_error);
    
    double pitch_rate_error = desired_pitch_rate_ - state.angular_velocity[1];
    inputs.elevator = pitch_rate_controller_->compute(pitch_rate_error);
    
    double yaw_rate_error = desired_yaw_rate_ - state.angular_velocity[2];
    inputs.rudder = yaw_rate_controller_->compute(yaw_rate_error);
    
    // 空速控制（简化处理）
    inputs.throttle = 0.6; // 可以添加空速PID控制
    
    // 控制量限制
    inputs.aileron = std::max(-0.5, std::min(0.5, inputs.aileron));
    inputs.elevator = std::max(-0.5, std::min(0.5, inputs.elevator));
    inputs.rudder = std::max(-0.5, std::min(0.5, inputs.rudder));
    inputs.throttle = std::max(0.0, std::min(1.0, inputs.throttle));
    
    return inputs;
}

void CascadeController::reset() {
    heading_controller_->reset();
    altitude_controller_->reset();
    airspeed_controller_->reset();
    roll_controller_->reset();
    pitch_controller_->reset();
    roll_rate_controller_->reset();
    pitch_rate_controller_->reset();
    yaw_rate_controller_->reset();
}

// FlightDynamics实现
std::pair<std::array<double, 3>, std::array<double, 3>> 
FlightDynamics::computeAeroForces(const UAVState& state, const ControlInputs& inputs) {
    const auto& aero = params_.aero;
    
    // 计算攻角和侧滑角
    double V = sqrt(state.velocity[0]*state.velocity[0] + 
                   state.velocity[1]*state.velocity[1] + 
                   state.velocity[2]*state.velocity[2]);
    
    if (V < 1e-6) V = 1e-6; // 避免除零
    
    double alpha = atan2(state.velocity[2], state.velocity[0]); // 攻角
    double beta = asin(state.velocity[1] / V);                 // 侧滑角
    
    // 动压
    double dynamic_pressure = 0.5 * params_.air_density * V * V;
    
    // 升力系数
    double CL = aero.CL0 + aero.CL_alpha * alpha + aero.CL_delta_e * inputs.elevator;
    
    // 阻力系数
    double CD = aero.CD0 + aero.k * CL * CL;
    
    // 侧力系数
    double CY = aero.Cy_beta * beta;
    
    // 气动力（风轴）
    double lift = dynamic_pressure * params_.wing_area * CL;
    double drag = dynamic_pressure * params_.wing_area * CD;
    double side_force = dynamic_pressure * params_.wing_area * CY;
    
    // 转换到机体轴
    std::array<double, 3> forces = {
        -drag * cos(alpha) + lift * sin(alpha),  // X
        side_force,                              // Y
        -drag * sin(alpha) - lift * cos(alpha)   // Z
    };
    
    // 力矩系数
    double Cl = aero.Cl_beta * beta + aero.Cl_delta_a * inputs.aileron + 
                aero.Cl_p * state.angular_velocity[0] * params_.chord / (2.0 * V);
    
    double Cm = aero.Cm0 + aero.Cm_alpha * alpha + aero.Cm_delta_e * inputs.elevator +
                aero.Cm_q * state.angular_velocity[1] * params_.chord / (2.0 * V);
    
    double Cn = aero.Cn_beta * beta + aero.Cn_delta_r * inputs.rudder +
                aero.Cn_r * state.angular_velocity[2] * params_.wing_span / (2.0 * V);
    
    // 气动力矩
    std::array<double, 3> moments = {
        dynamic_pressure * params_.wing_area * params_.wing_span * Cl,  // L
        dynamic_pressure * params_.wing_area * params_.chord * Cm,      // M
        dynamic_pressure * params_.wing_area * params_.wing_span * Cn   // N
    };
    
    return {forces, moments};
}

double FlightDynamics::computeThrust(double throttle) {
    // 简化推力模型：推力与油门成正比
    return throttle * 15.0; // 最大推力15N
}

std::array<double, 12> FlightDynamics::computeDerivatives(const UAVState& state, const ControlInputs& inputs) {
    std::array<double, 12> derivatives = {0};
    
    // 获取气动力和力矩
    auto [aero_forces, aero_moments] = computeAeroForces(state, inputs);
    
    // 推力
    double thrust = computeThrust(inputs.throttle);
    
    // 总力（机体坐标系）
    std::array<double, 3> total_forces = {
        aero_forces[0] + thrust,
        aero_forces[1],
        aero_forces[2]
    };
    
    // 重力在机体坐标系下的投影
    double roll = state.attitude[0];
    double pitch = state.attitude[1];
    
    total_forces[0] -= params_.mass * params_.gravity * sin(pitch);
    total_forces[1] += params_.mass * params_.gravity * cos(pitch) * sin(roll);
    total_forces[2] += params_.mass * params_.gravity * cos(pitch) * cos(roll);
    
    // 位置导数（需要转换到地球坐标系）
    auto R = rotationMatrix(roll, pitch, state.attitude[2]);
    for (int i = 0; i < 3; ++i) {
        derivatives[i] = 0;
        for (int j = 0; j < 3; ++j) {
            derivatives[i] += R[i][j] * state.velocity[j];
        }
    }
    
    // 速度导数（机体坐标系）
    const auto& omega = state.angular_velocity;
    derivatives[3] = total_forces[0] / params_.mass + omega[2] * state.velocity[1] - omega[1] * state.velocity[2];
    derivatives[4] = total_forces[1] / params_.mass + omega[0] * state.velocity[2] - omega[2] * state.velocity[0];
    derivatives[5] = total_forces[2] / params_.mass + omega[1] * state.velocity[0] - omega[0] * state.velocity[1];
    
    // 姿态导数
    derivatives[6] = omega[0] + (omega[1] * sin(roll) + omega[2] * cos(roll)) * tan(pitch);
    derivatives[7] = omega[1] * cos(roll) - omega[2] * sin(roll);
    derivatives[8] = (omega[1] * sin(roll) + omega[2] * cos(roll)) / cos(pitch);
    
    // 角速度导数
    const auto& I = params_.inertia;
    derivatives[9] = (aero_moments[0] + (I[1] - I[2]) * omega[1] * omega[2]) / I[0];
    derivatives[10] = (aero_moments[1] + (I[2] - I[0]) * omega[2] * omega[0]) / I[1];
    derivatives[11] = (aero_moments[2] + (I[0] - I[1]) * omega[0] * omega[1]) / I[2];
    
    return derivatives;
}

UAVState FlightDynamics::update(const UAVState& state, const ControlInputs& inputs, double dt) {
    // 4阶Runge-Kutta积分
    auto k1 = computeDerivatives(state, inputs);
    
    UAVState temp_state = state;
    for (int i = 0; i < 12; ++i) {
        if (i < 3) temp_state.position[i] += 0.5 * dt * k1[i];
        else if (i < 6) temp_state.velocity[i-3] += 0.5 * dt * k1[i];
        else if (i < 9) temp_state.attitude[i-6] += 0.5 * dt * k1[i];
        else temp_state.angular_velocity[i-9] += 0.5 * dt * k1[i];
    }
    
    auto k2 = computeDerivatives(temp_state, inputs);
    
    temp_state = state;
    for (int i = 0; i < 12; ++i) {
        if (i < 3) temp_state.position[i] += 0.5 * dt * k2[i];
        else if (i < 6) temp_state.velocity[i-3] += 0.5 * dt * k2[i];
        else if (i < 9) temp_state.attitude[i-6] += 0.5 * dt * k2[i];
        else temp_state.angular_velocity[i-9] += 0.5 * dt * k2[i];
    }
    
    auto k3 = computeDerivatives(temp_state, inputs);
    
    temp_state = state;
    for (int i = 0; i < 12; ++i) {
        if (i < 3) temp_state.position[i] += dt * k3[i];
        else if (i < 6) temp_state.velocity[i-3] += dt * k3[i];
        else if (i < 9) temp_state.attitude[i-6] += dt * k3[i];
        else temp_state.angular_velocity[i-9] += dt * k3[i];
    }
    
    auto k4 = computeDerivatives(temp_state, inputs);
    
    // 更新状态
    UAVState new_state = state;
    for (int i = 0; i < 12; ++i) {
        double increment = dt * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) / 6.0;
        
        if (i < 3) new_state.position[i] += increment;
        else if (i < 6) new_state.velocity[i-3] += increment;
        else if (i < 9) new_state.attitude[i-6] += increment;
        else new_state.angular_velocity[i-9] += increment;
    }
    
    // 更新辅助变量
    new_state.airspeed = sqrt(new_state.velocity[0]*new_state.velocity[0] + 
                             new_state.velocity[1]*new_state.velocity[1] + 
                             new_state.velocity[2]*new_state.velocity[2]);
    new_state.altitude = -new_state.position[2]; // Z轴向下为正
    new_state.heading = new_state.attitude[2];
    
    // 角度归一化
    for (int i = 0; i < 3; ++i) {
        new_state.attitude[i] = normalizeAngle(new_state.attitude[i]);
    }
    new_state.heading = normalizeAngle(new_state.heading);
    
    return new_state;
}

// UAVSimulation实现
void UAVSimulation::setInitialState(const UAVState& initial_state) {
    current_state_ = initial_state;
    simulation_time_ = 0.0;
    outer_loop_counter_ = 0;
    middle_loop_counter_ = 0;
    controller_.reset();
}

void UAVSimulation::step(const Commands& commands) {
    // 控制频率管理
    bool run_outer = (outer_loop_counter_ % 5 == 0);   // 20Hz (0.01*5 = 0.05s)
    bool run_middle = (middle_loop_counter_ % 2 == 0); // 50Hz (0.01*2 = 0.02s)
    
    // 执行控制循环
    if (run_outer) {
        controller_.outerLoop(commands, current_state_);
    }
    
    if (run_middle) {
        controller_.middleLoop(current_state_);
    }
    
    // 内环总是执行（100Hz）
    ControlInputs inputs = controller_.innerLoop(current_state_);
    
    // 更新动力学
    current_state_ = dynamics_.update(current_state_, inputs, dt_);
    
    // 更新时间和计数器
    simulation_time_ += dt_;
    outer_loop_counter_++;
    middle_loop_counter_++;
}

void UAVSimulation::reset() {
    simulation_time_ = 0.0;
    outer_loop_counter_ = 0;
    middle_loop_counter_ = 0;
    controller_.reset();
    current_state_ = UAVState(); // 重置为默认状态
}

/**
 * @brief 仿真数据记录器类
 */
class SimulationLogger {
private:
    struct LogData {
        double time;
        UAVState state;
        ControlInputs inputs;
        Commands commands;
    };
    
    std::vector<LogData> log_data_;
    
public:
    /**
     * @brief 记录仿真数据
     */
    void log(double time, const UAVState& state, const ControlInputs& inputs, const Commands& commands) {
        log_data_.push_back({time, state, inputs, commands});
    }
    
    /**
     * @brief 导出数据到CSV文件
     * @param filename 文件名
     */
    void exportToCSV(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        // 写入表头
        file << "时间,X位置,Y位置,Z位置,高度,U速度,V速度,W速度,空速,";
        file << "滚转角,俯仰角,偏航角,航向,滚转率,俯仰率,偏航率,";
        file << "油门,副翼,升降舵,方向舵,目标高度,目标航向,目标空速\n";
        
        // 写入数据
        for (const auto& data : log_data_) {
            const auto& s = data.state;
            const auto& i = data.inputs;
            const auto& c = data.commands;
            
            file << data.time << ","
                 << s.position[0] << "," << s.position[1] << "," << s.position[2] << "," << s.altitude << ","
                 << s.velocity[0] << "," << s.velocity[1] << "," << s.velocity[2] << "," << s.airspeed << ","
                 << s.attitude[0] << "," << s.attitude[1] << "," << s.attitude[2] << "," << s.heading << ","
                 << s.angular_velocity[0] << "," << s.angular_velocity[1] << "," << s.angular_velocity[2] << ","
                 << i.throttle << "," << i.aileron << "," << i.elevator << "," << i.rudder << ","
                 << c.desired_altitude << "," << c.desired_heading << "," << c.desired_airspeed << "\n";
        }
        
        file.close();
    }
    
    /**
     * @brief 清空记录
     */
    void clear() {
        log_data_.clear();
    }
    
    /**
     * @brief 获取记录数量
     */
    size_t size() const {
        return log_data_.size();
    }
};

/**
 * @brief 配置管理器类
 */
class ConfigManager {
public:
    /**
     * @brief 从文件加载UAV参数
     * @param filename 配置文件名
     * @return UAV参数
     */
    static UAVParameters loadUAVParameters(const std::string& filename) {
        UAVParameters params;
        
        // 这里可以实现JSON/YAML文件解析
        // 为简化示例，使用默认参数
        
        return params;
    }
    
    /**
     * @brief 保存UAV参数到文件
     * @param params UAV参数
     * @param filename 配置文件名
     */
    static void saveUAVParameters(const UAVParameters& params, const std::string& filename) {
        // 实现参数保存逻辑
    }
    
    /**
     * @brief 创建典型小型固定翼无人机参数
     * @return 典型参数配置
     */
    static UAVParameters createTypicalUAV() {
        UAVParameters params;
        
        // 基本物理参数
        params.mass = 2.0;              // 2kg小型无人机
        params.wing_area = 0.3;         // 0.3m² 机翼面积
        params.wing_span = 1.5;         // 1.5m 翼展
        params.chord = 0.2;             // 0.2m 平均气动弦长
        
        // 转动惯量（典型值）
        params.inertia = {0.1, 0.15, 0.2};
        
        // 环境参数
        params.gravity = 9.81;
        params.air_density = 1.225;
        
        // 气动参数（基于文档典型值）
        params.aero.CL0 = 0.2;
        params.aero.CL_alpha = 5.7;
        params.aero.CL_delta_e = 0.8;
        params.aero.CD0 = 0.02;
        params.aero.k = 0.066;
        params.aero.Cm0 = 0.05;
        params.aero.Cm_alpha = -0.38;
        params.aero.Cm_delta_e = -1.1;
        
        // 侧向气动参数
        params.aero.Cy_beta = -0.3;
        params.aero.Cl_beta = -0.05;
        params.aero.Cn_beta = 0.08;
        params.aero.Cl_delta_a = 0.15;
        params.aero.Cn_delta_r = -0.12;
        
        // 阻尼系数
        params.aero.Cl_p = -0.5;
        params.aero.Cm_q = -12.0;
        params.aero.Cn_r = -0.15;
        
        return params;
    }
};

/**
 * @brief 航路点类
 */
struct Waypoint {
    double x, y, z;                 // 位置 (m)
    double desired_airspeed;        // 期望空速 (m/s)
    double tolerance;               // 到达容差 (m)
    
    Waypoint(double x_, double y_, double z_, double speed = 15.0, double tol = 10.0)
        : x(x_), y(y_), z(z_), desired_airspeed(speed), tolerance(tol) {}
};

/**
 * @brief 航路点导航器类
 */
class WaypointNavigator {
private:
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    
public:
    WaypointNavigator() : current_waypoint_index_(0) {}
    
    /**
     * @brief 添加航路点
     */
    void addWaypoint(const Waypoint& wp) {
        waypoints_.push_back(wp);
    }
    
    /**
     * @brief 清除所有航路点
     */
    void clearWaypoints() {
        waypoints_.clear();
        current_waypoint_index_ = 0;
    }
    
    /**
     * @brief 获取当前目标指令
     * @param current_state 当前状态
     * @return 目标指令
     */
    Commands getCurrentCommands(const UAVState& current_state) {
        Commands commands;
        
        if (waypoints_.empty() || current_waypoint_index_ >= waypoints_.size()) {
            // 没有航路点，保持当前状态
            commands.desired_altitude = current_state.altitude;
            commands.desired_heading = current_state.heading;
            commands.desired_airspeed = current_state.airspeed;
            return commands;
        }
        
        const auto& target_wp = waypoints_[current_waypoint_index_];
        
        // 计算到目标点的距离和航向
        double dx = target_wp.x - current_state.position[0];
        double dy = target_wp.y - current_state.position[1];
        double distance = sqrt(dx*dx + dy*dy);
        
        // 检查是否到达当前航路点
        double dz = abs(target_wp.z - current_state.altitude);
        if (distance < target_wp.tolerance && dz < target_wp.tolerance) {
            current_waypoint_index_++;
            if (current_waypoint_index_ < waypoints_.size()) {
                return getCurrentCommands(current_state); // 递归获取下一个航路点指令
            }
        }
        
        // 设置目标指令
        commands.desired_heading = atan2(dy, dx);
        commands.desired_altitude = target_wp.z;
        commands.desired_airspeed = target_wp.desired_airspeed;
        
        return commands;
    }
    
    /**
     * @brief 获取当前航路点索引
     */
    size_t getCurrentWaypointIndex() const {
        return current_waypoint_index_;
    }
    
    /**
     * @brief 检查是否完成所有航路点
     */
    bool isCompleted() const {
        return current_waypoint_index_ >= waypoints_.size();
    }
    
    /**
     * @brief 重置导航器
     */
    void reset() {
        current_waypoint_index_ = 0;
    }
};

/**
 * @brief 风场模型类
 */
class WindModel {
private:
    std::array<double, 3> constant_wind_;  // 恒定风速 [北, 东, 下] (m/s)
    std::array<double, 3> turbulence_;     // 湍流强度 (m/s)
    double time_;
    
public:
    WindModel() : constant_wind_({0, 0, 0}), turbulence_({0, 0, 0}), time_(0) {}
    
    /**
     * @brief 设置恒定风速
     * @param north 北向风速 (m/s)
     * @param east 东向风速 (m/s) 
     * @param down 下向风速 (m/s)
     */
    void setConstantWind(double north, double east, double down = 0) {
        constant_wind_ = {north, east, down};
    }
    
    /**
     * @brief 设置湍流强度
     * @param turbulence 湍流强度 (m/s)
     */
    void setTurbulence(double turbulence) {
        turbulence_ = {turbulence, turbulence, turbulence * 0.5};
    }
    
    /**
     * @brief 获取当前风速（地球坐标系）
     * @param dt 时间步长
     * @return 风速向量 [北, 东, 下] (m/s)
     */
    std::array<double, 3> getWindVelocity(double dt) {
        time_ += dt;
        
        std::array<double, 3> wind = constant_wind_;
        
        // 添加简单的正弦湍流
        if (turbulence_[0] > 0) {
            wind[0] += turbulence_[0] * sin(2.0 * time_) * cos(3.1 * time_);
            wind[1] += turbulence_[1] * cos(1.7 * time_) * sin(2.3 * time_);
            wind[2] += turbulence_[2] * sin(1.1 * time_) * cos(1.9 * time_);
        }
        
        return wind;
    }
};

/**
 * @brief 增强的无人机仿真系统类
 */
class AdvancedUAVSimulation : public UAVSimulation {
private:
    SimulationLogger logger_;
    WindModel wind_model_;
    WaypointNavigator navigator_;
    ControlInputs last_inputs_;
    bool logging_enabled_;
    
public:
    AdvancedUAVSimulation(const UAVParameters& params, double dt = 0.01)
        : UAVSimulation(params, dt), logging_enabled_(false) {}
    
    /**
     * @brief 启用/禁用数据记录
     */
    void enableLogging(bool enable = true) {
        logging_enabled_ = enable;
        if (!enable) logger_.clear();
    }
    
    /**
     * @brief 设置风场模型
     */
    void setWindModel(const WindModel& wind) {
        wind_model_ = wind;
    }
    
    /**
     * @brief 设置航路点导航
     */
    void setWaypoints(const std::vector<Waypoint>& waypoints) {
        navigator_.clearWaypoints();
        for (const auto& wp : waypoints) {
            navigator_.addWaypoint(wp);
        }
    }
    
    /**
     * @brief 自动航路点导航步进
     */
    void stepWithNavigation() {
        Commands commands = navigator_.getCurrentCommands(getCurrentState());
        step(commands);
        
        // 记录数据
        if (logging_enabled_) {
            logger_.log(getSimulationTime(), getCurrentState(), last_inputs_, commands);
        }
    }
    
    /**
     * @brief 手动指令步进（重载）
     */
    void step(const Commands& commands) override {
        UAVSimulation::step(commands);
        
        // 记录数据
        if (logging_enabled_) {
            logger_.log(getSimulationTime(), getCurrentState(), last_inputs_, commands);
        }
    }
    
    /**
     * @brief 导出仿真数据
     */
    void exportData(const std::string& filename) const {
        logger_.exportToCSV(filename);
    }
    
    /**
     * @brief 获取导航状态
     */
    bool isNavigationCompleted() const {
        return navigator_.isCompleted();
    }
    
    /**
     * @brief 获取当前航路点索引
     */
    size_t getCurrentWaypointIndex() const {
        return navigator_.getCurrentWaypointIndex();
    }
    
    /**
     * @brief 重置仿真（重载）
     */
    void reset() override {
        UAVSimulation::reset();
        navigator_.reset();
        logger_.clear();
    }
};

// 使用示例和测试函数
/**
 * @brief 性能评估工具
 */
class PerformanceAnalyzer {
public:
    /**
     * @brief 分析轨迹跟踪性能
     * @param logger 仿真记录器
     * @return 性能指标
     */
    static void analyzePerformance(const SimulationLogger& logger) {
        if (logger.size() == 0) {
            printf("没有仿真数据可分析\n");
            return;
        }
        
        // 这里可以添加具体的性能分析逻辑
        printf("=== 仿真性能分析 ===\n");
        printf("数据点数量: %zu\n", logger.size());
        printf("注：详细性能分析功能待实现\n");
    }
    
    /**
     * @brief 计算跟踪误差统计
     */
    static void calculateTrackingErrors(const std::vector<Waypoint>& waypoints,
                                       const std::vector<UAVState>& states) {
        // 实现轨迹跟踪误差计算
    }
};

/**
 * @brief 仿真测试套件
 */
class SimulationTestSuite {
public:
    /**
     * @brief 基础悬停测试
     */
    static void testHover() {
        printf("=== 执行悬停测试 ===\n");
        
        auto params = ConfigManager::createTypicalUAV();
        AdvancedUAVSimulation sim(params, 0.01);
        sim.enableLogging(true);
        
        // 设置初始状态
        UAVState initial_state;
        initial_state.position = {0, 0, -100};
        initial_state.velocity = {15, 0, 0};
        initial_state.attitude = {0, 0, 0};
        sim.setInitialState(initial_state);
        
        // 悬停指令
        Commands hover_commands;
        hover_commands.desired_altitude = 100;
        hover_commands.desired_heading = 0;
        hover_commands.desired_airspeed = 15;
        
        // 运行仿真30秒
        for (int i = 0; i < 3000; ++i) {
            sim.step(hover_commands);
            
            if (i % 500 == 0) {
                const auto& state = sim.getCurrentState();
                printf("时间: %.1fs, 高度: %.1fm, 航向: %.2frad, 空速: %.1fm/s\n",
                       sim.getSimulationTime(), state.altitude, state.heading, state.airspeed);
            }
        }
        
        sim.exportData("hover_test.csv");
        printf("悬停测试完成，数据已保存到 hover_test.csv\n\n");
    }
    
    /**
     * @brief 航路点导航测试
     */
    static void testWaypointNavigation() {
        printf("=== 执行航路点导航测试 ===\n");
        
        auto params = ConfigManager::createTypicalUAV();
        AdvancedUAVSimulation sim(params, 0.01);
        sim.enableLogging(true);
        
        // 设置初始状态
        UAVState initial_state;
        initial_state.position = {0, 0, -100};
        initial_state.velocity = {15, 0, 0};
        initial_state.attitude = {0, 0, 0};
        sim.setInitialState(initial_state);
        
        // 设置航路点
        std::vector<Waypoint> waypoints = {
            Waypoint(0, 0, 100, 15),      // 起点
            Waypoint(200, 0, 120, 18),    // 第一个航路点
            Waypoint(200, 200, 100, 15),  // 第二个航路点
            Waypoint(0, 200, 110, 16),    // 第三个航路点
            Waypoint(0, 0, 100, 15)       // 回到起点
        };
        sim.setWaypoints(waypoints);
        
        // 运行仿真直到完成所有航路点
        int max_steps = 15000; // 最大150秒
        for (int i = 0; i < max_steps && !sim.isNavigationCompleted(); ++i) {
            sim.stepWithNavigation();
            
            if (i % 1000 == 0) {
                const auto& state = sim.getCurrentState();
                printf("时间: %.1fs, 位置: (%.1f, %.1f, %.1f), 当前航路点: %zu\n",
                       sim.getSimulationTime(), 
                       state.position[0], state.position[1], state.altitude,
                       sim.getCurrentWaypointIndex());
            }
        }
        
        if (sim.isNavigationCompleted()) {
            printf("航路点导航测试完成！\n");
        } else {
            printf("航路点导航测试超时\n");
        }
        
        sim.exportData("waypoint_navigation_test.csv");
        printf("数据已保存到 waypoint_navigation_test.csv\n\n");
    }
    
    /**
     * @brief 风扰动测试
     */
    static void testWindDisturbance() {
        printf("=== 执行风扰动测试 ===\n");
        
        auto params = ConfigManager::createTypicalUAV();
        AdvancedUAVSimulation sim(params, 0.01);
        sim.enableLogging(true);
        
        // 设置风场
        WindModel wind;
        wind.setConstantWind(5.0, 2.0, 0.0);  // 5m/s北风，2m/s东风
        wind.setTurbulence(1.0);               // 1m/s湍流强度
        sim.setWindModel(wind);
        
        // 设置初始状态
        UAVState initial_state;
        initial_state.position = {0, 0, -100};
        initial_state.velocity = {15, 0, 0};
        initial_state.attitude = {0, 0, 0};
        sim.setInitialState(initial_state);
        
        // 直线飞行指令
        Commands commands;
        commands.desired_altitude = 100;
        commands.desired_heading = 0.5;  // 30度航向
        commands.desired_airspeed = 18;
        
        // 运行仿真60秒
        for (int i = 0; i < 6000; ++i) {
            sim.step(commands);
            
            if (i % 1000 == 0) {
                const auto& state = sim.getCurrentState();
                printf("时间: %.1fs, 位置: (%.1f, %.1f), 航向误差: %.2frad\n",
                       sim.getSimulationTime(),
                       state.position[0], state.position[1],
                       commands.desired_heading - state.heading);
            }
        }
        
        sim.exportData("wind_disturbance_test.csv");
        printf("风扰动测试完成，数据已保存到 wind_disturbance_test.csv\n\n");
    }
    
    /**
     * @brief 运行所有测试
     */
    static void runAllTests() {
        printf("开始运行无人机仿真测试套件...\n\n");
        
        testHover();
        testWaypointNavigation();
        testWindDisturbance();
        
        printf("所有测试完成！\n");
    }
};

// 使用示例主函数
/*
int main() {
    try {
        // 方式1: 运行标准测试套件
        SimulationTestSuite::runAllTests();
        
        // 方式2: 自定义仿真
        printf("\n=== 自定义仿真示例 ===\n");
        
        // 创建典型无人机配置
        auto params = ConfigManager::createTypicalUAV();
        
        // 创建高级仿真系统
        AdvancedUAVSimulation sim(params, 0.01);
        sim.enableLogging(true);
        
        // 设置初始状态
        UAVState initial_state;
        initial_state.position = {0, 0, -50};     // 50m高度
        initial_state.velocity = {12, 0, 0};      // 12m/s初始速度
        initial_state.attitude = {0, 0.1, 0};     // 轻微仰角
        sim.setInitialState(initial_state);
        
        // 创建复杂航路点任务
        std::vector<Waypoint> mission_waypoints = {
            Waypoint(0, 0, 80, 15),           // 起飞到80m
            Waypoint(300, 0, 100, 20),        // 向东飞行300m
            Waypoint(300, 300, 120, 18),      // 向北飞行300m，爬升到120m
            Waypoint(0, 300, 100, 16),        // 向西返回，下降到100m
            Waypoint(0, 0, 60, 12)            // 返回起点，准备降落
        };
        sim.setWaypoints(mission_waypoints);
        
        // 添加环境干扰
        WindModel mission_wind;
        mission_wind.setConstantWind(3.0, 1.0);   // 轻度风场
        mission_wind.setTurbulence(0.5);           // 轻度湍流
        sim.setWindModel(mission_wind);
        
        printf("开始执行任务...\n");
        
        // 执行任务
        int step_count = 0;
        while (!sim.isNavigationCompleted() && step_count < 20000) {
            sim.stepWithNavigation();
            step_count++;
            
            // 每5秒输出一次状态
            if (step_count % 500 == 0) {
                const auto& state = sim.getCurrentState();
                printf("任务时间: %.1fs | 位置: (%.0f, %.0f, %.0f) | 航路点: %zu/%zu\n",
                       sim.getSimulationTime(),
                       state.position[0], state.position[1], state.altitude,
                       sim.getCurrentWaypointIndex() + 1, mission_waypoints.size());
            }
        }
        
        if (sim.isNavigationCompleted()) {
            printf("任务成功完成！\n");
        } else {
            printf("任务执行超时\n");
        }
        
        // 导出详细数据
        sim.exportData("mission_flight_log.csv");
        printf("详细飞行数据已保存到 mission_flight_log.csv\n");
        
        // 性能分析
        PerformanceAnalyzer::analyzePerformance(sim.getLogger());
        
    } catch (const std::exception& e) {
        printf("仿真出错: %s\n", e.what());
        return -1;
    }
    
    return 0;
}
*/

/**
 * @brief 编译说明
 * 
 * 编译命令：
 * g++ -std=c++17 -O2 -o uav_sim uav_simulation.cpp -lm
 * 
 * 依赖库：
 * - 标准C++17库
 * - 数学库 (-lm)
 * 
 * 可选优化：
 * - 集成Eigen库进行矩阵运算优化
 * - 添加JSON/YAML配置文件支持
 * - 集成可视化库(如OpenGL/Matplotlib)
 * - 添加实时通信接口(ROS/MAVLink)
 */