#include "mani_controller/pid_controller.hpp"
#include <cmath>

PidController::PidController()
    : initialized_(false)
{
    const int n_joints = 5;  // 관절 개수
    
    // Unit to current conversion factors (A per unit)
    // ID 1: XM430-W350 -> 2.69 mA/unit = 2.69e-3 A/unit
    // ID 2, 3, 4, 5 -> 1.0 mA/unit = 1.0e-3 A/unit
    unit_to_current_.resize(n_joints);
    unit_to_current_ << 2.69e-3,  // ID 1
                        1.0e-3,    // ID 2`
                        1.0e-3,    // ID 3
                        1.0e-3,    // ID 4
                        1.0e-3;    // ID 5
    
    // PID 게인 초기화 (기본값, 필요시 조정)
    Kp_.resize(n_joints);
    Kd_.resize(n_joints);
    Ki_.resize(n_joints);
    Kp_.setConstant(1.0);  // 기본값
    Kd_.setConstant(0.1);   // 기본값
    Ki_.setConstant(0.01); // 기본값
    
    // PID 상태 변수 초기화
    theta_integral_.resize(n_joints);
    prev_error_.resize(n_joints);
    theta_integral_.setZero();
    prev_error_.setZero();
}

Eigen::VectorXd PidController::update(const Eigen::VectorXd &desired_theta,
                                      const DynamixelSdkInterface::State &state,
                                      double dt)
{
    const std::size_t n = state.position.size();
    if (n == 0 || desired_theta.size() != static_cast<Eigen::Index>(n)) {
        return Eigen::VectorXd::Zero(n);
    }
    
    // 현재 관절 각도를 radian으로 변환
    Eigen::VectorXd current_theta(static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        current_theta(static_cast<int>(i)) = 
            static_cast<double>(state.position[i]) * DynamixelSdkInterface::UNIT2RAD;
    }
    
    // 현재 속도를 rad/s로 변환
    Eigen::VectorXd current_theta_dot(static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        current_theta_dot(static_cast<int>(i)) = 
            static_cast<double>(state.velocity[i]) * DynamixelSdkInterface::UNIT2RADPERSEC;
    }
    
    // 에러 계산
    Eigen::VectorXd error = desired_theta - current_theta;
    
    // 적분값 업데이트
    if (!initialized_) {
        // 첫 호출시 초기화
        prev_error_ = error;
        theta_integral_.setZero();
        initialized_ = true;
    }
    
    // 적분 누적 (windup 방지를 위한 클리핑은 필요시 추가)
    theta_integral_ += error * dt;
    
    // 미분 계산 (에러의 미분 = desired_dot - current_dot)
    // desired_dot가 없으므로 에러의 변화율로 근사
    Eigen::VectorXd error_dot = (error - prev_error_) / dt;
    if (!std::isfinite(dt) || dt <= 0.0) {
        error_dot.setZero();
    }
    
    // PID 제어 출력 계산
    // output = Kp * error + Kd * error_dot + Ki * integral
    Eigen::VectorXd output(static_cast<int>(n));
    for (int i = 0; i < static_cast<int>(n); ++i) {
        output(i) = Kp_(i) * error(i) + 
                    Kd_(i) * error_dot(i) + 
                    Ki_(i) * theta_integral_(i);
    }
    
    // 이전 에러 저장
    prev_error_ = error;
    
    // 출력 = desired_theta + PID 보정값
    return desired_theta + output;
}

void PidController::reset()
{
    theta_integral_.setZero();
    prev_error_.setZero();
    initialized_ = false;
}

