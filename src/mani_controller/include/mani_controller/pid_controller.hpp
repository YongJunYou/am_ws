#pragma once

#include <Eigen/Dense>
#include <vector>
#include "mani_controller/dynamixel_sdk_interface.hpp"

class PidController
{
public:
    PidController();
    
    // PID 제어 업데이트
    // desired_theta: 원하는 관절 각도 (radian)
    // state: 현재 Dynamixel 상태
    // dt: 시간 간격 (seconds)
    // 반환: PID 제어 출력 (goal position in radian)
    Eigen::VectorXd update(const Eigen::VectorXd &desired_theta,
                          const DynamixelSdkInterface::State &state,
                          double dt);
    
    // 적분값 리셋
    void reset();

private:
    // Unit to current conversion factors (A per unit)
    Eigen::VectorXd unit_to_current_;
    
    // PID 게인
    Eigen::VectorXd Kp_;  // Proportional gain
    Eigen::VectorXd Kd_;  // Derivative gain
    Eigen::VectorXd Ki_;  // Integral gain
    
    // PID 상태 변수
    Eigen::VectorXd theta_integral_;  // 적분값
    Eigen::VectorXd prev_error_;       // 이전 에러 (미분 계산용)
    bool initialized_;                 // 초기화 여부
};

