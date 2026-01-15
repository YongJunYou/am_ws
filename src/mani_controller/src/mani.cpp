#include "mani_controller/mani.hpp"
#include <sstream>
#include <cmath>

RobotArm::RobotArm()
: Node("robot_arm")
{
    // config에서 받아올 파라미터 선언
    this->declare_parameter<std::string>("port");
    this->declare_parameter<int>("baudrate");
    this->declare_parameter<double>("protocol_version");
    this->declare_parameter<std::vector<int64_t>>("dxl_ids");
    this->declare_parameter<int>("control_mode");

    // config에서 받아와서 파라미터 파라미터 저장
    std::string port                    = this->get_parameter("port").as_string();
    std::vector<int64_t> dxl_ids_param  = this->get_parameter("dxl_ids").as_integer_array();
    double protocol_version             = this->get_parameter("protocol_version").as_double();
    int baudrate                        = this->get_parameter("baudrate").as_int();
    int control_mode                    = this->get_parameter("control_mode").as_int();
    
    // 관절 개수 가져오기
    const size_t n_joints = dxl_ids_param.size();

    // int64_t를 uint8_t로 변환
    std::vector<uint8_t> dxl_ids;
    dxl_ids.reserve(dxl_ids_param.size());
    for (auto id : dxl_ids_param) {
        if (id < 0 || id > 255) {
            RCLCPP_WARN(this->get_logger(), "Invalid Dynamixel ID: %ld, skipping", id);
            continue;
        }
        dxl_ids.push_back(static_cast<uint8_t>(id));
    }

    // Desired theta 초기화 (관절 개수에 맞춤)
    desired_theta_ = Eigen::VectorXd::Zero(static_cast<int>(n_joints));

    // DynamixelSdkInterface / pid 제어기를 파라미터 값으로 초기화
    dxl_interface_ = std::make_unique<DynamixelSdkInterface>(port, baudrate, protocol_version, dxl_ids, control_mode);
    gravitational_moment_calculator_ = std::make_unique<GravitationalMomentCalculator>();

    // ROS2 Publisher 초기화
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // ROS2 Subscriber 초기화
    desired_theta_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "desired_theta", 10,
        std::bind(&RobotArm::desiredThetaCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotArm created.");
    RCLCPP_INFO(this->get_logger(), "port: %s @ baud: %d", port.c_str(), baudrate);
    RCLCPP_INFO(this->get_logger(), "Publishing joint states to topic: joint_states");
    RCLCPP_INFO(this->get_logger(), "Subscribing to desired theta from topic: desired_theta");
}

void RobotArm::run()
{
    RCLCPP_INFO(this->get_logger(), "Starting control loop...");

    last_time_ = this->now();
    const double control_period = 0.005;  // 5ms
    rclcpp::Rate rate(1.0 / control_period);  // 200 Hz

    DynamixelSdkInterface::State dxl_state;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (!std::isfinite(dt) || dt <= 0.0) {
            dt = control_period;
        }

        // 1. state 읽기
        if (!dxl_interface_->readOnce(dxl_state))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read Dynamixel state");
            continue;
        }
            // ROS2 topic으로 발행
        publishJointState(dxl_state);
        
        
        // 3. PID 제어기 업데이트
        // Eigen::VectorXd desired_theta_local;
        // {
        //     std::lock_guard<std::mutex> lock(desired_theta_mutex_);
        //     desired_theta_local = desired_theta_;
        // }
        
        // // desired_theta와 state 크기 확인
        // if (desired_theta_local.size() > 0 && 
        //     desired_theta_local.size() == static_cast<Eigen::Index>(dxl_state.position.size())) {
            
        //     // PID 제어기로 goal position 계산
        //     const Eigen::VectorXd q_d = pid_controller_.update(desired_theta_local, dxl_state, dt);
            
        //     // 4. Dynamixel에 쓰기 (radian을 unit으로 변환)
        //     if (q_d.size() == static_cast<Eigen::Index>(dxl_state.position.size())) {
        //         std::vector<int32_t> q_d_unit(q_d.size());
        //         for (int i = 0; i < q_d.size(); ++i) {
        //             // RAD2UNIT = 4096.0 / (PI * 2.0)
        //             // std::round()를 사용하여 반올림
        //             q_d_unit[i] = static_cast<int32_t>(std::round(q_d(i) * DynamixelSdkInterface::RAD2UNIT));
        //         }
                
        //         if (!dxl_->writeGoalPositions(q_d_unit)) {
        //             RCLCPP_WARN_THROTTLE(
        //                 this->get_logger(),
        //                 *this->get_clock(),
        //                 1.0,  // 1초마다 출력
        //                 "Failed to write goal positions to Dynamixel");
        //         }
        //     }
        // }
        
        rate.sleep();  // 여기서 다음 주기까지 블록
    }
}

void RobotArm::publishJointState(const DynamixelSdkInterface::State &state)
{
    if (!joint_state_pub_) return;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    
    const std::size_t n = state.position.size();
    if (n == 0) return;

    // Joint names
    msg.name.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.name[i] = "joint_" + std::to_string(i + 1);
    }

    // Position (radian)
    msg.position.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.position[i] = static_cast<double>(state.position[i]) * 
                          DynamixelSdkInterface::UNIT2RAD;
    }

    // Velocity (rad/s)
    msg.velocity.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        msg.velocity[i] = static_cast<double>(state.velocity[i]) * 
                          DynamixelSdkInterface::UNIT2RADPERSEC;
    }

    msg.effort.resize(n);
    for (std::size_t i = 0; i < n && i < state.current.size(); ++i) {
        // Current를 토크로 변환 (간단한 근사값, 실제로는 모터 스펙에 따라 다름)
        const double current_amp = static_cast<double>(state.current[i]) * 0.00269; // 예시 값
        msg.effort[i] = current_amp * 0.1; // 예시 변환 계수 (실제 값으로 교체 필요)
    }

    joint_state_pub_->publish(msg);
}

void RobotArm::desiredThetaCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty desired theta message");
        return;
    }
    desired_theta_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        desired_theta_(i) = msg->data[i];
    }

    RCLCPP_DEBUG(this->get_logger(), "Received desired theta with %zu joints", msg->data.size());
}

