#pragma once

#include <vector>
#include <string>
#include <memory>
#include <numbers>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Dynamixel SDK를 직접 사용해서
// - 포트/보드레이트 설정
// - Torque ON/OFF
// - Operating mode / Profile 설정
// - GroupBulkRead로 pos/vel/load 읽기
// - GroupBulkWrite로 goal position 쓰기
// 를 담당하는 유틸 클래스
class DynamixelSdkInterface
{
public:
  static constexpr double PI = std::numbers::pi_v<double>;
  static constexpr double UNIT2RAD = 2.0 * PI / 4096.0;                 // UNIT: 4096 = 1REV
  static constexpr double UNIT2DEG = 360.0 / 4096.0;
  static constexpr double DEG2UNIT = 4096.0 / 360.0;
  static constexpr double UNIT2RADPERSEC = (0.229 * 2.0 * PI) / 60.0;     // UNIT: 1 = 0.229 REV/MIN
  static constexpr double RADPERSEC2UNIT = 60.0 / (0.229 * 2.0 * PI);
  static constexpr double UNIT2PERCENT = 0.1;

  struct States
  {
    std::vector<int32_t> goal_positions;  // PRESENT_POSITION (32bit signed)
    std::vector<int32_t> positions;  // PRESENT_POSITION (32bit signed)
    std::vector<int32_t> velocities;  // PRESENT_VELOCITY (32bit signed)
    std::vector<int16_t> loads;   // PRESENT_LOAD  (16bit signed)
  };

  //생성자
  DynamixelSdkInterface
  (
    const std::string &port,
    int baudrate,
    double protocol_version,
    const std::vector<uint8_t> &dxl_ids,
    int operating_mode,
    const std::string &profile,
    double profile_velocity,
    double profile_acceleration
  );
  //소멸자
  ~DynamixelSdkInterface();
  
  //함수
  bool readOnce(States &out_states);
  bool writeGoalPositions(const std::vector<int32_t> &goals_unit);
  bool writeGoalPositionsDeg(const std::vector<double> &goals_deg);

private:
  void initPort();
  void initMotors();    // torque off → mode/profile 설정 → torque on
  void initBulkRead();  // PRESENT_* 3개를 GroupBulkRead에 등록

  //  config의 파라미터들 =====
  std::string port_;
  int         baudrate_;
  double      protocol_version_;
  std::vector<uint8_t> dxl_ids_all_;
  int         operating_mode_;
  std::string profile_;
  double      profile_velocity_;
  double      profile_acceleration_;

  // control table
  uint16_t addr_torque_enable_;
  uint16_t addr_drive_mode_;
  uint16_t addr_operating_mode_;
  uint16_t addr_profile_acc_;
  uint16_t addr_profile_vel_;
  uint16_t addr_goal_position_;
  uint16_t addr_present_load_; //present_load
  uint16_t addr_present_velocity_;
  uint16_t addr_present_position_;

  uint16_t len_torque_enable_;
  uint16_t len_drive_mode_;
  uint16_t len_operating_mode_;
  uint16_t len_profile_acc_;
  uint16_t len_profile_vel_;
  uint16_t len_goal_position_;
  uint16_t len_present_load_; //present_load
  uint16_t len_present_velocity_;
  uint16_t len_present_position_;
  uint16_t len_present_all_;

  uint8_t torque_enable_value_;
  uint8_t torque_disable_value_;

  // SDK objects
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
  std::unique_ptr<dynamixel::GroupBulkRead>  group_bulk_read_;
  std::unique_ptr<dynamixel::GroupBulkWrite> group_bulk_write_;

  rclcpp::Logger logger_;
};
