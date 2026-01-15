#include "mani_controller/mani.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotArm>();
    node->run();

    rclcpp::shutdown();
    return 0;
}
