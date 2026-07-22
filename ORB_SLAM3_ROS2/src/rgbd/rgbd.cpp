#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc != 3)
    {
        std::cerr << std::endl
                  << "Usage: ros2 run orbslam3 rgbd path_to_vocab path_to_settings"
                  << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM3::System SLAM(
        argv[1],
        argv[2],
        ORB_SLAM3::System::RGBD,
        false); //GUI on or off

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
