#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace UNITREE_LEGGED_SDK;
UDP *low_udp;
UDP *high_udp;
LowCmd low_cmd = {0};
LowState low_state = {0};
HighCmd high_cmd = {0};
HighState high_state = {0};

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;
rclcpp::TimerBase::SharedPtr timer;
long high_count = 0;
long low_count = 0;

void timerCallback()
{
    high_udp->SetSend(high_cmd);
    high_udp->Send();
    ros2_unitree_legged_msgs::msg::HighState high_state_ros;
    high_udp->Recv();
    high_udp->GetRecv(high_state);
    high_state_ros = state2rosMsg(high_state);
    pub_high->publish(high_state_ros);
}

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    high_cmd = rosMsg2Cmd(msg);

    high_udp->SetSend(high_cmd);
    high_udp->Send();

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    high_udp->Recv();
    high_udp->GetRecv(high_state);

    high_state_ros = state2rosMsg(high_state);

    pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    low_cmd = rosMsg2Cmd(msg);

    low_udp->SetSend(low_cmd);
    low_udp->Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    low_udp->Recv();
    low_udp->GetRecv(low_state);

    low_state_ros = state2rosMsg(low_state);

    pub_low->publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " [LOWLEVEL/HIGHLEVEL] [TIMER_INTERVAL_MS]" << std::endl;
        return -1;
    }

    auto node = rclcpp::Node::make_shared("node_ros2_udp");

    // Parse timeout value from the command line arguments
    int timeout_ms;
    try {
        timeout_ms = std::stoi(argv[2]);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid timeout value: " << argv[2] << std::endl;
        return -1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Timeout value out of range: " << argv[2] << std::endl;
        return -1;
    }

    timer = node->create_wall_timer(std::chrono::milliseconds(timeout_ms), timerCallback);

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        printf("low level runing!\n");
        low_udp = new UDP(LOWLEVEL, 8090, "192.168.123.10", 8007);
        low_udp->InitCmdData(low_cmd);
        printf("creating topics!\n");
        // pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        // sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);

        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level runing!\n");
        high_udp = new UDP(HIGHLEVEL, 8090, "192.168.123.220", 8082);
        high_udp->InitCmdData(high_cmd);
        pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
        sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}