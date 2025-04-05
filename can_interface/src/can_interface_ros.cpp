#include "can_interface_ros.hpp"
#include <spdlog/spdlog.h>
#include "canfd.h"

CANInterface::CANInterface() : Node("can_interface_node") {
    extract_parameters();
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10,
        std::bind(&CANInterface::joy_callback, this, std::placeholders::_1));
    pwm_pub_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CANInterface::encoder_angles_callback, this));

    last_msg_time_ = this->now();
    spdlog::info("CAN interface node started");
}

void CANInterface::extract_parameters() {
    this->declare_parameter<std::string>("topics.joy");
    this->declare_parameter<std::string>("topics.pwm");
    this->declare_parameter<int>("pwm.gain");
    this->declare_parameter<int>("pwm.idle");
    this->declare_parameter<std::string>("can.interface");

    this->joy_topic_ = this->get_parameter("topics.joy").as_string();
    this->pwm_topic_ = this->get_parameter("topics.pwm").as_string();
    this->pwm_gain_ = this->get_parameter("pwm.gain").as_int();
    this->pwm_idle_ = this->get_parameter("pwm.idle").as_int();
    this->can_interface_ = this->get_parameter("can.interface").as_string();
}


void CANInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    double shoulder_value = msg->axes[1];
    double wrist_value = msg->axes[0];
    double grip_value = msg->axes[3];

    std::uint16_t shoulder_pwm = this->joy_to_pwm(shoulder_value);
    std::uint16_t wrist_pwm = this->joy_to_pwm(wrist_value);
    std::uint16_t grip_pwm = this->joy_to_pwm(grip_value);

    canMsg.data[0] = (shoulder_pwm);
    canMsg.data[1] = (wrist_pwm);
    canMsg.data[2] = (grip_pwm);

    canMsg.id = 0x46C;

    std_msgs::msg::Int16MultiArray pwm_msg = vec_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    canfd_send(&canMsg);

    if (msg->buttons[0]) {
        // todo
    } else if (msg->buttons[1]) {
        // todo
    }
}

void CANInterface::encoder_angles_callback() {
    std::vector<double> angles = gripper_driver_->encoder_read();
    if (angles.empty()) {
        return;
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"shoulder", "wrist", "grip"};
    joint_state_msg.position = angles;

    joint_state_pub_->publish(joint_state_msg);
}

std_msgs::msg::Int16MultiArray CANInterface::vec_to_msg(
    std::vector<std::uint16_t> vec) {
    std_msgs::msg::Int16MultiArray msg;
    for (std::uint16_t value : vec) {
        msg.data.push_back(value);
    }
    return msg;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANInterface>());
    rclcpp::shutdown();
    return 0;
}
