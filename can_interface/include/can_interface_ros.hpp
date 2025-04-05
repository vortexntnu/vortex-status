#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <ranges>
#include "can_interface_driver.h"

class CANInterface : public rclcpp::Node {
   public:
    CANInterface();

   private:
    /**
     * @brief Extract parameters from the config file.
     */
    void extract_parameters();

    void set_publisher_and_subsribers();

    /**
     * @brief Callback function for the joystick message.
     * @param msg The joystick message.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void encoder_angles_callback();

    std::uint16_t joy_to_pwm(const double joy_value);
    /**
     * @brief Convert a vector of PWM values to a ROS message.
     * @param vec The vector of PWM values.
     * @return The ROS message.
     */
    std_msgs::msg::Int16MultiArray vec_to_msg(std::vector<std::uint16_t> vec);

    std::string joy_topic_;
    std::string pwm_topic_;
    int pwm_gain_;
    int pwm_idle_;
    std::string can_interface_;

    CANFD_Message canMsg;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_msg_time_;
};

#endif  // CAN_INTERFACE_HPP
