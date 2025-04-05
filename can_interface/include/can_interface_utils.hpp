#ifndef CAN_INTERFACE_UTILS_HPP
#define CAN_INTERFACE_UTILS_HPP


#include <cstdint>



  
static inline std::uint16_t joy_to_pwm(std::uint16_t pwm_idle, std::uint16_t pwm_gain, const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle + pwm_gain * joy_value);
}



#endif // !CAN_INTERFACE_UTILS_HPP
