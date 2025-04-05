#include "can_interface_utils.hpp"
#include <cstdint>
#include "can_interface_ros.hpp"

std::vector<double> convert_angles_to_radians(const uint8_t* can_data) {
    std::vector<double> encoder_angles;
    encoder_angles.reserve(NUM_ANGLES);
    for (std::size_t i = 0; i < NUM_ANGLES; ++i) {
        std::array<std::uint8_t, 2> pair = {can_data[2 * i],
                                            can_data[2 * i + 1]};
        std::uint16_t raw_angle = can_to_encoder_angles(pair);
        encoder_angles.push_back(raw_angle_to_radians(raw_angle));
    }
    return encoder_angles;
}
