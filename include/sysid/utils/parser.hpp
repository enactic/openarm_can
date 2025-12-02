#pragma once
#include <string>
#include <vector>

#include "sysid/signal/factory.hpp" // for Type enum

namespace sysid::utils {

struct MotorConfig {
    int send_can_id = 0;
    std::string signal_type = "none";  // e.g., "sine", "trap", "saw", "step", "none"
    double amplitude = 0.0;
    double frequency = 0.0;
    double phase = 0.0;
    double shift = 0.0;  // vertical offset
};

struct Config {
    std::string can_interface;
    std::vector<MotorConfig> motors;
};

// Parse an input file and return global + per-motor configuration
Config parse_input_file(const std::string& filename);

} // namespace sysid::utils
