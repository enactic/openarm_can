#include "sysid/utils/parser.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

namespace sysid::utils {

Config parse_input_file(const std::string& filename) {
    Config cfg;
    std::ifstream infile(filename);
    if (!infile) throw std::runtime_error("Cannot open input file: " + filename);

    MotorConfig current_motor;
    bool in_motor_section = false;

    std::string line;
    while (std::getline(infile, line)) {
        // remove comments
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos)
            line = line.substr(0, comment_pos);

        // trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.empty()) continue;

        auto eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = line.substr(0, eq_pos);
        std::string value = line.substr(eq_pos + 1);

        // trim whitespace
        key.erase(0, key.find_first_not_of(" \t\r\n"));
        key.erase(key.find_last_not_of(" \t\r\n") + 1);
        value.erase(0, value.find_first_not_of(" \t\r\n"));
        value.erase(value.find_last_not_of(" \t\r\n") + 1);

        if (key == "send_can_id") {
            if (in_motor_section) {
                cfg.motors.push_back(current_motor);
            }
            current_motor = MotorConfig();
            in_motor_section = true;
        }

        if (!in_motor_section) {
            if (key == "can_interface") cfg.can_interface = value;
        } else {
            if (key == "send_can_id") current_motor.send_can_id = std::stoi(value);
            else if (key == "signal") current_motor.signal_type = value;
            else if (key == "amplitude") current_motor.amplitude = std::stod(value);
            else if (key == "frequency") current_motor.frequency = std::stod(value);
            else if (key == "phase") current_motor.phase = std::stod(value);
            else if (key == "shift") current_motor.shift = std::stod(value);
        }
    }

    if (in_motor_section) cfg.motors.push_back(current_motor);

    return cfg;
}

} // namespace sysid::utils