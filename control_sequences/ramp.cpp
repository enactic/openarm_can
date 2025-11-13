#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <map>
#include <stdexcept>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

std::map<std::string, std::string> parse_input_file(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        throw std::runtime_error("Could not open input file: " + filename);
    }

    std::map<std::string, std::string> params;
    std::string line;

    while (std::getline(infile, line)) {
        // Remove comments
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos)
            line = line.substr(0, comment_pos);

        // Trim whitespace
        auto trim = [](std::string& s) {
            size_t start = s.find_first_not_of(" \t\r\n");
            size_t end = s.find_last_not_of(" \t\r\n");
            s = (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
        };
        trim(line);

        if (line.empty()) continue;

        // Parse key = value
        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = line.substr(0, eq_pos);
        std::string value = line.substr(eq_pos + 1);
        trim(key);
        trim(value);

        if (!key.empty())
            params[key] = value;
    }

    return params;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file.in>\n";
        return 1;
    }

    try {
        auto params = parse_input_file(argv[1]);

        // Input parameters
        uint32_t send_can_id = std::stoul(params.at("send_can_id"));
        std::string can_interface = params.at("can_interface");
        double max_torque = std::stod(params.at("max_torque"));
        double rise_width = std::stod(params.at("rise_width"));
        double plateau_width = std::stod(params.at("plateau_width"));
        double fall_width = std::stod(params.at("fall_width"));
        double sample_rate_hz = std::stod(params.at("resolution"));

        // Optional parameter
        std::string test_name = "default";
        if (params.find("test_name") != params.end()) {
            test_name = params["test_name"];
        }

        double dt_us = static_cast<int>(1e6 / sample_rate_hz);
        int zero_steps_start = std::max(1, static_cast<int>(0.5 * sample_rate_hz));
        int zero_steps_end = std::max(1, static_cast<int>(1.0 * sample_rate_hz));
        int ramp_up_steps = std::max(1, static_cast<int>(rise_width * sample_rate_hz));
        int plateau_steps = std::max(1, static_cast<int>(plateau_width * sample_rate_hz));
        int ramp_down_steps = std::max(1, static_cast<int>(fall_width * sample_rate_hz));

        // Print everything out
        std::cout << "=== OpenArm Motor Control Configuration ===\n"
                << "Send CAN ID: " << send_can_id << "\n"
                << "CAN Interface: " << can_interface << "\n"
                << "\nTrapezoid Profile:\n"
                << "  Height: " << max_torque << " Nm\n"
                << "  Rise width: " << rise_width << " s\n"
                << "  Plateau width: " << plateau_width << " s\n"
                << "  Fall width: " << fall_width << " s\n"
                << "  Resolution: " << sample_rate_hz << " Hz (dt_us = " << dt_us << ")\n";


        // Initialize OpenArm with CAN interface
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface, true);

        // Initialize all arm motors
        std::cout << "Initializing motors..." << std::endl;
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310
        };
        std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Open CSV file
        namespace fs = std::filesystem;
        fs::create_directories("./data/ramp");
        std::ostringstream filename;
        filename << "data/ramp/motor" << send_can_id << "_ramp_" << test_name << ".csv";
        std::ofstream csv_file(filename.str());

        // Header
        csv_file << "Torque" << send_can_id << ",Time_s";
        for (size_t i = 1; i < openarm.get_arm().get_motors().size() + 1; ++i) {
            csv_file << ",Pos" << i << ",Vel" << i;
        }
        csv_file << "\n";

        // Set callback mode for state monitoring
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        std::cout << "\n=== Enabling Motor ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        std::map<uint32_t, size_t> motor_index;
        for (size_t i = 0; i < send_can_ids.size(); ++i) {
            motor_index[send_can_ids[i]] = i;
        }
        auto control_motor = [&](uint32_t target_id, double torque) {
            std::vector<openarm::damiao_motor::MITParam> params(send_can_ids.size());
            
            for (size_t i = 0; i < params.size(); ++i) {
                if (i == motor_index.at(target_id))
                    params[i] = {0, 0, 0, 0, torque};  // Apply torque to target motor
                else
                    params[i] = {30, 1, 0, 0, 0};      // Force other motors to hold position
            }

            openarm.get_arm().mit_control_all(params);
        };

        // Reset to Zero
        for (int i = 1; i <= 2000; i++) {
            openarm.get_arm().mit_control_all({
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{50, 1, 0, 0, 0}
            });
            openarm.recv_all(500);
        }

        // Start the clock
        auto start_time = std::chrono::steady_clock::now();
        auto log_motor = [&](double torque) {
            openarm.refresh_all();
            const auto& motors = openarm.get_arm().get_motors();

            double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time
            ).count();

            csv_file << torque << "," << t;
            for (const auto& m : motors) {
                csv_file << "," << m.get_position()
                        << "," << m.get_velocity();
            }
            csv_file << "\n";
        };

        // --- Zero torque start ---
        for (int i = 0; i < zero_steps_start; i++) {
            control_motor(send_can_id, 0.0);
            openarm.recv_all(dt_us);
            log_motor(0.0);
        }

        // --- Ramp up ---
        for (int i = 1; i <= ramp_up_steps; i++) {
            double torque = max_torque * i / ramp_up_steps;
            control_motor(send_can_id, torque);
            openarm.recv_all(dt_us);
            log_motor(torque);
        }

        // --- Hold ---
        for (int i = 0; i < plateau_steps; i++) {
            control_motor(send_can_id, max_torque);
            openarm.recv_all(dt_us);
            log_motor(max_torque);
        }

        // --- Ramp down ---
        for (int i = ramp_down_steps; i >= 0; i--) {
            double torque = max_torque * i / ramp_down_steps;
            control_motor(send_can_id, torque);
            openarm.recv_all(dt_us);
            log_motor(torque);
        }

        // --- Zero torque end ---
        for (int i = 0; i < zero_steps_end; i++) {
            control_motor(send_can_id, 0.0);
            openarm.recv_all(dt_us);
            log_motor(0.0);
        }

        // Reset to Zero
        for (int i = 1; i <= 4000; i++) {
            double strength = std::round((i / 4000.0) * 100.0);
            openarm.get_arm().mit_control_all({
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{strength, 1, 0, 0, 0}
            });
            openarm.recv_all(500);
        }

        openarm.disable_all();
        openarm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
