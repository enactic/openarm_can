#include "sysid/utils/parser.hpp"
#include "sysid/signal/factory.hpp"
#include <openarm/can/socket/openarm.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

class Logger {
public:
    static void write_csv(const std::string& filename, const std::vector<std::string>& buffer, size_t num_motors) {
        std::ofstream file(filename);
        file << "time";
        for (size_t i = 0; i < num_motors; ++i)
            file << ",m" << i << "_pos,m" << i << "_vel,m" << i << "_tor,m" << i << "_cmd";
        file << "\n";
        for (const auto& row : buffer) file << row << "\n";
    }
};

int main(int argc, char* argv[]) {
    if (argc < 4 || std::string(argv[2]) != "--seconds") {
        std::cerr << "Usage: " << argv[0] << " <input_file.in> --seconds <duration>\n";
        return 1;
    }

    std::string input_file = argv[1];
    double duration = std::stod(argv[3]);

    try {
        auto cfg = sysid::utils::parse_input_file(input_file);

        // Print configuration for debug
        std::cout << "CAN interface: " << cfg.can_interface << "\n";
        for (const auto& m : cfg.motors) {
            std::cout << "Motor " << m.send_can_id
                      << " | type: " << m.signal_type
                      << " | amp: " << m.amplitude
                      << " | freq: " << m.frequency
                      << " | phase: " << m.phase
                      << " | shift: " << m.shift << "\n";
        }

        // Construct signals
        std::vector<sysid::signal::Signal> signals;
        for (const auto& m : cfg.motors) {
            sysid::signal::Type type;
            if (m.signal_type == "sine") type = sysid::signal::Type::Sine;
            else if (m.signal_type == "saw") type = sysid::signal::Type::Saw;
            else if (m.signal_type == "trap") type = sysid::signal::Type::Trap;
            else if (m.signal_type == "step") type = sysid::signal::Type::Step;
            else type = sysid::signal::Type::Sine; // fallback

            signals.emplace_back(type, m.amplitude, m.frequency, m.phase, m.shift);
        }

        // Initialize OpenArm
        openarm::can::socket::OpenArm openarm(cfg.can_interface, true);
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
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();

        // Reset to Zero
        for (int i = 1; i <= 2000; i++) {
            openarm.get_arm().mit_control_all({
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0},
                openarm::damiao_motor::MITParam{5, 1, 0, 0, 0}
            });
            openarm.recv_all();
        }

        std::vector<std::string> log_buffer;
        auto start_time = std::chrono::steady_clock::now();

        // Main loop
        while (true) {
            auto now = std::chrono::steady_clock::now();
            double t = std::chrono::duration<double>(now - start_time).count();
            if (t > duration) break;

            // compute torques
            std::vector<double> torques;
            for (auto& s : signals)
                torques.push_back(s.value(t));

            // send torques to motors
            std::vector<openarm::damiao_motor::MITParam> params(cfg.motors.size());
            for (size_t i = 0; i < params.size(); ++i)
                params[i] = {0,0,0,0, torques[i]};

            openarm.get_arm().mit_control_all(params);

            // read and log
            openarm.refresh_all();
            openarm.recv_all();
            const auto& motors = openarm.get_arm().get_motors();

            std::string row = std::to_string(t);
            for (size_t i = 0; i < motors.size(); ++i) {
                const auto& m = motors[i];
                row += "," + std::to_string(m.get_position())
                    + "," + std::to_string(m.get_velocity())
                    + "," + std::to_string(m.get_torque())
                    + "," + std::to_string(torques[i]);
            }

            log_buffer.push_back(row);
        }

        openarm.disable_all();
        openarm.recv_all(1000);

        Logger::write_csv("openarm_log.csv", log_buffer, cfg.motors.size());

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
