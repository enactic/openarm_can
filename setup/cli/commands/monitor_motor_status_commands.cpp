#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

int run_monitor(const std::string& interface, bool use_arm_ids,
                const std::vector<std::string>& custom_ids_str, int interval_ms, int duration_ms) {
    std::vector<uint32_t> send_ids;
    if (use_arm_ids)
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
    for (const auto& id_str : custom_ids_str) {
        try {
            send_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID '" << id_str << "'\n";
            return 1;
        }
    }

    if (send_ids.empty()) {
        std::cerr << "✗ Error: No target IDs specified.\n";
        return 1;
    }

    try {
        openarm::can::socket::OpenArm openarm(interface, true);
        std::vector<openarm::damiao_motor::MotorType> types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);

        openarm.init_arm_motors(types, send_ids, recv_ids);

        // --- STEP 1: Enable motors for monitoring ---
        std::cout << ">>> Enabling motors and starting monitor..." << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Wait for power-on
        openarm.recv_all();

        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            if (elapsed >= duration_ms) break;

            openarm.refresh_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            openarm.recv_all();

            // Clear Screen
            std::cout << "\033[2J\033[1;1H";

            std::cout << "========================================================================="
                         "=====\n";
            std::cout << "  OPENARM HEALTH CHECK | Elapsed: " << (double)elapsed / 1000.0 << "s / "
                      << (double)duration_ms / 1000.0 << "s\n";
            std::cout << "  Motors are currently ENABLED. Check values for any anomalies.\n";
            std::cout << "========================================================================="
                         "=====\n";
            std::cout << std::left << std::setw(8) << "ID" << std::setw(14) << "Pos(rad)"
                      << std::setw(14) << "Vel(rad/s)" << std::setw(14) << "Torque(Nm)"
                      << std::setw(10) << "T(MOS)"
                      << "T(Rtr)\n";
            std::cout << "-------------------------------------------------------------------------"
                         "-----\n";

            const auto& motors = openarm.get_arm().get_motors();
            for (size_t i = 0; i < motors.size(); ++i) {
                const auto& m = motors[i];
                std::cout << std::left << "0x" << std::hex << send_ids[i] << std::dec
                          << std::setw(6) << "" << std::fixed << std::setprecision(3)
                          << std::setw(14) << m.get_position() << std::setw(14) << m.get_velocity()
                          << std::setw(14) << m.get_torque() << std::setprecision(1)
                          << std::setw(10) << m.get_state_tmos() << m.get_state_trotor() << "\n";
            }
            std::cout << "========================================================================="
                         "=====\n";
            std::flush(std::cout);

            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }

        // --- STEP 2: Disable motors before exiting ---
        std::cout << "\n>>> Disabling motors and finishing..." << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::cout << ">>> All motors DISARMED safely.\n";

    } catch (const std::exception& e) {
        std::cerr << "✗ Monitor Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

}  // namespace openarm::cli