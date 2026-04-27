#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

/**
 * @brief Controls the operational state (Enable/Disable) of the arm motors.
 * This function initializes the CAN interface, registers the motors, and sends state transition
 * commands.
 */
int run_motor_state_control(const std::string& interface, bool use_arm_ids,
                            const std::vector<std::string>& custom_ids_str, bool enable) {
    std::vector<uint32_t> send_ids;

    // 1. Populate the list of target CAN IDs
    if (use_arm_ids) {
        // IDs 1-8 are reserved for the standard arm configuration
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
    }

    for (const auto& id_str : custom_ids_str) {
        try {
            // Parses both decimal and hex (if prefixed with 0x) strings
            send_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID format provided: '" << id_str << "'\n";
            return 1;
        }
    }

    if (send_ids.empty()) {
        std::cerr << "✗ Error: No target IDs specified. Please use --arm or provide specific IDs "
                     "via --id.\n";
        return 1;
    }

    try {
        // 2. Initialize the OpenArm SocketCAN interface
        std::cout << ">>> Opening " << interface << " (CAN-FD Enabled)..." << std::endl;
        openarm::can::socket::OpenArm openarm(interface, true);

        // 3. Register and initialize motor components
        // DaMiao motors typically use: Response_ID = Master_ID + 0x10
        std::vector<openarm::damiao_motor::MotorType> motor_types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) {
            recv_ids.push_back(id + 0x10);
        }

        std::cout << ">>> Initializing " << send_ids.size() << " DM4310 motor(s)..." << std::endl;
        openarm.init_arm_motors(motor_types, send_ids, recv_ids);

        // 4. Execute the state transition
        // Set callback mode to IGNORE to bypass telemetry processing during the state change
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        if (enable) {
            std::cout << ">>> Action: ENABLING torque output for target motors..." << std::endl;
            openarm.enable_all();
        } else {
            std::cout << ">>> Action: DISABLING torque output for target motors..." << std::endl;
            openarm.disable_all();
        }

        // 5. Synchronization wait
        // Wait 2ms (2000us) to allow motors to process the command and clear the RX buffer
        openarm.recv_all(2000);

        std::cout << "✓ State change request successful for IDs: ";
        for (auto id : send_ids) {
            std::cout << (id < 16 ? "0x0" : "0x") << std::hex << id << " ";
        }
        std::cout << std::dec << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "✗ System Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

}  // namespace openarm::cli