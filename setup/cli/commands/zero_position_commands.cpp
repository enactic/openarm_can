#include <linux/can.h>

#include <chrono>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

/**
 * @brief Calibrates the mechanical zero position for specified motors.
 * This follows the DaMiao protocol sequence: Disable -> Set Zero -> Disable.
 */
int run_set_zero(const std::string& interface, bool use_arm_ids,
                 const std::vector<std::string>& custom_ids_str) {
    std::vector<uint32_t> target_ids;

    // Parse target IDs
    if (use_arm_ids) {
        // Default arm motor IDs (1 through 8)
        for (uint32_t i = 1; i <= 8; ++i) target_ids.push_back(i);
    }

    for (const auto& id_str : custom_ids_str) {
        try {
            target_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID format '" << id_str << "'\n";
            return 1;
        }
    }

    if (target_ids.empty()) {
        std::cerr << "✗ Error: No target motor IDs specified.\n";
        return 1;
    }

    try {
        // Use Classic CAN (CAN 2.0) for configuration sequences to ensure compatibility
        openarm::canbus::CANSocket socket(interface, false);
        std::cout << "Initializing Zero Position Calibration sequence on " << interface << "...\n";

        for (uint32_t id : target_ids) {
            std::cout << ">>> Processing Motor ID: " << id << std::endl;

            // 1. Disable Frame: 0xFFFFFFFFFFFFFFFD
            struct can_frame msg_disable;
            msg_disable.can_id = id;
            msg_disable.can_dlc = 8;
            for (int i = 0; i < 7; i++) msg_disable.data[i] = 0xFF;
            msg_disable.data[7] = 0xFD;

            // 2. Set Zero Frame: 0xFFFFFFFFFFFFFFFE
            struct can_frame msg_zero;
            msg_zero.can_id = id;
            msg_zero.can_dlc = 8;
            for (int i = 0; i < 7; i++) msg_zero.data[i] = 0xFF;
            msg_zero.data[7] = 0xFE;

            // Execute Calibration Sequence (Mirroring original Bash logic)
            std::cout << "  [Step 1/3] Sending 'Disable' command..." << std::endl;
            socket.write_can_frame(msg_disable);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "  [Step 2/3] Sending 'Set Zero' command (current position = 0 rad)..."
                      << std::endl;
            socket.write_can_frame(msg_zero);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "  [Step 3/3] Sending 'Disable' command to confirm..." << std::endl;
            socket.write_can_frame(msg_disable);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "✓ Success: Motor " << id << " zero position has been calibrated.\n"
                      << std::endl;
        }

        std::cout << "Calibration sequence completed for all targets." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "✗ System Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

}  // namespace openarm::cli