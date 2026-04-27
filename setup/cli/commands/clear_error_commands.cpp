#include <linux/can.h>

#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include <string>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

int run_clear_error(const std::string& interface, bool use_arm_ids,
                    const std::vector<std::string>& custom_ids_str) {
    try {
        std::cout << ">>> Connecting to " << interface << " (Classic CAN mode)..." << std::endl;
        openarm::canbus::CANSocket socket(interface, false);

        std::vector<int> target_ids;
        if (use_arm_ids) {
            for (int i = 1; i <= 8; ++i) target_ids.push_back(i);
        }
        for (const auto& id_str : custom_ids_str) {
            target_ids.push_back(std::stoi(id_str));
        }

        if (target_ids.empty()) {
            std::cerr << "✗ No target IDs specified. Use --arm or --id." << std::endl;
            return 1;
        }

        for (int id : target_ids) {
            struct can_frame frame;
            frame.can_id = id;
            frame.can_dlc = 8;

            for (int i = 0; i < 7; i++) {
                frame.data[i] = 0xFF;
            }
            frame.data[7] = 0xFB;

            if (socket.write_can_frame(frame)) {
                std::cout << "✓ Sent Clear Error command to Motor ID: " << format_hex_id(id)
                          << std::endl;
            } else {
                std::cerr << "✗ Failed to send command to Motor ID: " << format_hex_id(id)
                          << std::endl;
            }
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}

}  // namespace openarm::cli