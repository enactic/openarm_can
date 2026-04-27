#include <linux/can.h>

#include <chrono>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include <thread>

#include "cli.hpp"

namespace openarm::cli {

int run_change_id(const std::string& interface, int current_id, int new_slave_id, int new_master_id,
                  bool save) {
    try {
        std::cout << "Connecting to " << interface << " (Classic CAN mode)..." << std::endl;

        // Config frames (0x7FF) must be sent via Classic CAN (Standard Frame)
        openarm::canbus::CANSocket socket(interface, false);

        // Helper to send configuration frames via 0x7FF system ID
        auto send_config = [&](uint16_t target_id, uint8_t rid, uint32_t value) {
            struct can_frame frame;
            frame.can_id = 0x7FF;
            frame.can_dlc = 8;
            frame.data[0] = target_id & 0xFF;
            frame.data[1] = (target_id >> 8) & 0xFF;
            frame.data[2] = 0x55;  // Command: Write Parameter
            frame.data[3] = rid;   // Register ID
            frame.data[4] = value & 0xFF;
            frame.data[5] = (value >> 8) & 0xFF;
            frame.data[6] = (value >> 16) & 0xFF;
            frame.data[7] = (value >> 24) & 0xFF;
            return socket.write_can_frame(frame);
        };

        std::cout << "---------------------------------------------------------" << std::endl;

        // --- Step 1: Change Slave ID (ESC_ID) ---
        // Target original ID for this initial command
        std::cout << "[1/3] Changing Slave ID: " << format_hex_id(current_id) << " -> "
                  << format_hex_id(new_slave_id) << "..." << std::endl;
        send_config(current_id, 0x08, new_slave_id);

        // Wait for motor RAM to update the ID
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // --- Step 2: Change Master ID (MST_ID) ---
        // Switch target to New ID as the motor now identifies as the new ID in RAM
        std::cout << "[2/3] Changing Master ID: -> " << format_hex_id(new_master_id)
                  << " (via New ID " << format_hex_id(new_slave_id) << ")..." << std::endl;
        send_config(new_slave_id, 0x07, new_master_id);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // --- Step 3: Flash Save Operation ---
        if (save) {
            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "⚠️  WARNING: FLASH WRITE OPERATION" << std::endl;
            std::cout << "   - Motor Flash memory has a limit of ~10,000 write cycles."
                      << std::endl;
            std::cout << "   - Do NOT execute this in a frequent loop." << std::endl;
            std::cout << "   - Motor will be DISABLED (Torque OFF) during this process."
                      << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;

            std::cout << "[3/3] Executing Flash Save via New ID..." << std::endl;

            // A. Disable motor (Required before saving to Flash)
            struct can_frame dis;
            dis.can_id = new_slave_id;
            dis.can_dlc = 8;
            for (int i = 0; i < 7; i++) dis.data[i] = 0xFF;
            dis.data[7] = 0xFD;  // Disable constant
            socket.write_can_frame(dis);

            // Wait for state transition
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // B. Send Save Command
            struct can_frame sv;
            sv.can_id = 0x7FF;
            sv.can_dlc = 8;
            sv.data[0] = new_slave_id & 0xFF;
            sv.data[1] = (new_slave_id >> 8) & 0xFF;
            sv.data[2] = 0xAA;  // Command: Save to Flash
            for (int i = 3; i < 8; i++) sv.data[i] = 0x00;

            if (socket.write_can_frame(sv)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "✓ ID Configuration saved to Flash successfully." << std::endl;
            }
        }

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "✓ ID Change Sequence Completed." << std::endl;
        std::cout << "  Active Slave ID (RAM): " << format_hex_id(new_slave_id) << std::endl;
        std::cout << "  Active Master ID (RAM): " << format_hex_id(new_master_id) << std::endl;
        std::cout << "⚠️  POWER CYCLE REQUIRED to apply changes permanently." << std::endl;
        std::cout << "---------------------------------------------------------" << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}

}  // namespace openarm::cli