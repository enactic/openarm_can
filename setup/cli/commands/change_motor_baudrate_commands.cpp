#include <linux/can.h>

#include <chrono>
#include <iostream>
#include <map>
#include <openarm/canbus/can_socket.hpp>
#include <thread>

#include "cli.hpp"

namespace openarm::cli {

int run_change_baud(const std::string& interface, int baudrate, int canid, bool flash) {
    // Mapping of baudrate values to DaMiao register codes
    static const std::map<int, uint8_t> BAUDRATE_MAP = {
        {125000, 0},  {200000, 1},  {250000, 2},  {500000, 3},  {1000000, 4},  {2000000, 5},
        {2500000, 6}, {3200000, 7}, {4000000, 8}, {5000000, 9}, {8000000, 10}, {10000000, 11}};

    if (BAUDRATE_MAP.find(baudrate) == BAUDRATE_MAP.end()) {
        std::cerr << "✗ Error: Unsupported baudrate " << baudrate << " bps" << std::endl;
        return 1;
    }

    try {
        std::cout << "Connecting to " << interface << " (Classic CAN mode)..." << std::endl;
        openarm::canbus::CANSocket socket(interface, false);

        std::cout << "---------------------------------------------------------" << std::endl;

        // --- Step 1: Write New Baudrate ---
        std::cout << "[1/2] Writing baudrate " << baudrate << " to Motor ID " << canid << "..."
                  << std::endl;

        struct can_frame frame;
        frame.can_id = 0x7FF;  // System Configuration ID
        frame.can_dlc = 8;
        frame.data[0] = canid & 0xFF;
        frame.data[1] = (canid >> 8) & 0xFF;
        frame.data[2] = 0x55;  // Command: Write Parameter
        frame.data[3] = 0x23;  // Register: RID_BAUD (35)
        frame.data[4] = BAUDRATE_MAP.at(baudrate);
        for (int i = 5; i < 8; i++) frame.data[i] = 0x00;

        if (!socket.write_can_frame(frame)) {
            std::cerr << "✗ Error: Failed to write CAN frame." << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // --- Step 2: Flash Save Operation ---
        if (flash) {
            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "⚠️  WARNING: FLASH WRITE OPERATION" << std::endl;
            std::cout << "   - Motor Flash memory has a limit of ~10,000 write cycles."
                      << std::endl;
            std::cout << "   - Do NOT execute this in a frequent loop." << std::endl;
            std::cout << "   - Motor will be DISABLED (Torque OFF) during this process."
                      << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;

            std::cout << "[2/2] Executing Flash Save..." << std::endl;

            // A. Disable motor (Mandatory for Flash Save)
            struct can_frame dis;
            dis.can_id = canid;
            dis.can_dlc = 8;
            for (int i = 0; i < 7; i++) dis.data[i] = 0xFF;
            dis.data[7] = 0xFD;  // Disable command
            socket.write_can_frame(dis);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // B. Send Save Command
            struct can_frame sv;
            sv.can_id = 0x7FF;
            sv.can_dlc = 8;
            sv.data[0] = canid & 0xFF;
            sv.data[1] = (canid >> 8) & 0xFF;
            sv.data[2] = 0xAA;  // Command: Save to Flash
            for (int i = 3; i < 8; i++) sv.data[i] = 0x00;

            if (socket.write_can_frame(sv)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
                std::cout << "✓ Parameters saved to Flash successfully." << std::endl;
            }
        }

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "✓ Baudrate Change Sequence Completed." << std::endl;
        std::cout << "⚠️  POWER CYCLE REQUIRED to apply changes permanently." << std::endl;
        std::cout << "---------------------------------------------------------" << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "✗ Exception: " << e.what() << std::endl;
        return 1;
    }
}

}  // namespace openarm::cli