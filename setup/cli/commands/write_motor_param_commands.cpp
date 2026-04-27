#include <linux/can.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

#include "cli.hpp"

namespace openarm::cli {

/**
 * @brief Writes a parameter to a specific register (RID) of the motor.
 * Automatically handles float/int conversion based on the RID.
 */
int run_write_param(const std::string& interface, uint32_t can_id, int rid, float value,
                    bool save) {
    try {
        std::cout << "Connecting to " << interface << " (Classic CAN mode)..." << std::endl;
        openarm::canbus::CANSocket socket(interface, false);

        // --- 1. Prepare Data based on RID type ---
        uint32_t raw_value = 0;
        using namespace openarm::damiao_motor;
        RID reg = static_cast<RID>(rid);

        // Check if the RID should be treated as a float
        // (Gains, Inertia, Limits, etc. are floats in DM motors)
        bool is_float = true;
        if (reg == RID::MST_ID || reg == RID::ESC_ID || reg == RID::can_br ||
            reg == RID::CTRL_MODE || reg == RID::NPP || reg == RID::dir) {
            is_float = false;
        }

        if (is_float) {
            // Bit-copy float to uint32 (IEEE 754)
            std::memcpy(&raw_value, &value, sizeof(float));
        } else {
            // Simple integer cast
            raw_value = static_cast<uint32_t>(value);
        }

        // --- 2. Send Write Command (0x55) ---
        std::cout << "Writing value " << value << (is_float ? " (float)" : " (int)") << " to RID "
                  << rid << " for Motor " << format_hex_id(can_id) << "..." << std::endl;

        struct can_frame frame;
        frame.can_id = 0x7FF;
        frame.can_dlc = 8;
        frame.data[0] = can_id & 0xFF;
        frame.data[1] = (can_id >> 8) & 0xFF;
        frame.data[2] = 0x55;  // Write command
        frame.data[3] = static_cast<uint8_t>(rid);
        frame.data[4] = raw_value & 0xFF;
        frame.data[5] = (raw_value >> 8) & 0xFF;
        frame.data[6] = (raw_value >> 16) & 0xFF;
        frame.data[7] = (raw_value >> 24) & 0xFF;

        if (!socket.write_can_frame(frame)) {
            throw std::runtime_error("Failed to send CAN frame");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // --- 3. Save to Flash if requested ---
        if (save) {
            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "⚠️  WARNING: FLASH WRITE OPERATION (Limit: ~10,000 cycles)" << std::endl;

            // A. Disable motor
            struct can_frame dis;
            dis.can_id = can_id;
            dis.can_dlc = 8;
            for (int i = 0; i < 7; i++) dis.data[i] = 0xFF;
            dis.data[7] = 0xFD;
            socket.write_can_frame(dis);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // B. Save command
            struct can_frame sv;
            sv.can_id = 0x7FF;
            sv.can_dlc = 8;
            sv.data[0] = can_id & 0xFF;
            sv.data[1] = (can_id >> 8) & 0xFF;
            sv.data[2] = 0xAA;  // Save
            for (int i = 3; i < 8; i++) sv.data[i] = 0x00;

            if (socket.write_can_frame(sv)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "✓ Parameters saved to Flash." << std::endl;
            }
        }

        std::cout << "✓ Write Complete." << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}

}  // namespace openarm::cli