#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

int run_can_configure(const std::vector<std::string>& interfaces, int bitrate, int dbitrate,
                      bool fd_mode, const std::string& sample_point,
                      const std::string& dsample_point, const std::string& dsjw, int restart_ms) {
    // Create a local copy of the interfaces list for manipulation
    std::vector<std::string> target_interfaces = interfaces;

    // Fallback to default interfaces if the list is empty
    if (target_interfaces.empty()) {
        target_interfaces = {"can0", "can1", "can2", "can3"};
    }

    for (const auto& iface : target_interfaces) {
        std::cout << ">>> Configuring interface: " << iface << "..." << std::endl;

        // 1. Bring the interface DOWN to apply changes
        // Using sudo to ensure permissions; redirecting stderr to null for a cleaner output
        std::string cmd_down = "sudo ip link set " + iface + " down 2>/dev/null";
        std::system(cmd_down.c_str());

        // 2. Construct the configuration command
        // Note: bitrates and restart-ms are converted to strings via std::to_string
        std::string cmd_set = "sudo ip link set " + iface + " type can bitrate " +
                              std::to_string(bitrate) + " sample-point " + sample_point +
                              " restart-ms " + std::to_string(restart_ms);

        // Append CAN FD specific parameters if mode is enabled
        if (fd_mode) {
            cmd_set += " dbitrate " + std::to_string(dbitrate) + " fd on" + " dsample-point " +
                       dsample_point + " dsjw " + dsjw;
        }

        std::cout << "    Executing: " << cmd_set << std::endl;

        // Execute the 'ip link set' command
        int ret = std::system(cmd_set.c_str());
        if (ret != 0) {
            std::cerr << "✗ Error: Failed to apply CAN parameters to " << iface << std::endl;
            continue;  // Attempt to configure the next interface in the list
        }

        // 3. Bring the interface back UP
        std::string cmd_up = "sudo ip link set " + iface + " up";
        ret = std::system(cmd_up.c_str());
        if (ret != 0) {
            std::cerr << "✗ Error: Failed to bring up interface " << iface << std::endl;
            continue;
        }

        // Final status report for the current interface
        std::cout << "✓ Success: " << iface << " is now UP and ACTIVE." << std::endl;
        std::cout << "    Mode: " << (fd_mode ? "CAN-FD" : "Classic CAN") << std::endl;
        std::cout << "    Nominal Bitrate: " << bitrate << " bps (SP: " << sample_point << ")"
                  << std::endl;
        if (fd_mode) {
            std::cout << "    Data Bitrate: " << dbitrate << " bps (DSP: " << dsample_point
                      << ", DSJW: " << dsjw << ")" << std::endl;
        }
        std::cout << "    Auto-restart: " << restart_ms << " ms" << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
    }

    return 0;
}

}  // namespace openarm::cli