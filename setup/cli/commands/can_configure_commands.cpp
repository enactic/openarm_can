// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

int run_can_configure(const std::vector<std::string>& interfaces, int bitrate, int dbitrate,
                      bool fd_mode, const std::string& sample_point,
                      const std::string& dsample_point, const std::string& dsjw, int restart_ms) {
    std::vector<std::string> target_interfaces = interfaces;
    if (target_interfaces.empty()) {
        target_interfaces = {"can0", "can1", "can2", "can3"};
    }

    // Header: show what we are about to apply
    std::cout << "=========================================================\n";
    std::cout << " CAN CONFIGURE\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " Target    :";
    for (const auto& i : target_interfaces) std::cout << " " << i;
    std::cout << "\n";
    std::cout << " Mode      : " << (fd_mode ? "CAN-FD" : "Classic CAN") << "\n";
    std::cout << " Bitrate   : " << bitrate << " bps  (SP: " << sample_point << ")\n";
    if (fd_mode) {
        std::cout << " Data rate : " << dbitrate << " bps  (DSP: " << dsample_point
                  << ", DSJW: " << dsjw << ")\n";
    }
    std::cout << " Restart   : " << restart_ms << " ms\n";
    std::cout << "=========================================================\n\n";

    int failed = 0;

    for (const auto& iface : target_interfaces) {
        std::cout << ">>> [" << iface << "] Applying..." << std::endl;

        std::string cmd_down = "sudo ip link set " + iface + " down 2>/dev/null";
        std::system(cmd_down.c_str());

        std::string cmd_set = "sudo ip link set " + iface + " type can bitrate " +
                              std::to_string(bitrate) + " sample-point " + sample_point +
                              " restart-ms " + std::to_string(restart_ms);
        if (fd_mode) {
            cmd_set += " dbitrate " + std::to_string(dbitrate) + " fd on dsample-point " +
                       dsample_point + " dsjw " + dsjw;
        }

        std::cout << "    " << cmd_set << std::endl;
        int ret = std::system(cmd_set.c_str());
        if (ret != 0) {
            std::cerr << "✗ [" << iface << "] Failed to apply CAN parameters." << std::endl;
            ++failed;
            continue;
        }

        std::string cmd_up = "sudo ip link set " + iface + " up";
        ret = std::system(cmd_up.c_str());
        if (ret != 0) {
            std::cerr << "✗ [" << iface << "] Failed to bring up interface." << std::endl;
            ++failed;
            continue;
        }

        std::cout << "✓ [" << iface << "] UP and active." << std::endl;
    }

    // Summary
    int total = static_cast<int>(target_interfaces.size());
    std::cout << "\n---------------------------------------------------------\n";
    if (failed == 0) {
        std::cout << "✓ All " << total << " interface(s) configured successfully.\n";
    } else {
        std::cerr << "✗ " << failed << "/" << total << " interface(s) failed.\n";
        std::cout << "  " << (total - failed) << "/" << total
                  << " interface(s) configured successfully.\n";
    }
    std::cout << "---------------------------------------------------------\n";

    return failed > 0 ? 1 : 0;
}

}  // namespace openarm::cli
