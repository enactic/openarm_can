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

int run_can_configure(const std::vector<std::string>& interfaces,
                      const CanConfigureOptions& options) {
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
    std::cout << " Mode      : " << (options.fd_mode ? "CAN-FD" : "Classic CAN") << "\n";
    std::cout << " Bitrate   : " << options.bitrate << " bps  (SP: " << options.sample_point
              << ")\n";
    if (options.fd_mode) {
        std::cout << " Data rate : " << options.dbitrate << " bps  (DSP: " << options.dsample_point
                  << ", DSJW: " << options.dsjw << ")\n";
    }
    std::cout << " Restart   : " << options.restart_ms << " ms\n";
    std::cout << " TX queue  : "
              << (options.txqueuelen > 0 ? std::to_string(options.txqueuelen) + " frames"
                                         : "kernel default")
              << "\n";
    std::cout << "=========================================================\n\n";

    int failed = 0;

    for (const auto& iface : target_interfaces) {
        std::cout << ">>> [" << iface << "] Applying..." << std::endl;

        std::string cmd_down = "sudo ip link set " + iface + " down 2>/dev/null";
        std::system(cmd_down.c_str());

        std::string cmd_set = "sudo ip link set " + iface + " type can bitrate " +
                              std::to_string(options.bitrate) + " sample-point " +
                              options.sample_point + " restart-ms " +
                              std::to_string(options.restart_ms);
        if (options.fd_mode) {
            cmd_set += " dbitrate " + std::to_string(options.dbitrate) + " fd on dsample-point " +
                       options.dsample_point + " dsjw " + options.dsjw;
        }

        std::cout << "    " << cmd_set << std::endl;
        int ret = std::system(cmd_set.c_str());
        if (ret != 0) {
            std::cerr << "✗ [" << iface << "] Failed to apply CAN parameters." << std::endl;
            ++failed;
            continue;
        }

        // Only touched when asked for. The CAN default of 10 frames is short on
        // purpose: a deep queue delivers stale commands, and for a control loop
        // a dropped frame beats one that arrives late. Raising it by a cycle or
        // two absorbs jitter, raising it far does not.
        if (options.txqueuelen > 0) {
            std::string cmd_q =
                "sudo ip link set " + iface + " txqueuelen " + std::to_string(options.txqueuelen);
            std::cout << "    " << cmd_q << std::endl;
            if (std::system(cmd_q.c_str()) != 0)
                std::cerr << "! [" << iface << "] Failed to set txqueuelen; continuing."
                          << std::endl;
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
