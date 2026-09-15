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

#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace openarm::cli {

// ========================================================================
// [ Network & Hardware ]
// ========================================================================

// Parameters applied by can_configure. The member initializers are the
// defaults of the can_configure subcommand and are also what discover restores
// the interface to after scanning, so the two cannot drift apart.
struct CanConfigureOptions {
    int bitrate = 1000000;
    int dbitrate = 5000000;
    bool fd_mode = true;
    std::string sample_point = "0.75";
    std::string dsample_point = "0.75";
    std::string dsjw = "2";
    // 0 keeps the controller stopped after a bus-off instead of silently
    // restarting it. An auto-restart hides the fault: the link is back within
    // milliseconds while control was interrupted the whole time.
    int restart_ms = 0;
    // 0 leaves the kernel's default. The CAN default of 10 frames is short on
    // purpose: a deep queue delivers stale commands.
    int txqueuelen = 0;
};

int run_can_configure(const std::vector<std::string>& interfaces,
                      const CanConfigureOptions& options);

int run_discover(const std::string& interface, int max_id, bool full_scan = false);

int run_change_id(const std::string& interface, int current_id, int new_slave_id, int new_master_id,
                  bool save);

int run_change_baud(const std::string& interface, int baudrate, int canid, bool flash);

int run_read_params(const std::string& interface, bool use_arm_ids,
                    const std::vector<std::string>& custom_ids_str);

int run_write_param(const std::string& interface, uint32_t can_id, int rid, float value, bool save);

int run_set_zero(const std::string& interface, bool use_arm_ids,
                 const std::vector<std::string>& custom_ids_str);

// ========================================================================
// [ Operation & Debug ]
// ========================================================================

int run_motor_state_control(const std::string& interface, bool use_arm_ids,
                            const std::vector<std::string>& custom_ids_str, bool enable);

int run_clear_error(const std::string& interface, bool use_arm_ids,
                    const std::vector<std::string>& custom_ids_str);

int run_monitor(const std::string& interface, bool use_arm_ids,
                const std::vector<std::string>& custom_ids_str, int interval_ms, int duration_ms);

int run_diagnose(const std::string& interface, bool use_arm_ids,
                 const std::vector<std::string>& custom_ids_str, int duration_ms, int interval_ms,
                 bool want_explain = false);

// ========================================================================
// [ Shared Utilities ]
// ========================================================================

inline std::string format_hex_id(uint32_t id) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << id;
    return ss.str();
}

}  // namespace openarm::cli
