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

int run_can_configure(const std::vector<std::string>& interfaces, int bitrate, int dbitrate,
                      bool fd_mode, const std::string& sp, const std::string& dsp,
                      const std::string& dsjw, int restart_ms);

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

// ========================================================================
// [ Shared Utilities ]
// ========================================================================

inline std::string format_hex_id(uint32_t id) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << id;
    return ss.str();
}

}  // namespace openarm::cli
