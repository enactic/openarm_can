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

#include <chrono>
#include <iostream>
#include <map>
#include <openarm/canbus/can_socket.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

namespace {

struct MotorResult {
    uint16_t id;
    std::string status;
    bool ok;
};

void print_results(const std::vector<MotorResult>& results) {
    std::cout << "---------------------------------------------------------" << std::endl;
    for (const auto& r : results) {
        std::cout << "  " << (r.ok ? "✓" : "✗") << " Motor " << format_hex_id(r.id) << ": "
                  << r.status << std::endl;
    }
    std::cout << "---------------------------------------------------------" << std::endl;
}

}  // namespace

// The motor switches to the new baudrate right after acknowledging the write,
// so the sequence is: confirm every motor answers, write all of them on the
// current interface settings, move the interface to the new baudrate, confirm
// every motor answers there, and only then save. A failure before the save
// leaves flash untouched, and a power cycle returns every motor to where it
// started.
int run_change_baud(const std::string& interface, int baudrate,
                    const std::vector<std::string>& ids_str, bool flash) {
    // Mapping of baudrate values to DaMiao register codes
    static const std::map<int, uint8_t> BAUDRATE_MAP = {
        {125000, 0},  {200000, 1},  {250000, 2},  {500000, 3},  {1000000, 4},  {2000000, 5},
        {2500000, 6}, {3200000, 7}, {4000000, 8}, {5000000, 9}, {8000000, 10}, {10000000, 11}};

    if (BAUDRATE_MAP.find(baudrate) == BAUDRATE_MAP.end()) {
        std::cerr << "✗ Error: Unsupported baudrate " << baudrate << " bps" << std::endl;
        return 1;
    }

    std::vector<uint16_t> ids;
    for (const auto& id_str : ids_str) {
        try {
            ids.push_back(static_cast<uint16_t>(std::stoul(id_str, nullptr, 0)));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID format '" << id_str << "'\n";
            return 1;
        }
    }
    if (ids.empty()) {
        std::cerr << "✗ Error: No target motor IDs specified.\n";
        return 1;
    }

    const uint8_t code = BAUDRATE_MAP.at(baudrate);
    const uint8_t rid = static_cast<uint8_t>(damiao_motor::RID::can_br);

    // Up to 1 Mbps the motor runs Classic CAN; above it, CAN FD with a 1 Mbps
    // arbitration phase.
    CanConfigureOptions target;
    target.fd_mode = baudrate > 1000000;
    target.bitrate = target.fd_mode ? 1000000 : baudrate;
    target.dbitrate = baudrate;

    std::vector<uint16_t> to_change;

    try {
        // FD enabled so that the motors' replies are received on an FD bus.
        openarm::canbus::CANSocket socket(interface, true);

        // --- Step 1: Every motor must answer before anything is written ---
        std::cout << "[1/4] Checking motors on " << interface << "..." << std::endl;
        std::vector<MotorResult> results;
        bool all_answer = true;
        for (uint16_t id : ids) {
            auto current = param_read(socket, id, rid);
            if (!current) {
                results.push_back({id, "no answer", false});
                all_answer = false;
            } else if (*current == code) {
                results.push_back({id, "already can_br=" + std::to_string(code), true});
            } else {
                results.push_back(
                    {id, "can_br=" + std::to_string(*current) + " -> " + std::to_string(code),
                     true});
                to_change.push_back(id);
            }
        }
        print_results(results);
        if (!all_answer) {
            std::cerr << "✗ Not every motor answers. Nothing was written.\n"
                      << "  Check the IDs, power, and that the interface matches the motors' "
                         "current baudrate (see 'discover')."
                      << std::endl;
            return 1;
        }
        if (to_change.empty() && !flash) {
            std::cout << "✓ All motors already at " << baudrate << " bps. Nothing to do."
                      << std::endl;
            return 0;
        }

        // --- Step 2: Write on the current settings ---
        // Only the acknowledgement can be checked here: it is the last frame
        // the motor sends at its old baudrate.
        if (!to_change.empty()) {
            std::cout << "[2/4] Writing can_br=" << static_cast<int>(code) << "..." << std::endl;
            for (uint16_t id : to_change) {
                if (!motor_disable(socket, id) || !param_write_acked(socket, id, rid, code)) {
                    std::cerr << "✗ Motor " << format_hex_id(id)
                              << " did not acknowledge. Flash was not written.\n"
                              << "  Power cycle the motors to return them to their saved "
                                 "baudrate."
                              << std::endl;
                    return 1;
                }
                std::cout << "  ✓ Motor " << format_hex_id(id) << " acknowledged" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "✗ Exception: " << e.what() << std::endl;
        return 1;
    }

    // --- Step 3: Follow the motors to the new baudrate ---
    if (!to_change.empty()) {
        std::cout << "[3/4] Moving " << interface << " to " << baudrate << " bps..." << std::endl;
        if (run_can_configure({interface}, target) != 0) {
            std::cerr << "✗ Failed to reconfigure " << interface << ". Flash was not written.\n"
                      << "  Power cycle the motors to return them to their saved baudrate."
                      << std::endl;
            return 1;
        }
    }

    try {
        // A new socket: the old one was bound across the interface going down.
        openarm::canbus::CANSocket socket(interface, true);

        std::cout << "[3/4] Verifying at " << baudrate << " bps..." << std::endl;
        std::vector<MotorResult> results;
        bool all_ok = true;
        for (uint16_t id : ids) {
            auto value = param_read(socket, id, rid);
            if (value && *value == code) {
                results.push_back({id, "answers with can_br=" + std::to_string(code), true});
            } else {
                results.push_back(
                    {id, value ? "reads can_br=" + std::to_string(*value) : "no answer", false});
                all_ok = false;
            }
        }
        print_results(results);
        if (!all_ok) {
            std::cerr << "✗ Not every motor answers at the new baudrate. Flash was not written.\n"
                      << "  Power cycle the motors to return them to their saved baudrate."
                      << std::endl;
            return 1;
        }

        // --- Step 4: Flash Save Operation ---
        if (flash) {
            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "[!]  WARNING: FLASH WRITE OPERATION" << std::endl;
            std::cout << "   - Motor Flash memory has a limit of ~10,000 write cycles."
                      << std::endl;
            std::cout << "   - Do NOT execute this in a frequent loop." << std::endl;
            std::cout << "   - Motor will be DISABLED (Torque OFF) during this process."
                      << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;

            std::cout << "[4/4] Executing Flash Save..." << std::endl;
            results.clear();
            for (uint16_t id : ids) {
                auto value = param_save(socket, id) ? param_read(socket, id, rid) : std::nullopt;
                if (value && *value == code) {
                    results.push_back({id, "saved", true});
                } else {
                    results.push_back({id, "no answer after save", false});
                    all_ok = false;
                }
            }
            print_results(results);
            if (!all_ok) {
                std::cerr << "✗ Some motors did not answer after the save. Power cycle and run "
                             "this command again without --save to see where they stand."
                          << std::endl;
                return 1;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "✗ Exception: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "✓ All motors run at " << baudrate << " bps." << std::endl;
    if (flash) {
        std::cout << "[!]  Power cycle, then run this command again without --save to confirm\n"
                     "     the motors come back at "
                  << baudrate << " bps." << std::endl;
    } else {
        std::cout << "[!]  Not saved (--save not given): a power cycle reverts them." << std::endl;
    }
    return 0;
}

}  // namespace openarm::cli
