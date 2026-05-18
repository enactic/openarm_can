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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "openarm/can/socket/openarm.hpp"
#include "openarm/damiao_motor/dm_motor_constants.hpp"

// Expand comma-separated or space-separated ID tokens into a flat list.
// Examples:
//   "1,2,3"      -> {"1", "2", "3"}
//   {"1,2", "3"} -> {"1", "2", "3"}
static std::vector<std::string> expand_ids(const std::vector<std::string>& raw) {
    std::vector<std::string> result;
    for (const auto& token : raw) {
        std::stringstream ss(token);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) result.push_back(item);
        }
    }
    return result;
}

int main(int argc, char** argv) {
    const std::string banner = R"(
    ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗     ██████╗██╗     ██╗
    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║    ██╔════╝██║     ██║
    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║    ██║     ██║     ██║
    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║    ██║     ██║     ██║
    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║    ╚██████╗███████╗██║
     ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝    ╚═════╝╚══════╝╚═╝
    )";

    CLI::App app{banner + "\nMulti-Damiao-Motor High-Speed Monitor & Configuration Tool",
                 "openarm-can-cli"};

    // Require exactly one subcommand; shows help if none provided
    app.require_subcommand(1);
    app.set_help_flag("-h,--help", "Print this help message and exit");

    // ========================================================================
    // Global options
    // ========================================================================
    static std::string global_iface = "can0";
    app.add_option("-i,--interface", global_iface, "SocketCAN interface (default: can0)")
        ->default_val("can0");

    // ========================================================================
    // [ Network & Hardware ]
    // ========================================================================

    // --- can_configure: Setup SocketCAN parameters (bitrate, sample points, etc.) ---
    auto* can_configure =
        app.add_subcommand("can_configure", "Setup SocketCAN interface (default: 1Mbps/5Mbps FD)")
            ->group("[ Network & Hardware ]");

    // static int cc_bitrate = 1000000;
    // static int cc_dbitrate = 8000000;
    // static bool cc_fd_mode = true;
    // static std::string cc_sample_point = "0.75";
    // static std::string cc_dsample_point = "0.6";
    // static std::string cc_dsjw = "3";
    // static int cc_restart_ms = 100;

    static int cc_bitrate = 1000000;
    static int cc_dbitrate = 5000000;
    static bool cc_fd_mode = true;
    static std::string cc_sample_point = "0.75";
    static std::string cc_dsample_point = "0.75";
    static std::string cc_dsjw = "2";
    static int cc_restart_ms = 100;

    can_configure->add_option("-b,--bitrate", cc_bitrate, "Set arbitration phase bitrate")
        ->default_val("1000000");

    // can_configure->add_option("-d,--dbitrate", cc_dbitrate, "Set CAN FD data phase bitrate")
    //     ->default_val(8000000);
    // can_configure->add_option("--sp", cc_sample_point, "Sample point for arbitration phase")
    //     ->default_val("0.75");
    // can_configure->add_option("--dsp", cc_dsample_point, "Sample point for data phase")
    //     ->default_val("0.6");
    // can_configure->add_option("--dsjw", cc_dsjw, "Data Synchronization Jump Width")
    //     ->default_val("3");

    can_configure->add_option("-d,--dbitrate", cc_dbitrate, "Set CAN FD data phase bitrate")
        ->default_val("5000000");  // 8000000 → 5000000
    can_configure->add_option("--dsp", cc_dsample_point, "Sample point for data phase")
        ->default_val("0.75");  // "0.6" → "0.75"
    can_configure->add_option("--sp", cc_sample_point, "Sample point for arbitration phase")
        ->default_val("0.75");
    can_configure->add_option("--dsjw", cc_dsjw, "Data Synchronization Jump Width")
        ->default_val("2");  // "3" → "2"
    can_configure->add_option("--rm", cc_restart_ms, "Auto-restart time in milliseconds")
        ->default_val("100");
    can_configure->add_flag("!--no-fd,--fd", cc_fd_mode, "Enable CAN FD mode");

    can_configure->callback([&]() {
        // Check if -i was explicitly provided by comparing to default value
        std::vector<std::string> target_ifaces;
        if (app["--interface"]->count() > 0) {
            target_ifaces = {global_iface};
        } else {
            target_ifaces = {"can0", "can1", "can2", "can3"};
        }
        int result = openarm::cli::run_can_configure(target_ifaces, cc_bitrate, cc_dbitrate,
                                                     cc_fd_mode, cc_sample_point, cc_dsample_point,
                                                     cc_dsjw, cc_restart_ms);
        if (result != 0) {
            throw CLI::RuntimeError("can_configure failed.", result);
        }
    });
    // --- discover: Scan for active motors on the bus ---
    auto* discover =
        app.add_subcommand("discover", "Scan CAN bus for connected motors (default: 1M/5M/8M/10M)")
            ->group("[ Network & Hardware ]");

    static int disc_max_id = 16;
    static bool disc_full_scan = false;

    discover->add_option("-m,--max-id", disc_max_id, "Max ID to scan (default: 16)")
        ->default_val("16");
    discover->add_flag("--full-scan", disc_full_scan,
                       "Scan all 12 baudrates (default: 1M/5M/8M/10M only)");

    discover->callback([&]() {
        int result = openarm::cli::run_discover(global_iface, disc_max_id, disc_full_scan);
        if (result != 0) {
            throw CLI::RuntimeError("discover failed.", result);
        }
    });

    // ========================================================================
    // [ Motor Setup ]
    // ========================================================================

    // --- change_id: Modify motor CAN ID ---
    auto* change_id =
        app.add_subcommand("change_id", "Change Master/Slave CAN ID")->group("[ Motor Setup ]");

    static int current_id = 1;
    static int new_slave_id = 1;
    static int new_master_id = 17;
    static bool save_to_flash = false;

    change_id->add_option("-c,--current", current_id, "Current Slave ID")->required();
    change_id->add_option("-s,--new-slave", new_slave_id, "New Slave ID")->required();
    change_id->add_option("-m,--new-master", new_master_id, "New Master ID")->default_val("17");
    change_id->add_flag("--save", save_to_flash, "Save configuration to motor Flash memory");

    change_id->callback([&]() {
        std::cout << ">>> Executing change_id on " << global_iface << "..." << std::endl;
        int result = openarm::cli::run_change_id(global_iface, current_id, new_slave_id,
                                                 new_master_id, save_to_flash);
        if (result != 0) {
            throw CLI::RuntimeError("ID change failed.", result);
        }
    });

    // --- change_baud: Modify motor internal communication speed ---
    auto* change_baud = app.add_subcommand("change_baud", "Change motor internal baudrate")
                            ->group("[ Motor Setup ]");

    static int cb_baud = 0;
    static int cb_id = 0;
    static bool cb_flash = false;

    change_baud
        ->add_option("-b,--baudrate", cb_baud, "Target baudrate (e.g. 1000000, 8000000, 10000000)")
        ->required();
    change_baud->add_option("-c,--canid", cb_id, "Target Motor ID (0-255)")->required();
    change_baud->add_flag("--save", cb_flash, "Save parameters to motor flash memory");

    change_baud->callback([&]() {
        int result = openarm::cli::run_change_baud(global_iface, cb_baud, cb_id, cb_flash);
        if (result != 0) {
            throw CLI::RuntimeError("change_baud failed.", result);
        }
    });

    // --- show_param: Read all motor internal parameters (PID, Limits, etc.) ---
    auto* show_param =
        app.add_subcommand("show_param",
                           "Read all motor internal parameters (default: arm IDs 1-8)")
            ->group("[ Motor Setup ]");

    static bool sp_arm = true;
    static std::vector<std::string> sp_ids;

    show_param->add_flag("-a,--arm,!--no-arm", sp_arm, "Read from arm motors (IDs 1-8) [default]")
        ->default_val("true");
    show_param->add_option("--id", sp_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");

    show_param->callback([&]() {
        auto ids = expand_ids(sp_ids);
        if (!ids.empty()) sp_arm = false;  // --id overrides --arm
        int result = openarm::cli::run_read_params(global_iface, sp_arm, ids);
        if (result != 0) {
            std::cerr << "[ERROR] Failed to read parameters from one or more motors. "
                      << "Check wiring, CAN interface, and motor IDs." << std::endl;
            throw CLI::RuntimeError("show_param failed.", result);
        }
    });

    // --- write_param: Update internal registers ---
    auto* write_param = app.add_subcommand("write_param", "Write motor internal parameters")
                            ->group("[ Motor Setup ]");

    static uint32_t wp_id = 1;
    static int wp_rid = 0;
    static float wp_val = 0.0f;
    static bool wp_save = false;

    write_param->add_option("-c,--id", wp_id, "Target motor ID")->required();
    write_param->add_option("-r,--rid", wp_rid, "Register ID (RID)")->required();
    write_param->add_option("-v,--value", wp_val, "Value to write")->required();
    write_param->add_flag("--save", wp_save, "Save to motor Flash (⚠️ Limit: ~10,000 cycles)");

    write_param->callback([&]() {
        int result = openarm::cli::run_write_param(global_iface, wp_id, wp_rid, wp_val, wp_save);
        if (result != 0) {
            throw CLI::RuntimeError("write_param failed.", result);
        }
    });

    // --- set_zero: Calibrate mechanical zero position ---
    auto* set_zero =
        app.add_subcommand("set_zero",
                           "Set current position as mechanical zero (default: arm IDs 1-8)")
            ->group("[ Motor Setup ]");

    static bool sz_arm = true;
    static std::vector<std::string> sz_ids;

    set_zero->add_flag("-a,--arm,!--no-arm", sz_arm, "Apply to all arm motors (IDs 1-8) [default]")
        ->default_val("true");
    set_zero->add_option("--id", sz_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");

    set_zero->callback([&]() {
        auto ids = expand_ids(sz_ids);
        if (!ids.empty()) sz_arm = false;  // --id overrides --arm
        int result = openarm::cli::run_set_zero(global_iface, sz_arm, ids);
        if (result != 0) {
            throw CLI::RuntimeError("set_zero failed.", result);
        }
    });

    // ========================================================================
    // [ Operation & Debug ]
    // ========================================================================

    // --- enable: Enable motor power output ---
    auto* enable =
        app.add_subcommand("enable", "Enable motor output - Torque ON (default: arm IDs 1-8)")
            ->group("[ Operation & Debug ]");

    static bool en_arm = true;
    static std::vector<std::string> en_ids;

    enable->add_flag("-a,--arm,!--no-arm", en_arm, "Enable all arm motors (IDs 1-8) [default]")
        ->default_val("true");
    enable->add_option("--id", en_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");

    enable->callback([&]() {
        auto ids = expand_ids(en_ids);
        if (!ids.empty()) en_arm = false;  // --id overrides --arm
        int result = openarm::cli::run_motor_state_control(global_iface, en_arm, ids, true);
        if (result != 0) {
            throw CLI::RuntimeError("enable failed.", result);
        }
    });

    // --- disable: Disable motor power output ---
    auto* disable =
        app.add_subcommand("disable", "Disable motor output - Torque OFF (default: arm IDs 1-8)")
            ->group("[ Operation & Debug ]");

    static bool dis_arm = true;
    static std::vector<std::string> dis_ids;

    disable->add_flag("-a,--arm,!--no-arm", dis_arm, "Disable all arm motors (IDs 1-8) [default]")
        ->default_val("true");
    disable->add_option("--id", dis_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");

    disable->callback([&]() {
        auto ids = expand_ids(dis_ids);
        if (!ids.empty()) dis_arm = false;  // --id overrides --arm
        int result = openarm::cli::run_motor_state_control(global_iface, dis_arm, ids, false);
        if (result != 0) {
            throw CLI::RuntimeError("disable failed.", result);
        }
    });

    // --- clear_error: Reset motor error flags ---
    auto* clear_error =
        app.add_subcommand("clear_error", "Reset motor error state (default: arm IDs 1-8)")
            ->group("[ Operation & Debug ]");

    static bool ce_arm = true;
    static std::vector<std::string> ce_ids;

    clear_error
        ->add_flag("-a,--arm,!--no-arm", ce_arm,
                   "Clear errors on all arm motors (IDs 1-8) [default]")
        ->default_val("true");
    clear_error->add_option("--id", ce_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");

    clear_error->callback([&]() {
        auto ids = expand_ids(ce_ids);
        if (!ids.empty()) ce_arm = false;  // --id overrides --arm
        int result = openarm::cli::run_clear_error(global_iface, ce_arm, ids);
        if (result != 0) {
            throw CLI::RuntimeError("clear_error failed.", result);
        }
    });

    // --- monitor: Live telemetry dashboard ---
    auto* monitor =
        app.add_subcommand("monitor", "Live dashboard: pos/vel/torque/temp (default: arm IDs 1-8)")
            ->group("[ Operation & Debug ]");
    static bool mon_arm = true;
    static std::vector<std::string> mon_ids;
    static int mon_interval = 100;   // ms
    static int mon_duration = 6000;  // ms

    monitor->add_flag("-a,--arm,!--no-arm", mon_arm, "Monitor all arm motors (IDs 1-8) [default]")
        ->default_val("true");
    monitor->add_option("--id", mon_ids, "Target motor IDs (e.g. --id 1,2,3  or  --id 1 2 3)");
    monitor->add_option("-t,--tick", mon_interval, "Update interval in milliseconds")
        ->default_val("100");
    monitor->add_option("-d,--duration", mon_duration, "Total monitoring duration in milliseconds")
        ->default_val("6000");

    monitor->callback([&]() {
        auto ids = expand_ids(mon_ids);
        if (!ids.empty()) mon_arm = false;  // --id overrides --arm
        int result =
            openarm::cli::run_monitor(global_iface, mon_arm, ids, mon_interval, mon_duration);
        if (result != 0) {
            throw CLI::RuntimeError("monitor failed.", result);
        }
    });

    // ========================================================================
    // Execution - Parse arguments and dispatch subcommands
    // ========================================================================
    CLI11_PARSE(app, argc, argv);

    return 0;
}
