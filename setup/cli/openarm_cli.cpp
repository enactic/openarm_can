#include <linux/can.h>
#include <linux/can/raw.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "openarm/can/socket/openarm.hpp"
#include "openarm/damiao_motor/dm_motor_constants.hpp"

int main(int argc, char** argv) {
    // Define a cool ASCII banner
    std::string banner = R"(
    ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗     ██████╗██╗     ██╗
    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║    ██╔════╝██║     ██║
    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║    ██║     ██║     ██║
    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║    ██║     ██║     ██║
    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║    ╚██████╗███████╗██║
    ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝    ╚═════╝╚══════╝╚═╝
    )";

    CLI::App app{banner + "\nMulti-Motor High-Speed Monitor & Configuration Tool", "openarm-can"};

    // Require at least one subcommand (shows help automatically if no arguments are provided)
    app.require_subcommand(1);
    app.set_help_flag("-h,--help", "Print this help message and exit");

    // ========================================================================
    // Global Options
    // ========================================================================
    // Global SocketCAN interface used across all subcommands
    static std::string global_iface = "can0";
    app.add_option("-i,--interface", global_iface, "SocketCAN interface (default: can0)")
        ->default_val("can0");

    // ========================================================================
    // [ Network & Hardware ] - Group for physical interface setup
    // ========================================================================

    // --- can_configure: Setup SocketCAN parameters (bitrate, sample points, etc.) ---
    auto* can_configure =
        app.add_subcommand("can_configure", "Setup SocketCAN interface (baudrate, SP, SJW)")
            ->group("[ Network & Hardware ]");

    static std::vector<std::string> cc_interfaces;
    static int cc_bitrate = 1000000;
    static int cc_dbitrate = 8000000;
    static bool cc_fd_mode = true;
    static std::string cc_sample_point = "0.75";
    static std::string cc_dsample_point = "0.6";
    static std::string cc_dsjw = "3";
    static int cc_restart_ms = 100;

    can_configure->add_option("interfaces", cc_interfaces,
                              "Target CAN interfaces (e.g. can0 can1)");
    can_configure->add_option("-b,--bitrate", cc_bitrate, "Set arbitration phase bitrate")
        ->default_val(1000000);
    can_configure->add_option("-d,--dbitrate", cc_dbitrate, "Set CAN FD data phase bitrate")
        ->default_val(8000000);
    can_configure->add_option("--sp", cc_sample_point, "Sample point for arbitration phase")
        ->default_val("0.75");
    can_configure->add_option("--dsp", cc_dsample_point, "Sample point for data phase")
        ->default_val("0.6");
    can_configure->add_option("--dsjw", cc_dsjw, "Data Synchronization Jump Width")
        ->default_val("3");
    can_configure->add_option("--rm", cc_restart_ms, "Auto-restart time in milliseconds")
        ->default_val(100);
    can_configure->add_flag("--no-fd,!--fd", cc_fd_mode, "Disable CAN FD mode");

    can_configure->callback([&]() {
        openarm::cli::run_can_configure(cc_interfaces, cc_bitrate, cc_dbitrate, cc_fd_mode,
                                        cc_sample_point, cc_dsample_point, cc_dsjw, cc_restart_ms);
    });

    // --- discover: Scan for active motors on the bus ---
    auto* discover = app.add_subcommand("discover", "Scan CAN bus for connected motors")
                         ->group("[ Network & Hardware ]");

    static int disc_max_id = 64;  // Default scan range: 1 to 64
    discover->add_option("-m,--max-id", disc_max_id, "Max ID to scan (default: 64)")
        ->default_val(64);

    discover->callback([&]() { openarm::cli::run_discover(global_iface, disc_max_id); });

    // ========================================================================
    // [ Motor Setup ] - Group for non-realtime motor configuration
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
    change_id->add_option("-m,--new-master", new_master_id, "New Master ID")->default_val(17);
    change_id->add_flag("--save", save_to_flash, "Save configuration to motor Flash memory");

    change_id->callback([&]() {
        std::cout << ">>> Executing change_id..." << std::endl;
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

    change_baud->callback(
        [&]() { openarm::cli::run_change_baud(global_iface, cb_baud, cb_id, cb_flash); });

    // --- show_param: Read all motor internal parameters (PID, Limits, etc.) ---
    auto* show_param = app.add_subcommand("show_param", "Read all motor internal parameters")
                           ->group("[ Motor Setup ]");

    static bool sp_arm = false;
    static std::vector<std::string> sp_ids;

    show_param->add_flag("--arm", sp_arm, "Read from arm motors (IDs 1-8)");
    show_param->add_option("--id", sp_ids, "Target motor IDs");

    show_param->callback([&]() { openarm::cli::run_read_params(global_iface, sp_arm, sp_ids); });

    // --- write_param: Update internal registers (Aligned with show_param style) ---
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

    write_param->callback(
        [&]() { openarm::cli::run_write_param(global_iface, wp_id, wp_rid, wp_val, wp_save); });

    // --- set_zero: Calibrate mechanical zero position ---
    auto* set_zero =
        app.add_subcommand("set_zero", "Set current position as mechanical zero (0 rad)")
            ->group("[ Motor Setup ]");

    static bool sz_arm = false;
    static std::vector<std::string> sz_ids;

    set_zero->add_flag("--arm", sz_arm, "Apply to all arm motors (IDs 1-8)");
    set_zero->add_option("--id", sz_ids, "Specific target motor IDs");

    set_zero->callback([&]() { openarm::cli::run_set_zero(global_iface, sz_arm, sz_ids); });

    // ========================================================================
    // [ Operation & Debug ] - Group for realtime control and monitoring
    // ========================================================================

    // --- enable: Enable motor power output ---
    auto* enable = app.add_subcommand("enable", "Enable motor output (Torque ON)")
                       ->group("[ Operation & Debug ]");

    static bool en_arm = false;
    static std::vector<std::string> en_ids;

    enable->add_flag("--arm", en_arm, "Enable all arm motors (IDs 1-8)");
    enable->add_option("--id", en_ids, "Specific motor IDs to enable");

    enable->callback(
        [&]() { openarm::cli::run_motor_state_control(global_iface, en_arm, en_ids, true); });

    // --- disable: Disable motor power output ---
    auto* disable = app.add_subcommand("disable", "Disable motor output (Torque OFF)")
                        ->group("[ Operation & Debug ]");

    static bool dis_arm = false;
    static std::vector<std::string> dis_ids;

    disable->add_flag("--arm", dis_arm, "Disable all arm motors (IDs 1-8)");
    disable->add_option("--id", dis_ids, "Specific motor IDs to disable");

    disable->callback(
        [&]() { openarm::cli::run_motor_state_control(global_iface, dis_arm, dis_ids, false); });

    // --- clear_error: Reset motor error flags ---
    auto* clear_error = app.add_subcommand("clear_error", "Reset motor error state")
                            ->group("[ Operation & Debug ]");

    static bool ce_arm = false;
    static std::vector<std::string> ce_ids;

    clear_error->add_flag("--arm", ce_arm, "Clear errors on all arm motors (IDs 1-8)");
    clear_error->add_option("--id", ce_ids, "Specific motor IDs to clear errors");

    clear_error->callback([&]() { openarm::cli::run_clear_error(global_iface, ce_arm, ce_ids); });

    // --- monitor ---
    auto* monitor =
        app.add_subcommand("monitor", "Live dashboard for position, velocity, torque, temp")
            ->group("[ Operation & Debug ]");

    static bool mon_arm = false;
    static std::vector<std::string> mon_ids;
    static int mon_interval = 100;   // Default: 100ms
    static int mon_duration = 3000;  // Default: 3000ms (3s)

    monitor->add_flag("--arm", mon_arm, "Monitor all arm motors (IDs 1-8)");
    monitor->add_option("--id", mon_ids, "Specific motor IDs to monitor");
    monitor
        ->add_option("-t,--tick", mon_interval,
                     "Update interval in milliseconds")  // Prevent conflict with global -i
        ->default_val(100);
    monitor->add_option("-d,--duration", mon_duration, "Total monitoring duration in milliseconds")
        ->default_val(3000);

    monitor->callback([&]() {
        openarm::cli::run_monitor(global_iface, mon_arm, mon_ids, mon_interval, mon_duration);
    });

    // ========================================================================
    // Execution - Parsing and subcommand dispatching
    // ========================================================================
    CLI11_PARSE(app, argc, argv);

    return 0;
}