// Copyright 2025 Enactic, Inc.
// (License details omitted for brevity)

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t keep_running = 1;
void signal_handler(int signum) { keep_running = 0; }

void print_usage(const char* program_name) {
    std::cout << "Usage (Single Motor) : " << program_name
              << " <send_id> <recv_id> <frequency_hz> [interface] [-fd]" << std::endl;
    std::cout << "Usage (All Motors)   : " << program_name
              << " --all <frequency_hz> [interface] [-fd]" << std::endl;
    std::cout << "Example: " << program_name << " --all 500 can0 -fd" << std::endl;
}

void display_stats(const std::vector<openarm::damiao_motor::Motor>& motors, double rtt_us,
                   double target_hz, double actual_hz, uint64_t count) {
    std::cout << "\033[2J\033[H";
    std::cout << "====================================================================="
              << std::endl;
    std::cout << "          OpenArm Multi-Motor High-Speed Monitor (1s Refresh)        "
              << std::endl;
    std::cout << "====================================================================="
              << std::endl;
    std::cout << " [Network Metrics]  Loop Count: " << count << std::endl;
    std::cout << "  Target Freq    : " << std::fixed << std::setprecision(1) << target_hz << " Hz"
              << std::endl;
    std::cout << "  Actual Freq    : \033[1;36m" << std::fixed << std::setprecision(1) << actual_hz
              << " Hz\033[0m";
    if (actual_hz < target_hz * 0.95) std::cout << "  \033[1;31m[LOW!]\033[0m";
    std::cout << "\n  RTT (Last)     : \033[1;32m" << std::fixed << std::setprecision(2) << rtt_us
              << " us\033[0m" << std::endl;
    std::cout << "---------------------------------------------------------------------"
              << std::endl;

    std::cout << " [Motor Status]" << std::endl;
    std::cout << " ID(S/R) | Position (rad) | Velocity (rad/s) | Torque (Nm) | Temp(C)"
              << std::endl;
    std::cout << "---------+----------------+------------------+-------------+---------"
              << std::endl;

    for (const auto& motor : motors) {
        std::cout << " " << std::setw(2) << motor.get_send_can_id() << "/" << std::setw(2)
                  << motor.get_recv_can_id() << "  | " << std::setw(14) << std::fixed
                  << std::setprecision(4) << motor.get_position() << " | " << std::setw(16)
                  << std::fixed << std::setprecision(4) << motor.get_velocity() << " | "
                  << std::setw(11) << std::fixed << std::setprecision(4) << motor.get_torque()
                  << " | " << std::setw(6) << std::fixed << std::setprecision(1)
                  << motor.get_state_tmos() << std::endl;
    }

    std::cout << "====================================================================="
              << std::endl;
    std::cout << " Press Ctrl+C to shutdown safely." << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::vector<uint32_t> send_ids;
    std::vector<uint32_t> recv_ids;
    std::vector<openarm::damiao_motor::MotorType> types;
    double target_hz = 0;
    std::string interface = "can0";
    bool use_fd = false;

    int arg_idx = 1;
    std::string first_arg = argv[arg_idx];

    if (first_arg == "--all") {
        for (uint32_t i = 1; i <= 8; ++i) {
            send_ids.push_back(i);
            recv_ids.push_back(i + 0x10);
            types.push_back(openarm::damiao_motor::MotorType::DM4310);
        }
        arg_idx++;
    } else {
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
        send_ids.push_back(std::stoul(argv[arg_idx++]));
        recv_ids.push_back(std::stoul(argv[arg_idx++]));
        types.push_back(openarm::damiao_motor::MotorType::DM4310);
    }

    if (arg_idx < argc) {
        target_hz = std::stod(argv[arg_idx++]);
    } else {
        print_usage(argv[0]);
        return 1;
    }

    for (; arg_idx < argc; ++arg_idx) {
        if (std::string(argv[arg_idx]) == "-fd")
            use_fd = true;
        else
            interface = argv[arg_idx];
    }

    try {
        openarm::can::socket::OpenArm openarm(interface, use_fd);

        openarm.init_arm_motors(types, send_ids, recv_ids);
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        std::cout << "Initializing " << send_ids.size() << " motor(s)... Target: " << target_hz
                  << "Hz on " << interface << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        auto cycle_duration = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / target_hz));
        auto next_wakeup = std::chrono::steady_clock::now();
        auto last_stats_time = std::chrono::steady_clock::now();

        uint64_t loop_count = 0;
        uint64_t last_loop_count = 0;
        double latest_rtt_us = 0;
        double actual_hz = 0;

        std::vector<openarm::damiao_motor::MITParam> zero_cmd(send_ids.size(), {0, 0, 0, 0, 0});

        while (keep_running) {
            auto now = std::chrono::steady_clock::now();

            next_wakeup += cycle_duration;
            if (next_wakeup < now) {
                next_wakeup = now + cycle_duration;
            }

            auto t_send = std::chrono::steady_clock::now();
            openarm.get_arm().mit_control_all(zero_cmd);
            openarm.recv_all(100);
            auto t_recv = std::chrono::steady_clock::now();
            latest_rtt_us = std::chrono::duration<double, std::micro>(t_recv - t_send).count();

            auto elapsed_stats = std::chrono::duration<double>(now - last_stats_time).count();
            if (elapsed_stats >= 1.0) {
                actual_hz = (loop_count - last_loop_count) / elapsed_stats;

                const auto& motors = openarm.get_arm().get_motors();
                if (!motors.empty()) {
                    display_stats(motors, latest_rtt_us, target_hz, actual_hz, loop_count);
                }
                last_stats_time = now;
                last_loop_count = loop_count;
            }

            loop_count++;

            auto t_pre_sleep = std::chrono::steady_clock::now();
            if (next_wakeup - t_pre_sleep > std::chrono::microseconds(1000)) {
                std::this_thread::sleep_until(next_wakeup - std::chrono::microseconds(500));
            }
            while (std::chrono::steady_clock::now() < next_wakeup) {
#if defined(__i386__) || defined(__x86_64__)
                __builtin_ia32_pause();
#endif
            }
        }

        std::cout << "\nTerminating safely..." << std::endl;
        openarm.disable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        openarm.recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}