#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

// Configuration for all supported baudrates
struct BaudSetting {
    int bitrate;
    int dbitrate;
    std::string label;
};

static const std::map<int, BaudSetting> SCAN_BAUDRATE_MAP = {
    {0, {125000, 125000, "125 kbps"}},        {1, {200000, 200000, "200 kbps"}},
    {2, {250000, 250000, "250 kbps"}},        {3, {500000, 500000, "500 kbps"}},
    {4, {1000000, 1000000, "1 Mbps"}},        {5, {1000000, 2000000, "2 Mbps (FD)"}},
    {6, {1000000, 2500000, "2.5 Mbps (FD)"}}, {7, {1000000, 3200000, "3.2 Mbps (FD)"}},
    {8, {1000000, 4000000, "4 Mbps (FD)"}},   {9, {1000000, 5000000, "5 Mbps (FD)"}},
    {10, {1000000, 8000000, "8 Mbps (FD)"}},  {11, {1000000, 10000000, "10 Mbps (FD)"}}};

struct DiscoveredMotor {
    uint32_t send_id;
    uint32_t recv_id;
    int baud_code;
    std::string baud_label;

    bool operator<(const DiscoveredMotor& other) const {
        if (send_id != other.send_id) return send_id < other.send_id;
        if (recv_id != other.recv_id) return recv_id < other.recv_id;
        return baud_code < other.baud_code;
    }
};

// Helper: Visual progress bar
void print_progress(int current, int total, const std::string& info) {
    float progress = (float)current / total;
    int barWidth = 30;
    std::cout << "\r[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << "% | " << info << std::flush;
}

// Helper: Reconfigure CAN interface with user-specified sample points
bool reconfigure_can_interface(const std::string& iface, int br, int dbr) {
    std::string fd_opt = (br != dbr) ? "fd on" : "fd off";

    // User-specified: Nominal Sample-Point 0.75
    std::string cmd = "sudo ip link set " + iface + " type can bitrate " + std::to_string(br) +
                      " sample-point 0.75";

    if (br != dbr) {
        // User-specified: Data Sample-Point 0.60
        // Mandatory: dsjw 1 to prevent kernel timing errors at 8M/10M
        cmd += " dbitrate " + std::to_string(dbr) + " " + fd_opt + " dsample-point 0.60 dsjw 1";
    }
    cmd += " restart-ms 100 2>/dev/null";

    (void)system(("sudo ip link set " + iface + " down 2>/dev/null").c_str());
    int res = system(cmd.c_str());
    (void)system(("sudo ip link set " + iface + " up 2>/dev/null").c_str());

    return (res == 0);
}

// Main discovery loop
int run_discover(const std::string& interface, int max_id) {
    std::set<DiscoveredMotor> found_motors;
    const int total_bauds = 12;

    std::cout << "=========================================================\n";
    std::cout << " OPENARM DEEP DISCOVERY MODE\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " [Policy] Supports Master ID: Factory(0x00) & Offset(+0x10)\n";
    std::cout << " [Timing] SP: 0.75 / DSP: 0.60 / DSJW: 1\n";
    std::cout << " Scanning Range: 0x01 to " << format_hex_id(max_id) << "\n";
    std::cout << "=========================================================\n\n";

    for (int b = 0; b < total_bauds; ++b) {
        auto setting = SCAN_BAUDRATE_MAP.at(b);
        print_progress(b, total_bauds, "Testing " + setting.label + "...");

        if (!reconfigure_can_interface(interface, setting.bitrate, setting.dbitrate)) continue;

        // Stabilize interface
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        for (int id = 1; id <= max_id; ++id) {
            uint32_t recv_candidates[2] = {(uint32_t)(id + 0x10), 0x00};

            for (uint32_t rid : recv_candidates) {
                try {
                    // Create fresh instance per check to avoid internal library state corruption
                    openarm::can::socket::OpenArm openarm(interface,
                                                          (setting.bitrate != setting.dbitrate));

                    openarm.init_arm_motors({openarm::damiao_motor::MotorType::DM4310},
                                            {(uint32_t)id}, {rid});
                    openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

                    bool detected = false;
                    for (int retry = 0; retry < 2; ++retry) {
                        openarm.get_arm().query_param_all((int)openarm::damiao_motor::RID::MST_ID);

                        for (int k = 0; k < 2; ++k) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(15));
                            openarm.recv_all();
                        }

                        double val = openarm.get_arm().get_motor(0).get_param(
                            (int)openarm::damiao_motor::RID::MST_ID);
                        if (std::isfinite(val) && val != -1.0) {
                            detected = true;
                            break;
                        }
                    }

                    if (detected) {
                        found_motors.insert({(uint32_t)id, rid, b, setting.label});
                    }
                } catch (...) {
                }
            }
        }
    }

    print_progress(total_bauds, total_bauds, "Scan Complete!            \n\n");

    if (found_motors.empty()) {
        std::cout << "[!] No motors detected. Check wiring and power.\n";
    } else {
        std::cout << "=========================================================\n";
        std::cout << " DISCOVERY SUMMARY (Total: " << found_motors.size() << " motors found)\n";
        std::cout << "---------------------------------------------------------\n";
        std::cout << std::left << std::setw(12) << "Send ID" << std::setw(12) << "Recv ID"
                  << "Internal Baudrate Setting\n";
        std::cout << "---------------------------------------------------------\n";

        for (const auto& m : found_motors) {
            std::cout << std::left << std::setw(12) << format_hex_id(m.send_id) << std::setw(12)
                      << format_hex_id(m.recv_id) << m.baud_label << " (Code: " << m.baud_code
                      << ")\n";
        }
        std::cout << "=========================================================\n";
    }
    return 0;
}

}  // namespace openarm::cli