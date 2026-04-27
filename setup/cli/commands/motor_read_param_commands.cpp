#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <vector>

#include "cli.hpp"

namespace openarm::cli {

std::string rid_to_string(int rid) {
    using namespace openarm::damiao_motor;
    switch (static_cast<RID>(rid)) {
        case RID::UV_Value:
            return "Under-Voltage Threshold";
        case RID::KT_Value:
            return "KT Value (Torque Const)";
        case RID::OT_Value:
            return "Over-Temp Threshold";
        case RID::OC_Value:
            return "Over-Current Threshold";
        case RID::ACC:
            return "Acceleration";
        case RID::DEC:
            return "Deceleration";
        case RID::MAX_SPD:
            return "Max Speed";
        case RID::MST_ID:
            return "Master ID";
        case RID::ESC_ID:
            return "Motor (ESC) ID";
        case RID::TIMEOUT:
            return "CAN Timeout";
        case RID::CTRL_MODE:
            return "Control Mode";
        case RID::Damp:
            return "Damping Ratio";
        case RID::Inertia:
            return "Inertia";
        case RID::hw_ver:
            return "Hardware Version";
        case RID::sw_ver:
            return "Software Version";
        case RID::SN:
            return "Serial Number";

        case RID::NPP:
            return "Number of Pole Pairs";
        case RID::Rs:
            return "Stator Resistance (Rs)";
        case RID::LS:
            return "Stator Inductance (Ls)";
        case RID::Flux:
            return "Rotor Flux";
        case RID::Gr:
            return "Gear Ratio";

        case RID::PMAX:
            return "Position Limit (PMAX)";
        case RID::VMAX:
            return "Velocity Limit (VMAX)";
        case RID::TMAX:
            return "Torque Limit (TMAX)";
        case RID::I_BW:
            return "Current Loop Bandwidth";
        case RID::KP_ASR:
            return "Speed Loop KP";
        case RID::KI_ASR:
            return "Speed Loop KI";
        case RID::KP_APR:
            return "Position Loop KP";
        case RID::KI_APR:
            return "Position Loop KI";
        case RID::OV_Value:
            return "Over-Voltage Threshold";

        case RID::GREF:
            return "GREF (Gravity Reference)";
        case RID::Deta:
            return "Deta (Smoothing Filter)";
        case RID::V_BW:
            return "Velocity Loop Bandwidth";
        case RID::IQ_c1:
            return "Current Loop C1";
        case RID::VL_c1:
            return "Velocity Loop C1";

        case RID::can_br:
            return "CAN Baudrate Code";
        case RID::sub_ver:
            return "Sub Version";

        case RID::u_off:
            return "I-Phase U Offset";
        case RID::v_off:
            return "I-Phase V Offset";
        case RID::k1:
            return "Gain K1";
        case RID::k2:
            return "Gain K2";
        case RID::m_off:
            return "Mechanical Offset";
        case RID::dir:
            return "Motor Direction";

        case RID::p_m:
            return "Position Mode Offset";
        case RID::xout:
            return "XOUT (Internal Status)";

        default:
            return "Register " + std::to_string(rid);
    }
}

/**
 * @brief Reads all internal parameters from specified motors and prints them.
 */
int run_read_params(const std::string& interface, bool use_arm_ids,
                    const std::vector<std::string>& custom_ids_str) {
    std::vector<uint32_t> send_ids;

    // 1. Resolve target motor IDs
    if (use_arm_ids) {
        for (uint32_t i = 1; i <= 8; ++i) send_ids.push_back(i);
    }
    for (const auto& id_str : custom_ids_str) {
        try {
            send_ids.push_back(std::stoul(id_str, nullptr, 0));
        } catch (...) {
            std::cerr << "✗ Error: Invalid ID '" << id_str << "'\n";
            return 1;
        }
    }

    if (send_ids.empty()) {
        std::cerr << "✗ Error: No target IDs specified.\n";
        return 1;
    }

    try {
        // 2. Initialize CAN communication
        std::cout << ">>> Connecting to " << interface << " (FD Mode)..." << std::endl;
        openarm::can::socket::OpenArm openarm(interface, true);

        std::vector<openarm::damiao_motor::MotorType> types(
            send_ids.size(), openarm::damiao_motor::MotorType::DM4310);
        std::vector<uint32_t> recv_ids;
        for (auto id : send_ids) recv_ids.push_back(id + 0x10);

        openarm.init_arm_motors(types, send_ids, recv_ids);
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

        // 3. Batch query all registers (0 to 81)
        std::cout << ">>> Querying all registers for " << send_ids.size()
                  << " motors. Please wait...\n";

        for (int r = 0; r < (int)openarm::damiao_motor::RID::COUNT; ++r) {
            openarm.query_param_all(r);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(35));  // Delay to avoid bus saturation
            openarm.recv_all();
        }

        // 4. Formatting and output
        const auto& motors = openarm.get_arm().get_motors();
        for (size_t i = 0; i < motors.size(); ++i) {
            uint32_t sid = send_ids[i];
            std::cout << "\n==================================================\n";
            std::cout << " MOTOR ID: 0x" << std::hex << sid << std::dec << " (Response ID: 0x"
                      << std::hex << recv_ids[i] << std::dec << ")\n";
            std::cout << "==================================================\n";

            // Connectivity check
            if (!std::isfinite(motors[i].get_param((int)openarm::damiao_motor::RID::MST_ID))) {
                std::cout << "  [!] NO RESPONSE FROM MOTOR\n";
                continue;
            }

            std::cout << std::left << std::setw(30) << "Parameter Name" << " | " << "Value" << "\n";
            std::cout << "--------------------------------------------------\n";

            for (int r = 0; r < (int)openarm::damiao_motor::RID::COUNT; ++r) {
                double val = motors[i].get_param(r);
                if (std::isfinite(val) && val != -1.0) {
                    std::cout << std::left << std::setw(30) << rid_to_string(r) << " | ";

                    using namespace openarm::damiao_motor;
                    RID reg = static_cast<RID>(r);

                    if (reg == RID::MST_ID || reg == RID::ESC_ID || reg == RID::can_br ||
                        reg == RID::CTRL_MODE || reg == RID::hw_ver || reg == RID::sw_ver ||
                        reg == RID::SN) {
                        std::cout << std::fixed << std::setprecision(0) << val;
                    } else if (reg == RID::Inertia || reg == RID::Damp || reg == RID::Rs ||
                               reg == RID::LS || reg == RID::Flux) {
                        std::cout << std::scientific << std::setprecision(6) << val;
                    } else {
                        std::cout << std::fixed << std::setprecision(4) << val;
                    }

                    std::cout << std::defaultfloat << "\n";
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "✗ System Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

}  // namespace openarm::cli