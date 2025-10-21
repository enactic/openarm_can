#include <filesystem>
#include <chrono>
#include <iostream>
#include <thread>
#include <fstream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>


int main(int argc, char* argv[]) {
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0]
                << " <send_can_id> <recv_can_id> <can_interface> -fd <max_current>\n";
        return 1;
    }

    uint32_t send_can_id = std::stoul(argv[1]);
    uint32_t recv_can_id = std::stoul(argv[2]);
    std::string can_interface = argv[3];

    // Check that fourth argument is literally "-fd"
    if (std::string(argv[4]) != "-fd") {
        std::cerr << "Error: Expected '-fd' as fourth argument\n";
        return 1;
    }

    bool use_fd = true;
    double max_current = std::stod(argv[5]);
    int steps = 20;
    int hold_ms = 2000;
    int step_delay = 50;

    std::cout << "Motor Test | send: " << send_can_id
            << " recv: " << recv_can_id
            << " iface: " << can_interface
            << " FD: " << (use_fd ? "yes" : "no")
            << " max_current: " << max_current
            << "\n";

    try {
        std::cout << "=== OpenArm Motor Control Script ===" << std::endl;
        std::cout << "Send CAN ID: " << send_can_id << std::endl;
        std::cout << "Receive CAN ID: " << recv_can_id << std::endl;
        std::cout << "CAN Interface: " << can_interface << std::endl;
        std::cout << "CAN-FD Enabled: " << (use_fd ? "Yes" : "No") << std::endl;
        std::cout << std::endl;

        // Initialize OpenArm with CAN interface
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface,
                                              use_fd);  // Use specified interface and FD setting

        // Initialize single motor
        std::cout << "Initializing motor..." << std::endl;
        openarm.init_arm_motors({openarm::damiao_motor::MotorType::DM8009}, {send_can_id},
                                {recv_can_id});

        // Set callback mode to param for initial parameter reading
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

        // Query motor parameters (Master ID and Baudrate)
        std::cout << "Reading motor parameters..." << std::endl;
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::can_br));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Get motor and verify parameters
        const auto& motors = openarm.get_arm().get_motors();
        if (!motors.empty()) {
            const auto& motor = motors[0];
            double queried_mst_id =
                motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
            double queried_baudrate =
                motor.get_param(static_cast<int>(openarm::damiao_motor::RID::can_br));

            std::cout << "\n=== Motor Parameters ===" << std::endl;
            std::cout << "Send CAN ID: " << motor.get_send_can_id() << std::endl;
            std::cout << "Queried Master ID: " << queried_mst_id << std::endl;
            std::cout << "Queried Baudrate (1-9): " << queried_baudrate << std::endl;

            // Verify recv_can_id matches queried master ID
            if (static_cast<uint32_t>(queried_mst_id) != recv_can_id) {
                std::cerr << "Error: Queried Master ID (" << queried_mst_id
                          << ") does not match provided recv_can_id (" << recv_can_id << ")"
                          << std::endl;
                return 1;
            }
            std::cout << "✓ Master ID verification passed" << std::endl;
        }

        // Open CSV file
        namespace fs = std::filesystem;
        fs::create_directories("./data");
        std::ostringstream filename;
        filename << "data/motor_log.csv";
        std::ofstream csv_file(filename.str());

        auto log_motor = [&](double current) {
            openarm.refresh_all();
            for (const auto& motor : openarm.get_arm().get_motors()) {
                csv_file << motor.get_position() << ","
                         << motor.get_velocity() << ","
                         << current << "\n";
            }
        };

        // Switch to state callback mode for motor status updates
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Enable the motor
        std::cout << "\n=== Enabling Motor ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        
        // Ramp up
        for (int i = 1; i <= steps; i++) {
            double current = max_current * i / steps;
            openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0,0,0,0,current}});
            openarm.recv_all(step_delay * 1000);
            log_motor(current);
        }

        // Hold
        auto hold_start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - hold_start < std::chrono::milliseconds(hold_ms)) {
            openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0,0,0,0,max_current}});
            openarm.recv_all(step_delay * 1000);
            openarm.refresh_all();
            log_motor(max_current);
        }

        // Ramp down
        for (int i = steps; i >= 0; i--) {
            double current = max_current * i / steps;
            openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0,0,0,0,current}});
            openarm.recv_all(step_delay * 1000);
            openarm.refresh_all();
            log_motor(current);
        }

        openarm.disable_all();
        openarm.recv_all(500);
        std::cout << "=== Test Complete ===\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }

    return 0;
}
