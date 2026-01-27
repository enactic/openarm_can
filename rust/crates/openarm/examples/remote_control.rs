//! Remote motor control via xoq P2P CAN bridge.
//!
//! This example demonstrates controlling OpenArm motors over a remote CAN connection
//! using xoq's P2P networking.
//!
//! # Usage
//!
//! First, start the CAN server on the machine with CAN hardware:
//! ```bash
//! cd ~/Documents/wser
//! cargo run --example can_server --features "iroh,can" -- can0
//! ```
//!
//! Then run this client on any machine:
//! ```bash
//! cd ~/Documents/work/openarm_can/rust
//! cargo run --example remote_control --features remote -- <server-id>
//! ```
//!
//! The server-id is printed when the CAN server starts.

use openarm::{CallbackMode, ControlMode, MITParam, MotorType, RemoteOpenArm};
use std::thread;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // Get server ID from command line
    let server_id = std::env::args().nth(1).expect(
        "Usage: remote_control <server-id>\n\n\
         The server-id is printed when running the CAN server:\n\
         cargo run --example can_server --features \"iroh,can\" -- can0",
    );

    println!("Connecting to remote CAN server: {}...", &server_id[..20.min(server_id.len())]);

    // Connect to remote CAN bus
    let mut arm = RemoteOpenArm::new(&server_id, false)?;

    println!("Connected! Initializing motors...");

    // Initialize motors (same API as local OpenArm)
    // Example: 2 arm motors on a typical OpenArm setup
    arm.init_arm_motors(
        &[MotorType::DM4310, MotorType::DM4310],
        &[0x01, 0x02],   // Send CAN IDs
        &[0x11, 0x12],   // Receive CAN IDs
        Some(&[ControlMode::MIT, ControlMode::MIT]),
    )?;

    println!("Motors initialized. Enabling...");

    // Enable motors
    arm.enable_all()?;
    thread::sleep(Duration::from_millis(100));

    // Set callback mode to receive state updates
    arm.set_callback_mode_all(CallbackMode::STATE);

    println!("Motors enabled. Starting control loop...");
    println!("Press Ctrl+C to stop.\n");

    // Simple control loop - send position commands and read state
    let mut iteration = 0;
    loop {
        // Generate a simple sinusoidal position
        let t = iteration as f64 * 0.01;
        let position = 0.5 * (t * 2.0 * std::f64::consts::PI / 2.0).sin();

        // Create MIT control parameters for each motor
        let params = vec![
            MITParam {
                kp: 10.0,
                kd: 1.0,
                q: position,
                dq: 0.0,
                tau: 0.0,
            },
            MITParam {
                kp: 10.0,
                kd: 1.0,
                q: -position, // Mirror motion for second motor
                dq: 0.0,
                tau: 0.0,
            },
        ];

        // Send control commands (same API as local)
        arm.arm().mit_control_all(&params)?;

        // Receive motor state feedback
        let frames_received = arm.recv_all(10_000)?; // 10ms timeout

        if frames_received > 0 {
            // Get motor states
            let motors = arm.arm().get_motors();
            for (i, motor) in motors.iter().enumerate() {
                println!(
                    "Motor {}: pos={:6.3} rad, vel={:6.2} rad/s, torque={:5.2} Nm",
                    i,
                    motor.get_position(),
                    motor.get_velocity(),
                    motor.get_torque()
                );
            }
        }

        iteration += 1;
        thread::sleep(Duration::from_millis(10));

        // Exit after some iterations for demo
        if iteration > 500 {
            break;
        }
    }

    // Disable motors before exiting
    println!("\nDisabling motors...");
    arm.disable_all()?;
    println!("Done!");

    Ok(())
}
