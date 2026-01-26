"""Simple movement test for OpenArm using Rust implementation."""

import time
import openarm_can as oa

# Create OpenArm instance
arm = oa.OpenArm("can0", True)

# Initialize motors
arm.init_arm_motors(
    [oa.MotorType.DM4310, oa.MotorType.DM4310],
    [0x01, 0x02],
    [0x11, 0x12]
)
arm.init_gripper_motor(oa.MotorType.DM4310, 0x07, 0x17)

print("Motors initialized")

# Enable all motors
arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
arm.enable_all()
arm.recv_all(2000)
arm.set_callback_mode_all(oa.CallbackMode.STATE)
print("Motors enabled")

# MIT control parameters
KP = 3.0   # Position gain
KD = 0.5   # Damping gain

def move_to(positions, duration=1.0, steps=50):
    """Move arm to target positions over duration."""
    for i in range(steps):
        arm.get_arm().mit_control_all([
            oa.MITParam(KP, KD, positions[0], 0, 0),
            oa.MITParam(KP, KD, positions[1], 0, 0)
        ])
        arm.get_gripper().mit_control_all([
            oa.MITParam(KP, KD, positions[2], 0, 0)
        ])
        arm.recv_all(2000)
        time.sleep(duration / steps)

    # Print final positions
    motors = arm.get_arm().get_motors()
    gripper = arm.get_gripper().get_motor()
    print(f"  Arm: [{motors[0].get_position():.3f}, {motors[1].get_position():.3f}], Gripper: {gripper.get_position():.3f}")

print("\n=== Movement Test ===\n")

# Move to position A (small offset)
print("Moving to position A (+0.3, -0.2, 0)...")
move_to([0.3, -0.2, 0.0], duration=1.0)

time.sleep(0.5)

# Move to position B
print("Moving to position B (-0.2, +0.3, -0.5)...")
move_to([-0.2, 0.3, -0.5], duration=1.0)

time.sleep(0.5)

# Move to position C
print("Moving to position C (+0.2, +0.2, -0.8)...")
move_to([0.2, 0.2, -0.8], duration=1.0)

time.sleep(0.5)

# Return to zero
print("Returning to zero...")
move_to([0.0, 0.0, 0.0], duration=1.0)

time.sleep(0.5)

# Disable motors
arm.disable_all()
arm.recv_all(2000)
print("\nMotors disabled. Test complete!")
