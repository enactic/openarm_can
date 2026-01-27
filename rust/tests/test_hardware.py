"""
Hardware compliance test for openarm_can Python bindings.

Tests all CAN bus operations that require actual hardware.
Run with: python tests/test_hardware.py

REQUIRES: CAN interface (can0) with motors connected.
"""

import time
import openarm_can as oa


# Configuration - adjust these to match your hardware setup
CAN_INTERFACE = "can0"
ENABLE_FD = True
ARM_MOTOR_TYPES = [oa.MotorType.DM4310, oa.MotorType.DM4310]
ARM_SEND_IDS = [0x01, 0x02]
ARM_RECV_IDS = [0x11, 0x12]
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x07
GRIPPER_RECV_ID = 0x17


class TestOpenArm:
    """Test OpenArm high-level interface."""

    def __init__(self):
        self.arm = None

    def setup(self):
        """Initialize OpenArm instance."""
        self.arm = oa.OpenArm(CAN_INTERFACE, ENABLE_FD)
        self.arm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
        self.arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
        print(f"  OpenArm initialized: {self.arm}")

    def teardown(self):
        """Cleanup."""
        if self.arm:
            try:
                self.arm.disable_all()
            except:
                pass
        self.arm = None

    def test_openarm_creation(self):
        """Test OpenArm can be created and initialized."""
        self.setup()
        try:
            assert self.arm is not None
            assert self.arm.enable_fd == ENABLE_FD
            print(f"  OpenArm creation OK")
        finally:
            self.teardown()

    def test_get_arm_component(self):
        """Test get_arm returns ArmComponent."""
        self.setup()
        try:
            arm_comp = self.arm.get_arm()
            assert arm_comp is not None
            assert arm_comp.motor_count() == len(ARM_MOTOR_TYPES)
            print(f"  get_arm: {arm_comp} OK")
        finally:
            self.teardown()

    def test_get_gripper_component(self):
        """Test get_gripper returns GripperComponent."""
        self.setup()
        try:
            gripper = self.arm.get_gripper()
            assert gripper is not None
            print(f"  get_gripper: {gripper} OK")
        finally:
            self.teardown()

    def test_enable_disable_all(self):
        """Test enable_all and disable_all."""
        self.setup()
        try:
            self.arm.set_callback_mode_all(oa.CallbackMode.IGNORE)

            # Enable
            self.arm.enable_all()
            self.arm.recv_all()
            print(f"  enable_all OK")

            # Disable
            self.arm.disable_all()
            self.arm.recv_all()
            print(f"  disable_all OK")
        finally:
            self.teardown()

    def test_refresh_all(self):
        """Test refresh_all."""
        self.setup()
        try:
            self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
            self.arm.refresh_all()
            count = self.arm.recv_all()
            assert count >= 0
            print(f"  refresh_all: received {count} frames OK")
        finally:
            self.teardown()

    def test_refresh_one(self):
        """Test refresh_one."""
        self.setup()
        try:
            self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
            self.arm.refresh_one(0)
            count = self.arm.recv_all()
            assert count >= 0
            print(f"  refresh_one(0): received {count} frames OK")
        finally:
            self.teardown()

    def test_recv_all(self):
        """Test recv_all with timeout."""
        self.setup()
        try:
            self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
            self.arm.refresh_all()

            # Test with different timeouts
            count1 = self.arm.recv_all(500)  # 500us
            count2 = self.arm.recv_all(1000)  # 1000us

            print(f"  recv_all(500): {count1}, recv_all(1000): {count2} OK")
        finally:
            self.teardown()

    def test_set_callback_mode_all(self):
        """Test set_callback_mode_all."""
        self.setup()
        try:
            # Test all callback modes
            self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
            print(f"  set_callback_mode_all(STATE) OK")

            self.arm.set_callback_mode_all(oa.CallbackMode.PARAM)
            print(f"  set_callback_mode_all(PARAM) OK")

            self.arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
            print(f"  set_callback_mode_all(IGNORE) OK")
        finally:
            self.teardown()

    def test_query_param_all(self):
        """Test query_param_all."""
        self.setup()
        try:
            self.arm.set_callback_mode_all(oa.CallbackMode.PARAM)
            self.arm.query_param_all(oa.MotorVariable.PMAX)
            count = self.arm.recv_all()
            print(f"  query_param_all(PMAX): received {count} frames OK")
        finally:
            self.teardown()


class TestArmComponent:
    """Test ArmComponent interface."""

    def __init__(self):
        self.openarm = None

    def setup(self):
        """Initialize OpenArm and get arm component."""
        self.openarm = oa.OpenArm(CAN_INTERFACE, ENABLE_FD)
        self.openarm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
        self.openarm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

    def teardown(self):
        """Cleanup."""
        if self.openarm:
            try:
                self.openarm.disable_all()
            except:
                pass
        self.openarm = None

    def test_get_motors(self):
        """Test get_motors returns motor list."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            motors = arm.get_motors()
            assert len(motors) == len(ARM_MOTOR_TYPES)
            for i, motor in enumerate(motors):
                assert motor.motor_type == ARM_MOTOR_TYPES[i]
                assert motor.send_can_id == ARM_SEND_IDS[i]
                assert motor.recv_can_id == ARM_RECV_IDS[i]
                print(f"    Motor {i}: {motor}")
            print(f"  get_motors OK")
        finally:
            self.teardown()

    def test_get_motor_by_index(self):
        """Test get_motor by index."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            motor0 = arm.get_motor(0)
            motor1 = arm.get_motor(1)
            assert motor0.send_can_id == ARM_SEND_IDS[0]
            assert motor1.send_can_id == ARM_SEND_IDS[1]
            print(f"  get_motor(0): {motor0} OK")
            print(f"  get_motor(1): {motor1} OK")
        finally:
            self.teardown()

    def test_motor_count(self):
        """Test motor_count."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            assert arm.motor_count() == len(ARM_MOTOR_TYPES)
            print(f"  motor_count: {arm.motor_count()} OK")
        finally:
            self.teardown()

    def test_enable_disable(self):
        """Test enable_all and disable_all."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.IGNORE)

            arm.enable_all()
            arm.recv_all()
            time.sleep(0.01)
            print(f"  arm.enable_all OK")

            arm.disable_all()
            arm.recv_all()
            time.sleep(0.01)
            print(f"  arm.disable_all OK")
        finally:
            self.teardown()

    def test_refresh_all(self):
        """Test refresh_all."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.STATE)
            arm.refresh_all()
            count = arm.recv_all()
            print(f"  arm.refresh_all: received {count} frames OK")
        finally:
            self.teardown()

    def test_refresh_one(self):
        """Test refresh_one."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.STATE)
            arm.refresh_one(0)
            count = arm.recv_all()
            print(f"  arm.refresh_one(0): received {count} frames OK")
        finally:
            self.teardown()

    def test_mit_control_one(self):
        """Test mit_control_one."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.STATE)
            arm.enable_all()
            arm.recv_all()
            time.sleep(0.01)

            # Send zero torque command
            param = oa.MITParam(0, 0, 0, 0, 0)
            arm.mit_control_one(0, param)
            arm.recv_all()
            time.sleep(0.01)
            print(f"  arm.mit_control_one(0, zero) OK")

            arm.disable_all()
            arm.recv_all()
        finally:
            self.teardown()

    def test_mit_control_all(self):
        """Test mit_control_all."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.STATE)
            arm.enable_all()
            arm.recv_all()
            time.sleep(0.01)

            # Send zero torque to all
            params = [oa.MITParam(0, 0, 0, 0, 0) for _ in range(len(ARM_MOTOR_TYPES))]
            arm.mit_control_all(params)
            arm.recv_all()
            time.sleep(0.01)
            print(f"  arm.mit_control_all OK")

            arm.disable_all()
            arm.recv_all()
        finally:
            self.teardown()

    def test_query_param_one(self):
        """Test query_param_one."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.PARAM)
            arm.query_param_one(0, oa.MotorVariable.PMAX)
            count = arm.recv_all()
            print(f"  arm.query_param_one(0, PMAX): received {count} frames OK")
        finally:
            self.teardown()

    def test_query_param_all(self):
        """Test query_param_all."""
        self.setup()
        try:
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.PARAM)
            arm.query_param_all(oa.MotorVariable.VMAX)
            count = arm.recv_all()
            print(f"  arm.query_param_all(VMAX): received {count} frames OK")
        finally:
            self.teardown()

    def test_motor_state_read(self):
        """Test reading motor state after refresh."""
        self.setup()
        try:
            time.sleep(0.02)  # Allow buffer to clear
            arm = self.openarm.get_arm()
            arm.set_callback_mode_all(oa.CallbackMode.STATE)
            arm.refresh_all()
            arm.recv_all()

            motors = arm.get_motors()
            for i, motor in enumerate(motors):
                pos = motor.get_position()
                vel = motor.get_velocity()
                tau = motor.get_torque()
                print(f"    Motor {i}: pos={pos:.4f}, vel={vel:.4f}, tau={tau:.4f}")

            print(f"  motor state read OK")
        finally:
            self.teardown()


class TestGripperComponent:
    """Test GripperComponent interface."""

    def __init__(self):
        self.openarm = None

    def setup(self):
        """Initialize OpenArm and get gripper component."""
        self.openarm = oa.OpenArm(CAN_INTERFACE, ENABLE_FD)
        self.openarm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
        self.openarm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

    def teardown(self):
        """Cleanup."""
        if self.openarm:
            try:
                self.openarm.disable_all()
            except:
                pass
        self.openarm = None

    def test_get_motor(self):
        """Test get_motor returns gripper motor."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            motor = gripper.get_motor()
            assert motor.motor_type == GRIPPER_MOTOR_TYPE
            assert motor.send_can_id == GRIPPER_SEND_ID
            assert motor.recv_can_id == GRIPPER_RECV_ID
            print(f"  gripper.get_motor: {motor} OK")
        finally:
            self.teardown()

    def test_get_motors(self):
        """Test get_motors returns motor list."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            motors = gripper.get_motors()
            assert len(motors) == 1
            print(f"  gripper.get_motors: {len(motors)} motor(s) OK")
        finally:
            self.teardown()

    def test_enable_disable(self):
        """Test enable_all and disable_all."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            gripper.set_callback_mode_all(oa.CallbackMode.IGNORE)

            gripper.enable_all()
            gripper.recv_all()
            print(f"  gripper.enable_all OK")

            gripper.disable_all()
            gripper.recv_all()
            print(f"  gripper.disable_all OK")
        finally:
            self.teardown()

    def test_refresh_all(self):
        """Test refresh_all."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            gripper.set_callback_mode_all(oa.CallbackMode.STATE)
            gripper.refresh_all()
            count = gripper.recv_all()
            print(f"  gripper.refresh_all: received {count} frames OK")
        finally:
            self.teardown()

    def test_mit_control_one(self):
        """Test mit_control_one."""
        self.setup()
        try:
            time.sleep(0.02)  # Allow buffer to clear
            gripper = self.openarm.get_gripper()
            gripper.set_callback_mode_all(oa.CallbackMode.STATE)
            gripper.enable_all()
            gripper.recv_all()
            time.sleep(0.01)

            # Send zero torque
            param = oa.MITParam(0, 0, 0, 0, 0)
            gripper.mit_control_one(0, param)
            gripper.recv_all()
            time.sleep(0.01)
            print(f"  gripper.mit_control_one(0, zero) OK")

            gripper.disable_all()
            gripper.recv_all()
        finally:
            self.teardown()

    def test_mit_control_all(self):
        """Test mit_control_all."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            gripper.set_callback_mode_all(oa.CallbackMode.STATE)
            gripper.enable_all()
            gripper.recv_all()

            # Send zero torque
            params = [oa.MITParam(0, 0, 0, 0, 0)]
            gripper.mit_control_all(params)
            gripper.recv_all()
            print(f"  gripper.mit_control_all OK")

            gripper.disable_all()
            gripper.recv_all()
        finally:
            self.teardown()

    def test_set_limit(self):
        """Test set_limit."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            gripper.set_limit(10.0, 0.5)
            print(f"  gripper.set_limit(10.0, 0.5) OK")
        finally:
            self.teardown()

    def test_motor_state_read(self):
        """Test reading motor state."""
        self.setup()
        try:
            gripper = self.openarm.get_gripper()
            gripper.set_callback_mode_all(oa.CallbackMode.STATE)
            gripper.refresh_all()
            gripper.recv_all()

            motor = gripper.get_motor()
            pos = motor.get_position()
            vel = motor.get_velocity()
            tau = motor.get_torque()
            print(f"    Gripper: pos={pos:.4f}, vel={vel:.4f}, tau={tau:.4f}")
            print(f"  gripper motor state read OK")
        finally:
            self.teardown()


class TestCANSocket:
    """Test CANSocket low-level interface."""

    def test_socket_creation(self):
        """Test CANSocket can be created."""
        socket = oa.CANSocket(CAN_INTERFACE, ENABLE_FD, 100)
        assert socket.is_open()
        assert socket.interface == CAN_INTERFACE
        assert socket.enable_fd == ENABLE_FD
        print(f"  CANSocket: {socket} OK")
        socket.close()

    def test_socket_close(self):
        """Test CANSocket close."""
        socket = oa.CANSocket(CAN_INTERFACE, ENABLE_FD, 100)
        assert socket.is_open()
        socket.close()
        assert not socket.is_open()
        print(f"  CANSocket close OK")

    def test_socket_reinitialize(self):
        """Test CANSocket reinitialize."""
        socket = oa.CANSocket(CAN_INTERFACE, ENABLE_FD, 100)
        socket.close()
        socket.initialize_socket()
        assert socket.is_open()
        print(f"  CANSocket reinitialize OK")
        socket.close()

    def test_set_recv_timeout(self):
        """Test set_recv_timeout."""
        socket = oa.CANSocket(CAN_INTERFACE, ENABLE_FD, 100)
        socket.set_recv_timeout(500)
        socket.set_recv_timeout(1000)
        print(f"  set_recv_timeout OK")
        socket.close()

    def test_is_data_available(self):
        """Test is_data_available."""
        socket = oa.CANSocket(CAN_INTERFACE, ENABLE_FD, 100)
        # Just test it doesn't crash, result depends on bus activity
        available = socket.is_data_available(100)
        print(f"  is_data_available: {available} OK")
        socket.close()

    def test_write_read_can_frame(self):
        """Test write and read CAN frame."""
        socket = oa.CANSocket(CAN_INTERFACE, False, 1000)  # Standard CAN

        # Write a frame
        frame = oa.CanFrame(0x7FF, [0x01, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00])
        socket.write_can_frame(frame)
        print(f"  write_can_frame OK")

        # Try to read (may or may not get response)
        response = socket.read_can_frame()
        print(f"  read_can_frame: {response} OK")

        socket.close()

    def test_write_read_canfd_frame(self):
        """Test write and read CAN-FD frame."""
        if not ENABLE_FD:
            print(f"  Skipping CAN-FD test (FD not enabled)")
            return

        socket = oa.CANSocket(CAN_INTERFACE, True, 1000)  # CAN-FD

        # Write a frame
        frame = oa.CanFdFrame(0x7FF, [0x01, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00], 0)
        socket.write_canfd_frame(frame)
        print(f"  write_canfd_frame OK")

        # Try to read
        response = socket.read_canfd_frame()
        print(f"  read_canfd_frame: {response} OK")

        socket.close()


def run_all_tests():
    """Run all hardware compliance tests."""
    test_classes = [
        ("CANSocket", TestCANSocket),
        ("OpenArm", TestOpenArm),
        ("ArmComponent", TestArmComponent),
        ("GripperComponent", TestGripperComponent),
    ]

    total_passed = 0
    total_failed = 0

    for name, test_class in test_classes:
        print(f"\n{'='*60}")
        print(f"Testing: {name}")
        print('='*60)

        instance = test_class()
        methods = [m for m in dir(instance) if m.startswith('test_')]

        for method_name in methods:
            method = getattr(instance, method_name)
            try:
                method()
                total_passed += 1
            except Exception as e:
                print(f"  FAILED: {method_name}")
                print(f"    Error: {e}")
                total_failed += 1

    print(f"\n{'='*60}")
    print(f"HARDWARE TEST SUMMARY: {total_passed} passed, {total_failed} failed")
    print('='*60)

    return total_failed == 0


if __name__ == "__main__":
    import sys

    print("OpenArm CAN Hardware Compliance Tests")
    print("=====================================")
    print(f"Interface: {CAN_INTERFACE}")
    print(f"CAN-FD: {ENABLE_FD}")
    print(f"Arm motors: {len(ARM_MOTOR_TYPES)}")
    print(f"Gripper motor: {GRIPPER_MOTOR_TYPE}")
    print()

    success = run_all_tests()
    sys.exit(0 if success else 1)
