"""
Compliance test for openarm_can Python bindings.

Tests all public API functions and classes to ensure they work correctly.
Most tests do not require CAN hardware.
"""

import openarm_can as oa


class TestEnums:
    """Test all enum types and their values."""

    def test_motor_type_values(self):
        """Test MotorType enum has all expected variants."""
        expected = [
            ("DM3507", 0),
            ("DM4310", 1),
            ("DM4310_48V", 2),
            ("DM4340", 3),
            ("DM4340_48V", 4),
            ("DM6006", 5),
            ("DM8006", 6),
            ("DM8009", 7),
            ("DM10010L", 8),
            ("DM10010", 9),
            ("DMH3510", 10),
            ("DMH6215", 11),
            ("DMG6220", 12),
        ]
        for name, value in expected:
            motor_type = getattr(oa.MotorType, name)
            assert int(motor_type) == value, f"MotorType.{name} should be {value}, got {int(motor_type)}"
            print(f"  MotorType.{name} = {value} OK")

    def test_control_mode_values(self):
        """Test ControlMode enum has all expected variants."""
        expected = [
            ("MIT", 1),
            ("POS_VEL", 2),
            ("VEL", 3),
            ("POS_FORCE", 4),
        ]
        for name, value in expected:
            mode = getattr(oa.ControlMode, name)
            assert int(mode) == value, f"ControlMode.{name} should be {value}"
            print(f"  ControlMode.{name} = {value} OK")

    def test_callback_mode_values(self):
        """Test CallbackMode enum has all expected variants."""
        expected = [
            ("STATE", 0),
            ("PARAM", 1),
            ("IGNORE", 2),
        ]
        for name, value in expected:
            mode = getattr(oa.CallbackMode, name)
            assert int(mode) == value, f"CallbackMode.{name} should be {value}"
            print(f"  CallbackMode.{name} = {value} OK")

    def test_motor_variable_values(self):
        """Test MotorVariable enum has expected variants."""
        # Test a subset of important variables
        expected = [
            ("UV_Value", 0),
            ("KT_Value", 1),
            ("ACC", 4),
            ("DEC", 5),
            ("MAX_SPD", 6),
            ("MST_ID", 7),
            ("ESC_ID", 8),
            ("TIMEOUT", 9),
            ("CTRL_MODE", 10),
            ("PMAX", 21),
            ("VMAX", 22),
            ("TMAX", 23),
            ("run_state", 56),
            ("error_state", 80),
            ("CUR_angle", 81),
        ]
        for name, value in expected:
            var = getattr(oa.MotorVariable, name)
            assert int(var) == value, f"MotorVariable.{name} should be {value}"
            print(f"  MotorVariable.{name} = {value} OK")


class TestDataStructures:
    """Test all data structure classes."""

    def test_limit_param(self):
        """Test LimitParam creation and field access."""
        param = oa.LimitParam(12.5, 30.0, 10.0)
        assert param.p_max == 12.5, f"p_max should be 12.5, got {param.p_max}"
        assert param.v_max == 30.0, f"v_max should be 30.0, got {param.v_max}"
        assert param.t_max == 10.0, f"t_max should be 10.0, got {param.t_max}"

        # Test setters
        param.p_max = 15.0
        param.v_max = 25.0
        param.t_max = 8.0
        assert param.p_max == 15.0
        assert param.v_max == 25.0
        assert param.t_max == 8.0

        # Test repr
        repr_str = repr(param)
        assert "LimitParam" in repr_str
        print(f"  LimitParam: {repr_str} OK")

    def test_mit_param(self):
        """Test MITParam creation and field access."""
        param = oa.MITParam(2.0, 0.5, 1.0, 0.1, 0.05)
        assert param.kp == 2.0
        assert param.kd == 0.5
        assert param.q == 1.0
        assert param.dq == 0.1
        assert param.tau == 0.05

        # Test default values
        param_default = oa.MITParam()
        assert param_default.kp == 0.0
        assert param_default.kd == 0.0

        # Test setters
        param.kp = 3.0
        assert param.kp == 3.0

        print(f"  MITParam: {repr(param)} OK")

    def test_pos_vel_param(self):
        """Test PosVelParam creation and field access."""
        param = oa.PosVelParam(1.5, 2.5)
        assert param.q == 1.5
        assert param.dq == 2.5

        # Test default values
        param_default = oa.PosVelParam()
        assert param_default.q == 0.0
        assert param_default.dq == 0.0

        print(f"  PosVelParam: {repr(param)} OK")

    def test_pos_force_param(self):
        """Test PosForceParam creation and field access."""
        param = oa.PosForceParam(1.0, 2.0, 3.0)
        assert param.q == 1.0
        assert param.dq == 2.0
        assert param.i == 3.0

        print(f"  PosForceParam: {repr(param)} OK")

    def test_motor_state_result(self):
        """Test MotorStateResult creation and field access."""
        result = oa.MotorStateResult(1.5, 2.5, 0.5, 45, 50, True)
        assert result.position == 1.5
        assert result.velocity == 2.5
        assert result.torque == 0.5
        assert result.t_mos == 45
        assert result.t_rotor == 50
        assert result.valid == True

        # Test default values
        result_default = oa.MotorStateResult()
        assert result_default.position == 0.0
        assert result_default.valid == False

        print(f"  MotorStateResult: {repr(result)} OK")

    def test_param_result(self):
        """Test ParamResult creation and field access."""
        result = oa.ParamResult(10, 123.456, True)
        assert result.rid == 10
        assert result.value == 123.456
        assert result.valid == True

        print(f"  ParamResult: {repr(result)} OK")

    def test_can_frame(self):
        """Test CanFrame creation and field access."""
        frame = oa.CanFrame(0x100, [1, 2, 3, 4, 5, 6, 7, 8])
        assert frame.can_id == 0x100
        assert list(frame.data) == [1, 2, 3, 4, 5, 6, 7, 8]

        print(f"  CanFrame: {repr(frame)} OK")

    def test_can_fd_frame(self):
        """Test CanFdFrame creation and field access."""
        frame = oa.CanFdFrame(0x200, [1, 2, 3, 4], 0x01)
        assert frame.can_id == 0x200
        assert list(frame.data) == [1, 2, 3, 4]
        assert frame.flags == 0x01

        # Test default flags
        frame_default = oa.CanFdFrame(0x200, [1, 2, 3, 4])
        assert frame_default.flags == 0

        print(f"  CanFdFrame: {repr(frame)} OK")

    def test_can_packet(self):
        """Test CANPacket creation and field access."""
        packet = oa.CANPacket(0x01, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        assert packet.send_can_id == 0x01
        assert list(packet.data) == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]

        print(f"  CANPacket: {repr(packet)} OK")


class TestMotor:
    """Test Motor class."""

    def test_motor_creation(self):
        """Test Motor creation with different parameters."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        assert motor.motor_type == oa.MotorType.DM4310
        assert motor.send_can_id == 0x01
        assert motor.recv_can_id == 0x11
        assert motor.control_mode == oa.ControlMode.MIT  # default

        print(f"  Motor default control mode: {repr(motor)} OK")

    def test_motor_with_control_mode(self):
        """Test Motor creation with explicit control mode."""
        motor = oa.Motor(oa.MotorType.DM4340, 0x02, 0x12, oa.ControlMode.POS_VEL)
        assert motor.motor_type == oa.MotorType.DM4340
        assert motor.control_mode == oa.ControlMode.POS_VEL

        print(f"  Motor with POS_VEL: {repr(motor)} OK")

    def test_motor_state_methods(self):
        """Test Motor state getter methods."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)

        # Initial state should be zeros
        assert motor.get_position() == 0.0
        assert motor.get_velocity() == 0.0
        assert motor.get_torque() == 0.0
        assert motor.get_state_tmos() == 0
        assert motor.get_state_trotor() == 0
        assert motor.is_enabled() == False

        print(f"  Motor state methods OK")

    def test_motor_all_types(self):
        """Test Motor creation with all motor types."""
        motor_types = [
            oa.MotorType.DM3507,
            oa.MotorType.DM4310,
            oa.MotorType.DM4310_48V,
            oa.MotorType.DM4340,
            oa.MotorType.DM4340_48V,
            oa.MotorType.DM6006,
            oa.MotorType.DM8006,
            oa.MotorType.DM8009,
            oa.MotorType.DM10010L,
            oa.MotorType.DM10010,
            oa.MotorType.DMH3510,
            oa.MotorType.DMH6215,
            oa.MotorType.DMG6220,
        ]
        for mt in motor_types:
            motor = oa.Motor(mt, 0x01, 0x11)
            assert motor.motor_type == mt
            print(f"    Motor type {mt} OK")

        print(f"  All motor types OK")


class TestEncoder:
    """Test CanPacketEncoder class."""

    def test_encoder_creation(self):
        """Test CanPacketEncoder can be created."""
        encoder = oa.CanPacketEncoder()
        assert encoder is not None
        print(f"  CanPacketEncoder creation OK")

    def test_enable_command(self):
        """Test create_enable_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_enable_command(motor)

        assert packet.send_can_id == 0x01
        assert len(packet.data) == 8
        # Enable command ends with 0xFC
        assert packet.data[-1] == 0xFC

        print(f"  create_enable_command: {repr(packet)} OK")

    def test_disable_command(self):
        """Test create_disable_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_disable_command(motor)

        assert packet.send_can_id == 0x01
        assert len(packet.data) == 8
        # Disable command ends with 0xFD
        assert packet.data[-1] == 0xFD

        print(f"  create_disable_command: {repr(packet)} OK")

    def test_set_zero_command(self):
        """Test create_set_zero_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_set_zero_command(motor)

        assert packet.send_can_id == 0x01
        assert len(packet.data) == 8
        # Set zero command ends with 0xFE
        assert packet.data[-1] == 0xFE

        print(f"  create_set_zero_command: {repr(packet)} OK")

    def test_refresh_command(self):
        """Test create_refresh_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_refresh_command(motor)

        # Refresh uses broadcast ID 0x7FF
        assert packet.send_can_id == 0x7FF
        assert len(packet.data) == 8

        print(f"  create_refresh_command: {repr(packet)} OK")

    def test_mit_control_command(self):
        """Test create_mit_control_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        param = oa.MITParam(2.0, 0.5, 0.0, 0.0, 0.0)
        packet = oa.CanPacketEncoder.create_mit_control_command(motor, param)

        assert packet.send_can_id == 0x01
        assert len(packet.data) == 8

        print(f"  create_mit_control_command: {repr(packet)} OK")

    def test_posvel_control_command(self):
        """Test create_posvel_control_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11, oa.ControlMode.POS_VEL)
        param = oa.PosVelParam(1.0, 2.0)
        packet = oa.CanPacketEncoder.create_posvel_control_command(motor, param)

        # POS_VEL mode uses CAN ID offset 0x100
        assert packet.send_can_id == 0x101
        assert len(packet.data) == 8

        print(f"  create_posvel_control_command: {repr(packet)} OK")

    def test_posforce_control_command(self):
        """Test create_posforce_control_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11, oa.ControlMode.POS_FORCE)
        param = oa.PosForceParam(1.0, 2.0, 0.5)
        packet = oa.CanPacketEncoder.create_posforce_control_command(motor, param)

        # POS_FORCE mode uses CAN ID offset 0x300
        assert packet.send_can_id == 0x301
        assert len(packet.data) == 8

        print(f"  create_posforce_control_command: {repr(packet)} OK")

    def test_set_control_mode_command(self):
        """Test create_set_control_mode_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_set_control_mode_command(motor, oa.ControlMode.POS_VEL)

        assert packet.send_can_id == 0x7FF
        assert len(packet.data) == 8

        print(f"  create_set_control_mode_command: {repr(packet)} OK")

    def test_query_param_command(self):
        """Test create_query_param_command."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)
        packet = oa.CanPacketEncoder.create_query_param_command(motor, oa.MotorVariable.PMAX)

        assert packet.send_can_id == 0x7FF
        assert len(packet.data) == 8

        print(f"  create_query_param_command: {repr(packet)} OK")


class TestDecoder:
    """Test CanPacketDecoder class."""

    def test_decoder_creation(self):
        """Test CanPacketDecoder can be created."""
        decoder = oa.CanPacketDecoder()
        assert decoder is not None
        print(f"  CanPacketDecoder creation OK")

    def test_parse_motor_state_data(self):
        """Test parse_motor_state_data."""
        motor = oa.Motor(oa.MotorType.DM4310, 0x01, 0x11)

        # Create a valid state response (8 bytes)
        # Format: [id, pos_h, pos_l, vel_h, vel_l, tau_h, tau_l, temp]
        data = [0x01, 0x7F, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x28]

        result = oa.CanPacketDecoder.parse_motor_state_data(motor, data)
        assert isinstance(result, oa.MotorStateResult)
        assert result.valid == True

        print(f"  parse_motor_state_data: {repr(result)} OK")

    def test_parse_motor_param_data(self):
        """Test parse_motor_param_data."""
        # Create a valid param response (8 bytes)
        # Format depends on the parameter type
        data = [0x00, 0x15, 0x00, 0x00, 0x41, 0x48, 0x00, 0x00]

        result = oa.CanPacketDecoder.parse_motor_param_data(data)
        assert isinstance(result, oa.ParamResult)

        print(f"  parse_motor_param_data: {repr(result)} OK")


class TestCANDevice:
    """Test CANDevice class."""

    def test_can_device_creation(self):
        """Test CANDevice creation."""
        device = oa.CANDevice(0x01, 0x11)
        assert device.send_can_id == 0x01
        assert device.recv_can_id == 0x11

        print(f"  CANDevice: {repr(device)} OK")


class TestExceptions:
    """Test exception handling."""

    def test_can_socket_exception_exists(self):
        """Test CANSocketException is accessible."""
        assert hasattr(oa, 'CANSocketException')
        print(f"  CANSocketException exists OK")

    def test_can_socket_exception_raised(self):
        """Test CANSocketException is raised for invalid interface."""
        try:
            socket = oa.CANSocket("nonexistent_interface", False, 100)
            assert False, "Should have raised CANSocketException"
        except oa.CANSocketException as e:
            assert "nonexistent_interface" in str(e) or "No such device" in str(e)
            print(f"  CANSocketException raised correctly: {e}")

    def test_openarm_exception_on_invalid_interface(self):
        """Test OpenArm raises exception for invalid interface."""
        try:
            arm = oa.OpenArm("nonexistent_can", False)
            assert False, "Should have raised CANSocketException"
        except oa.CANSocketException as e:
            print(f"  OpenArm CANSocketException raised correctly: {e}")


class TestModuleAttributes:
    """Test module-level attributes."""

    def test_all_classes_exported(self):
        """Test all expected classes are exported."""
        expected_classes = [
            "MotorType",
            "ControlMode",
            "CallbackMode",
            "MotorVariable",
            "LimitParam",
            "MITParam",
            "PosVelParam",
            "PosForceParam",
            "MotorStateResult",
            "ParamResult",
            "CanFrame",
            "CanFdFrame",
            "CANPacket",
            "Motor",
            "CANSocket",
            "CANDevice",
            "OpenArm",
            "ArmComponent",
            "GripperComponent",
            "CanPacketEncoder",
            "CanPacketDecoder",
            "CANSocketException",
        ]

        for cls_name in expected_classes:
            assert hasattr(oa, cls_name), f"Missing class: {cls_name}"
            print(f"    {cls_name} exported OK")

        print(f"  All {len(expected_classes)} classes exported OK")


def run_all_tests():
    """Run all compliance tests."""
    test_classes = [
        ("Enums", TestEnums),
        ("Data Structures", TestDataStructures),
        ("Motor", TestMotor),
        ("Encoder", TestEncoder),
        ("Decoder", TestDecoder),
        ("CANDevice", TestCANDevice),
        ("Exceptions", TestExceptions),
        ("Module Attributes", TestModuleAttributes),
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
    print(f"SUMMARY: {total_passed} passed, {total_failed} failed")
    print('='*60)

    return total_failed == 0


if __name__ == "__main__":
    import sys
    success = run_all_tests()
    sys.exit(0 if success else 1)
