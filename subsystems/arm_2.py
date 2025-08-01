from typing import Callable
from math import cos, pi, acos, atan2, sin

from ntcore import NetworkTableInstance

from commands2 import Subsystem

from wpilib import (
    RobotBase,
    SmartDashboard,
    Mechanism2d,
    Color8Bit,
)

from wpilib.simulation import RoboRioSim, SingleJointedArmSim

from wpimath.geometry import Rotation2d, Transform2d
from wpimath.system.plant import DCMotor
from wpimath.units import meters, amperes, inchesToMeters
from wpimath.controller import ArmFeedforward

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    MotorOutputConfigs,
    FeedbackConfigs,
    Slot0Configs,
    Slot1Configs,
    Slot2Configs,
    CANcoderConfiguration,
    MagnetSensorConfigs,
    CurrentLimitsConfigs,
)

from commands2 import Command, InstantCommand

from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.controls import Follower, PositionVoltage
from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
    GravityTypeValue,
    SensorDirectionValue,
)


class JointConstants:
    START_SETPOINT: Rotation2d = Rotation2d(0)  # the starting angle of the arm

    MOTOR_IDs: list[int] = [
        10
    ]  # the CAN IDs of each motor, the first is the master, all others are followers
    MASTER_INVERSION: bool = (
        False  # whether to invert the master motor, all others will be inverted relative to this. Default is CW Positive
    )
    INVERSIONS_TO_MASTER: list[bool] = [
        False,
    ]  # at [0] is master, its value is ignored, all others are whether to invert the motor relative to the master
    SENSOR_TO_MECHANISM_RATIO: float = (
        1.0  # the ratio of the sensor's rotations to the mechanism's rotations
    )
    BRAKE_ENABLED: bool = (
        True  # whether to set brake as the default neutral mode for the motors
    )
    CURRENT_LIMIT: amperes = 40  # the stator limit applied to the motors
    CLOSED_LOOP_CONFIG: Slot0Configs = (
        Slot0Configs()
        .with_k_p(0.0)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_g(0.0)
        .with_k_s(0.0)
        .with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        .with_gravity_type(GravityTypeValue.ARM_COSINE)
    )  # configuration for the closed loop control of the arm
    SLOT_1_CONFIG: Slot1Configs | None = (
        None  # optional configuration for alternate hardware closed loop configurations, used for advanced controls based on changing arm properties
    )
    SLOT_2_CONFIG: Slot2Configs | None = (
        None  # optional configuration for alternate hardware closed loop configuration, used for advanced controls based on changing arm properties
    )
    CANCODER_ID: int | None = None  # ID for a remote cancoder, if used
    CANCODER_INVERTED: bool | None = True  # whether the cancoder is inverted, if used
    CANCODER_OFFSET_ROTATIONS: float = (
        0.0  # the offset of the cancoder in rotations, if used
    )
    MOTOR_TYPE: DCMotor = DCMotor.krakenX60(
        1
    )  # The kind and quantity of motors controlling the system. Used exclusively for simulation

    LENGTH: meters = inchesToMeters(
        30
    )  # the length from the pivot to the end of the arm in meters
    MOI: float = 0.25  # Moment of intertia in kg*m^2. This should come from CAD
    MIN_ANGLE: Rotation2d = Rotation2d.fromDegrees(
        -120
    )  # the minimum angle of the arm. Used for simulation and to limit the arm's setpoints
    MAX_ANGLE: Rotation2d = Rotation2d.fromDegrees(
        120
    )  # the maximum angle of the arm. Used for simulation and to limit the arm's setpoints

    # allow for each value to be set to a different value than default, only optionally
    def __init__(
        self,
        *,
        START_SETPOINT: Rotation2d | None = None,
        MOTOR_IDs: list[int] | None = None,
        MASTER_INVERSION: bool | None = None,
        INVERSIONS_TO_MASTER: list[bool] | None = None,
        MOTOR_TO_SENSOR_RATIO: float | None = None,
        SENSOR_TO_MECHANISM_RATIO: float | None = None,
        BRAKE_ENABLED: bool | None = None,
        CURRENT_LIMIT: amperes | None = None,
        CLOSED_LOOP_CONFIG: Slot0Configs | None = None,
        SLOT_1_CONFIG: Slot1Configs | None = None,
        SLOT_2_CONFIG: Slot2Configs | None = None,
        CANCODER_ID: int | None = None,
        CANCODER_INVERTED: bool | None = None,
        CANCODER_OFFSET_ROTATIONS: float | None = None,
        MOTOR_TYPE: DCMotor | None = None,
        LENGTH: meters | None = None,
        MOI: float | None = None,
        MIN_ANGLE: Rotation2d | None = None,
        MAX_ANGLE: Rotation2d | None = None,
    ) -> None:
        if START_SETPOINT is not None:
            self.START_SETPOINT = START_SETPOINT
        if MOTOR_IDs is not None:
            self.MOTOR_IDs = MOTOR_IDs
        if MASTER_INVERSION is not None:
            self.MASTER_INVERSION = MASTER_INVERSION
        if INVERSIONS_TO_MASTER is not None:
            self.INVERSIONS_TO_MASTER = INVERSIONS_TO_MASTER
        if MOTOR_TO_SENSOR_RATIO is not None:
            self.MOTOR_TO_SENSOR_RATIO = MOTOR_TO_SENSOR_RATIO
        if SENSOR_TO_MECHANISM_RATIO is not None:
            self.SENSOR_TO_MECHANISM_RATIO = SENSOR_TO_MECHANISM_RATIO
        if BRAKE_ENABLED is not None:
            self.BRAKE_ENABLED = BRAKE_ENABLED
        if CURRENT_LIMIT is not None:
            self.CURRENT_LIMIT = CURRENT_LIMIT
        if CLOSED_LOOP_CONFIG is not None:
            self.CLOSED_LOOP_CONFIG = CLOSED_LOOP_CONFIG
        if SLOT_1_CONFIG is not None:
            self.SLOT_1_CONFIG = SLOT_1_CONFIG
        if SLOT_2_CONFIG is not None:
            self.SLOT_2_CONFIG = SLOT_2_CONFIG
        if CANCODER_ID is not None:
            self.CANCODER_ID = CANCODER_ID
        if CANCODER_INVERTED is not None:
            self.CANCODER_INVERTED = CANCODER_INVERTED
        if CANCODER_OFFSET_ROTATIONS is not None:
            self.CANCODER_OFFSET_ROTATIONS = CANCODER_OFFSET_ROTATIONS
        if MOTOR_TYPE is not None:
            self.MOTOR_TYPE = MOTOR_TYPE
        if LENGTH is not None:
            self.LENGTH = LENGTH
        if MOI is not None:
            self.MOI = MOI
        if MIN_ANGLE is not None:
            self.MIN_ANGLE = MIN_ANGLE
        if MAX_ANGLE is not None:
            self.MAX_ANGLE = MAX_ANGLE


class Constants:
    CANBUS: str = (
        ""  # The canbus to be used by ctre products, "" will be "rio1", "canivore1" is a likely canivore bus name, etc
    )
    NETTABLE_NAME: str = "000Arm2"  # the name to be used for networktables logging
    J1_CONSTANTS: JointConstants = JointConstants()
    J2_CONSTANTS: JointConstants = JointConstants()
    J2_KG: float = 0.55 / 2  # the k_g value for joint 2. It considers the angle of j1

    def __init__(self) -> None:
        self.J1_CONSTANTS = JointConstants(
            CURRENT_LIMIT=120,
            MOTOR_IDs=[10],
            CLOSED_LOOP_CONFIG=Slot0Configs()
            .with_k_p(15.0)
            .with_k_i(0.0)
            .with_k_d(0.75)
            .with_k_s(0.0)
            .with_k_g(0.562)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            ),
            SENSOR_TO_MECHANISM_RATIO=15.0,
        )
        self.J2_CONSTANTS = JointConstants(
            CURRENT_LIMIT=280,
            MOTOR_IDs=[11],
            CLOSED_LOOP_CONFIG=Slot0Configs()
            .with_k_p(550.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            # do not use this kg, use the Constants.J2_KG value instead for J1 angle compensation
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            ),
            # MOI=0.25,
            SENSOR_TO_MECHANISM_RATIO=30.0,
            MIN_ANGLE=Rotation2d.fromDegrees(-180),
        )


class Arm2(Subsystem):
    enable: bool = False
    """
    A two-jointed arm subsystem with independent control of each joint.

    Note that if if if slot1 and slot2 exist and both of their selectors are true, the arm will use slot2 for closed loop control
    """

    j1_cancoder: CANcoder | None = None
    j2_cancoder: CANcoder | None = None

    def __init__(
        self,
        consts: Constants | None = None,
        *,
        select_slot_1: Callable[[], bool] | None = None,
        select_slot_2: Callable[[], bool] | None = None,
    ):
        self.consts = consts if consts is not None else Constants()
        self.setName(self.consts.NETTABLE_NAME)
        self.nettable = NetworkTableInstance.getDefault().getTable(
            self.consts.NETTABLE_NAME
        )
        self.j1_motors: list[TalonFX] = []
        self.j2_motors: list[TalonFX] = []

        self.select_slot_1 = select_slot_1
        self.select_slot_2 = select_slot_2

        if self.consts.J1_CONSTANTS.CANCODER_ID is not None:
            self.j1_cancoder = CANcoder(
                self.consts.J1_CONSTANTS.CANCODER_ID, self.consts.CANBUS
            )
            self.j1_cancoder.configurator.apply(
                CANcoderConfiguration().with_magnet_sensor(
                    MagnetSensorConfigs()
                    .with_sensor_direction(
                        SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                        if self.consts.J1_CONSTANTS.CANCODER_INVERTED
                        else SensorDirectionValue.CLOCKWISE_POSITIVE
                    )
                    .with_magnet_offset(
                        self.consts.J1_CONSTANTS.CANCODER_OFFSET_ROTATIONS
                    )
                )
            )

        if self.consts.J2_CONSTANTS.CANCODER_ID is not None:
            self.j2_cancoder = CANcoder(
                self.consts.J2_CONSTANTS.CANCODER_ID, self.consts.CANBUS
            )
            self.j2_cancoder.configurator.apply(
                CANcoderConfiguration().with_magnet_sensor(
                    MagnetSensorConfigs()
                    .with_sensor_direction(
                        SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                        if self.consts.J2_CONSTANTS.CANCODER_INVERTED
                        else SensorDirectionValue.CLOCKWISE_POSITIVE
                    )
                    .with_magnet_offset(
                        self.consts.J2_CONSTANTS.CANCODER_OFFSET_ROTATIONS
                    )
                )
            )

        # Initialize Joint 1 motors
        for idx, (id, inverted) in enumerate(
            zip(
                self.consts.J1_CONSTANTS.MOTOR_IDs,
                self.consts.J1_CONSTANTS.INVERSIONS_TO_MASTER,
            )
        ):
            motor = TalonFX(id, self.consts.CANBUS)

            motor.configurator.apply(
                TalonFXConfiguration()
                .with_motor_output(
                    MotorOutputConfigs()
                    .with_inverted(
                        InvertedValue.CLOCKWISE_POSITIVE
                        if idx == 0 and self.consts.J1_CONSTANTS.MASTER_INVERSION
                        else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                    )
                    .with_neutral_mode(
                        NeutralModeValue.BRAKE
                        if self.consts.J1_CONSTANTS.BRAKE_ENABLED
                        else NeutralModeValue.COAST
                    )
                )
                .with_feedback(
                    FeedbackConfigs()
                    .with_feedback_remote_sensor_id(
                        self.consts.J1_CONSTANTS.CANCODER_ID
                    )
                    .with_sensor_to_mechanism_ratio(
                        self.consts.J1_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
                    )
                    if self.consts.J1_CONSTANTS.CANCODER_ID and idx == 0
                    else FeedbackConfigs().with_sensor_to_mechanism_ratio(
                        self.consts.J1_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
                    )
                )
                .with_slot0(self.consts.J1_CONSTANTS.CLOSED_LOOP_CONFIG)
                .with_slot1(
                    self.consts.J1_CONSTANTS.SLOT_1_CONFIG
                    if self.consts.J1_CONSTANTS.SLOT_1_CONFIG is not None
                    else Slot1Configs()
                )
                .with_slot2(
                    self.consts.J1_CONSTANTS.SLOT_2_CONFIG
                    if self.consts.J1_CONSTANTS.SLOT_2_CONFIG is not None
                    else Slot2Configs()
                )
                .with_current_limits(
                    CurrentLimitsConfigs().with_stator_current_limit(
                        self.consts.J1_CONSTANTS.CURRENT_LIMIT
                    )
                )
            )

            if idx > 0:
                motor.set_control(
                    Follower(self.consts.J1_CONSTANTS.MOTOR_IDs[0], inverted)
                )
            self.j1_motors.append(motor)

        # Initialize Joint 2 motors
        for idx, (id, inverted) in enumerate(
            zip(
                self.consts.J2_CONSTANTS.MOTOR_IDs,
                self.consts.J2_CONSTANTS.INVERSIONS_TO_MASTER,
            )
        ):
            motor = TalonFX(id, self.consts.CANBUS)

            motor.configurator.apply(
                TalonFXConfiguration()
                .with_motor_output(
                    MotorOutputConfigs()
                    .with_inverted(
                        InvertedValue.CLOCKWISE_POSITIVE
                        if idx == 0 and self.consts.J2_CONSTANTS.MASTER_INVERSION
                        else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                    )
                    .with_neutral_mode(
                        NeutralModeValue.BRAKE
                        if self.consts.J2_CONSTANTS.BRAKE_ENABLED
                        else NeutralModeValue.COAST
                    )
                )
                .with_feedback(
                    FeedbackConfigs()
                    .with_feedback_remote_sensor_id(
                        self.consts.J2_CONSTANTS.CANCODER_ID
                    )
                    .with_sensor_to_mechanism_ratio(
                        self.consts.J2_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
                    )
                    if self.consts.J2_CONSTANTS.CANCODER_ID and idx == 0
                    else FeedbackConfigs().with_sensor_to_mechanism_ratio(
                        self.consts.J2_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
                    )
                )
                .with_slot0(self.consts.J2_CONSTANTS.CLOSED_LOOP_CONFIG)
                .with_slot1(
                    self.consts.J2_CONSTANTS.SLOT_1_CONFIG
                    if self.consts.J2_CONSTANTS.SLOT_1_CONFIG is not None
                    else Slot1Configs()
                )
                .with_slot2(
                    self.consts.J2_CONSTANTS.SLOT_2_CONFIG
                    if self.consts.J2_CONSTANTS.SLOT_2_CONFIG is not None
                    else Slot2Configs()
                )
                .with_current_limits(
                    CurrentLimitsConfigs().with_stator_current_limit(
                        self.consts.J2_CONSTANTS.CURRENT_LIMIT
                    )
                )
            )

            if idx > 0:
                motor.set_control(
                    Follower(self.consts.J2_CONSTANTS.MOTOR_IDs[0], inverted)
                )
            self.j2_motors.append(motor)

        # Setup position and velocity signals for both joints
        self.j1_position_signal_rotations = (
            self.j1_cancoder.get_position()
            if self.j1_cancoder
            else self.j1_motors[0].get_position()
        )
        self.j1_velocity_signal_rotations = (
            self.j1_cancoder.get_velocity()
            if self.j1_cancoder
            else self.j1_motors[0].get_velocity()
        )

        self.j2_position_signal_rotations = (
            self.j2_cancoder.get_position()
            if self.j2_cancoder
            else self.j2_motors[0].get_position()
        )
        self.j2_velocity_signal_rotations = (
            self.j2_cancoder.get_velocity()
            if self.j2_cancoder
            else self.j2_motors[0].get_velocity()
        )

        self.j1_setpoint = self.consts.J1_CONSTANTS.START_SETPOINT
        self.j2_setpoint = self.consts.J2_CONSTANTS.START_SETPOINT

        self.j2_ff = ArmFeedforward(0, self.consts.J2_KG, 0, 0)

        if RobotBase.isSimulation():
            self.j1_arm_sim = SingleJointedArmSim(
                self.consts.J1_CONSTANTS.MOTOR_TYPE,
                self.consts.J1_CONSTANTS.SENSOR_TO_MECHANISM_RATIO,
                self.consts.J1_CONSTANTS.MOI,
                self.consts.J1_CONSTANTS.LENGTH,
                self.consts.J1_CONSTANTS.MIN_ANGLE.radians(),
                self.consts.J1_CONSTANTS.MAX_ANGLE.radians(),
                True,
                self.get_j1_angle().radians(),
            )

            # The J2 arm can not be simulated with a SingleJointedArmSim because it will not consider the J1 arm's angle

        # Calculate total reach for mechanism sizing
        total_length = self.consts.J1_CONSTANTS.LENGTH + self.consts.J2_CONSTANTS.LENGTH
        self.mech = Mechanism2d(
            w := (4.4 * total_length * 100),
            h := (4.4 * total_length * 100),
        )
        self.mech_root = self.mech.getRoot("Arm2", w / 2, h / 2)
        SmartDashboard.putData("Arm2 Mechanism", self.mech)

        # Create ligaments for both joints
        self.j1_lig = self.mech_root.appendLigament(
            "J1Ligament",
            self.consts.J1_CONSTANTS.LENGTH * 100,
            self.get_j1_angle().degrees(),
        )
        self.j2_lig = self.j1_lig.appendLigament(
            "J2Ligament",
            self.consts.J2_CONSTANTS.LENGTH * 100,
            self.get_j2_angle().degrees(),
        )

        self.j1_setpoint_lig = self.mech_root.appendLigament(
            "J1SetpointLigament",
            self.consts.J1_CONSTANTS.LENGTH * 100,
            self.get_j1_angle().degrees(),
            color=Color8Bit(0, 0, 255),
        )
        self.j2_setpoint_lig = self.j1_setpoint_lig.appendLigament(
            "J2SetpointLigament",
            self.consts.J2_CONSTANTS.LENGTH * 100,
            self.get_j2_angle().degrees(),
            color=Color8Bit(0, 0, 255),
        )

        SmartDashboard.putData(self)
        super().__init__()

    def periodic(self):
        """
        Log data to networktables and update both joint motors
        """
        BaseStatusSignal.refresh_all(
            [
                self.j1_position_signal_rotations,
                self.j1_velocity_signal_rotations,
                self.j2_position_signal_rotations,
                self.j2_velocity_signal_rotations,
            ]
        )

        # Log Joint 1 Inputs
        j1_position = self.get_j1_angle().degrees() / 360
        j1_velocity = self.j1_velocity_signal_rotations.value
        self.nettable.putNumber("J1/Position/Rotations", j1_position)
        self.nettable.putNumber("J1/Position/Degrees", j1_position * 360)
        self.nettable.putNumber("J1/Position/Radians", j1_position * 2 * pi)

        self.nettable.putNumber("J1/Velocity/Rotations", j1_velocity)
        self.nettable.putNumber("J1/Velocity/Degrees", j1_velocity * 360)
        self.nettable.putNumber("J1/Velocity/Radians", j1_velocity * 2 * pi)

        self.nettable.putNumber(
            "J1/Setpoint/Rotations", self.j1_setpoint.degrees() / 360
        )
        self.nettable.putNumber("J1/Setpoint/Degrees", self.j1_setpoint.degrees())
        self.nettable.putNumber("J1/Setpoint/Radians", self.j1_setpoint.radians())

        # Log Joint 2 Inputs
        j2_position = self.get_j2_angle().degrees() / 360
        j2_velocity = self.j2_velocity_signal_rotations.value
        self.nettable.putNumber("J2/Position/Rotations", j2_position)
        self.nettable.putNumber("J2/Position/Degrees", j2_position * 360)
        self.nettable.putNumber("J2/Position/Radians", j2_position * 2 * pi)

        self.nettable.putNumber("J2/Velocity/Rotations", j2_velocity)
        self.nettable.putNumber("J2/Velocity/Degrees", j2_velocity * 360)
        self.nettable.putNumber("J2/Velocity/Radians", j2_velocity * 2 * pi)

        self.nettable.putNumber(
            "J2/Setpoint/Rotations", self.j2_setpoint.degrees() / 360
        )
        self.nettable.putNumber("J2/Setpoint/Degrees", self.j2_setpoint.degrees())
        self.nettable.putNumber("J2/Setpoint/Radians", self.j2_setpoint.radians())

        slot = 0
        if self.select_slot_1 is not None:
            if self.select_slot_1():
                slot = 1
        if self.select_slot_2 is not None:
            if self.select_slot_2():
                slot = 2

        # Control both joints
        self.j1_motors[0].set_control(
            PositionVoltage(self.j1_setpoint.degrees() / 360, slot=slot)
        )

        ff = self.j2_ff.calculate(
            (self.get_j1_angle() + self.get_j2_angle()).radians(),
            self.j2_velocity_signal_rotations.value,
        )

        self.j2_motors[0].set_control(
            PositionVoltage(
                self.j2_setpoint.degrees() / 360, slot=slot, feed_forward=ff
            )
        )

        # Log Joint 1 Outputs
        for idx, motor in enumerate(self.j1_motors):
            self.nettable.putNumber(f"J1/Motor{idx}/output (%)", motor.get())
            self.nettable.putNumber(
                f"J1/Motor{idx}/setpoint (rotations)",
                motor.get_closed_loop_reference().value_as_double,
            )
            self.nettable.putNumber(
                f"J1/Motor{idx}/Error (rotations)",
                motor.get_closed_loop_error().value_as_double,
            )

        # Log Joint 2 Outputs
        for idx, motor in enumerate(self.j2_motors):
            self.nettable.putNumber(f"J2/Motor{idx}/output (%)", motor.get())
            self.nettable.putNumber(
                f"J2/Motor{idx}/setpoint (rotations)",
                motor.get_closed_loop_reference().value_as_double,
            )
            self.nettable.putNumber(
                f"J2/Motor{idx}/Error (rotations)",
                motor.get_closed_loop_error().value_as_double,
            )

        # Update mechanism visualization
        self.j1_lig.setAngle(self.get_j1_angle().degrees())
        self.j2_lig.setAngle(self.get_j2_angle().degrees())

        self.j1_setpoint_lig.setAngle(self.j1_setpoint.degrees())
        self.j2_setpoint_lig.setAngle((self.j2_setpoint).degrees())

        return super().periodic()

    def simulationPeriodic(self):
        if not self.enable:
            return
        """
        Update the simulation model for both joints.
        Set the motors and simulate
        """
        # Simulate Joint 1
        self.j1_arm_sim.setInputVoltage(RoboRioSim.getVInVoltage())
        self.j1_arm_sim.setInput([self.j1_motors[0].get() * RoboRioSim.getVInVoltage()])
        self.j1_arm_sim.update(0.02)

        j1_vel = (
            self.j1_arm_sim.getVelocity()
            / (2 * pi)
            * self.consts.J1_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
        )

        for motor in self.j1_motors:
            motor.sim_state.add_rotor_position(j1_vel * 0.02)
            motor.sim_state.set_rotor_velocity(j1_vel)

        if self.j1_cancoder:
            j1_cancoder_vel = j1_vel * self.consts.J1_CONSTANTS.MOTOR_TO_SENSOR_RATIO
            self.j1_cancoder.sim_state.add_position(j1_cancoder_vel)
            self.j1_cancoder.sim_state.set_velocity(j1_cancoder_vel)

        """
        Physics happens here because J2 is dependent on J1
        and wpilib does not support multi-joint arms in simulation
        Instead, we just trick the SingleJointedArmSim by giving it the J1 angle
        This arm will go slow because it thinks that it is constantly accelerating since there is no state saved
        """
        j2_sim = SingleJointedArmSim(
            self.consts.J2_CONSTANTS.MOTOR_TYPE,
            self.consts.J2_CONSTANTS.SENSOR_TO_MECHANISM_RATIO,
            self.consts.J2_CONSTANTS.MOI,
            self.consts.J2_CONSTANTS.LENGTH,
            self.consts.J2_CONSTANTS.MIN_ANGLE.radians()
            - self.get_j1_angle().radians(),
            self.consts.J2_CONSTANTS.MAX_ANGLE.radians()
            + self.get_j1_angle().radians(),
            True,
            (self.get_j1_angle() + self.get_j2_angle()).radians(),
        )

        j2_sim.setInputVoltage(RoboRioSim.getVInVoltage())
        j2_sim.setInput([self.j2_motors[0].get() * RoboRioSim.getVInVoltage()])
        j2_sim.update(0.02)

        j2_vel = (
            j2_sim.getVelocity()
            / (2 * pi)
            * self.consts.J2_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
        )

        for motor in self.j2_motors:
            motor.sim_state.add_rotor_position(j2_vel * 0.02)
            motor.sim_state.set_rotor_velocity(j2_vel)

        if self.j2_cancoder:
            j2_cancoder_vel = j2_vel
            self.j2_cancoder.sim_state.add_position(j2_cancoder_vel * 0.02)
            self.j2_cancoder.sim_state.set_velocity(j2_cancoder_vel)

        # Set combined current draw
        RoboRioSim.setVInCurrent(
            self.j1_arm_sim.getCurrentDraw() + j2_sim.getCurrentDraw()
        )

        return super().simulationPeriodic()

    def get_j1_angle(self) -> Rotation2d:
        """
        Returns the current angle of joint 1 as a Rotation2d.
        Considers scaling factors set in Constants
        """
        if self.j1_cancoder:
            return Rotation2d.fromRotations(
                self.j1_position_signal_rotations.value_as_double
                / self.consts.J1_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
            )

        return Rotation2d.fromRotations(
            self.j1_position_signal_rotations.value_as_double
        )

    def get_j2_angle(self) -> Rotation2d:
        """
        Returns the current angle of joint 2 as a Rotation2d.
        Considers scaling factors set in Constants
        """
        if self.j2_cancoder:
            return Rotation2d.fromRotations(
                self.j2_position_signal_rotations.value_as_double
                / self.consts.J2_CONSTANTS.SENSOR_TO_MECHANISM_RATIO
            )

        return Rotation2d.fromRotations(
            self.j2_position_signal_rotations.value_as_double
        )

    def _set_j1_setpoint(self, setpoint: Rotation2d) -> None:
        """
        A (mostly) private method to set the setpoint of joint 1.
        Users should use the set_j1_setpoint method which returns a command
        This should only be used in advanced cases or nested in a subclassed command
        """
        if setpoint.degrees() < self.consts.J1_CONSTANTS.MIN_ANGLE.degrees():
            setpoint = self.consts.J1_CONSTANTS.MIN_ANGLE
        elif setpoint.degrees() > self.consts.J1_CONSTANTS.MAX_ANGLE.degrees():
            setpoint = self.consts.J1_CONSTANTS.MAX_ANGLE
        self.j1_setpoint = setpoint

    def _set_j2_setpoint(self, setpoint: Rotation2d) -> None:
        """
        A (mostly) private method to set the setpoint of joint 2.
        Users should use the set_j2_setpoint method which returns a command
        This should only be used in advanced cases or nested in a subclassed command
        """
        if setpoint.degrees() < self.consts.J2_CONSTANTS.MIN_ANGLE.degrees():
            setpoint = self.consts.J2_CONSTANTS.MIN_ANGLE
        elif setpoint.degrees() > self.consts.J2_CONSTANTS.MAX_ANGLE.degrees():
            setpoint = self.consts.J2_CONSTANTS.MAX_ANGLE
        self.j2_setpoint = setpoint

    def _set_setpoint(self, position: Transform2d) -> None:
        """
        A (mostly) private method to set the setpoint of both joints.
        Users should use the set_setpoint method which returns a command
        This should only be used in advanced cases or nested in a subclassed command

        The x and y components of the position will be the position of the end of the arm in meters.
        The rotation component will be the angle of the end of the arm in radians.

        If the position cannot be reached, the setpoint will be set to the closest reachable position.
        The ambiguity in the law of cos will be resolved by choosing the angle that is closest to the requested angle.

        Math based on law of sin/cos
        """
        # first, constrain the endpoints do be within the arm's reach
        dist = position.translation().norm()
        x = (
            position.translation().x
            / dist
            * min(
                dist,
                self.consts.J1_CONSTANTS.LENGTH
                + self.consts.J2_CONSTANTS.LENGTH
                - 0.001,
            )
        )
        y = (
            position.translation().y
            / dist
            * min(
                dist,
                self.consts.J1_CONSTANTS.LENGTH
                + self.consts.J2_CONSTANTS.LENGTH
                - 0.001,
            )
        )
        dist = (x**2 + y**2) ** 0.5

        alpha = acos(
            (
                x**2
                + y**2
                - self.consts.J1_CONSTANTS.LENGTH**2
                - self.consts.J2_CONSTANTS.LENGTH**2
            )
            / (2 * self.consts.J1_CONSTANTS.LENGTH * self.consts.J2_CONSTANTS.LENGTH)
        )

        beta = atan2(y, x) - atan2(
            (self.consts.J2_CONSTANTS.LENGTH * sin(alpha)),
            (
                self.consts.J1_CONSTANTS.LENGTH
                + self.consts.J2_CONSTANTS.LENGTH * cos(alpha)
            ),
        )

        def angle_diff(a, b):
            return abs(((a - b + pi) % (2 * pi)) - pi)

        final_angle = beta + (pi - alpha)

        if angle_diff(final_angle, position.rotation().radians()) < angle_diff(
            position.rotation().radians(), final_angle
        ):
            print("Optimize")
            alpha = -alpha
            beta = -beta
        else:
            print("No Optimize")

        self._set_j1_setpoint(Rotation2d(beta))
        self._set_j2_setpoint(Rotation2d(alpha))

        return None

    def set_setpoint(self, position: Transform2d) -> Command:
        """
        Sets the setpoint of both joints to the given position.
        The x and y components of the position will be the position of the end of the arm in meters.
        The rotation component will be the angle of the end of the arm in radians.
        """
        return InstantCommand(self._set_setpoint(position), self)

    def get_position(self) -> Transform2d:
        """
        Returns the current position of the end of the arm as a Transform2d.
        The translation will be the position of the end of the arm in meters.
        The rotation will be the angle of the end of the arm in radians.
        """
        j1_angle = self.get_j1_angle()
        j2_angle = self.get_j2_angle()
        end_j1 = Transform2d(
            self.consts.J1_CONSTANTS.LENGTH * j1_angle.cos(),
            self.consts.J1_CONSTANTS.LENGTH * j1_angle.sin(),
            j1_angle,
        )
        j2_transform = Transform2d(
            self.consts.J2_CONSTANTS.LENGTH * j2_angle.cos(),
            self.consts.J2_CONSTANTS.LENGTH * j2_angle.sin(),
            j2_angle,
        )
        return end_j1 + j2_transform
