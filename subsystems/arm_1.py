from typing import Callable
from math import pi

from ntcore import NetworkTableInstance

from commands2 import Subsystem

from wpilib import (
    RobotBase,
    SmartDashboard,
    Mechanism2d,
    MechanismLigament2d,
    MechanismRoot2d,
)
from wpilib.simulation import SingleJointedArmSim, RoboRioSim

from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor
from wpimath.units import meters, amperes, inchesToMeters

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


class Constants:
    START_SETPOINT: Rotation2d = Rotation2d(0)  # the starting angle of the arm
    CANBUS: str = (
        ""  # The canbus to be used by ctre products, "" will be "rio1", "canivore1" is a likely canivore bus name, etc
    )
    NETTABLE_NAME: str = "000Arm"  # the name to be used for networktables loggin
    MOTOR_IDs: list[int] = [
        10
    ]  # the CAN IDs of each motor, the first is the master, all others are followers
    MASTER_INVERSION: bool = (
        False  # whether to invert the master motor, all others will be inverted relative to this. Default is CW Positive
    )
    INVERSIONS_TO_MASTER: list[bool] = [
        False,
    ]  # at [0] is master, its value is ignored, all others are whether to invert the motor relative to the master
    MOTOR_TO_SENSOR_RATIO: float = (
        1.0  # the ratio of the motor's rotations to the (remote) sensor's rotations, 1 if just the motor is used
    )
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


class Arm1(Subsystem):
    """
    A simple single jointed arm subsystem to be configured with the Constants class from the same file
    Can be composed into a multijointed arm with a seperate subsystem for each joint
    and vizualized with Mechanism2d by passing parent mechanism to the constructor
    This has poor sim fidelity as each parent arm does not consider any children arms and their impact on the parent
    They also do not know the angle of the parent, so will consider gravity very poorly
    The arm2 and arm3 (TODO) have better sim performance but can be more complicated

    Note that if if if slot1 and slot2 exist and both of their selectors are true, the arm will use slot2 for closed loop control
    """

    cancoder: CANcoder | None = None

    def __init__(
        self,
        consts: Constants | None = None,
        *,
        mech_parent: MechanismRoot2d | MechanismLigament2d | None = None,
        select_slot_1: Callable[[], bool] | None = None,
        select_slot_2: Callable[[], bool] | None = None,
    ):
        self.consts = consts if consts is not None else Constants()
        self.setName(self.consts.NETTABLE_NAME)
        self.nettable = NetworkTableInstance.getDefault().getTable(
            self.consts.NETTABLE_NAME
        )
        self.motors: list[TalonFX] = []

        self.select_slot_1 = select_slot_1
        self.select_slot_2 = select_slot_2

        if self.consts.CANCODER_ID is not None:
            self.cancoder = CANcoder(self.consts.CANCODER_ID, self.consts.CANBUS)
            self.cancoder.configurator.apply(
                CANcoderConfiguration().with_magnet_sensor(
                    MagnetSensorConfigs()
                    .with_sensor_direction(
                        SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                        if self.consts.CANCODER_INVERTED
                        else SensorDirectionValue.CLOCKWISE_POSITIVE
                    )
                    .with_magnet_offset(self.consts.CANCODER_OFFSET_ROTATIONS)
                )
            )

        for idx, (id, inverted) in enumerate(
            zip(self.consts.MOTOR_IDs, self.consts.INVERSIONS_TO_MASTER)
        ):
            motor = TalonFX(id, self.consts.CANBUS)

            motor.configurator.apply(
                TalonFXConfiguration()
                .with_motor_output(
                    MotorOutputConfigs()
                    .with_inverted(
                        InvertedValue.CLOCKWISE_POSITIVE
                        if idx == 0 and self.consts.MASTER_INVERSION
                        else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                    )
                    .with_neutral_mode(
                        NeutralModeValue.BRAKE
                        if self.consts.BRAKE_ENABLED
                        else NeutralModeValue.COAST
                    )
                )
                .with_feedback(
                    FeedbackConfigs()
                    .with_feedback_remote_sensor_id(self.consts.CANCODER_ID)
                    .with_rotor_to_sensor_ratio(self.consts.MOTOR_TO_SENSOR_RATIO)
                    .with_sensor_to_mechanism_ratio(
                        self.consts.SENSOR_TO_MECHANISM_RATIO
                    )
                    if self.consts.CANCODER_ID and idx == 0
                    else FeedbackConfigs()
                    .with_rotor_to_sensor_ratio(self.consts.MOTOR_TO_SENSOR_RATIO)
                    .with_sensor_to_mechanism_ratio(
                        self.consts.SENSOR_TO_MECHANISM_RATIO
                    )
                )
                .with_slot0(self.consts.CLOSED_LOOP_CONFIG)
                .with_slot1(
                    self.consts.SLOT_1_CONFIG
                    if self.consts.SLOT_1_CONFIG is not None
                    else Slot1Configs()
                )
                .with_slot2(
                    self.consts.SLOT_2_CONFIG
                    if self.consts.SLOT_2_CONFIG is not None
                    else Slot2Configs()
                )
                .with_current_limits(
                    CurrentLimitsConfigs().with_stator_current_limit(
                        self.consts.CURRENT_LIMIT
                    )
                )
            )

            if idx > 0:
                motor.set_control(Follower(self.consts.MOTOR_IDs[0], inverted))
            self.motors.append(motor)

        self.position_signal_rotations = (
            self.cancoder.get_position()
            if self.cancoder
            else self.motors[0].get_position()
        )
        self.velocity_signal_rotations = (
            self.cancoder.get_velocity()
            if self.cancoder
            else self.motors[0].get_velocity()
        )

        self.setpoint = self.consts.START_SETPOINT

        if RobotBase.isSimulation():
            self.arm_sim = SingleJointedArmSim(
                self.consts.MOTOR_TYPE,
                self.consts.MOTOR_TO_SENSOR_RATIO
                * self.consts.SENSOR_TO_MECHANISM_RATIO,
                self.consts.MOI,
                self.consts.LENGTH,
                self.consts.MIN_ANGLE.radians(),
                self.consts.MAX_ANGLE.radians(),
                True,
                self.get_angle().radians(),
            )

        if mech_parent is None:
            # this does not current handle if the child is larger than the parent
            self.mech = Mechanism2d(
                w := (4.4 * self.consts.LENGTH * 100),
                h := (4.4 * self.consts.LENGTH * 100),
            )
            self.mech_root = self.mech.getRoot("Arm1", w / 2, h / 2)
            SmartDashboard.putData("Arm1 Mechanism", self.mech)
        else:
            self.mech_root = mech_parent
        self.lig = self.mech_root.appendLigament(
            "Arm1Ligament", self.consts.LENGTH * 100, self.get_angle().degrees()
        )

        SmartDashboard.putData(self)
        super().__init__()

    def periodic(self):
        """
        Log data to networktables and update the master motor
        """
        BaseStatusSignal.refresh_all(
            [self.position_signal_rotations, self.velocity_signal_rotations]
        )
        # Log Inputs
        position = self.get_angle().degrees() / 360
        velocity = self.velocity_signal_rotations.value
        self.nettable.putNumber("Position/Rotations", position)
        self.nettable.putNumber("Position/Degrees", position * 360)
        self.nettable.putNumber("Position/Radians", position * 2 * pi)

        self.nettable.putNumber("Velocity/Rotations", velocity)
        self.nettable.putNumber("Velocity/Degrees", velocity * 360)
        self.nettable.putNumber("Velocity/Radians", velocity * 2 * pi)

        self.nettable.putNumber("Setpoint/Rotations", self.setpoint.degrees() / 360)
        self.nettable.putNumber("Setpoint/Degrees", self.setpoint.degrees())
        self.nettable.putNumber("Setpoint/Radians", self.setpoint.radians())

        slot = 0
        if self.select_slot_1 is not None:
            if self.select_slot_1():
                slot = 1
        if self.select_slot_2 is not None:
            if self.select_slot_2():
                slot = 2

        # Work
        self.motors[0].set_control(
            PositionVoltage(self.setpoint.degrees() / 360, slot=slot)
        )

        # Log Outputs
        for idx, motor in enumerate(self.motors):
            self.nettable.putNumber(f"Motor{idx}/output (%)", motor.get())

            self.nettable.putNumber(
                f"Motor{idx}/setpoint (rotations)",
                motor.get_closed_loop_reference().value_as_double,
            )

            self.nettable.putNumber(
                f"Motor{idx}/Error (rotations)",
                motor.get_closed_loop_error().value_as_double,
            )

        self.lig.setAngle(self.get_angle().degrees())

        return super().periodic()

    def simulationPeriodic(self):
        """
        Update the simulation model.
        Set the motors and simulate
        """
        self.arm_sim.setInputVoltage(RoboRioSim.getVInVoltage())
        self.arm_sim.setInput([self.motors[0].get() * RoboRioSim.getVInVoltage()])
        self.arm_sim.update(0.02)

        vel = (
            self.arm_sim.getVelocity()
            / (2 * pi)
            * (
                self.consts.SENSOR_TO_MECHANISM_RATIO
                * self.consts.MOTOR_TO_SENSOR_RATIO
            )
        )
        for motor in self.motors:
            motor.sim_state.add_rotor_position(vel * 0.02)
            motor.sim_state.set_rotor_velocity(vel)

        if self.cancoder:
            cancoder_vel = vel * self.consts.MOTOR_TO_SENSOR_RATIO
            self.cancoder.sim_state.add_position(cancoder_vel)
            self.cancoder.sim_state.set_velocity(cancoder_vel)

        RoboRioSim.setVInCurrent(self.arm_sim.getCurrentDraw())

        return super().simulationPeriodic()

    def get_angle(self) -> Rotation2d:
        """
        Returns the current angle of the end of the arm as a Rotation2d.
        Considers scaling factors set in Constants
        """
        if self.cancoder:
            return Rotation2d.fromRotations(
                self.position_signal_rotations.value_as_double
                / self.consts.SENSOR_TO_MECHANISM_RATIO
            )

        return Rotation2d.fromRotations(self.position_signal_rotations.value_as_double)

    def _set_setpoint(self, setpoint: Rotation2d) -> None:
        """
        A (mostly) private method to set the setpoint of the arm.
        Users should use the set_setpoint method which returns a command
        This should only be used in advanced cases or nested in a subclassed command
        """
        if setpoint.degrees() < self.consts.MIN_ANGLE.degrees():
            setpoint = self.consts.MIN_ANGLE
        elif setpoint.degrees() > self.consts.MAX_ANGLE.degrees():
            setpoint = self.consts.MAX_ANGLE
        self.setpoint = setpoint

    def set_setpoint(self, setpoint: Rotation2d) -> Command:
        """
        Return a command to set the setpoint of the arm.
        This is the preferred way to set the setpoint of the arm.
        The setpoint is clamped to the min and max angles of the arm.
        """
        return InstantCommand(lambda: self._set_setpoint(setpoint), self).withName(
            f"Set Setpoint to {setpoint.degrees()} degrees"
        )

    def get_setpoint(self) -> Rotation2d:
        """
        Returns the current setpoint of the arm.
        """
        return self.setpoint
