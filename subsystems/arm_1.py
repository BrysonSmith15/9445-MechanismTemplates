from typing import Callable
from math import pi

from ntcore import NetworkTableInstance

from commands2 import Subsystem

from wpilib import RobotBase, SmartDashboard
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
    START_SETPOINT: Rotation2d = Rotation2d(0)
    CANBUS: str = ""
    NETTABLE_NAME: str = "000Arm"
    MOTOR_IDs: list[int] = [10]
    MASTER_INVERSION: bool = False
    INVERSIONS_TO_MASTER: list[bool] = [
        False,
        False,
        False,
    ]  # at [0] is master, its value is ignored
    MOTOR_TO_SENSOR_RATIO: float = 1.0
    SENSOR_TO_MECHANISM_RATIO: float = 15.0
    BRAKE_ENABLED: bool = True
    CURRENT_LIMIT: amperes = 80
    CLOSED_LOOP_CONFIG: Slot0Configs = (
        Slot0Configs()
        .with_k_p(25.0)
        .with_k_i(0)
        .with_k_d(2.0)
        .with_k_g(0.87)
        .with_k_s(0)
        .with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        .with_gravity_type(GravityTypeValue.ARM_COSINE)
    )
    SLOT_1_CONFIG: Slot1Configs | None = None
    SLOT_2_CONFIG: Slot2Configs | None = None
    CANCODER_ID: int | None = None
    CANCODER_INVERTED: bool | None = True
    CANCODER_OFFSET_ROTATIONS: float = 0.0
    # this should include the number of motors in the joint for simulation calculations
    MOTOR_TYPE: DCMotor = DCMotor.krakenX60(1)

    LENGTH: meters = inchesToMeters(19)
    MOI: float = 0.25  # Moment of intertia in kg*m^2
    MIN_ANGLE: Rotation2d = Rotation2d.fromDegrees(-120)
    MAX_ANGLE: Rotation2d = Rotation2d.fromDegrees(120)


class Arm1(Subsystem):
    cancoder: CANcoder | None = None

    def __init__(
        self,
        select_slot_1: Callable[[], bool] | None = None,
        select_slot_2: Callable[[], bool] | None = None,
    ):
        self.consts = Constants()
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
            self.nettable.putBoolean("Enable", False)

        SmartDashboard.putData(self)
        super().__init__()

    def periodic(self):
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

        # Work
        self.motors[0].set_control(
            PositionVoltage(
                self.setpoint.degrees() / 360  # / self.consts.SENSOR_TO_MECHANISM_RATIO
            )
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

        return super().periodic()

    def simulationPeriodic(self):
        if self.nettable.getBoolean("Enable", False):
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

        return super().simulationPeriodic()

    def get_angle(self) -> Rotation2d:
        if self.cancoder:
            return Rotation2d.fromRotations(
                self.position_signal_rotations.value_as_double
                / self.consts.SENSOR_TO_MECHANISM_RATIO
            )

        return Rotation2d.fromRotations(self.position_signal_rotations.value_as_double)

    def _set_setpoint(self, setpoint: Rotation2d) -> None:
        self.setpoint = setpoint

    def set_setpoint(self, setpoint: Rotation2d) -> Command:
        return InstantCommand(lambda: self._set_setpoint(setpoint), self).withName(
            f"Set Setpoint to {setpoint.degrees()} degrees"
        )

    def get_setpoint(self) -> Rotation2d:
        return self.setpoint
