from wpilib import TimedRobot
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.units import inchesToMeters
from commands2 import CommandScheduler, RepeatCommand, InstantCommand, WaitCommand


from subsystems.arm_2 import Arm2


class Robot(TimedRobot):
    def robotInit(self):
        self.arm = Arm2()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def teleopInit(self):
        self.arm.enable = True
        # self.arm._set_j1_setpoint(Rotation2d.fromDegrees(0))
        # self.arm._set_j2_setpoint(Rotation2d.fromDegrees(90))
        self.arm._set_setpoint(
            Transform2d(
                Translation2d(inchesToMeters(40), inchesToMeters(0)),
                Rotation2d.fromDegrees(45),
            )
        )

        return super().teleopInit()

    def autonomousInit(self) -> None:
        self.arm._set_setpoint(
            Transform2d(
                Translation2d(inchesToMeters(40), inchesToMeters(0)),
                Rotation2d.fromDegrees(-45),
            )
        )
        return super().autonomousInit()

    def testInit(self) -> None:
        self.arm._set_setpoint(
            Transform2d(
                Translation2d(inchesToMeters(0), inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
            )
        )
        return super().testInit()

    def teleopPeriodic(self) -> None:
        # print(
        #     (p := self.arm.get_position().translation()).x_feet * 12,
        #     p.y_feet * 12,
        #     self.arm.get_position().rotation().degrees(),
        # )
        return super().teleopPeriodic()
