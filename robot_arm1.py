from wpilib import TimedRobot
from commands2 import CommandScheduler, RepeatCommand, InstantCommand, WaitCommand
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter

from wpimath.system.plant import DCMotor

from subsystems.arm_1 import Arm1, Constants

from commands.arm1_goto_setpoint_profiled import Arm1GotoSetpointProfiled


class Robot(TimedRobot):
    def robotInit(self):
        self.arm = Arm1()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def teleopInit(self):
        # self.angle = Rotation2d.fromDegrees(0)
        RepeatCommand(
            Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(-90), 3).andThen(
                Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(90), 3)
            )
        ).schedule()

        return super().teleopInit()
