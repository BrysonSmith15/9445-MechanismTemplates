from wpilib import TimedRobot
from commands2 import CommandScheduler, RepeatCommand, InstantCommand, WaitCommand
from wpimath.geometry import Rotation2d

from subsystems.arm_1 import Arm1

from commands.arm1_goto_setpoint_profiled import Arm1GotoSetpointProfiled


class Robot(TimedRobot):
    def robotInit(self):
        self.arm = Arm1()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def teleopInit(self):
        RepeatCommand(
            Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(-90), 3).andThen(
                Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(90), 3)
            )
        ).schedule()

        return super().teleopInit()
