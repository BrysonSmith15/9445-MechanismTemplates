from wpilib import TimedRobot
from commands2 import CommandScheduler, RepeatCommand, InstantCommand, WaitCommand
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter

from wpimath.system.plant import DCMotor

from subsystems.arm_1 import Arm1

from commands.arm1_goto_setpoint_profiled import Arm1GotoSetpointProfiled


class Robot(TimedRobot):
    def robotInit(self):
        self.arm = Arm1()
        self.going_up = True
        self.limiter = SlewRateLimiter(200)

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def teleopInit(self):
        # self.angle = Rotation2d.fromDegrees(0)
        RepeatCommand(
            Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(-90), 3).andThen(
                Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(90), 1)
            )
        ).schedule()
        # Arm1GotoSetpointProfiled(self.arm, Rotation2d.fromDegrees(90), 10).schedule()
        self.arm.nettable.putBoolean("Enable", True)
        return super().teleopInit()

    def teleopPeriodic(self):
        # if (
        #     abs(self.angle.degrees() - self.arm.get_angle().degrees()) < 1
        #     and self.arm.setpoint == self.angle
        # ):
        #     self.angle = -Rotation2d.fromDegrees(
        #         self.angle.degrees() + 10 * (1 if self.angle.degrees() > 0 else -1)
        #     )
        #     if (
        #         self.angle.degrees() < self.arm.consts.MIN_ANGLE.degrees()
        #         or self.angle.degrees() > self.arm.consts.MAX_ANGLE.degrees()
        #     ):
        #         self.angle = (self.arm.consts.MIN_ANGLE + self.arm.consts.MAX_ANGLE) / 2

        # self.arm.set_setpoint(
        #     Rotation2d.fromDegrees(self.limiter.calculate(self.angle.degrees()))
        # ).schedule()
        return super().teleopPeriodic()


if __name__ == "__main__":
    from wpilib import run

    run(Robot)
