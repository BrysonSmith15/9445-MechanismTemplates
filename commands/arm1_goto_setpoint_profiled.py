from commands2 import Command
from subsystems.arm_1 import Arm1
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpimath.units import seconds


class Arm1GotoSetpointProfiled(Command):
    def __init__(self, arm: Arm1, setpoint: Rotation2d, time_to_target: seconds = 1.0):
        super().__init__()
        self.arm = arm
        self.setpoint = setpoint
        self.time_to_target = time_to_target
        self.limiter = SlewRateLimiter(1)
        self.target = Rotation2d()
        self.addRequirements(arm)

    def initialize(self):
        self.limiter = SlewRateLimiter(
            abs(
                (self.setpoint.degrees() - self.arm.get_angle().degrees())
                / self.time_to_target
            )
        )
        self.limiter.reset(self.arm.get_angle().degrees())

    def execute(self):
        self.target = self.limiter.calculate(self.setpoint.degrees())
        self.arm._set_setpoint(Rotation2d.fromDegrees(self.target))

    def end(self, interrupted: bool):
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return (
            abs(self.arm.get_angle().degrees() - self.setpoint.degrees()) < 1
            and self.target == self.setpoint.degrees()
        )  # achieved target and not moving
