from commands2 import Command
from subsystems.arm_1 import Arm1
from wpimath.geometry import Rotation2d


class Arm1GotoSetpoint(Command):
    def __init__(self, arm: Arm1, setpoint: Rotation2d):
        super().__init__()
        self.arm = arm
        self.setpoint = setpoint
        self.addRequirements(arm)

    def initialize(self):
        self.arm._set_setpoint(self.setpoint)

    def isFinished(self) -> bool:
        return abs(self.arm.get_angle().degrees() - self.setpoint.degrees()) < 1
