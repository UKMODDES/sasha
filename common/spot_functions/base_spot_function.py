from abc import ABC, abstractmethod

from bosdyn.client import Robot
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient


class BaseSpotFunction(ABC):
    robot: Robot = None
    robot_state_client: RobotStateClient = None
    lease_client: LeaseClient = None
    command_client: RobotCommandClient = None

    def prepare(
            self,
            robot: Robot,
            robot_state_client: RobotStateClient,
            lease_client: LeaseClient,
            command_client: RobotCommandClient
    ):
        self.robot = robot
        self.robot_state_client = robot_state_client
        self.lease_client = lease_client
        self.command_client = command_client

    @abstractmethod
    def execute(self, *args):
        pass
