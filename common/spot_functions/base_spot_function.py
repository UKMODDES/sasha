from abc import ABC, abstractmethod

from bosdyn.client import Robot
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_state import RobotStateClient


class BaseSpotFunction(ABC):
    robot_state_client: RobotStateClient = None
    lease_client: LeaseClient = None
    robot: Robot = None

    def prepare(
            self,
            robot_state_client: RobotStateClient,
            lease_client: LeaseClient,
            robot: Robot
    ):
        self.robot_state_client = robot_state_client
        self.lease_client = lease_client
        self.robot = robot

    @abstractmethod
    def execute(self, *args):
        pass
