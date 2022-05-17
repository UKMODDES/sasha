import argparse
from abc import ABC, abstractmethod
from argparse import Namespace

import bosdyn
import bosdyn.client.util
from bosdyn.client import Robot
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

from spot_common.spot_control.spot_actions.base_unit import make_robot_stand
from spot_common.spot_control.spot_power import power_on_robot, power_off_robot


class BaseSpotFunction(ABC):
    username = "example_username"
    password = "example_password"
    robot: Robot = None
    robot_state_client: RobotStateClient = None
    lease_client: LeaseClient = None
    command_client: RobotCommandClient = None

    def parse_arguments(self, argv) -> Namespace:
        parser = argparse.ArgumentParser()
        bosdyn.client.util.add_base_arguments(parser)
        self._add_arguments(parser)
        options = parser.parse_args(argv)
        return options

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

    def execute_function_for_robot(
            self,
            robot: Robot,
            options: Namespace
    ):
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            power_on_robot(robot)
            try:
                command_client = robot.ensure_client(RobotCommandClient.default_service_name)
                make_robot_stand(robot, command_client)
                self.prepare(robot, robot_state_client, lease_client, command_client)
                self.execute(options)
            finally:
                power_off_robot(robot)

    @abstractmethod
    def execute(self, options: Namespace):
        pass

    @abstractmethod
    def _add_arguments(self, parser: argparse.ArgumentParser):
        pass

    @staticmethod
    @abstractmethod
    def main(argv=None):
        pass
