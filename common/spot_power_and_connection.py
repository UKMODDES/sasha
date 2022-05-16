import os
from abc import ABC, abstractmethod

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.lease
from bosdyn.client import Robot
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient


def create_robot(
        client_name_prefix: str,
        hostname: str = "192.168.80.3",
        verbose: bool = False
) -> Robot:
    bosdyn.client.util.setup_logging(verbose)

    sdk = bosdyn.client.create_standard_sdk(client_name_prefix)
    robot = sdk.create_robot(hostname)
    return robot


def connect_to_robot(
        robot: Robot,
        username: str,
        password: str
):
    os.environ["BOSDYN_CLIENT_USERNAME"] = username
    os.environ["BOSDYN_CLIENT_PASSWORD"] = password
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()


def power_on_robot(
        robot: Robot
):
    # Now, we are ready to power on the robot. This call will block until the power
    # is on. Commands would fail if this did not happen. We can also check that the robot is
    # powered at any point.
    robot.logger.info("Powering on robot... This may take a several seconds.")
    robot.power_on(timeout_sec=20)
    assert robot.is_powered_on(), "Robot power on failed."
    robot.logger.info("Robot powered on.")


def make_robot_stand(
        robot: Robot
):
    # Tell the robot to stand up. The command service is used to issue commands to a robot.
    # The set of valid commands for a robot depends on hardware configuration. See
    # SpotCommandHelper for more detailed examples on command building. The robot
    # command service requires timesync between the robot and the client.
    robot.logger.info("Commanding robot to stand...")
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    blocking_stand(command_client, timeout_sec=10)
    robot.logger.info("Robot standing.")


def get_connected_robot(
        client_name_prefix: str,
        username: str,
        password: str,
        hostname: str = "192.168.80.3",
        verbose: bool = False
) -> Robot:
    robot = create_robot(client_name_prefix, hostname, verbose)
    connect_to_robot(robot, username, password)
    return robot


class SpotFunction(ABC):
    robot_state_client: RobotStateClient = None
    lease_client: bosdyn.client.lease.LeaseClient = None
    robot: Robot = None

    def prepare(
            self,
            robot_state_client: RobotStateClient,
            lease_client: bosdyn.client.lease.LeaseClient,
            robot: Robot
    ):
        self.robot_state_client = robot_state_client
        self.lease_client = lease_client
        self.robot = robot

    @abstractmethod
    def execute(self, *args):
        pass


def execute_function_for_robot(
        robot: Robot,
        spot_function: SpotFunction,
        *args
):
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        power_on_robot(robot)
        try:
            make_robot_stand(robot)
            spot_function.prepare(robot_state_client, lease_client, robot)
            spot_function.execute(args)
        finally:
            power_off_robot(robot)


def power_off_robot(
        robot: Robot
):
    robot.logger.info('Sitting down and turning off.')

    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")


class HelloSpotFunction(SpotFunction):
    def execute(self, robot: Robot, *args):
        robot.logger.info("Hello, Spot!")


class EchoSpotFunction(SpotFunction):
    def execute(self, robot: Robot, *args):
        message_to_echo: str = args[0]
        robot.logger.info(message_to_echo)
