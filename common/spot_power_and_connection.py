import os

import bosdyn.client
import bosdyn.client.util
from bosdyn.client import Robot


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


def get_powered_on_robot(
        client_name_prefix: str,
        username: str,
        password: str,
        hostname: str = "192.168.80.3",
        verbose: bool = False
) -> Robot:
    robot = create_robot(client_name_prefix, hostname, verbose)
    connect_to_robot(robot, username, password)
    return Robot


def power_down(
        robot: Robot
):
    robot.logger.info('Sitting down and turning off.')

    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")