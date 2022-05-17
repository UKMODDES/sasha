import os

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
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()


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
