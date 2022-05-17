import sys

import bosdyn
import bosdyn.client
import bosdyn.client.util

from common.spot_connection import get_connected_robot
from common.spot_functions.hello_spot_function import HelloSpotFunction
from common.spot_functions.spot_function_execution import execute_function_for_robot


def main():
    try:
        username = "example_username"
        password = "example_password"
        robot = get_connected_robot('HelloSpotClient', username, password)
        hello_spot_function = HelloSpotFunction()
        execute_function_for_robot(robot, hello_spot_function)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False


if __name__ == '__main__':
    if not main():
        sys.exit(1)
