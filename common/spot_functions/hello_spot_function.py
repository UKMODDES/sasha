import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client.util

from common.spot_connection import get_connected_robot
from common.spot_functions.base_spot_function import BaseSpotFunction
from common.spot_functions.spot_function_execution import execute_function_for_robot


class HelloSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        pass

    def execute(self, _options: Namespace):
        self.robot.logger.info("Hello, Spot!")

    @staticmethod
    def main(argv=None):
        try:
            username = "example_username"
            password = "example_password"
            robot = get_connected_robot('HelloSpotClient', username, password)
            hello_spot_function = HelloSpotFunction()
            options = hello_spot_function.parse_arguments(argv=None)
            execute_function_for_robot(robot, hello_spot_function, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Hello, Spot! threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not HelloSpotFunction.main():
        sys.exit(1)
