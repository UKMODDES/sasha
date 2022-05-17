import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client.util

from spot_common.spot_connection import get_connected_robot
from spot_common.spot_functions.base_spot_function import BaseSpotFunction


class HelloSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        pass

    def execute(self, _options: Namespace):
        self.robot.logger.info("Hello, Spot!")

    @staticmethod
    def main(argv=None):
        try:
            hello_spot_function = HelloSpotFunction()
            robot = get_connected_robot('HelloSpotClient', hello_spot_function.username, hello_spot_function.password)
            options = hello_spot_function.parse_arguments(argv=None)
            hello_spot_function.execute_function_for_robot(robot, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Hello, Spot! threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not HelloSpotFunction.main():
        sys.exit(1)
