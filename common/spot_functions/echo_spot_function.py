import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client
import bosdyn.client.util

from common.spot_connection import get_connected_robot
from common.spot_functions.base_spot_function import BaseSpotFunction
from common.spot_functions.spot_function_execution import execute_function_for_robot


class EchoSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument(
            '-m', '--message_to_echo', default='Hello, Spot!', nargs=1, help=
            'Pass a message to be echoed by Spot.'
        )

    def execute(self, options: Namespace):
        message_to_echo: str = options.message_to_echo
        self.robot.logger.info(message_to_echo)

    @staticmethod
    def main(argv=None):
        try:
            username = "example_username"
            password = "example_password"
            robot = get_connected_robot('HelloSpotClient', username, password)
            echo_spot_function = EchoSpotFunction()
            options = echo_spot_function.parse_arguments(argv)
            execute_function_for_robot(robot, echo_spot_function, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Echoing through Spot threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not EchoSpotFunction.main(sys.argv[1:]):
        sys.exit(1)
