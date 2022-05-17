import argparse
from argparse import Namespace

from common.spot_functions.base_spot_function import BaseSpotFunction


class EchoSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument(
            '-m', '--message_to_echo', default='Hello, Spot!', nargs=1, help=
            'Pass a message to be echoed by Spot.'
        )

    def execute(self, options: Namespace):
        message_to_echo: str = options.message_to_echo
        self.robot.logger.info(message_to_echo)
