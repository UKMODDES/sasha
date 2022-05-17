from argparse import Namespace

from common.spot_functions.base_spot_function import BaseSpotFunction


class EchoSpotFunction(BaseSpotFunction):
    def execute(self, options: Namespace):
        message_to_echo: str = options.message_to_echo
        self.robot.logger.info(message_to_echo)
