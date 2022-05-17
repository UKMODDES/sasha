from argparse import Namespace

from common.spot_functions.base_spot_function import BaseSpotFunction


class HelloSpotFunction(BaseSpotFunction):
    def execute(self, _options: Namespace):
        self.robot.logger.info("Hello, Spot!")
