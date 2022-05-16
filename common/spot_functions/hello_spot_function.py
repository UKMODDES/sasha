from bosdyn.client import Robot

from common.spot_functions.base_spot_function import BaseSpotFunction


class HelloSpotFunction(BaseSpotFunction):
    def execute(self, robot: Robot, *args):
        robot.logger.info("Hello, Spot!")
