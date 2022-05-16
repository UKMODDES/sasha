from bosdyn.client import Robot

from common.spot_functions.base_spot_function import BaseSpotFunction


class EchoSpotFunction(BaseSpotFunction):
    def execute(self, robot: Robot, *args):
        message_to_echo: str = args[0]
        robot.logger.info(message_to_echo)