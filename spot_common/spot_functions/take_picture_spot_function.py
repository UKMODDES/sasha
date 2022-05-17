import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient

from spot_common.spot_control.spot_actions.camera import make_robot_take_picture
from spot_common.spot_control.spot_connection import get_connected_robot
from spot_common.spot_functions.base_spot_function import BaseSpotFunction


class TakePictureSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument(
            '-i', '--image_source', default='frontleft_fisheye_image', nargs=1, help=
            'Which camera to use to take the picture.'
        )

    def execute(self, options: Namespace):
        image_source: str = options.image_source
        image_client = self.robot.ensure_client(ImageClient.default_service_name)
        image = make_robot_take_picture(self.robot, image_client, image_source)
        print(image)

    @staticmethod
    def main(argv=None):
        try:
            take_picture_spot_function = TakePictureSpotFunction()
            robot = get_connected_robot('image_capture', take_picture_spot_function.username, take_picture_spot_function.password)
            options = take_picture_spot_function.parse_arguments(argv)
            take_picture_spot_function.execute_function_for_robot(robot, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Echoing through Spot threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not TakePictureSpotFunction.main(sys.argv[1:]):
        sys.exit(1)
