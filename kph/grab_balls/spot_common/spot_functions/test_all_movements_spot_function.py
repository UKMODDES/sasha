import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client
import bosdyn.client.util
from bosdyn.api.geometry_pb2 import Vec2
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from spot_common.spot_control.spot_actions.arm import make_robot_unstow_arm, make_robot_pick_up_object_in_pixel
from spot_common.spot_control.spot_actions.base_unit import make_robot_stand, make_robot_take_stance, make_robot_walk_x, \
    make_robot_walk_y, make_robot_turn, make_robot_walk_x_y_yaw, make_robot_roll_over, make_robot_selfright, \
    make_robot_sit, set_robot_to_walk_mode, set_robot_to_crawl_mode, set_robot_to_amble_mode, set_robot_orientation, \
    make_robot_reset_height, set_robot_to_jog_mode, set_robot_to_hop_mode, make_robot_change_height
from spot_common.spot_control.spot_actions.camera import make_robot_take_picture
from spot_common.spot_control.spot_connection import get_connected_robot
from spot_common.spot_functions.base_spot_function import BaseSpotFunction


class TestAllMovementsSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        pass

    def execute(self, options: Namespace):
        make_robot_stand(self.robot, self.command_client)
        # make_robot_take_stance(self.robot, self.command_client, 0.5, 0.5)
        make_robot_walk_x(-1, self.robot_state_client, self.command_client)
        make_robot_walk_y(-1, self.robot_state_client, self.command_client)
        make_robot_turn(0.5, self.robot_state_client, self.command_client)
        make_robot_walk_x_y_yaw(-1, 1, 0.5, self.robot_state_client, self.command_client)
        # make_robot_roll_over(self.command_client)
        # make_robot_selfright(self.command_client)
        make_robot_sit(self.command_client)
        set_robot_to_walk_mode(self.command_client)
        set_robot_to_crawl_mode(self.command_client)
        set_robot_to_amble_mode(self.command_client)
        set_robot_orientation(self.command_client, 0.5, 0.5, 0.5, 0.5)
        make_robot_reset_height(self.command_client)
        set_robot_to_jog_mode(self.command_client)
        set_robot_to_hop_mode(self.command_client)
        make_robot_change_height(self.command_client, 0.2)
        image_client = self.robot.ensure_client(ImageClient.default_service_name)
        image = make_robot_take_picture(self.robot, image_client)
        make_robot_unstow_arm(self.robot, self.command_client)
        manipulation_api_client = self.robot.ensure_client(ManipulationApiClient.default_service_name)
        make_robot_pick_up_object_in_pixel(self.robot, image, Vec2(0, 0), self.robot_state_client, manipulation_api_client)


    @staticmethod
    def main(argv=None):
        try:
            test_all_movements_spot_function = TestAllMovementsSpotFunction()
            robot = get_connected_robot('HelloSpotClient', test_all_movements_spot_function.username, test_all_movements_spot_function.password)
            options = test_all_movements_spot_function.parse_arguments(argv)
            test_all_movements_spot_function.execute_function_for_robot(robot, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not TestAllMovementsSpotFunction.main(sys.argv[1:]):
        sys.exit(1)
