import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client
import bosdyn.client.util

from spot_common.spot_control.spot_actions.arm import make_robot_unstow_arm, make_robot_pick_up_object_in_pixel
from spot_common.spot_control.spot_actions.base_unit import make_robot_stand, make_robot_take_stance, make_robot_walk_x, \
    make_robot_walk_y, make_robot_turn, make_robot_walk_x_y_yaw, make_robot_roll_over, make_robot_selfright, \
    make_robot_sit, set_robot_to_walk_mode, set_robot_to_crawl_mode, set_robot_to_amble_mode, set_robot_orientation, \
    make_robot_reset_height, set_robot_to_jog_mode, set_robot_to_hop_mode, make_robot_change_height, \
    make_robot_walk_backwards_and_forwards, make_robot_walk_left_and_right, make_robot_turn_clockwise, \
    make_robot_turn_anticlockwise, make_robot_sit_and_stand_a_number_of_times, make_robot_shimmy_number_of_times
from spot_common.spot_control.spot_connection import get_connected_robot
from spot_common.spot_functions.base_spot_function import BaseSpotFunction


class GymnasticsRoutineSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        pass

    def dance(self):
        make_robot_stand(self.robot, self.command_client)
        set_robot_to_hop_mode(self.command_client)
        make_robot_walk_backwards_and_forwards(self.robot_state_client, self.command_client, 1, 2)
        make_robot_walk_left_and_right(self.robot_state_client, self.command_client, 1, 2)
        make_robot_walk_backwards_and_forwards(self.robot_state_client, self.command_client, 1, 1)
        make_robot_walk_left_and_right(self.robot_state_client, self.command_client, 1, 1)
        make_robot_walk_backwards_and_forwards(self.robot_state_client, self.command_client, 1, 1)
        make_robot_walk_left_and_right(self.robot_state_client, self.command_client, 1, 1)

        make_robot_turn_clockwise(self.robot_state_client, self.command_client)
        make_robot_turn_anticlockwise(self.robot_state_client, self.command_client)

        make_robot_sit_and_stand_a_number_of_times(self.robot, self.command_client, 2)

        make_robot_shimmy_number_of_times(self.command_client, 4, 0.5, 0.5, 0.5, 0.5)

        set_robot_orientation(self.command_client, 0.5, 0.5, 0.5, 0.5)
        make_robot_change_height(self.command_client, 0.2)
        make_robot_change_height(self.command_client, 0.8)
        make_robot_change_height(self.command_client, 0.2)
        make_robot_change_height(self.command_client, 0.8)

        make_robot_unstow_arm(self.robot, self.command_client)

    def execute(self, options: Namespace):
        for x in range(10):
            self.dance()

    @staticmethod
    def main(argv=None):
        try:
            test_all_movements_spot_function = GymnasticsRoutineSpotFunction()
            robot = get_connected_robot('HelloSpotClient', test_all_movements_spot_function.username, test_all_movements_spot_function.password)
            options = test_all_movements_spot_function.parse_arguments(argv)
            test_all_movements_spot_function.execute_function_for_robot(robot, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not GymnasticsRoutineSpotFunction.main(sys.argv[1:]):
        sys.exit(1)
