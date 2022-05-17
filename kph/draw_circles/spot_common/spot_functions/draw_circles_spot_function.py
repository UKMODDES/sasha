import argparse
import sys
from argparse import Namespace

import bosdyn
import bosdyn.client
import bosdyn.client.util
import numpy as np
from bosdyn.api import arm_surface_contact_service_pb2, geometry_pb2, arm_surface_contact_pb2, \
    trajectory_pb2
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b

from spot_common.spot_control.spot_connection import get_connected_robot
from spot_common.spot_functions.base_spot_function import BaseSpotFunction



class DrawCirclesSpotFunction(BaseSpotFunction):
    def _add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument(
            '-i', '--image_source', default='frontleft_fisheye_image', nargs=1, help=
            'Which camera to use to take the picture.'
        )

    def execute(self, options: Namespace):
        assert self.robot.has_arm(), "Robot requires an arm to run this example."

        arm_surface_contact_client = self.robot.ensure_client(ArmSurfaceContactClient.default_service_name)

        # Unstow the arm
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = self.command_client.robot_command(unstow)

        self.robot.logger.info("Unstow command issued.")
        block_until_arm_arrives(self.command_client, unstow_command_id, 3.0)

        # ----------
        #
        # Now we'll use the arm_surface_contact service to do an accurate position move with
        # some amount of force.
        #

        # Position of the hand:

        hand_start_x  = 0.75  # in front of the robot.
        hand_start_y = -0.14  # centered
        hand_z = 0  # will be ignored since we'll have a force in the Z axis.

        force_z = -0.05  # percentage of maximum press force, negative to press down
        # be careful setting this too large, you can knock the robot over
        percentage_press = geometry_pb2.Vec3(x=0, y=0, z=force_z)

        hand_vec3_start_rt_body = geometry_pb2.Vec3(x=hand_start_x, y=hand_start_y, z=hand_z)
        hand_vec3_end_rt_body = hand_vec3_start_rt_body

        # We want to point the hand straight down the entire time.
        qw = 0.707
        qx = 0
        qy = 0.707
        qz = 0
        body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Build a position trajectory
        body_T_hand1 = geometry_pb2.SE3Pose(position=hand_vec3_start_rt_body,
                                            rotation=body_Q_hand)
        body_T_hand2 = geometry_pb2.SE3Pose(position=hand_vec3_end_rt_body,
                                            rotation=body_Q_hand)

        robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        odom_T_hand1 = odom_T_flat_body * math_helpers.SE3Pose.from_obj(body_T_hand1)
        odom_T_hand2 = odom_T_flat_body * math_helpers.SE3Pose.from_obj(body_T_hand2)

        # Trajectory length
        trajectory_time = 5.0  # in seconds
        time_since_reference = seconds_to_duration(trajectory_time)

        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=odom_T_hand1.to_proto(), time_since_reference=seconds_to_duration(0))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=odom_T_hand2.to_proto(), time_since_reference=time_since_reference)

        hand_traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # Open the gripper
        gripper_cmd_packed = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
        gripper_command = gripper_cmd_packed.synchronized_command.gripper_command.claw_gripper_command

        radius = 0.06
        x_coords = []
        y_coords = []

        x_ = np.arange(hand_start_x - radius - 1, hand_start_x + radius + 1, dtype=float)
        y_ = np.arange(hand_start_y - radius - 1, hand_start_y + radius + 1, dtype=float)
        x, y = np.where((x_[:, np.newaxis] - hand_start_x) ** 2 + (y_ - hand_start_y) ** 2 <= radius ** 2)
        # x, y = np.where((np.hypot((x_-x0)[:,np.newaxis], y_-y0)<= radius)) # alternative implementation
        for x, y in zip(x_[x], y_[y]):
            x_coords.append(x)
            y_coords.append(y)

        _N_POINTS = len(x_coords)

        for ii in range(_N_POINTS + 1):
            cmd = arm_surface_contact_pb2.ArmSurfaceContact.Request(
                pose_trajectory_in_task=hand_traj,
                root_frame_name=ODOM_FRAME_NAME,
                press_force_percentage=percentage_press,
                x_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
                y_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
                z_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_FORCE,
                z_admittance=arm_surface_contact_pb2.ArmSurfaceContact.Request.
                    ADMITTANCE_SETTING_LOOSE,
                # Enable the cross term so that if the arm gets stuck in a rut, it will retract
                # upwards slightly, preventing excessive lateral forces.
                xy_to_z_cross_term_admittance=arm_surface_contact_pb2.ArmSurfaceContact.Request.
                    ADMITTANCE_SETTING_VERY_STIFF,
                gripper_command=gripper_command)

            # Enable walking
            cmd.is_robot_following_hand = True

            # A bias force (in this case, leaning forward) can help improve stability.
            bias_force_x = -25
            cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=x_[ii], y=y_[ii], z=0))

            proto = arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd)

            # Send the request
            self.robot.logger.info('Running arm surface contact...')
            arm_surface_contact_client.arm_surface_contact_command(proto)

    @staticmethod
    def main(argv=None):
        try:
            draw_circles_spot_function = DrawCirclesSpotFunction()
            robot = get_connected_robot('OlympicCirclesSpotClient', draw_circles_spot_function.username, draw_circles_spot_function.password)
            options = draw_circles_spot_function.parse_arguments(argv)
            draw_circles_spot_function.execute_function_for_robot(robot, options)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.error("Echoing through Spot threw an exception: %r", exc)
            return False


if __name__ == '__main__':
    if not DrawCirclesSpotFunction.main(sys.argv[1:]):
        sys.exit(1)
