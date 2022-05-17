# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import argparse
import math
import sys
import time

import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import robot_command_pb2, arm_surface_contact_service_pb2, geometry_pb2, arm_surface_contact_pb2, \
    trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration
from bosdyn.api import (arm_surface_contact_pb2, arm_surface_contact_service_pb2, geometry_pb2,
                        trajectory_pb2)
from bosdyn.client import math_helpers
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b

radius = 0.06
hand_z = 0  # will be ignored since we'll have a force in the Z axis.

force_z = -0.05  # percentage of maximum press force, negative to press down
# be careful setting this too large, you can knock the robot over
percentage_press = geometry_pb2.Vec3(x=0, y=0, z=force_z)

def draw_circle(hand_start_x, hand_start_y, robot_state):
    
        hand_vec3_start_rt_body = geometry_pb2.Vec3(x=hand_start_x, y=hand_start_y, z=hand_z)
        hand_vec3_end_rt_body = hand_vec3_start_rt_body
        print(hand_start_x)
        print(hand_start_y)
        print(radius)
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

def surface_contact(config):
    sdk = bosdyn.client.create_standard_sdk('ArmSurfaceContactExample')

    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot requires an arm to run this example."

    arm_surface_contact_client = robot.ensure_client(ArmSurfaceContactClient.default_service_name)

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
    #                                "such as the estop SDK example, to configure E-Stop."
                                    
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_state = robot_state_client.get_robot_state()

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take a several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # SpotCommandHelper for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")

        # Unstow the arm
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = command_client.robot_command(unstow)

        robot.logger.info("Unstow command issued.")
        block_until_arm_arrives(command_client, unstow_command_id, 3.0)

    starting_x  = 0.75  # in front of the robot.
    starting_y = -0.14  # centered

    #Draw top left circle
    draw_circle(starting_x, starting_y, robot_state)
        
    #Draw top middle circle
    draw_circle(starting_x, starting_y-0.14, robot_state)
        
    #Draw top right circle
    draw_circle(starting_x, starting_y-0.28, robot_state)
        
    #Draw bottom left circle
    draw_circle(starting_x-0.07, starting_y-0.07, robot_state)
        
    #Draw bottom right circle
    draw_circle(starting_x-0.07, starting_y-0.14, robot_state)
    
    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")


def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        surface_contact(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
