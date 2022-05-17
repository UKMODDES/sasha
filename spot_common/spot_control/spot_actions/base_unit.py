import math
import time

from bosdyn.api import basic_command_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.api.robot_command_pb2 import RobotCommand
from bosdyn.api.spot.robot_command_pb2 import MobilityParams, HINT_AUTO, HINT_SPEED_SELECT_TROT, HINT_CRAWL, HINT_AMBLE, \
    HINT_JOG, HINT_HOP
from bosdyn.client import Robot, frame_helpers, math_helpers
from bosdyn.client.frame_helpers import get_se2_a_tform_b, BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.geometry import EulerZXY
from numpy.random._examples.cffi.extending import state


def make_robot_stand(
        robot: Robot,
        command_client: RobotCommandClient
):
    # Tell the robot to stand up. The command service is used to issue commands to a robot.
    # The set of valid commands for a robot depends on hardware configuration. See
    # SpotCommandHelper for more detailed examples on command building. The robot
    # command service requires timesync between the robot and the client.
    robot.logger.info("Commanding robot to stand...")
    blocking_stand(command_client, timeout_sec=10)
    robot.logger.info("Robot standing.")


def make_robot_sit(
        robot: Robot,
        command_client: RobotCommandClient
):
    # Tell the robot to stand up. The command service is used to issue commands to a robot.
    # The set of valid commands for a robot depends on hardware configuration. See
    # SpotCommandHelper for more detailed examples on command building. The robot
    # command service requires timesync between the robot and the client.
    robot.logger.info("Commanding robot to sit...")
    blocking_sit(command_client, timeout_sec=10)
    robot.logger.info("Robot sitting.")


def make_robot_take_stance(
        robot: Robot,
        command_client: RobotCommandClient,
        x_offset: float,
        y_offset: float
):
    robot.logger.info("Commanding robot to take stance...")
    vo_T_body = frame_helpers.get_se2_a_tform_b(state.kinematic_state.transforms_snapshot,
                                                frame_helpers.VISION_FRAME_NAME,
                                                frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

    pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, y_offset, 0)
    pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, -y_offset, 0)
    pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, y_offset, 0)
    pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, -y_offset, 0)

    stance_cmd = RobotCommandBuilder.stance_command(
        frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position,
        pos_fr_rt_vision.position, pos_hl_rt_vision.position, pos_hr_rt_vision.position)

    # Update end time
    stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
        robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))

    # Send the command
    command_client.robot_command(stance_cmd)

    robot.logger.info("Robot has taken stance.")


def make_robot_walk_to_pose(
        body_tform_goal: SE2Pose,
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    # robot.logger.info("Commanding robot to take stance...")
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=ODOM_FRAME_NAME, params=RobotCommandBuilder.mobility_params(stair_hint=False))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            return True


def make_robot_walk_x(
        dx: float,
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=0, angle=0)
    make_robot_walk_to_pose(body_tform_goal, robot_state_client, robot_command_client)


def make_robot_walk_y(
        dy: float,
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=0, y=dy, angle=0)
    make_robot_walk_to_pose(body_tform_goal, robot_state_client, robot_command_client)


def make_robot_turn(
        yaw: float,
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=0, y=0, angle=yaw)
    make_robot_walk_to_pose(body_tform_goal, robot_state_client, robot_command_client)


def make_robot_turn_clockwise(
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    make_robot_turn(math.pi*2, robot_state_client, robot_command_client)


def make_robot_turn_anticlockwise(
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):
    make_robot_turn(-(math.pi*2), robot_state_client, robot_command_client)


def make_robot_sit_and_stand(
        robot: Robot,
        command_client: RobotCommandClient
):
    make_robot_sit(robot, command_client)
    make_robot_stand(robot, command_client)


def make_robot_sit_and_stand_a_number_of_times(
        robot: Robot,
        command_client: RobotCommandClient,
        number_of_times: int
):
    for x in range(number_of_times):
        make_robot_sit_and_stand(robot, command_client)


def make_robot_walk_x_y_yaw(
        dx: float,
        dy: float,
        yaw: float,
        robot_state_client: RobotStateClient,
        robot_command_client: RobotCommandClient
):

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=yaw)
    make_robot_walk_to_pose(body_tform_goal, robot_state_client, robot_command_client)


def make_robot_walk_backwards_and_forwards(
        robot_state_client: RobotStateClient,
        command_client: RobotCommandClient,
        dx: float,
        number_of_times: int
):
    for x in range(number_of_times):
        make_robot_walk_x(dx, robot_state_client, command_client)
        make_robot_walk_x(-dx, robot_state_client, command_client)


def make_robot_walk_left_and_right(
        robot_state_client: RobotStateClient,
        command_client: RobotCommandClient,
        dy: float,
        number_of_times: int):
    for x in range(number_of_times):
        make_robot_walk_y(dy, robot_state_client, command_client)
        make_robot_walk_y(-dy, robot_state_client, command_client)


def _issue_command(
        command_client: RobotCommandClient,
        command: RobotCommand
):
    command_client.robot_command_async(command, end_time_secs=None)


def make_robot_roll_over(
        command_client: RobotCommandClient
):
    """Executes the battery-change pose command which causes the robot to sit down if
    standing then roll to its [right]/left side for easier battery changing.
    """
    command = RobotCommandBuilder.battery_change_pose_command(
        dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT)
    _issue_command(command_client, command)


def make_robot_selfright(
        command_client: RobotCommandClient
):
    """Executes selfright command, which causes the robot to automatically turn if
    it is on its back.
    """

    command = RobotCommandBuilder.selfright_command()
    _issue_command(command_client, command)


def set_robot_to_walk_mode(
        command_client: RobotCommandClient
):
    mobility_params = MobilityParams(
                locomotion_hint=HINT_SPEED_SELECT_TROT, stair_hint=0)

    cmd = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    _issue_command(command_client, cmd)


def set_robot_to_crawl_mode(
        command_client: RobotCommandClient
):
    mobility_params = MobilityParams(
        locomotion_hint=HINT_CRAWL, stair_hint=0)

    cmd = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    _issue_command(command_client, cmd)


def set_robot_to_amble_mode(
        command_client: RobotCommandClient
):
    mobility_params = MobilityParams(
        locomotion_hint=HINT_AMBLE, stair_hint=0)

    cmd = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    _issue_command(command_client, cmd)


def set_robot_orientation(
        command_client: RobotCommandClient,
        yaw=0.0,
        roll=0.0,
        pitch=0.0,
        height=0.0
):
    orientation = EulerZXY(yaw, roll, pitch)
    cmd = RobotCommandBuilder.synchro_stand_command(body_height=height,
                                                    footprint_R_body=orientation)
    _issue_command(command_client, cmd)


def make_robot_shimmy_number_of_times(
        command_client: RobotCommandClient,
        number_of_times: int,
        yaw=0.0,
        roll=0.0,
        pitch=0.0,
        height=0.0
):
    for x in range(number_of_times):
        set_robot_orientation(command_client, yaw, roll, pitch, height)
        set_robot_orientation(command_client, -yaw, roll, -pitch, height)
        set_robot_orientation(command_client, yaw, -roll, pitch, -height)
        set_robot_orientation(command_client, -yaw, -roll, -pitch, -height)


def make_robot_reset_height(
        command_client: RobotCommandClient
):
    set_robot_orientation(command_client, height=0.0)


def set_robot_to_jog_mode(
        command_client: RobotCommandClient
):
    make_robot_reset_height(command_client)
    mobility_params = MobilityParams(
        locomotion_hint=HINT_JOG, stair_hint=0)
    cmd = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    _issue_command(command_client, cmd)


def set_robot_to_hop_mode(
        command_client: RobotCommandClient
):
    make_robot_reset_height(command_client)
    mobility_params = MobilityParams(
        locomotion_hint=HINT_HOP, stair_hint=0)

    cmd = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    _issue_command(command_client, cmd)


def make_robot_change_height(
        command_client: RobotCommandClient,
        new_height: float
):
    set_robot_orientation(command_client, height=new_height)
