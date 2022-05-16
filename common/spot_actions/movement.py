import time

from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import Robot, frame_helpers, math_helpers
from bosdyn.client.frame_helpers import get_se2_a_tform_b, BODY_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
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


def make_robot_walk(
        body_tform_goal: SE2Pose,
        robot_state_client: RobotStateClient,
        frame_name: str,
        robot_command_client: RobotCommandClient
):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=False))
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
        time.sleep(1)


def make_robot_walk_x(
        dx: float,
        robot_state_client: RobotStateClient,
        frame_name: str,
        robot_command_client: RobotCommandClient
):

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=0, angle=0)
    make_robot_walk(body_tform_goal, robot_state_client, frame_name, robot_command_client)


def make_robot_walk_y(
        dy: float,
        robot_state_client: RobotStateClient,
        frame_name: str,
        robot_command_client: RobotCommandClient
):

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=0, y=dy, angle=0)
    make_robot_walk(body_tform_goal, robot_state_client, frame_name, robot_command_client)


def make_robot_turn(
        yaw: float,
        robot_state_client: RobotStateClient,
        frame_name: str,
        robot_command_client: RobotCommandClient
):

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=0, y=0, angle=yaw)
    make_robot_walk(body_tform_goal, robot_state_client, frame_name, robot_command_client)
