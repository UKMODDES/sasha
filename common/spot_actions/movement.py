import time

from bosdyn.client import Robot, frame_helpers, math_helpers
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder
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

