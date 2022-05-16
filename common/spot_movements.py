from bosdyn.client import Robot
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, block_until_arm_arrives


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


def make_robot_unstow_arm(
        robot: Robot,
        command_client: RobotCommandClient
):
    # Unstow the arm
    unstow = RobotCommandBuilder.arm_ready_command()

    # Issue the command via the RobotCommandClient
    unstow_command_id = command_client.robot_command(unstow)

    robot.logger.info("Unstow command issued.")
    block_until_arm_arrives(command_client, unstow_command_id, 3.0)
