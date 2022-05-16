from bosdyn.api import arm_surface_contact_pb2
from bosdyn.api.arm_surface_contact_pb2 import ArmSurfaceContact
from bosdyn.api.geometry_pb2 import Vec3
from bosdyn.api.trajectory_pb2 import SE3Trajectory
from bosdyn.client import Robot
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.image import ImageClient


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


def make_robot_take_picture(
        robot: Robot,
        image_client: ImageClient,
        image_source: str = 'frontleft_fisheye_image'
):
    # Take a picture with a camera
    robot.logger.info('Getting an image from: ' + image_source)
    image_responses = image_client.get_image_from_sources([image_source])

    if len(image_responses) != 1:
        print('Got invalid number of images: ' + str(len(image_responses)))
        print(image_responses)
        image = None
    else:
        image = image_responses[0]

    return image


def make_robot_open_gripper(
        hand_pose_trajectory: SE3Trajectory,
        press_force_percentage: Vec3
) -> ArmSurfaceContact.Request:
    # Open the gripper
    gripper_cmd_packed = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    gripper_command = gripper_cmd_packed.synchronized_command.gripper_command.claw_gripper_command

    cmd = arm_surface_contact_pb2.ArmSurfaceContact.Request(
        pose_trajectory_in_task=hand_pose_trajectory,
        root_frame_name=ODOM_FRAME_NAME,
        press_force_percentage=press_force_percentage,
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
    return cmd
