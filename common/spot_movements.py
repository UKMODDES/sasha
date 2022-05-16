import time

from bosdyn.api import arm_surface_contact_pb2, manipulation_api_pb2, geometry_pb2
from bosdyn.api.arm_surface_contact_pb2 import ArmSurfaceContact
from bosdyn.api.geometry_pb2 import Vec3, Vec2
from bosdyn.api.image_pb2 import Image
from bosdyn.api.manipulation_api_pb2 import PickObjectInImage
from bosdyn.api.trajectory_pb2 import SE3Trajectory
from bosdyn.client import Robot, math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state import RobotStateClient


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
) -> Image:
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


def _add_grasp_constraint(
        grasp: PickObjectInImage,
        robot_state_client: RobotStateClient,
        force_top_down_grasp: bool = False,
        force_horizontal_grasp: bool = False,
        force_45_angle_grasp: bool = False,
        force_squeeze_grasp: bool = False):
    # There are 3 types of constraints:
    #   1. Vector alignment
    #   2. Full rotation
    #   3. Squeeze grasp
    #
    # You can specify more than one if you want and they will be OR'ed together.

    # For these options, we'll use a vector alignment constraint.
    use_vector_constraint = force_top_down_grasp or force_horizontal_grasp

    # Specify the frame we're using.
    grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

    if use_vector_constraint:
        if force_top_down_grasp:
            # Add a constraint that requests that the x-axis of the gripper is pointing in the
            # negative-z direction in the vision frame.

            # The axis on the gripper is the x-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

            # The axis in the vision frame is the negative z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

        if force_horizontal_grasp:
            # Add a constraint that requests that the y-axis of the gripper is pointing in the
            # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

            # The axis on the gripper is the y-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

            # The axis in the vision frame is the positive z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vo)

        # We'll take anything within about 10 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

    elif force_45_angle_grasp:
        # Demonstration of a RotationWithTolerance constraint.  This constraint allows you to
        # specify a full orientation you want the hand to be in, along with a threshold.
        #
        # You might want this feature when grasping an object with known geometry and you want to
        # make sure you grasp a specific part of it.
        #
        # Here, since we don't have anything in particular we want to grasp,  we'll specify an
        # orientation that will have the hand aligned with robot and rotated down 45 degrees as an
        # example.

        # First, get the robot's position in the world.
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # Rotation from the body to our desired grasp.
        body_Q_grasp = math_helpers.Quat.from_pitch(0.785398)  # 45 degrees
        vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

        # Turn into a proto
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

        # We'll accept anything within +/- 10 degrees
        constraint.rotation_with_tolerance.threshold_radians = 0.17

    elif force_squeeze_grasp:
        # Tell the robot to just squeeze on the ground at the given point.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.squeeze_grasp.SetInParent()


def make_robot_pick_up_object_in_pixel(
        robot: Robot,
        image: Image,
        pick_vec: Vec2,
        robot_state_client: RobotStateClient,
        manipulation_api_client: ManipulationApiClient,
        force_top_down_grasp: bool = False,
        force_horizontal_grasp: bool = False,
        force_45_angle_grasp: bool = False,
        force_squeeze_grasp: bool = False
):
    # Build the proto
    grasp = manipulation_api_pb2.PickObjectInImage(
        pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
        frame_name_image_sensor=image.shot.frame_name_image_sensor,
        camera_model=image.source.pinhole)

    # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
    _add_grasp_constraint(grasp, robot_state_client, force_top_down_grasp, force_horizontal_grasp, force_45_angle_grasp, force_squeeze_grasp)

    # Ask the robot to pick up the object
    grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

    # Send the request
    cmd_response = manipulation_api_client.manipulation_api_command(
        manipulation_api_request=grasp_request)

    # Get feedback from the robot
    while True:
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_response.manipulation_cmd_id)

        # Send the request
        response = manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)

        print('Current state: ',
              manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

        if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
            break

        time.sleep(0.25)

    robot.logger.info('Finished grasp.')
    time.sleep(4.0)


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
