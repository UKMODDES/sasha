import numpy as np
import cv2
from skimage import measure, morphology
from spot_common.spot_control import spot_connection
from spot_common.spot_control import spot_power
from spot_common.spot_control.spot_actions import arm
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder

def get_midpoint_blue(cv2_img, on_screen=False):
    result = cv2_img.copy()

    hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
    
    lower_col = np.array([69, 100, 100])
    upper_col = np.array([110, 255, 255])
    
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_col, upper_col)

    out = morphology.remove_small_objects(mask, min_size=500)

    inter = cv2.morphologyEx(out, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))

    contours, _ = cv2.findContours(inter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if (contours):

        largest_contour = max(contours, key=cv2.contourArea) 
    
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
    
        if (on_screen):
            cv2.drawContours(result, largest_contour, -1, (255,0,0), 10)
            cv2.circle(result, (cx, cy), 7, (0, 0, 255), -1)
            (h,w) = result.shape[:2]
            r = 2
            resized_result = cv2.resize(result, (int(w/r), int(h/r)))
            cv2.imshow("result", resized_result)
            cv2.waitKey(0)
    
        return Vec2(x=cx, y=cy)

    else:
        return None

def get_hand_color_image(image_client):
    image_responses = image_client.get_image_from_sources(['hand_color_image'])
    if len(image_responses) != 1:
        print('Error: did not get exactly one image response.')
        sys.exit(1)

    resp = image_responses[0]

    # Display the image to the user
    image = image_responses[0]
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
    else:
        dtype = np.uint8
    img = np.fromstring(image.shot.image.data, dtype=dtype)
    if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(image.shot.image.rows, image.shot.image.cols)
    else:
        img = cv2.imdecode(img, -1)

    return img

def main():
    robot = spot_connection.get_connected_robot("prefix",  "192.168.80.3", False)
    spot_power.power_on_robot(robot)
    image_client = robot.ensure_client(GripperCameraParamClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    manip_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    balls_in_image = True

    while balls_in_image:
        # unstow arm
        arm.make_robot_unstow_arm(robot, command_client)
        # open claw
        arm_img = get_hand_color_image(image_client)
        cv2.imwrite("/app/armpic.png", arm_img.copy())
        ball_midpoint = get_midpoint_blue(arm_img, True)

        if (ball_midpoint != None):
            print(ball_midpoint)
            arm.make_robot_pick_up_object_in_pixel(robot, arm_img.copy(), ball_midpoint, force_top_down_grasp=True)

            # move arm to side
            # open claw

            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        else:
            balls_in_image = False
        # stow arm
        arm.make_robot_stow_arm(robot, command_client)

    spot_power.power_off_robot(robot)

if __name__=="__main__":
    main()
