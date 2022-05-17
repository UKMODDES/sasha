from bosdyn.api.image_pb2 import Image
from bosdyn.client import Robot
from bosdyn.client.image import ImageClient


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
