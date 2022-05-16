import bosdyn.client
import bosdyn.client.util
import bosdyn.client.lease
from bosdyn.client import Robot
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

from common.spot_functions.base_spot_function import BaseSpotFunction
from common.spot_movements import make_robot_stand
from common.spot_power import power_on_robot, power_off_robot


def execute_function_for_robot(
        robot: Robot,
        spot_function: BaseSpotFunction,
        *args
):
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        power_on_robot(robot)
        try:
            command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            make_robot_stand(robot, command_client)
            spot_function.prepare(robot, robot_state_client, lease_client, command_client)
            spot_function.execute(args)
        finally:
            power_off_robot(robot)
