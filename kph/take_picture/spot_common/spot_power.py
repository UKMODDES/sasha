from bosdyn.client import Robot


def power_on_robot(
        robot: Robot
):
    # Now, we are ready to power on the robot. This call will block until the power
    # is on. Commands would fail if this did not happen. We can also check that the robot is
    # powered at any point.
    robot.logger.info("Powering on robot... This may take a several seconds.")
    robot.power_on(timeout_sec=20)
    assert robot.is_powered_on(), "Robot power on failed."
    robot.logger.info("Robot powered on.")


def power_off_robot(
        robot: Robot
):
    robot.logger.info('Sitting down and turning off.')

    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")