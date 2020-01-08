from robot_control_class import RobotControl

import time

robot = RobotControl()

moving = int(input("Entry with the time your robot must move:"))
def move_x_seconds(moving):

    robot.move_straight()
    time.sleep(moving)
    robot.stop_robot()

move_x_seconds(3)