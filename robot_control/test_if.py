from robot_control_class import RobotControl

rc = RobotControl()

wall = rc.get_laser(360)

if wall<=1:
    rc.stop_robot()
else:
    rc.move_straight()
print("The laser value received was: ", wall)

