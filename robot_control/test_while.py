from robot_control_class import RobotControl

rc = RobotControl()

wall_proximity = rc.get_laser(360)

while wall_proximity>=1:
        rc.move_straight()
        wall_proximity = rc.get_laser(360)
        print("The robot is located %f meter from the wall" % wall_proximity)

rc.stop_robot()

print("The wall is at %f meters, Stop it!!!" % wall_proximity)

