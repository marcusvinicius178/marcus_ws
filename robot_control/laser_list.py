from robot_control_class import RobotControl

robot = RobotControl()

def get_laser_summit(beam20,beam220,beam500):

    a = robot.get_laser_summit(beam20)
    b = robot.get_laser_summit(beam220)
    c = robot.get_laser_summit(beam500)

    return [a, b, c]

laser = get_laser_summit(20, 220, 500)

print (laser[0])
print (laser[1])
print (laser[2])