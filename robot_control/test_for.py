from robot_control_class import RobotControl

rc = RobotControl()

maximo = 0

maior = rc.get_laser_full()
#maior2 = [20, 40, 90]

for value in maior:
    if value > maximo:
        maximo = value

print "Max value of laser data:%f " % maximo

