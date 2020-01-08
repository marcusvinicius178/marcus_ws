from robot_control_class import RobotControl


robot = RobotControl()

def movimento():

    global a
    global b

    a = "forward"
    b = "counter-clockwise"

movimento()
direcao = robot.move_straight_time(a, 0.75, 5)
giro = robot.turn(b, 0.45, 10)


print ("O robo se deslocou com os seguinte parametros:", direcao )

print ("O robo girou com os seguintes parametros:", giro )