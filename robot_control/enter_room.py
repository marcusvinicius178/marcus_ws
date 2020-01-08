from robot_control_class import RobotControl

robotcontrol = RobotControl()

robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 6)
robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 7)