from robot_control_class import RobotControl
import time

robotcontrol = RobotControl()

class MoveRobot:

    def __init__(self):
        self.robot= RobotControl()
        self.is_out_maze = False

    def check_out_maze(self):
        left = self.robot.get_laser(719)
        right = self.robot.get_laser(0)
        print ("Left side laser reading is %f" % left)
        print ("Right side laser reading is %f" % right)
        if str(left) == 'inf' and str(right) == 'inf':
            self.is_out_maze = True
            self.robot.stop_robot()
            print ("Turtlebot is out of the maze!")
        else:
            print ("Turtlebot is not out of the maze yet!")

    def direction_to_turn(self):
        left = self.robot.get_laser(719)
        right = self.robot.get_laser(0)
        print ("Left side laser reading is %f" % left)
        print ("Right side laser reading is %f" % right)
        if left > right:
            print ("Let's turn to the left!")
            self.robot.turn('counterclockwise', 0.2, 7.7)
        else:
            print ("Let's turn to the right!")
            self.robot.turn('clockwise', 0.2, 7.7)

    def main(self):
        while not self.is_out_maze:
            front = self.robot.get_laser(360)
            while front > 1.2 and not self.is_out_maze:
                print ("Keep moving forward!")
                self.robot.move_straight()
                front = self.robot.get_laser(360)
            self.robot.stop_robot()
            print ("Wall is near... Stop!")
            self.direction_to_turn()
            self.check_out_maze()

mr = MoveRobot()
mr.main()


