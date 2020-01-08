from robot_control_class import RobotControl

import time

robotcontrol = RobotControl()

class MoveRobot:
    def __init__(self, motion, clockwise, speed, time):
        self.robot = RobotControl()
        self.motion = motion
        self.clockwise = clockwise
        self.speed = speed
        self.time = time
        self.time_turn = 2.5
        self.turn_speed = speed
        self.full_laser = self.robot.get_laser_full()
        self.full_laser2 = self.robot.get_laser_full()
        #Code above initialize the construct and the variable class wich belongs the class MoveROBOT
    def out_maze(self):
        corner = 0
        maximo = 0
        #While loop used to find the labirint exit, by searching the door though the sensor "inf" value
        #When the door is found then start the door_maze function below this one
        #The full laser data is get setting the maximp< infinito, so it will always get the data
        while(maximo < float("inf")):
            self.full_laser = self.robot.get_laser_full()
            maximo = 0
#The maximo was set to =0 above to enter in the while loop,but must be update in this loop to achieve a value
#bigger than 1 and so the robot contemplate a escape of a wall that it is close it. The code below is useful
#to let the robot get free from a "trap", in other words when it be very close to a wall
#The block below will turn the robot until if finds the higher distance value, while it is close the wall
            while(maximo < 1):
                #For loop to get the maximum value from the laser scan reading
                for value in self.full_laser:
                    if value > maximo:
                        maximo = value

                if(maximo < 1):
                    self.turn()
            print("Current Maximum laser distace is %f" % maximo)
#After getting the maximum value you need to make the robot move in its direction.Turn the robot around
#till its frontal laser is the close enough to the maximum value - this is represented in the code by max-1
#and max!=infinite arguments. now its facing the right direction, or at least almost there.The codebelow
#will turn the robot around until the FRONT LASER BEAM = 360 find the max distance and get out the turn loop
#To be honest the loop would calculate forever the max value because it has 720 beams so each beam should be
#compared while turning to give back the biggest value. This is done faster using a tolerance,. This tolerance
#is put with maximo - [value] in this case 1 meter, so when the robot beams have a value of max aroun 6 meters
#for example it will fast accecpt the distance of 5 meter as the "infinite" distance and get out the turn loop
#to start the move_straight loop
            while(self.robot.get_front_laser() < maximo -1 and maximo != float("inf")):
                print("Robot frontal laser distance: %f", self.robot.get_front_laser())
                self.turn()
#The code above is turning the robot until it finds the "first" distance shorter or close to the infinite
#distance value, when it arrives in this value (represented by max-1) it will get out the loop and stop
#turning. Then the code below enters in action ( to be honest is being compiled in paralell) so if the robot
#is not turning mean it is going ahead until find a wall (laser<1), then this programming block will stop_robot
# and the previous blocks will work (turn and turn the robot until find again an "almost-infinite" value)
#When one of these 3 blocks be reached the function stop_robot will work, avoiding simultaneously confusion
#while executing a move
            while (self.robot.get_front_laser() > 1):
                self.move_straight()
            self.stop_robot()

    def door_maze(self):
        #Define the region frontal the robot to laser reading, 270 - 470 correspond 45 degrees of range which
        #the robot will consider to calculate the max distace === infinit
        #Here the laser scan data will calculate the most distance value = infinit. If not achieve the infinit
        #value it means that it is near a wall and thus will go to the second loop (to turn and find the next
        #laser scan data which is "infinite") the counter pass reading all the matrix laser data values, until
        #get out the loop and then move_straight
#Her if the robot frontal (45 degrees cone range: 270:450) is already getting the infinite value, so the robot
#will go toward it. The counter +=1 is just to read all the values inside user laser ( the beams 270 to 450). If
#any of them calculate an infinite value the robot will go and try get out the maze
        i_counter = 0
        self.full_laser = self.robot.get_laser_full()
        use_laser = self.full_laser[270:450]
        for i in use_laser:
            if i == float("inf"):
                i_counter += 1

        #Just enter in this loop if the door is not find at first
        #Then the robot starts turn around until its facing the door
        #The counter is used to define wheter the robot is in the rigth direction or not
        #In case its not, then the robot will turn again till the criteria is met
#If the robot still did not find the infinite value (more distance from a wall), so it will need to turns
#because maybe it is turned with its back for the free destination. So the counter will read again the
#laser scan front robot range and turn and turn until find the infinite value (more distant from a obstacle)
#to get out the loop ( already read all the use_laser matrix values with counter and comapared to find the big one)
#and all the beams=90. So it can get out the loop, because found the greatest value and move straight out the maze!
        while(i_counter < 90):
            self.turn()
            self.full_laser = self.robot.get_laser_full()
            use_laser = self.full_laser[270:450]

            i_counter = 0
            for i in use_laser:
                if i == float("inf"):
                    i_counter += 1

        self.move_straight_time()



    def move_straight_time(self):
        self.robot.move_straight_time(self.motion, self.speed, self.time)
    def move_straight(self):
        self.robot.move_straight()
    def turn(self):
        self.robot.turn(self.clockwise,self.turn_speed,self.time_turn)

    def stop_robot(self):
        self.robot.stop_robot()

move_inside = MoveRobot('forward', 'clockwise', 0.75, 5)
move_inside.out_maze()
move_inside.door_maze()


