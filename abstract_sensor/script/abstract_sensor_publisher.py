#! /usr/bin/env python

import rospy
from abstract_sensor.msg import Abstractsensor

rospy.init_node('abstract_sensor_publisher_node')
pub = rospy.Publisher('/abstractsensorpublisher', Abstractsensor, queue_size=1)
rate = rospy.Rate(2)
sensor_data = Abstractsensor() #create and object of the class Twist, a class among oters from geometry_msgs

Abstractsensor.location = [2, 4, 40, 35, 2, 1, 30, 0.5]
Abstractsensor.covariance = [1, 2, 3, 4,
                             5, 6, 7, 8,
                             9, 10, 11, 12,
                             13, 14, 15, 16]
Abstractsensor.x = 1.5
Abstractsensor.y = 3
Abstractsensor.xerror = 0.2
Abstractsensor.yerror = 0.15
Abstractsensor.existence_probability = 0.87
Abstractsensor.object_class = ["car", "truck", "motorcycle", "pedestrian", "bike"]
Abstractsensor.object_detected_feature = ["RL, RR"]


while not rospy.is_shutdown():
  pub.publish(sensor_data)
  rate.sleep()