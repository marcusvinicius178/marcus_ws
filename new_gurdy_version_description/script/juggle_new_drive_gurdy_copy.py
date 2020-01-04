#!/usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs import Twist
"""
Topics To Write on:
type: std_msgs/Float64
/new_gurdy_drive/joint_state_controller/command
/new_gurdy_drive/head_aux_1_joint_position_controller/command
/new_gurdy_drive/head_aux_2_joint_position_controller/command
/new_gurdy_drive/head_aux_3_joint_position_controller/command
/new_gurdy_drive/auxs_upperlegM1_joint_position_controller/command
/new_gurdy_drive/auxs_upperlegM1_joint_position_controller/command
/new_gurdy_drive/auxs_upperlegM1_joint_position_controller/command
/new_gurdy_drive/upperlegM1_lowerlegM1_joint_position_controller/command
/new_gurdy_drive/upperlegM2_lowerlegM2_joint_position_controller/command
/new_gurdy_drive/upperlegM3_lowerlegM3_joint_position_controller/command
/new_gurdy_drive/internal_lowerleg1_joint_position_controller/command
/new_gurdy_drive/internal_lowerleg1_joint_position_controller/command
/new_gurdy_drive/internal_lowerleg1_joint_position_controller/command
/new_gurdy_drive/basefoot_peg_M1_joint_position_controller/command
/new_gurdy_drive/basefoot_peg_M2_joint_position_controller/command
/new_gurdy_drive/basefoot_peg_M3_joint_position_controller/command
"""

class gurdyJointMover(object):
#Publishing values for head-auxiliar links' joints
    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("New_Gurdy_Drive JointMover Initialising...")

        self.pub_head_aux_1_joint_position = rospy.Publisher(
            '/new_gurdy_drive/head_aux_1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_head_aux_2_joint_position = rospy.Publisher(
            '/new_gurdy_drive/head_aux_2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_head_aux_3_joint_position = rospy.Publisher(
            '/new_gurdy_drive/head_aux_3_joint_position_controller/command',
            Float64,
            queue_size=1)
#Publishing values for auxiliar-upperlegs links' joints
        self.pub_auxs_upperlegM1_joint_position = rospy.Publisher(
            '/new_gurdy_drive/auxs_upperlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_auxs_upperlegM2_joint_position = rospy.Publisher(
            '/new_gurdy_drive/auxs_upperlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_auxs_upperlegM3_joint_position = rospy.Publisher(
            '/new_gurdy_drive/auxs_upperlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
#Publishing values for upperlegs-lowerlegs links' joints
        self.pub_upperlegM1_lowerlegM1_joint_position = rospy.Publisher(
            '/new_gurdy_drive/upperlegM1_lowerlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM2_lowerlegM2_joint_position = rospy.Publisher(
            '/new_gurdy_drive/upperlegM2_lowerlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM3_lowerlegM3_joint_position = rospy.Publisher(
            '/new_gurdy_drive/upperlegM3_lowerlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
#Publishing values for lowerlegs-internal links' joints
        self.pub_internal_lowerleg1_joint_position = rospy.Publisher(
            '/new_gurdy_drive/internal_lowerleg1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_internal_lowerleg2_joint_position = rospy.Publisher(
            '/new_gurdy_drive/internal_lowerleg2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_internal_lowerleg3_joint_position = rospy.Publisher(
            '/new_gurdy_drive/internal_lowerleg3_joint_position_controller/command',
            Float64,
            queue_size=1)
#Publishing values for internal-foot links'joints NOT NECESSARY - THERE ARE WHELLS PLUGIN
    #    self.pub_basefoot_peg_M1_joint_position= rospy.Publisher(
    #        '/new_gurdy_drive/basefoot_peg_M1_joint_position_controller/command',
    #        Float64,
    #        queue_size=1)
    #    self.pub_basefoot_peg_M2_joint_position = rospy.Publisher(
    #        '/new_gurdy_drive/basefoot_peg_M2_joint_position_controller/command',
    #        Float64,
    #        queue_size=1)
    #    self.pub_basefoot_peg_M3_joint_position = rospy.Publisher(
    #        '/new_gurdy_drive/basefoot_peg_M3_joint_position_controller/command',
    #        Float64,
    #        queue_size=1)

        joint_states_topic_name = "/new_gurdy_drive/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.gurdy_joints_callback)
        gurdy_joints_data = None
        rate = rospy.Rate(2)
        while gurdy_joints_data is None:
            try:
                gurdy_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass
            rate.sleep()

        self.gurdy_joint_dictionary = dict(zip(gurdy_joints_data.name, gurdy_joints_data.position))

    def move_gurdy_all_joints(self, head_aux1_angle, head_aux2_angle, head_aux3_angle,
    aux_upperlegM1_angle, aux_upperlegM2_angle, aux_upperlegM3_angle,
    upper_lowerlegM1_angle, upper_lowerlegM2_angle, upper_lowerlegM3_angle,
    lower_internallegM1_value, lower_internallegM2_value, lower_internallegM3_value):


        head_aux1 = Float64()
        head_aux1.data = head_aux1_angle
        head_aux2 = Float64()
        head_aux2.data = head_aux2_angle
        head_aux3 = Float64()
        head_aux3.data = head_aux3_angle

        aux_upperlegM1 = Float64()
        aux_upperlegM1.data = aux_upperlegM1_angle
        aux_upperlegM2 = Float64()
        aux_upperlegM2.data = aux_upperlegM2_angle
        aux_upperlegM3 = Float64()
        aux_upperlegM3.data = aux_upperlegM3_angle

        upper_lowerlegM1 = Float64()
        upper_lowerlegM1.data = upper_lowerlegM1_angle
        upper_lowerlegM2 = Float64()
        upper_lowerlegM2.data = upper_lowerlegM2_angle
        upper_lowerlegM3 = Float64()
        upper_lowerlegM3.data = upper_lowerlegM3_angle

        lower_internallegM1 = Float64()
        lower_internallegM1.data = lower_internallegM1_value
        lower_internallegM2 = Float64()
        lower_internallegM2.data = lower_internallegM2_value
        lower_internallegM3 = Float64()
        lower_internallegM3.data = lower_internallegM3_value


        self.pub_head_aux_1_joint_position.publish(head_aux1)
        self.pub_head_aux_2_joint_position.publish(head_aux2)
        self.pub_head_aux_3_joint_position.publish(head_aux3)

        self.pub_auxs_upperlegM1_joint_position.publish(aux_upperlegM1)
        self.pub_auxs_upperlegM2_joint_position.publish(aux_upperlegM2)
        self.pub_auxs_upperlegM3_joint_position.publish(aux_upperlegM3)

        self.pub_upperlegM1_lowerlegM1_joint_position.publish(upper_lowerlegM1)
        self.pub_upperlegM2_lowerlegM2_joint_position.publish(upper_lowerlegM2)
        self.pub_upperlegM3_lowerlegM3_joint_position.publish(upper_lowerlegM3)

        self.pub_internal_lowerleg1_joint_position.publish(lower_internallegM1)
        self.pub_internal_lowerleg2_joint_position.publish(lower_internallegM2)
        self.pub_internal_lowerleg3_joint_position.publish(lower_internallegM3)


    def gurdy_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.gurdy_joint_dictionary = dict(zip(msg.name, msg.position))


    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def gurdy_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint', 'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.gurdy_joint_dictionary.get(joint_name)
        if not joint_reading:
            print "self.gurdy_joint_dictionary="+str(self.gurdy_joint_dictionary)
            print "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error
#similar recebe valor anterior (o dif_angles verifica se o valor lido na junta "angle=join_reading"
#é o mesmo [se já chegou no valor que o usuário entrou "value"], se o valor = dif_angles for menor que a
#tolerancia "error<0.1", então é porque já chegou). se menor que 0.1 similar retorna TRUE ou então False

        return similar

    def gurdy_movement_look(self, head_aux1_angle, head_aux2_angle, head_aux3_angle, aux_upperlegM1_angle,
    aux_upperlegM2_angle, aux_upperlegM3_angle,upperlegM1_angle, upperlegM2_angle, upperlegM3_angle,
    lower_internallegM1_value,lower_internallegM2_value, lower_internallegM3_value):
        """
        Move:
        'head_aux1_joint',
        'head_aux2_joint',
        'head_aux3_joint',
        'auxs_upperlegM1_joint',
        'auxs_upperlegM2_joint',
        'auxs_upperlegM3_joint',
        'upperlegM1_lowerlegM1_joint',
        'upperlegM2_lowerlegM2_joint',
        'upperlegM3_lowerlegM3_joint',
        'internal_lowerlegM1_joint',
        'internal_lowerlegM2_joint',
        'internal_lowerlegM3_joint',

        :return:
        """
        check_rate = 5.0

        position_head_aux1 = head_aux1_angle
        position_head_aux2 = head_aux1_angle
        position_head_aux3 = head_aux1_angle

        position_auxs_upperlegM1 = aux_upperlegM1_angle
        position_auxs_upperlegM2 = aux_upperlegM2_angle
        position_auxs_upperlegM2 = aux_upperlegM3_angle


        position_upperlegM1_lowerlegM1 = upperlegM1_angle
        position_upperlegM2_lowerlegM2 = upperlegM2_angle
        position_upperlegM3_lowerlegM3 = upperlegM3_angle

        position_internal_lowerlegM1 = lower_internallegM1_value
        position_internal_lowerlegM2 = lower_internallegM2_value
        position_internal_lowerlegM3 = lower_internallegM3_value

        similar_head_aux1 = False
        similar_head_aux2 = False
        similar_head_aux3 = False

        similar_auxs_upperlegM1 = False
        similar_auxs_upperlegM2 = False
        similar_auxs_upperlegM3 = False

        similar_upperlegM1_lowerlegM1 = False
        similar_upperlegM2_lowerlegM2 = False
        similar_upperlegM3_lowerlegM3 = False

        similar_internal_lowerlegM1 = False
        similar_internal_lowerlegM2 = False
        similar_internal_lowerlegM3 = False

        rate = rospy.Rate(check_rate)
        while not (similar_head_aux1 and similar_head_aux2 and similar_head_aux3 and similar_auxs_upperlegM1 and similar_auxs_upperlegM2 and similar_auxs_upperlegM1
        and similar_upperlegM1_lowerlegM1 and similar_upperlegM2_lowerlegM2 and similar_upperlegM3_lowerlegM3 and similar_internal_lowerlegM1 and
        similar_internal_lowerlegM2 and similar_internal_lowerlegM3):
            self.move_gurdy_all_joints(position_head_aux1,
                                       position_head_aux2,
                                       position_head_aux3,
                                       position_auxs_upperlegM1,
                                       position_auxs_upperlegM2,
                                       position_auxs_upperlegM3,
                                       position_upperlegM1_lowerlegM1,
                                       position_upperlegM2_lowerlegM2,
                                       position_upperlegM3_lowerlegM3,
                                       position_internal_lowerlegM1,
                                       position_internal_lowerlegM2,
                                       position_internal_lowerlegM3)
            similar_head_aux1 = self.gurdy_check_continuous_joint_value(joint_name="head_aux_1_joint",
                                                                         value=position_head_aux1)
            similar_head_aux2 = self.gurdy_check_continuous_joint_value(joint_name="head_aux_2_joint",
                                                                         value=position_head_aux2)
            similar_head_aux3 = self.gurdy_check_continuous_joint_value(joint_name="head_aux_3_joint",
                                                                         value=position_head_aux3)
            similar_auxs_upperlegM1 = self.gurdy_check_continuous_joint_value(joint_name="auxs_upperlegM1_joint",
                                                                         value=position_auxs_upperlegM1)
            similar_auxs_upperlegM2 = self.gurdy_check_continuous_joint_value(joint_name="auxs_upperlegM2_joint",
                                                                         value=position_auxs_upperlegM2)
            similar_auxs_upperlegM3 = self.gurdy_check_continuous_joint_value(joint_name="auxs_upperlegM3_joint",
                                                                         value=position_auxs_upperlegM3)
            similar_upperlegM1_lowerlegM1 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM1_lowerlegM1_joint",
                                                                         value=position_upperlegM1_lowerlegM1)
            similar_upperlegM2_lowerlegM2 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM2_lowerlegM2_joint",
                                                                         value=position_upperlegM2_lowerlegM2)
            similar_upperlegM3_lowerlegM3 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM3_lowerlegM3_joint",
                                                                         value=position_upperlegM3_lowerlegM3)
            similar_internal_lowerlegM1 = self.gurdy_check_continuous_joint_value(joint_name="internal_lowerlegM1_joint",
                                                                         value=position_internal_lowerlegM1)
            similar_internal_lowerlegM2 = self.gurdy_check_continuous_joint_value(joint_name="internal_lowerlegM2_joint",
                                                                         value=position_internal_lowerlegM2)
            similar_internal_lowerlegM3 = self.gurdy_check_continuous_joint_value(joint_name="internal_lowerlegM3_joint",
                                                                         value=position_internal_lowerlegM3)

            rate.sleep()

    def gurdy_init_pos_sequence(self):
        """
        HEAD-AUX limits lower="-1.55" upper="1.55"
        AUX-UPPERLEG limits lower="-1.55" upper="0.0"
        UPPERLEG-LOWERLEG limits lower="-2.9" upper="1.5708"
        LOWERLEG-INTERNAL limits lower="0" upper="0.02"
        :return:
        """
        head_aux1_angle = -1.55
        head_aux2_angle = -1.55
        head_aux3_angle = -1.55
        aux_upperlegM1_angle = -1.55
        aux_upperlegM2_angle = -1.55
        aux_upperlegM3_angle = -1.55
        upper_lowerlegM1_angle = -2.9
        upper_lowerlegM2_angle = -2.9
        upper_lowerlegM3_angle = -2.9
        lower_internallegM1_value = 0.0
        lower_internallegM2_value = 0.0
        lower_internallegM3_value = 0.0
        self.gurdy_movement_look(head_aux1_angle,
                                 head_aux2_angle,
                                 head_aux3_angle,
                                 aux_upperlegM1_angle,
                                 aux_upperlegM2_angle,
                                 aux_upperlegM3_angle,
                                 upper_lowerlegM1_angle,
                                 upper_lowerlegM2_angle,
                                 upper_lowerlegM3_angle,
                                 lower_internallegM1_value,
                                 lower_internallegM2_value,
                                 lower_internallegM3_value)

        head_aux1_angle = 1.55
        head_aux2_angle = 1.55
        head_aux3_angle = 1.55
        aux_upperlegM1_angle = 0
        aux_upperlegM2_angle = 0
        aux_upperlegM3_angle = 0
        upper_lowerlegM1_angle = 1.5708
        upper_lowerlegM2_angle = 1.5708
        upper_lowerlegM3_angle = 1.5708
        lower_internallegM1_value = 0.02
        lower_internallegM2_value = 0.02
        lower_internallegM3_value = 0.02
        self.gurdy_movement_look(head_aux1_angle,
                                 head_aux2_angle,
                                 head_aux3_angle,
                                 aux_upperlegM1_angle,
                                 aux_upperlegM2_angle,
                                 aux_upperlegM3_angle,
                                 upper_lowerlegM1_angle,
                                 upper_lowerlegM2_angle,
                                 upper_lowerlegM3_angle,
                                 lower_internallegM1_value,
                                 lower_internallegM2_value,
                                 lower_internallegM3_value)

    def gurdy_hop(self, num_hops=15):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """

        upper_delta = 1
        basic_angle = -1.55
        angle_change = random.uniform(0.2, 0.7)
        head_aux1_angle = basic_angle
        lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0

        #self.gurdy_init_pos_sequence()
        for repetitions in range(num_hops):
            self.gurdy_movement_look(head_aux1_angle,
                                 head_aux2_angle,
                                 head_aux3_angle,
                                 aux_upperlegM1_angle,
                                 aux_upperlegM2_angle,
                                 aux_upperlegM3_angle,
                                 upper_lowerlegM1_angle,
                                 upper_lowerlegM2_angle,
                                 upper_lowerlegM3_angle,
                                 lower_internallegM1_value,
                                 lower_internallegM2_value,
                                 lower_internallegM3_value)

            upper_delta *= -1
            if upper_delta < 0:
                upperlegM_angle = basic_angle + angle_change
            else:
                upperlegM_angle = basic_angle
            lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0


    def gurdy_moverandomly(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = random.uniform(-1.55, 0.0)
        upperlegM2_angle = random.uniform(-1.55, 0.0)
        upperlegM3_angle = random.uniform(-1.55, 0.0)
        lowerlegM1_angle = random.uniform(-2.9, pi/2)
        lowerlegM2_angle = random.uniform(-2.9, pi/2)
        lowerlegM3_angle = random.uniform(-2.9, pi/2)
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Gurdy...")
        while not rospy.is_shutdown():
            #self.gurdy_init_pos_sequence()
            self.gurdy_moverandomly()
            self.gurdy_hop()

if __name__ == "__main__":
    gurdy_jointmover_object = gurdyJointMover()
    gurdy_jointmover_object.movement_random_loop()