import rospy
import math
from sensor_msgs.msg import PointCloud
from people_msgs.msg import PositionMeasurementArray


def face_cloud_callback(msg):
    rospy.loginfo("I received some point cloud info")
    print(msg)

def pos_meas_callback(data):
    rospy.loginfo("I received some position info")
    print len(data)

def listener():

    rospy.init_node('face_detector', anonymous=True)

    face_cloud = rospy.Subscriber("/face_detector/faces_cloud", PointCloud, face_cloud_callback)
    face_tracker = rospy.Subscriber("/face_detector/people_tracker_measurements_array", PositionMeasurementArray, pos_meas_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()