import rospy
import message_filters
import math
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

face = None
hand = None
glass = None
glove = None

def face_pose_callback(data):
    global face
    face = data

def hand_pose_callback(data):
    global hand
    hand = data
   
def vicon_glass_callback(data):
    global glass
    glass = data

def vicon_glove_callback(data):
    global glove, glass, hand, face
    glove = data

    vicon_rad = -math.atan2(glass.transform.translation.y - glove.transform.translation.y, glass.transform.translation.x - glove.transform.translation.x)
    vicon_deg = math.degrees(vicon_rad)

    face_rad = math.atan2(face.point.x - hand.point.x, face.point.z - hand.point.z)
    face_deg = math.degrees(face_rad)

    print vicon_deg, face_deg

rospy.init_node('accuracy')

rospy.Subscriber('/3dr/face_pose', PointStamped, face_pose_callback)
rospy.Subscriber('/3dr/right_hand_pose', PointStamped, hand_pose_callback)

rospy.Subscriber('/vicon/Glass2/Glass2', TransformStamped, vicon_glass_callback)
rospy.Subscriber('/vicon/Gloves1/Gloves1', TransformStamped, vicon_glove_callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
