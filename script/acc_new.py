import rospy
import message_filters
import math
import tf.transformations
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped


face = None
hand = None
glass = None
glove = None
camera = None
summ = 0
counter = 0
cam_roll = 0
cam_pitch = 0
cam_yaw = 0
def face_pose_callback(data):
    global face
    face = data

def hand_pose_callback(data):
    global hand
    hand = data
   
def vicon_glass_callback(data):
    global glass
    glass = data

def vicon_camera_callback(data):
    global camera, cam_roll, cam_pitch, cam_yaw
    camera = data
    quaternion = (camera.transform.rotation.x, camera.transform.rotation.y, camera.transform.rotation.z, camera.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    cam_roll = euler[0]# * 180/math.pi
    cam_pitch = euler[1]# * 180/math.pi
    cam_yaw = euler[2]# * 180/math.pi
#    print ("{:10.2f}".format(cam_roll*180/math.pi)+"{:10.2f}".format(cam_pitch*180/math.pi)+"{:10.2f}".format(cam_yaw*180/math.pi))

def vicon_glove_callback(data):
    global glove, glass, hand, face, summ, counter, cam_roll, cam_pitch, cam_yaw
    glove = data
#    sum = 0
#    counter = 0
    if (glass == None or glove == None or hand == None or face == None):
      return

    A = cam_yaw
    B = cam_pitch
    C = cam_roll
    # print "These-->" ,A * 180/ math.pi, C * 180/ math.pi, C * 180/ math.pi
    # A = math.pi 
    # B = 0
    # C = 0
    
    X = -glove.transform.translation.x + glass.transform.translation.x
    Y = glove.transform.translation.y - glass.transform.translation.y
    Z = glove.transform.translation.z - glass.transform.translation.z

    # X = 1
    # Y = 0
    # Z = 0

    Rx = (math.cos(A)*math.cos(B)*X) + ((math.cos(A)*math.sin(B)*math.sin(C)-math.sin(A)*math.cos(C))*Y) + ((math.cos(A)*math.sin(B)*math.cos(C)+math.sin(A)*math.sin(C))*Z)
    Ry = (math.sin(A)*math.cos(B)*X) + ((math.sin(A)*math.sin(B)*math.sin(C)+math.cos(A)*math.cos(C))*Y) + ((math.sin(A)*math.sin(B)*math.cos(C)-math.cos(A)*math.sin(C))*Z)
    Rz = (-math.sin(B)*X) + (math.cos(B)*math.cos(C)*Y) + (math.cos(B)*math.cos(C)*Z)

    # print Rx, Ry, Rz
    # print "HI", math.sin(math.pi/2)
    # vicon_yaw_rad = math.atan2(Ry, Rx)
    vicon_yaw_rad = math.atan2(Ry, Rx)
    vicon_yaw_deg = math.degrees(vicon_yaw_rad)

    vicon_pitch_rad = math.atan2(math.sqrt(math.pow(Rx, 2) + math.pow(Ry, 2)), Rz) - math.pi/2
    vicon_pitch_deg = math.degrees(vicon_pitch_rad)

#    vicon_rad = -math.atan2(glass.transform.translation.y - glove.transform.translation.y, glass.transform.translation.x - glove.transform.translation.x)
#    vicon_deg = math.degrees(vicon_rad)

    face_yaw_rad = math.atan2(face.point.x - hand.point.x, face.point.z - hand.point.z)
    face_yaw_deg = math.degrees(face_yaw_rad)

    face_pitch_rad = math.atan2(math.sqrt(math.pow(face.point.x - hand.point.x, 2) + math.pow(face.point.z - hand.point.z, 2)), face.point.y - hand.point.y) - math.pi/2
    face_pitch_deg = math.degrees(face_pitch_rad)

    print vicon_yaw_deg, vicon_pitch_deg, face_yaw_deg, face_pitch_deg
#    face_rad = math.atan2(face.point.x - hand.point.x, face.point.z - hand.point.z)
#    face_deg = math.degrees(face_rad)

#    print face_deg, vicon_deg

    accuracy = abs(vicon_yaw_deg - face_yaw_deg) / 180
    summ = summ + accuracy
 #   print summ
    if counter == 400:
      print face.point.z
      acy = summ / counter 
      summ = 0
      counter = 0
      print "Here---------------------------->", acy
    else:
      counter = counter + 1

#    print vicon_deg, face_deg, face.point.z
#    print accuracy, face.point.z
rospy.init_node('accuracy')
rospy.Subscriber('/3dr/face_ave_pose', PointStamped, face_pose_callback)
rospy.Subscriber('/3dr/hand_ave_pose', PointStamped, hand_pose_callback)

rospy.Subscriber('/vicon/Glass2/Glass2', TransformStamped, vicon_glass_callback)
rospy.Subscriber('/vicon/Cam23/Cam23', TransformStamped, vicon_camera_callback)
rospy.Subscriber('/vicon/Gloves1/Gloves1', TransformStamped, vicon_glove_callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
