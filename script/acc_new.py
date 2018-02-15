import rospy
import message_filters
import math
import tf.transformations
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

face_ave = None
hand_ave = None
face_med = None
hand_med = None
face_cls = None
hand_cls = None
face_ave_dbscan = None
hand_ave_dbscan = None
glass = None
glove = None
camera = None
summ = 0
counter = 0
cam_roll = 0
cam_pitch = 0
cam_yaw = 0
def face_ave_pose_callback(data):
    global face_ave
    face_ave = data

def hand_ave_pose_callback(data):
    global hand_ave
    hand_ave = data
   
def face_med_pose_callback(data):
    global face_med
    face_med = data

def hand_med_pose_callback(data):
    global hand_med
    hand_med = data
   
def face_cls_pose_callback(data):
    global face_cls
    face_cls = data

def hand_cls_pose_callback(data):
    global hand_cls
    hand_cls = data
   
def face_ave_dbscan_pose_callback(data):
    global face_ave_dbscan
    face_ave_dbscan = data

def hand_ave_dbscan_pose_callback(data):
    global hand_ave_dbscan
    hand_ave_dbscan = data
   
def vicon_glass_callback(data):
    global glass
    glass = data
    glass.transform.translation.z -= 0.17
    glass.transform.translation.y += 0.17

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
    global glove, glass, hand_ave, face_ave, summ, counter, cam_roll, cam_pitch, cam_yaw
    if camera == None:
      return 
    if glass == None:
      return

    glove = data
#    sum = 0
#    counter = 0
    if (glass == None or glove == None or hand_ave == None or face_ave == None):
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
    Rz = (-math.sin(B)*X) + (math.cos(B)*math.sin(C)*Y) + (math.cos(B)*math.cos(C)*Z)

    # print Rx, Ry, Rz
    # print "HI", math.sin(math.pi/2)
    # vicon_yaw_rad = math.atan2(Ry, Rx)
    vicon_yaw_rad = math.atan2(Ry, Rx)
    vicon_yaw_deg = math.degrees(vicon_yaw_rad)

    vicon_pitch_rad = math.atan2(math.sqrt(math.pow(Rx, 2) + math.pow(Ry, 2)), Rz) - math.pi/2
    vicon_pitch_deg = math.degrees(vicon_pitch_rad) 

#    vicon_rad = -math.atan2(glass.transform.translation.y - glove.transform.translation.y, glass.transform.translation.x - glove.transform.translation.x)
#    vicon_deg = math.degrees(vicon_rad)

    face_ave_yaw_rad = math.atan2(face_ave.point.x - hand_ave.point.x, face_ave.point.z - hand_ave.point.z)
    face_ave_yaw_deg = math.degrees(face_ave_yaw_rad)

    face_ave_pitch_rad = math.atan2(math.sqrt(math.pow(face_ave.point.x - hand_ave.point.x, 2) + math.pow(face_ave.point.z - hand_ave.point.z, 2)), face_ave.point.y - hand_ave.point.y) - math.pi/2
    face_ave_pitch_deg = math.degrees(face_ave_pitch_rad)

    face_med_yaw_rad = math.atan2(face_med.point.x - hand_med.point.x, face_med.point.z - hand_med.point.z)
    face_med_yaw_deg = math.degrees(face_med_yaw_rad)

    face_med_pitch_rad = math.atan2(math.sqrt(math.pow(face_med.point.x - hand_med.point.x, 2) + math.pow(face_med.point.z - hand_med.point.z, 2)), face_med.point.y - hand_med.point.y) - math.pi/2
    face_med_pitch_deg = math.degrees(face_med_pitch_rad)

    face_cls_yaw_rad = math.atan2(face_cls.point.x - hand_cls.point.x, face_cls.point.z - hand_cls.point.z)
    face_cls_yaw_deg = math.degrees(face_cls_yaw_rad)

    face_cls_pitch_rad = math.atan2(math.sqrt(math.pow(face_cls.point.x - hand_cls.point.x, 2) + math.pow(face_cls.point.z - hand_cls.point.z, 2)), face_cls.point.y - hand_cls.point.y) - math.pi/2
    face_cls_pitch_deg = math.degrees(face_cls_pitch_rad)

    face_ave_dbscan_yaw_rad = math.atan2(face_ave_dbscan.point.x - hand_ave_dbscan.point.x, face_ave_dbscan.point.z - hand_ave_dbscan.point.z)
    face_ave_dbscan_yaw_deg = math.degrees(face_ave_dbscan_yaw_rad)

    face_ave_dbscan_pitch_rad = math.atan2(math.sqrt(math.pow(face_ave_dbscan.point.x - hand_ave_dbscan.point.x, 2) + math.pow(face_ave_dbscan.point.z - hand_ave_dbscan.point.z, 2)), face_ave_dbscan.point.y - hand_ave_dbscan.point.y) - math.pi/2
    face_ave_dbscan_pitch_deg = math.degrees(face_ave_dbscan_pitch_rad)
    

# if math.fabs(vicon_pitch_deg - face_ave_pitch_deg) < 5:
#print "Vicon Yaw: " , vicon_yaw_deg, "Vicon Pitch: " , vicon_pitch_deg
#print "Yaw Average: " , face_ave_yaw_deg, "Pitch Average: " , face_ave_pitch_deg 
#print "Yaw Median: " , face_med_yaw_deg, "Pitch Median: " , face_med_pitch_deg 
#print "Yaw Closest: " , face_cls_yaw_deg, "Pitch Closest: " , face_cls_pitch_deg 
#print "Yaw Average DBSCAN: " , face_ave_dbscan_yaw_deg, "Pitch Average DBSCAN: " , face_ave_dbscan_pitch_deg 
#print "----------------------------------------------------------------------"
#print " "
    print "Vicon Yaw: " , vicon_yaw_deg, "Vicon Pitch: " , vicon_pitch_deg
    print "Yaw Average: " , math.fabs(face_ave_yaw_deg - vicon_yaw_deg), "Pitch Average: " , math.fabs(face_ave_pitch_deg - vicon_pitch_deg) 
#    print "Yaw Median: " ,  math.fabs(face_med_yaw_deg - vicon_yaw_deg), "Pitch Median: " , math.fabs(face_med_pitch_deg - vicon_pitch_deg)
#    print "Yaw Closest: " , math.fabs(face_cls_yaw_deg - vicon_yaw_deg), "Pitch Closest: " , math.fabs(face_cls_pitch_deg - vicon_pitch_deg)
    print "Yaw Average DBSCAN: " , math.fabs(face_ave_dbscan_yaw_deg -  vicon_yaw_deg), "Pitch Average DBSCAN: " , math.fabs(face_ave_dbscan_pitch_deg - vicon_pitch_deg)
    print "-------------------------------------------------"

#print " "
#    face_rad = math.atan2(face.point.x - hand.point.x, face.point.z - hand.point.z)
#    face_deg = math.degrees(face_rad)

#    print face_deg, vicon_deg

#    accuracy = abs(vicon_yaw_deg - face_yaw_deg) / 180
#    summ = summ + accuracy
#   print summ
#    if counter == 400:
#      print face.point.z
#      acy = summ / counter 
#      summ = 0
#      counter = 0
#      print "Here---------------------------->", acy
#    else:
#      counter = counter + 1

#    print vicon_deg, face_deg, face.point.z
#    print accuracy, face.point.z
rospy.init_node('accuracy')
rospy.Subscriber('/3dr/face_ave_pose', PointStamped, face_ave_pose_callback)
rospy.Subscriber('/3dr/hand_ave_pose', PointStamped, hand_ave_pose_callback)

rospy.Subscriber('/3dr/face_med_pose', PointStamped, face_med_pose_callback)
rospy.Subscriber('/3dr/hand_med_pose', PointStamped, hand_med_pose_callback)

rospy.Subscriber('/3dr/face_cls_pose', PointStamped, face_cls_pose_callback)
rospy.Subscriber('/3dr/hand_cls_pose', PointStamped, hand_cls_pose_callback)

rospy.Subscriber('/3dr/face_ave_dbscan_pose', PointStamped, face_ave_dbscan_pose_callback)
rospy.Subscriber('/3dr/hand_ave_dbscan_pose', PointStamped, hand_ave_dbscan_pose_callback)

rospy.Subscriber('/vicon/Glass2/Glass2', TransformStamped, vicon_glass_callback)
rospy.Subscriber('/vicon/Cam23/Cam23', TransformStamped, vicon_camera_callback)
rospy.Subscriber('/vicon/Gloves1/Gloves1', TransformStamped, vicon_glove_callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
