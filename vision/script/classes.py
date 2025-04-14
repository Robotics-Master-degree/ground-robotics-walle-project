from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CameraInfo,LaserScan
from geometry_msgs.msg import Vector3,PointStamped,PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from numpy import zeros
import rospy
import cv2
import tf
import math

#from detect_color import Blue_Filtering,Red_Filtering,Green_Filtering,direction
from functions import Blue_Filtering,Red_Filtering,Green_Filtering,direction

class image_converter:
  #Int64MultiArray blue_center()
  def __init__(self):
    self.image_pub = rospy.Publisher("vision/output_image",Image,queue_size=10)
    self.marker_blue = rospy.Publisher("vision/output_data/blue_direction",Float64MultiArray,queue_size=1)
    self.marker_red = rospy.Publisher("vision/output_data/red_direction",Float64MultiArray,queue_size=1)
    self.marker_green = rospy.Publisher("vision/output_data/green_direction",Float64MultiArray,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/astra/rgb/image_raw",Image,self.callback)
    self.image_sub_depth = rospy.Subscriber("/astra/depth/image_raw",Image,self.callback_depth)
    self.centers = dict()
    self.camera_info = rospy.Subscriber("astra/rgb/camera_info",CameraInfo,self.camera_info_callback)
    self.data_K=[]
    self.image_depth=[]

  def image_centers(self,image,center_dict,blue_center,red_center,green_center):
      image_depth=self.image_depth

      try:
          image,blue_center = Blue_Filtering(image,blue_center,image_depth)
          #print(blue_center)
      except:
          pass
      try:
          image,red_center = Red_Filtering(image,red_center,image_depth)
          #print(red_centers)
      except:
          pass
      try:
          image,green_center = Green_Filtering(image,green_center,image_depth)
      except:
          pass
      return image,center_dict,blue_center,red_center,green_center

  def callback_depth(self,data):
        try:
          cv_image_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
          self.image_depth=cv_image_depth
        except CvBridgeError as e:
          print(e)

  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    center_dict = {'blue':'None','red':'None','green':'None'}
    blue_center = Int64MultiArray()
    red_center = Int64MultiArray()
    green_center = Int64MultiArray()
    image,center_dict,blue_center,red_center,green_center = self.image_centers(cv_image,center_dict,blue_center,red_center,green_center)
    try:
        blue_direction = direction(blue_center,self.data_K,"blue")
        if blue_direction.layout.dim[0].size != 0:
            self.marker_blue.publish(blue_direction)
    except:
        pass
    try:
        red_direction = direction(red_center,self.data_K,"red")
        if red_direction.layout.dim[0].size != 0:
            self.marker_red.publish(red_direction)
    except:
        pass
    try:
        green_direction = direction(green_center,self.data_K,"green")
        if green_direction.layout.dim[0].size != 0:
            self.marker_green.publish(green_direction)
    except:
        pass

    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))




  def camera_info_callback(self,data):
        data_K = data.K
        self.data_K = data_K

class astra_converter:
    def __init__(self):
        self.marker_blue = rospy.Subscriber("vision/output_data/blue_direction",Float64MultiArray,self.blue_callback)
        self.image_raw = rospy.Subscriber("astra/depth/camera_info",CameraInfo,self.camera_info_call)
        self.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,self.red_callback)
        self.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,self.green_callback)
        self.marker_blue_pub = rospy.Publisher("vision/astra/blue_pose",PoseStamped,queue_size=5)
        self.marker_red_pub = rospy.Publisher("vision/astra/red_pose",PoseStamped,queue_size=5)
        self.marker_green_pub= rospy.Publisher("vision/astra/green_pose",PoseStamped,queue_size=5)
        self.camera_info = CameraInfo()
        self.listener = tf.TransformListener()

    def camera_info_call(self,data):
        self.camera_info.header = data.header


    def blue_callback(self,data):
        #print("blue")
        blue_pt = PointStamped()
        astra_blue_pt = PointStamped()
        blue_ps = PoseStamped()
        blue_pt.header.frame_id = self.camera_info.header.frame_id
        blue_ps.header.frame_id = self.camera_info.header.frame_id
        #astra_blue_pt.header = self.camera_info.header
        l =len(data.data)
        #print(blue_ps)
        i=0
        while i<l:
            blue_pt.point.x = round(data.data[i],3)
            blue_pt.point.y = round(data.data[i+1],3)
            blue_pt.point.z = round(data.data[i+2],3)
            #print(blue_pt)
            try:
                astra_blue_pt =self.listener.waitForTransform("camera_depth_optical_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                astra_blue_pt =self.listener.transformPoint("map",blue_pt)
                #print("astra_blue_pt",astra_blue_pt)
                #print(astra_blue_pt)
                blue_ps.header = astra_blue_pt.header
                blue_ps.pose.position = astra_blue_pt.point
                blue_ps.pose.orientation.w = 1.0
                #print(blue_ps)
                self.marker_blue_pub.publish(blue_ps)
            except(tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                astra_blue_pt = PointStamped()
                print("astra blue tf error")
                continue
            i+=3

    def red_callback(self,data):
        red_pt = PointStamped()
        astra_red_pt = PointStamped()
        red_ps = PoseStamped()
        red_pt.header.frame_id = self.camera_info.header.frame_id
        red_ps.header.frame_id = self.camera_info.header.frame_id
        #astra_blue_pt.header = self.camera_info.header
        l =len(data.data)
        #print(red_ps)
        i=0
        while i<l:
            red_pt.point.x = round(data.data[i],3)
            red_pt.point.y = round(data.data[i+1],3)
            red_pt.point.z = round(data.data[i+2],3)
            #print(red_pt)
            try:
                astra_red_pt =self.listener.waitForTransform("camera_depth_optical_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                astra_red_pt =self.listener.transformPoint("map",red_pt)
                #print("astra_red_pt",astra_red_pt)
                #print(astra_red_pt)
                red_ps.header = astra_red_pt.header
                red_ps.pose.position = astra_red_pt.point
                red_ps.pose.orientation.w = 1.0
                #print(red_ps)
                self.marker_red_pub.publish(red_ps)
            except(tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                astra_red_pt = PointStamped()
                print("astra red tf error")
                continue
            i+=3
    def green_callback(self,data):
        green_pt = PointStamped()
        astra_green_pt = PointStamped()
        green_ps = PoseStamped()
        green_pt.header.frame_id = self.camera_info.header.frame_id
        green_ps.header.frame_id = self.camera_info.header.frame_id
        #astra_blue_pt.header = self.camera_info.header
        l =len(data.data)
        #print(green_ps)
        i=0
        while i<l:
            green_pt.point.x = round(data.data[i],3)
            green_pt.point.y = round(data.data[i+1],3)
            green_pt.point.z = round(data.data[i+2],3)
            #print(green_pt)
            try:
                astra_green_pt =self.listener.waitForTransform("camera_depth_optical_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                astra_green_pt =self.listener.transformPoint("map",green_pt)
                #print("astra_green_pt",astra_green_pt)
                #print(astra_green_pt)
                green_ps.header = astra_green_pt.header
                green_ps.pose.position = astra_green_pt.point
                green_ps.pose.orientation.w = 1.0
                #print(green_ps)
                self.marker_green_pub.publish(green_ps)
            except(tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                astra_green_pt = PointStamped()
                print("astra green tf error")
                continue
            i+=3
class lidar_converter:
    def __init__(self):
        self.marker_blue = rospy.Subscriber("vision/output_data/blue_direction",Float64MultiArray,self.blue_callback)
        self.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,self.red_callback)
        self.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,self.green_callback)
        self.LaserSub = rospy.Subscriber("scan",LaserScan,self.LaserScan_callback)
        self.marker_blue_pub = rospy.Publisher("vision/RPlidar/blue_pose",PoseStamped,queue_size=5)
        self.marker_red_pub = rospy.Publisher("vision/RPlidar/red_pose",PoseStamped,queue_size=5)
        self.marker_green_pub= rospy.Publisher("vision/RPlidar/green_pose",PoseStamped,queue_size=5)
        self.laser = LaserScan()
        self.seq=0
        self.secs=0
        self.nsecs=0
        self.j=0
        #self.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,self.red_callback)
        #self.angle_increment = 0.2
        self.angle_increment = 0.0175018943846
        self.listener = tf.TransformListener()
        self.PoseStamped = PoseStamped()
        #listener2 = tf.TransformListener()
        #self.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,self.green_callback)

    def blue_callback(self,data):
        blue_pt = PointStamped()
        lidar_blue_pt = PointStamped()
        lidar_blue_pt_ = PointStamped()
        #blue_pt.header.seq = self.laser.header.seq
        blue_pt.header.stamp = self.laser.header.stamp
        blue_pt.header.frame_id = "camera_depth_optical_frame"
        l =len(data.data)
        #listener = tf.TransformListener()
        #listener2 = tf.TransformListener()
        i=0
        while i<l:
            blue_pt.point.x = round(data.data[i],3)
            blue_pt.point.y = round(data.data[i+1],3)
            blue_pt.point.z = round(data.data[i+2],3)
            try:
                lidar_blue_pt =self.listener.waitForTransform("camera_depth_optical_frame", "rplidar_frame", rospy.Time(0), rospy.Duration(5.0))
                lidar_blue_pt =self.listener.transformPoint("rplidar_frame",blue_pt)
                #print(lidar_blue_pt)
            except(tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                lidar_blue_pt = PointStamped()
                print("lidar tf error")
                #print("not rp_lidar_frame tf possible")
                continue
            angle =math.atan2(lidar_blue_pt.point.y,lidar_blue_pt.point.x)
            angle_increment = self.angle_increment
            distance = 0
            if (0 <= angle) and (angle<=math.pi/2.0):
                    pos = int(round(angle/angle_increment,0))
                    laser_data = self.laser
                    """print("laser_data",len(laser_data.ranges))
                    print("pos",pos)
                    print("laser_data_0",laser_data.ranges[0])
                    print("laser_data_90",laser_data.ranges[90])
                    print("laser_data_180",laser_data.ranges[180])
                    print("laser_data_270",laser_data.ranges[270])
                    print("laser_data_359",laser_data.ranges[359])"""
                    distance = laser_data.ranges[pos]
            elif (angle<0) and (abs(angle)<=math.pi/2.0):
                #pos = 399 - int(round(abs(angle)/angle_increment,0))
                pos = 359 - int(round(abs(angle)/angle_increment,0))
                laser_data = self.laser
                #print("laser_data",len(laser_data.ranges))
                distance = laser_data.ranges[pos]
            else:
                print("not angle located")
            #print("angle",angle*180/math.pi)
            #print("distance",distance)
            #print(i)
            #lidar_blue_pt.header.seq = i/3.0
            lidar_blue_pt.header.stamp = rospy.Time(0)
            lidar_blue_pt.header.frame_id = "rplidar_frame"
            #try:
            if (blue_pt.point.x == 1.0):
                lidar_blue_pt.point.x *= distance
                lidar_blue_pt.point.y *= distance
                lidar_blue_pt.point.z *= distance
                #print("point_x=1",lidar_blue_pt )
            else:
                lidar_blue_pt.point.x = (lidar_blue_pt.point.x/lidar_blue_pt.point.x)*distance
                lidar_blue_pt.point.y = (lidar_blue_pt.point.y/lidar_blue_pt.point.x)*distance
                lidar_blue_pt.point.z = (lidar_blue_pt.point.z/lidar_blue_pt.point.x)*distance
                #print("point_x!=1",lidar_blue_pt.point.x)
            #except:
            #    lidar_blue_pt = PointStamped()
            #    continue
            try:
                lidar_blue_pt_=self.listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(5.0))
                lidar_blue_pt_=self.listener.transformPoint("map", lidar_blue_pt)
                BluePoseStamped = PoseStamped()
                BluePoseStamped.header = lidar_blue_pt_.header
                BluePoseStamped.pose.position = lidar_blue_pt_.point
                BluePoseStamped.pose.orientation.x = 0
                BluePoseStamped.pose.orientation.y = 0
                BluePoseStamped.pose.orientation.z = 0
                BluePoseStamped.pose.orientation.w = 1
                self.marker_blue_pub.publish(BluePoseStamped)
            except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                lidar_blue_pt_ = PointStamped()
                #print(lidar_blue_pt)
                #print("TF blue Exception")
                continue
            #print("rp_lidar_frame")
            i+=3
            ##self.j+=1




    def LaserScan_callback(self,data):
        l=len(data.ranges)
        ranges= data.ranges
        self.laser = data
        self.angle_increment = data.angle_increment
        #print(self.angle_increment)
        #print(l)
        #print("0",data.ranges[0])
        #print("100",data.ranges[100])
        #print("200",data.ranges[200])
        #print("300",data.ranges[300])

    def red_callback(self,data):
        red_pt = PointStamped()
        red_pt.header.seq = self.laser.header.seq
        red_pt.header.stamp = self.laser.header.stamp
        red_pt.header.frame_id = "camera_depth_optical_frame"
        l =len(data.data)

        i=0
        #rint("red")
        while i<l:

            lidar_red_pt = PointStamped()
            lidar_red_pt_ = PointStamped()

            #print("red")
            red_pt.point.x = data.data[i]
            red_pt.point.y = data.data[i+1]
            red_pt.point.z = data.data[i+2]
            try:
                lidar_red_pt = self.listener.transformPoint("rplidar_frame",red_pt)
            except:
                lidar_red_pt = PointStamped()
                pass
            angle = math.atan2(lidar_red_pt.point.y,lidar_red_pt.point.x)
            angle_increment = self.angle_increment
            distance = 0
            if (0 < angle) and (angle<math.pi/2.0):
                #pos_ = angle/angle_increment
                pos = int(round(angle/angle_increment,0))
                #print("pos",pos)
                laser_data = self.laser
                distance = laser_data.ranges[pos]
                #print("distance",distance)
            elif (angle<0) and (abs(angle)<math.pi/2.0):
                #pos = 399 - int(round(abs(angle)/angle_increment,0))
                pos = 359 - int(round(abs(angle)/angle_increment,0))
                laser_data = self.laser
                #print("pos",pos)
                distance = laser_data.ranges[pos]
                #print("distance",distance)
            else:
                print("not angle located")
            lidar_red_pt.header.stamp = rospy.Time(0)
            lidar_red_pt.header.frame_id = "rplidar_frame"

            try:
                if (red_pt.point.z == 1.0):
                    lidar_red_pt.point.x *= distance
                    lidar_red_pt.point.y *= distance
                    lidar_red_pt.point.z *= distance

                else:
                    lidar_red_pt.point.x = (lidar_red_pt.point.x/lidar_red_pt.point.x)*distance
                    lidar_red_pt.point.y = (lidar_red_pt.point.y/lidar_red_pt.point.x)*distance
                    lidar_red_pt.point.z = (lidar_red_pt.point.z/lidar_red_pt.point.x)*distance
            except:
                lidar_red_pt = PointStamped()
                pass

            try:
                lidar_red_pt_=self.listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(1.0))
                lidar_red_pt_=self.listener.transformPoint("map", lidar_red_pt)
                RedPoseStamped = PoseStamped()
                RedPoseStamped.header = lidar_red_pt_.header
                RedPoseStamped.pose.position = lidar_red_pt_.point
                RedPoseStamped.pose.orientation.x = 0
                RedPoseStamped.pose.orientation.y = 0
                RedPoseStamped.pose.orientation.z = 0
                RedPoseStamped.pose.orientation.w = 1
                self.marker_red_pub.publish(RedPoseStamped)
                #print(lidar_red_pt_)
            except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                lidar_red_pt_ = PointStamped()
                print("TF Red exception")
                pass
            i+=3



    def green_callback(self,data):
            green_pt = PointStamped()
            green_pt.header.seq = self.laser.header.seq
            green_pt.header.stamp = self.laser.header.stamp
            green_pt.header.frame_id = "camera_depth_optical_frame"
            l =len(data.data)

            i=0
            #rint("green")
            while i<l:

                lidar_green_pt = PointStamped()
                lidar_green_pt_ = PointStamped()

                #print("green")
                green_pt.point.x = data.data[i]
                green_pt.point.y = data.data[i+1]
                green_pt.point.z = data.data[i+2]
                try:
                    lidar_green_pt = self.listener.transformPoint("rplidar_frame",green_pt)
                except:
                    lidar_green_pt = PointStamped()
                    pass
                angle = math.atan2(lidar_green_pt.point.y,lidar_green_pt.point.x)
                angle_increment = self.angle_increment
                distance = 0
                if (0 < angle) and (angle<math.pi/2.0):
                    #pos_ = angle/angle_increment
                    pos = int(round(angle/angle_increment,0))
                    #print("pos",pos)
                    laser_data = self.laser
                    distance = laser_data.ranges[pos]
                    #print("distance",distance)
                elif (angle<0) and (abs(angle)<math.pi/2.0):
                    #pos = 399 - int(round(abs(angle)/angle_increment,0))
                    pos = 359 - int(round(abs(angle)/angle_increment,0))
                    laser_data = self.laser
                    distance = laser_data.ranges[pos]
                    #print("distance",distance)
                else:
                    print("not angle located")
                lidar_green_pt.header.stamp = rospy.Time(0)
                lidar_green_pt.header.frame_id = "rplidar_frame"

                try:
                    if (green_pt.point.z == 1.0):
                        lidar_green_pt.point.x *= distance
                        lidar_green_pt.point.y *= distance
                        lidar_green_pt.point.z *= distance

                    else:
                        lidar_green_pt.point.x = (lidar_green_pt.point.x/lidar_green_pt.point.x)*distance
                        lidar_green_pt.point.y = (lidar_green_pt.point.y/lidar_green_pt.point.x)*distance
                        lidar_green_pt.point.z = (lidar_green_pt.point.z/lidar_green_pt.point.x)*distance
                except:
                    lidar_green_pt = PointStamped()
                    pass

                try:
                    lidar_green_pt_=self.listener.waitForTransform("rplidar_frame", "map", rospy.Time(0), rospy.Duration(1.0))
                    lidar_green_pt_=self.listener.transformPoint("map", lidar_green_pt)
                    greenPoseStamped = PoseStamped()
                    greenPoseStamped.header = lidar_green_pt_.header
                    greenPoseStamped.pose.position = lidar_green_pt_.point
                    greenPoseStamped.pose.orientation.x = 0
                    greenPoseStamped.pose.orientation.y = 0
                    greenPoseStamped.pose.orientation.z = 0
                    greenPoseStamped.pose.orientation.w = 1
                    self.marker_green_pub.publish(greenPoseStamped)
                    #print(lidar_green_pt_)
                except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException,tf.Exception):
                    lidar_green_pt_ = PointStamped()
                    print("TF green exception")
                    pass
                i+=3
