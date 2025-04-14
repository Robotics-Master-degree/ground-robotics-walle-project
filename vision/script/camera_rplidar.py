#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from center_contour import CenterContour
from cv_bridge import CvBridge, CvBridgeError
from classes import image_converter,lidar_converter,astra_converter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CameraInfo,LaserScan
from geometry_msgs.msg import Vector3,PointStamped,PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Empty
import sys
import cv2

"""def call_vision(req):
    lc = lidar_converter()
    print (req)
    return Empty()"""

def main():
    rospy.init_node('camera_rplidar', anonymous=True)
    #try:

    #s = rospy.Service('call_vision', Empty, call_vision)
    #centers = {}
    #od = obtain_direction()
    #print(ic.centers)
    #except:
    #    print("Error obtaining marker's center")

    ic = image_converter()
    ac = astra_converter()
    lc = lidar_converter()
    try:
      rate = rospy.Rate(5) # 10hz
      # cluster.publish_people()
      """ic.image_sub = rospy.Subscriber("/astra/rgb/image_raw",Image,ic.callback)
      ic.image_sub_depth = rospy.Subscriber("/astra/depth/image_raw",Image,ic.callback_depth)
      ic.camera_info = rospy.Subscriber("astra/rgb/camera_info",CameraInfo,ic.camera_info_callback)
      ac.marker_blue = rospy.Subscriber("vision/output_data/blue_direction",Float64MultiArray,ac.blue_callback)
      ac.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,ac.red_callback)
      ac.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,ac.green_callback)
      ac.image_raw = rospy.Subscriber("astra/depth/camera_info",CameraInfo,ac.camera_info_call)
      lc.marker_blue = rospy.Subscriber("vision/output_data/blue_direction",Float64MultiArray,lc.blue_callback)
      lc.marker_red = rospy.Subscriber("vision/output_data/red_direction",Float64MultiArray,lc.red_callback)
      lc.marker_green = rospy.Subscriber("vision/output_data/green_direction",Float64MultiArray,lc.green_callback)"""
      rospy.spin()
      rate.sleep()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
