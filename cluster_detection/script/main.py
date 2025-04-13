#!/usr/bin/env python
import rospy
#from sensor_msgs.msg import Image
#from center_contour import CenterContour
#from cv_bridge import CvBridge, CvBridgeError
#from classes import image_converter,lidar_converter
#from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from classes import clustering
import sys

"""def call_vision(req):
    lc = lidar_converter()
    print (req)
    return Empty()"""

def main():
    rospy.init_node('cluster_detection', anonymous=True)
    #try:

    #s = rospy.Service('call_vision', Empty, call_vision)
    #centers = {}
    cluster = clustering()
    #ic = image_converter()
    #lc = lidar_converter()
    #od = obtain_direction()
    #print(ic.centers)
    #except:
    #    print("Error obtaining marker's center")
    try:
      rate = rospy.Rate(5) # 10hz
      #cluster.vision_rplidar_blue_sub = rospy.Subscriber("/vision/RPlidar/blue_pose",PoseStamped,cluster.rplidar_blue_callback)
      #~cluster.vision_astra_blue_sub = rospy.Subscriber("/vision/astra/blue_pose",PoseStamped,cluster.astra_blue_callback)
      #cluster.publish_people()
      rospy.spin()
      rate.sleep()
    except KeyboardInterrupt:
      print("Shutting down")


if __name__ == '__main__':
    main()
