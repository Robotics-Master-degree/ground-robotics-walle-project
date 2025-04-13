import rospy
import numpy
from geometry_msgs.msg import Vector3,PointStamped,PoseStamped,PoseArray
from visualization_msgs.msg import MarkerArray
from function import check_waypoint,obtain_markerArray
class clustering:
    def __init__(self):
        self.vision_rplidar_blue_sub = rospy.Subscriber("/vision/RPlidar/blue_pose",PoseStamped,self.rplidar_blue_callback)
        self.vision_rplidar_red_sub = rospy.Subscriber("/vision/RPlidar/red_pose",PoseStamped,self.rplidar_red_callback)
        self.vision_rplidar_green_sub = rospy.Subscriber("/vision/RPlidar/green_pose",PoseStamped,self.rplidar_green_callback)
        """self.vision_astra_blue_sub = rospy.Subscriber("/vision/astra/blue_pose",PointStamped,self.astra_blue_callback)
        self.vision_astra_red_sub = rospy.Subscriber("/vision/astra/red_pose",PointStamped,self.astra_red_callback)
        self.vision_astra_green_sub = rospy.Subscriber("/vision/astra/green_pose",PointStamped,self.vision_astra_green_sub_callback)"""
        self.waypoints_people = PoseArray()
        self.waypoints_people.header.frame_id ="map"
        self.waypoints_danger_zones = PoseArray()
        self.waypoints_danger_zones.header.frame_id ="map"
        self.waypoints_wayout = PoseArray()
        self.waypoints_wayout.header.frame_id ="map"
        self.count_blue = 0
        self.count_red = 0
        self.count_green = 0
        self.waypoints_people_pub = rospy.Publisher("/output/waypoints/people",PoseArray,queue_size=10)
        self.waypoints_danger_zones_pub = rospy.Publisher("/output/waypoints/danger_zones",PoseArray,queue_size=10)
        self.waypoints_wayout_pub = rospy.Publisher("/output/waypoints/wayout",PoseArray,queue_size=10)
        self.markers_people_pub = rospy.Publisher("/output/markers/people",MarkerArray,queue_size=10)
        self.markers_danger_zones_pub = rospy.Publisher("/output/markers/danger_zones",MarkerArray,queue_size=10)
        self.markers_wayout_pub = rospy.Publisher("/output/markers/wayout",MarkerArray,queue_size=10)
        #self.waypoints_people_sub = rospy.Publisher("/output/waypoints/people",PoseArray,queue_size=10)
        #self.waypoints_danger_zones_sub = rospy.Publisher("/output/waypoints/danger_zones",PoseArray,queue_size=10)
        #self.waypoints_wayout_sub = rospy.Publisher("/output/waypoints/wayout",PoseArray,queue_size=10)
        self.dist = 1.3
        self.mkrarray_people = MarkerArray()
        self.mkrarray_danger_zones = MarkerArray()
        self.mkrarray_wayout = MarkerArray()


    def rplidar_blue_callback(self,data):
        #print(data)
        if self.count_blue == 30:
            self.waypoints_people = check_waypoint(data,self.waypoints_people,self.dist)
            self.waypoints_people_pub.publish(self.waypoints_people)
            mkrarray_people = MarkerArray()
            self.mkrarray_people = obtain_markerArray(self.waypoints_people,mkrarray_people,"person")
            self.markers_people_pub.publish(self.mkrarray_people)
            self.count_blue = 0
        else:
            self.count_blue+=1



    def rplidar_red_callback(self,data):
        if self.count_red == 30:
            self.waypoints_danger_zones = check_waypoint(data,self.waypoints_danger_zones,self.dist)
            self.waypoints_danger_zones_pub.publish(self.waypoints_danger_zones)
            mkrarray_danger_zones = MarkerArray()
            self.mkrarray_danger_zones = obtain_markerArray(self.waypoints_danger_zones,mkrarray_danger_zones,"danger_zone")
            self.markers_danger_zones_pub.publish(self.mkrarray_danger_zones)
            self.count_red = 0
        else:
            self.count_red+=1

    def rplidar_green_callback(self,data):
        #print(data)
        if self.count_green == 30:
            self.waypoints_wayout = check_waypoint(data,self.waypoints_wayout,self.dist)
            print(self.waypoints_wayout)
            self.waypoints_wayout_pub.publish(self.waypoints_wayout)
            mkrarray_wayout = MarkerArray()
            self.mkrarray_wayout = obtain_markerArray(self.waypoints_wayout,mkrarray_wayout,"wayout")
            self.markers_wayout_pub.publish(self.mkrarray_wayout)
            self.count_green = 0
        else:
            self.count_green +=1

    def astra_blue_callback(self,data):
        print("new astra person")
    def astra_red_callback(self,data):
        print("new astra zone")
    def astra_green_callback(self,data):
        print("new astra wayout")
