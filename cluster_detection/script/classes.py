import rospy
import numpy
from geometry_msgs.msg import Vector3,PointStamped,PoseStamped,PoseArray
from people_msgs.msg import People
from visualization_msgs.msg import MarkerArray
from function import check_waypoint,obtain_markerArray,obtain_people_danger_zones,check_waypoint_v2,double_check,double_check_v2,check_waypoint_v3,double_check_v3
class clustering:
    def __init__(self):
        self.vision_rplidar_blue_sub = rospy.Subscriber("/vision/RPlidar/blue_pose",PoseStamped,self.rplidar_blue_callback)
        self.vision_rplidar_red_sub = rospy.Subscriber("/vision/RPlidar/red_pose",PoseStamped,self.rplidar_red_callback)
        self.vision_rplidar_green_sub = rospy.Subscriber("/vision/RPlidar/green_pose",PoseStamped,self.rplidar_green_callback)
        self.vision_astra_blue_sub = rospy.Subscriber("/vision/astra/blue_pose",PoseStamped,self.astra_blue_callback)
        self.vision_astra_red_sub = rospy.Subscriber("/vision/astra/red_pose",PoseStamped,self.astra_red_callback)
        #self.vision_astra_green_sub = rospy.Subscriber("/vision/astra/green_pose",PoseStamped,self.astra_green_callback)

        self.waypoints_wayout = PoseArray()
        self.waypoints_wayout.header.frame_id ="map"

        self.count_blue_30 = 0
        self.count_red = 0
        self.count_green = 0
        self.waypoints_people_pub = rospy.Publisher("/output/waypoints/people",PoseArray,queue_size=10)
        self.waypoints_danger_zones_pub = rospy.Publisher("/output/waypoints/danger_zones",PoseArray,queue_size=10)
        self.waypoints_wayout_pub = rospy.Publisher("/output/waypoints/wayout",PoseArray,queue_size=10)
        self.markers_people_pub = rospy.Publisher("/output/markers/people",MarkerArray,queue_size=10)
        self.markers_danger_zones_pub = rospy.Publisher("/output/markers/danger_zones",MarkerArray,queue_size=10)
        self.markers_wayout_pub = rospy.Publisher("/output/markers/wayout",MarkerArray,queue_size=10)
        self.waypoints_people_sub = rospy.Publisher("/output/waypoints/people",PoseArray,queue_size=10)
        self.waypoints_danger_zones_pub = rospy.Publisher("/output/waypoints/danger_zones",PoseArray,queue_size=10)
        self.waypoints_wayout_sub = rospy.Publisher("/output/waypoints/wayout",PoseArray,queue_size=10)
        self.waypoints_people_danger_pub = rospy.Publisher("/output/people/danger_zones",People,queue_size=10)
        self.waypoints_people_wayout_pub = rospy.Publisher("/output/people/exit_zones",People,queue_size=10)
        self.mkrarray_people = MarkerArray()
        self.mkrarray_danger_zones = MarkerArray()
        self.mkrarray_wayout = MarkerArray()
        self.ps_temp_blue = numpy.array([0,0,0]) #y,z
        self.ps_temp_red = numpy.array([0,0,0]) #y,z
        self.ps_temp_green = numpy.array([0,0,0]) #y,z
        self.offset_y_min = 0.0
        self.offset_z_min = 0.0
        self.offset_y_max = 1.3
        self.offset_z_max = 1
        self.offset_dist = 0.5
        self.dist_temp = 1


        self.dist_cluster_person =0.8 #0.5#0.13 #0.17
        #self.dist_cluster_person = 7#0.17
        self.dist_danger_zone = 0.5#0.10
        #self.dist_danger_zone = 0.7
        self.dist_wayout = 0.3

        self.dist1 = 0.15;
        self.dist2 = 0.95;


        self.dist_double_check_person  = 1.3 # 1.3 #1.5 # 1.3
        self.dist_double_check_danger_zone = 0.8 #0.8
        self.dist_double_check_wayout = 1
        #new
        self.dist_blue = 1 #1.3
        self.dist_red = 1
        self.dist_green =  1.3
        self.people = People()
        self.wayout = People()
        #waypoints_people #new
        self.count_blue_astra = 0
        self.count_blue_rplidar = 0
        self.count_red_astra = 0
        self.count_red_rplidar = 0
        self.count_green_astra = 0
        self.count_green_rplidar = 0
        self.psarr = PoseArray()
        self.waypoints_people_rplidar = PoseArray()
        self.waypoints_people_astra = PoseArray()
        self.waypoints_people = PoseArray()
        self.waypoints_people.header.frame_id = "map"
        self.waypoints_danger_zones_astra = PoseArray()
        self.waypoints_danger_zones_rplidar = PoseArray()
        self.waypoints_danger_zones = PoseArray()
        self.waypoints_danger_zones.header.frame_id = "map"
        self.waypoints_wayout = PoseArray()
        self.waypoints_wayout_astra = PoseArray()
        self.waypoints_wayout_rplidar = PoseArray()
        self.waypoints_wayout.header.frame_id = "map"
        #self.dist_cluster

        #old
        self.psarr = PoseArray()
        self.psarr.header.frame_id = "map"
        self.waypoints_people_30 = PoseArray()
        self.waypoints_people_10 = PoseArray()


    def publish_people(self):
        #print("people")
        #print("self.waypoints_people_astra",self.waypoints_people_astra)
        #print("self.waypoints_people_rplidar",self.waypoints_people_rplidar)
        self.waypoints_people = double_check_v3(self.waypoints_people_astra,self.waypoints_people_rplidar,self.waypoints_people,self.dist_cluster_person,self.dist_double_check_person,self.dist1)
        self.waypoints_people_pub.publish(self.waypoints_people)
        mkrarray_people = MarkerArray()
        self.mkrarray_people = obtain_markerArray(self.waypoints_people,mkrarray_people,"person")
        self.markers_people_pub.publish(self.mkrarray_people)
        self.waypoints_people_rplidar = PoseArray()
        self.waypoints_people_astra = PoseArray()

    def astra_blue_callback(self,data):
        #print("blue")
        a= len(self.waypoints_people.poses)
        #print("len self.waypoints_people",a)
        if (self.count_blue_astra == 1) and (a<6): #and (len(self.waypoints_people.poses)<7):
            self.waypoints_people_astra = check_waypoint_v3(data,self.waypoints_people_astra,self.dist_blue)
            self.count_blue_astra =  0
            self.publish_people()
        else:
            self.count_blue_astra += 1



    def rplidar_blue_callback(self,data):
        #print("blue")
        a= len(self.waypoints_people.poses)
        #print("len self.waypoints_people_rplidar",a)
        if (self.count_blue_rplidar == 1) and (a<6): #and (len(self.waypoints_people.poses)<7):
            self.waypoints_people_rplidar = check_waypoint_v3(data,self.waypoints_people_rplidar,self.dist_blue)
            self.count_blue_rplidar = 0
            self.publish_people()
        else:
            self.count_blue_rplidar += 1


    def publish_danger_zones(self):
        #print("red")
        #print("self.waypoints_danger_zones_astra",self.waypoints_danger_zones_astra)
        #print("self.waypoints_danger_zones_rplidar",self.waypoints_danger_zones_rplidar)
        self.waypoints_danger_zones = double_check_v3(self.waypoints_danger_zones_astra,self.waypoints_danger_zones_rplidar,self.waypoints_danger_zones,self.dist_danger_zone,self.dist_double_check_danger_zone,self.dist2)
        self.waypoints_danger_zones_pub.publish(self.waypoints_danger_zones)
        mkrarray_danger_zones = MarkerArray()
        self.mkrarray_danger_zones = obtain_markerArray(self.waypoints_danger_zones,mkrarray_danger_zones,"danger_zone")
        self.markers_danger_zones_pub.publish(self.mkrarray_danger_zones)
        self.people = obtain_people_danger_zones(self.waypoints_danger_zones)
        self.waypoints_people_danger_pub.publish(self.people)
        self.waypoints_danger_zones_astra = PoseArray()
        self.waypoints_danger_zones_rplidar = PoseArray()

        #self.mkrarray_people = obtain_markerArray(self.waypoints_people,mkrarray_people,"person")
        #self.markers_people_pub.publish(self.mkrarray_people)


    def astra_red_callback(self,data):
        if self.count_red_astra == 5:
            self.waypoints_danger_zones_astra = check_waypoint_v3(data,self.waypoints_danger_zones_astra,self.dist_red)
            self.publish_danger_zones()
            self.count_red_astra =  0
        else:
            self.count_red_astra += 1


    def rplidar_red_callback(self,data):
        if self.count_red_rplidar == 1:
            self.waypoints_danger_zones_rplidar = check_waypoint_v3(data,self.waypoints_danger_zones_rplidar,self.dist_red)
            self.publish_danger_zones()
            self.count_red_rplidar =  0
        else:
            self.count_red_rplidar += 1



    def publish_wayout(self):
        #print("self.waypoints_wayout_rplidar",self.waypoints_wayout_rplidar)
        #print("self.waypoints_wayout_astra",self.waypoints_wayout_rplidar)
        self.waypoints_wayout = double_check_v3(self.waypoints_wayout_astra,self.waypoints_wayout_rplidar,self.waypoints_wayout,self.dist_wayout,self.dist_double_check_wayout,self.dist2)
        self.waypoints_wayout_pub.publish(self.waypoints_wayout)
        mkrarray_wayout = MarkerArray()
        self.mkrarray_wayout= obtain_markerArray(self.waypoints_wayout,mkrarray_wayout,"wayout")
        self.markers_wayout_pub.publish(self.mkrarray_wayout)
        self.wayout = obtain_people_danger_zones(self.waypoints_wayout)
        self.waypoints_people_wayout_pub.publish(self.wayout)
        #self.waypoints_wayout_rplidar = PoseArray()
        #self.waypoints_wayout_astra = PoseArray()

    #self.mkrarray_people = obtain_markerArray(self.waypoints_people,mkrarray_people,"person")
    #self.markers_people_pub.publish(self.mkrarray_people)


    def rplidar_green_callback(self,data):
        b = len(self.waypoints_wayout.poses)
        #print("b",b)
        if (self.count_green_rplidar == 10) and (b<1):
            self.waypoints_people_rplidar = check_waypoint_v3(data,self.waypoints_wayout_rplidar,self.dist_green)
            self.count_green_rplidar = 0
            self.publish_wayout()
        else:
            self.count_green_rplidar += 1

    def astra_green_callback(self,data):
        if self.count_green_astra == 20:
            self.waypoints_wayout_astra = check_waypoint_v3(data,self.waypoints_wayout_astra,self.dist_green)
            self.count_green_astra =  0
            self.publish_wayout()
        else:
            self.count_green_astra += 1
