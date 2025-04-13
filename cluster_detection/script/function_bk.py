import numpy
import rospy
from visualization_msgs.msg import Marker
def check_waypoint(pstamped,psarray,self_dist):
    #print(pstamped)
    ps = numpy.array((pstamped.pose.position.x,pstamped.pose.position.y,pstamped.pose.position.z))
    #print("ps",ps)
    if (len(psarray.poses) == 0):
        print(len(psarray.poses))
        psarray.poses.append(pstamped.pose)
    else:
        l = len(psarray.poses)
        i=0
        trobat = False
        while (i<l):
            psa = numpy.array((psarray.poses[i].position.x,psarray.poses[i].position.y,psarray.poses[i].position.z))
            dist = numpy.linalg.norm(psa-ps)
            #print(dist)
            if dist < self_dist:
                trobat = True
            i+=1
        if not trobat:
            psarray.poses.append(pstamped.pose)
    return psarray

def obtain_markerArray(psarray,mkrarray,cluster):
    l = len(psarray.poses)
    #print(l)
    i=0
    while i< l:
        Marker_ = Marker()
        Marker_.header.frame_id = "map"
        Marker_.header.stamp = rospy.Time(0)
        Marker_.ns = "basic_shapes"
        Marker_.id = i
        Marker_.type = Marker_.CUBE
        Marker_.action = 0
        Marker_.pose.position = psarray.poses[i].position
        Marker_.pose.orientation = psarray.poses[i].orientation
        Marker_.scale.x = 0.1;
        Marker_.scale.y = 0.2;
        Marker_.scale.z = 0.2;
        if cluster == "person":
            Marker_.color.r = 0
            Marker_.color.g = 0
            Marker_.color.b = 255
            Marker_.color.a = 1
        elif   cluster == "danger_zone":
            Marker_.color.r = 255
            Marker_.color.g = 0
            Marker_.color.b = 0
            Marker_.color.a = 1
        elif   cluster == "wayout":
            Marker_.color.r = 0
            Marker_.color.g = 255
            Marker_.color.b = 0
            Marker_.color.a = 1
        else:
            Marker_.color.r = 0
            Marker_.color.g = 0
            Marker_.color.b = 0
            Marker_.color.a = 1
        Marker_.lifetime = rospy.Duration()
        mkrarray.markers.append(Marker_)
        i+=1
    return mkrarray
