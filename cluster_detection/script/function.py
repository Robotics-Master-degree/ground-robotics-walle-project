import numpy
import rospy
import math
from visualization_msgs.msg import Marker
from people_msgs.msg import People
from people_msgs.msg import Person
from geometry_msgs.msg import Vector3,PointStamped,PoseStamped,PoseArray
def check_waypoint(pstamped,psarray,self,ps_temp):
   #print(pstamped)
    ps = numpy.array((pstamped.pose.position.y,pstamped.pose.position.z))
    #ps_temp = numpy.array((0,0,0))
    #print(colour)
    #print("ps",ps)
    if (len(psarray.poses) == 0): #there isn't any pose defined in psarray
        #print("first iteration, adding psarray")
        psarray.poses.append(pstamped.pose)

    else:
        l = len(psarray.poses)
        i=0
        trobat = False #there isn't any marke in posearray that is the same as this one
        #check2punts = False
        while (i<l):
            psa = numpy.array((psarray.poses[i].position.y,psarray.poses[i].position.z))
            #dist = numpy.linalg.norm(psa-ps)
            dist = numpy.linalg.norm(psa-ps)
            #print("psa",psa)
            #print("ps",ps)
            #print("dist",dist)
            if dist < self.dist:
                trobat = True
            i+=1
            if not trobat:
                #dist_p = numpy.linalg.norm(psa-ps)
                #ps is an array
                #print(self.ps_temp)
                punt_temp = numpy.array((ps_temp[1],ps_temp[2])) #catch previous point
                #dist_ps = numpy.linalg.norm(ps)
                #dist_temp = numpy.linalg.norm(self.ps_temp)
                computed_dist =  numpy.linalg.norm(ps-punt_temp)
                if computed_dist > self.dist_temp:
                    ps_temp = numpy.array([0,ps[0],ps[1]])
                    print("computed_dist")
                    psarray.poses.append(pstamped.pose)
                else:
                    offset_y = abs(ps[0]-ps_temp[1])
                    offset_z = abs(ps[1]-ps_temp[2])
                    #print("Distancia ps",dist_ps)
                    #print("Distancia temp",dist_temp)
                    #print("computed_dist_",computed_dist)
                    #print("offset_y_",offset_y)
                    #print("offset_z_",offset_z)
                    #if ((abs(ps[0]-self.ps_temp[0]) > 0.2 or abs(ps[1]-self.ps_temp[1])>abs(0.2))and abs(dist_ps-dist_temp) > abs(0.08)):
                    if  (offset_y > self.offset_y or  offset_z > self.offset_z and abs(computed_dist) > self.offset_dist):
                        ps_temp = numpy.array([0,ps[0],ps[1]])
                        #print("ps_temp",ps_temp)
                    else:
                        psarray.poses.append(pstamped.pose)
                        ps_temp = numpy.array([0,0,0])
                        #print("S'ha afegit punt!!!")

    return psarray,ps_temp


def double_check_v2(psarrayastra,psarraylidar,psarrayant,dist_cluster):
    #print(psarrayastra)
    #print(psarraylidar)
    #print("holis")
    l_astra = len(psarrayastra.poses)
    l_lidar = len(psarraylidar.poses)
    l_ant = len(psarrayant.poses)
    i=0
    trobada = False
    tractada = False
    if (l_astra != 0)  and (l_ant == 0) :
        if(abs(psarrayastra.poses[0].position.x) > 1.5):
            psarrayant.poses.append(psarrayastra.poses[0])
        else:
            print("not initialized")
    elif (l_lidar != 0) and (l_ant == 0):
        psarrayant.poses.append(psarraylidar.poses[0])
    else:
        for a in psarrayastra.poses:
            psastra = numpy.array((a.position.y,a.position.z))
            for l in psarraylidar.poses:
                pslidar = numpy.array((l.position.y,l.position.z))
                dist = numpy.linalg.norm(psastra-pslidar)
                #print("dist",dist)
                if abs(dist) < dist_cluster:
                    dist_1 = dist
                    trobada = True
                    #print("trobat",trobada)
                    for ant in psarrayant.poses:
                        psant = numpy.array((ant.position.y,ant.position.z))
                        dist2 = numpy.linalg.norm(psant - psastra)
                        #print("dist2",dist2)
                        if abs(dist2) < 0.3:
                            tractada = True
                            #print("tractada",tractada)
                        else:
                            tractada = False
                            #print("tractada",tractada)
                            print("a_",a)
                            print("l",l)
                            print("dist_1",dist_1)
                            print("dust2_2",dist2)
                            if (a.position.x > 1):
                                print("position>x")
                                psarrayant.poses.append(a)
                            else:
                                psarrayant.poses.append(l)
                                print("position<x")
                            break
                else:
                    trobada = False
                    print("trobat",trobada)
        trobada=False
        tractada=False
        """for ant in psarrayant.poses:
                psant = numpy.array((ant.position.y,ant.position.z))
                dist2 = numpy.linalg.norm(psant - psastra)
                if abs(dist2) < 0.5:
                    tractada = True
                    a_ = a
                    l_ = l
                else:
                    tractada = False
            if (not tractada) and trobada:
                a_ = a
                l_ = l
                print("adding new pose ")
                print("a_",a_)
                print("l",l_)
                print("dist_1",dist)
                print("dust2_2",dist2)
                if (a.position.x > 1):
                    print("position>x")
                    psarrayant.poses.append(a)
                else:
                    psarrayant.poses.append(l)
                    print("position<x")
            tractada = False
            trobada = False
        if (not tractada) and trobada:
            print("adding new pose ")
            print("a_",a_)
            print("l",l_)
            print("dist_1",dist_1)
            print("dust2_2",dist_2)
            if (a_.position.x > 1):
                print("position>x")
                psarrayant.poses.append(a_)
            else:
                psarrayant.poses.append(l_)
                print("position<x")
            #tractada = False
            #trobada = False
            tractada = False
        trobada = False
    #print(psarrayant)"""
    return psarrayant

def double_check_v3(psarrayastra,psarraylidar,psarrayant,dist_cluster,dist_double_check,distxy):
    #print("psarrayastra",psarrayastra)
    #print("psarraylidar",psarraylidar)
    #print("holis")
    #print(psarrayant)
    l_astra = len(psarrayastra.poses)
    #print("l_astra",l_astra)
    l_lidar = len(psarraylidar.poses)
    #print("l_liDar",l_lidar)
    l_ant = len(psarrayant.poses)
    #print("l_ant",l_ant)
    i=0
    trobada = False
    tractada = False
    if (l_astra != 0)  and (l_ant == 0) :
        if(abs(psarrayastra.poses[0].position.x) > 1.5):
            psarrayant.poses.append(psarrayastra.poses[0])
        else:
            print("not initialized")
    elif (l_lidar != 0) and (l_ant == 0):
        x = psarraylidar.poses[0].position.x
        y = psarraylidar.poses[0].position.y
        z = psarraylidar.poses[0].position.z
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            print("is nan")
        else:
            psarrayant.poses.append(psarraylidar.poses[0])
            print("adding rplidar pose")
            print("l", psarraylidar.poses[0])
    else:
        for a in psarrayastra.poses:
            psastra = numpy.array((a.position.x,a.position.y,a.position.z))
            psastra_ = numpy.array((a.position.y,a.position.z))
            for l in psarraylidar.poses:
                pslidar = numpy.array((l.position.x,l.position.y,l.position.z))
                dist = numpy.linalg.norm(psastra-pslidar)
                #print("dist",dist)
                if abs(dist) < dist_cluster:
                    #dist_1 = dist
                    trobada = True
                    print("hem trobat un punt comu lidar,astra",dist)
                    for ant in psarrayant.poses:
                        psant = numpy.array((ant.position.x,ant.position.y,ant.position.z))
                        psant_ = numpy.array((ant.position.y,ant.position.z))
                        dist2 = numpy.linalg.norm(psastra-psant)
                        dist2_ = numpy.linalg.norm(psastra_-psant_)
                        print("dist x,y",dist2_)
                        print("dist x,y,z", dist2)
                        #print("dist2",dist2)
                        #print("dist2_",dist2_)
                        #print("ant",ant)
                        if abs(dist2) < dist_double_check:
                            print("hem trobat un punt comu dins dels waypoints",dist2)
                            tractada = True
                        if abs(dist2_) < distxy:
                            #0.95 red
                            #print("hem trobat un punt comu dins dels waypoints",dist2_)
                            tractada = True
                        else:
                            #tractada = False
                            dist1n = dist
                            dist2n = dist2
                            dist2yz = dist2_
                            a_ = a
                            l_ = l
                            ant_ = ant

        if (not tractada) and trobada:
            print("adding new pose ")
            print("a_",a_)
            print("ant",ant_)
            print("l",l_)
            print("dist1n",dist1n)
            print("dust2_2",dist2n)
            print("dist2_yz",dist2yz)
            #print("ant",ant_)
            #posar doble check
            if (a_.position.x > 1):
                print("position>x")
                psarrayant.poses.append(a_)
            else:
                print("position<x")
                psarrayant.poses.append(l_)
            trobada=False
            tractada=False
                #break
        trobada=False
        tractada=False
        """l_astra = len(psarrayastra.poses)
        l_lidar = len(psarraylidar.poses)
        l_ant = len(psarrayant.poses)
        i=0
        trobada = False
        tractada = False
        if (l_astra != 0)  and (l_ant == 0) :
            if(abs(psarrayastra.poses[0].position.x) > 1.5):
                psarrayant.poses.append(psarrayastra.poses[0])
            else:
                print("not initialized")
        elif (l_lidar != 0) and (l_ant == 0):
            psarrayant.poses.append(psarraylidar.poses[0])
        else:
            for a in psarrayastra.poses:
                psastra = numpy.array((a.position.x,a.position.y,a.position.z))
                for l in psarraylidar.poses:
                    pslidar = numpy.array((l.position.x,l.position.y,l.position.z))
                    dist = numpy.linalg.norm(psastra-pslidar)
                    #print("dist",dist)
                    if abs(dist) < dist_cluster:
                        #print("l",l)
                        #print("a",a)
                        dist_1 = dist
                        trobada = True
                        #print("l")
                        #print("dist<distlcuster", dist)
                        for ant in psarrayant.poses:
                            psant = numpy.array((ant.position.x,ant.position.y,ant.position.z))
                            #print("psant",psant)
                            #print("psastra",psastra)
                            dist2 = numpy.linalg.norm(psant - psastra)
                            dist_2 = dist2
                            #print("dist2",dist2)
                            if abs(dist2) < 0.5:
                                tractada = True
                            else:
                                tractada = False
                                l_ = l
                                a_ = a
                    else:
                        trobada = False
                if (not tractada) and trobada:
                    print("adding new pose ")
                    print("a_",a_)
                    print("l",l_)
                    print("dist_1",dist_1)
                    print("dust2_2",dist_2)
                    if (a_.position.x > 1):
                        print("position>x")
                        psarrayant.poses.append(a_)
                    else:
                        psarrayant.poses.append(l_)
                        print("position<x")
                    #tractada = False
                    #trobada = False
                tractada = False
                trobada = False
        #print(psarrayant)"""
    return psarrayant



def double_check(psarray10,psarray30,psarr,dist_clusters):
    #print("psarray10",psarray10)
    #print("psarray30",psarray30)
    l10 = len(psarray10.poses)
    l30 = len(psarray30.poses)
    i=0
    trobat= False
    if (l10 !=0 ):
        psarr.header = psarray10.header
        #psarr.header.frame_id = "map"
    if (l30 !=0 ):
        psarr.header = psarray30.header
    if (l10 == 0) and (l30 != 0):
        print("case1")
        psarr = psarray30
        psarr.poses.append(psarray30.poses[0])
    elif (l10 !=0 ) and (l30 == 0):
        print("case2")
        #psarr = psarray10
        psarr.poses.append(psarray10.poses[0])
    elif (l10==1) and (l30 ==1):
        print("case3")
        ps10 = numpy.array((psarray10.poses[0].position.x,psarray10.poses[0].position.y,psarray10.poses[0].position.z))
        ps30 = numpy.array((psarray30.poses[0].position.x,psarray30.poses[0].position.y,psarray30.poses[0].position.z))
        dist = numpy.linalg.norm(ps10-ps30)
        if dist < dist_clusters:
            psarr.poses.append(psarray30.poses[0])
    elif l10 > l30:
        print("case4")
        while(i<l10):
            ps10 = numpy.array((psarray10.poses[i].position.x,psarray10.poses[i].position.y,psarray10.poses[i].position.z))
            for o in psarray30.poses:
                ps30 = numpy.array((o.position.x,o.position.y,o.position.z))
                dist = numpy.linalg.norm(ps10-ps30)
                if (dist < dist_clusters) and not trobat:
                    trobat = True
            if (trobat):
                psarr.poses.append(psarray10.poses[i])
                trobat = False
            i+=1

    elif l30 >= l10:
        print("case5")
        while(i<l30):
            ps30 = numpy.array((psarray30.poses[i].position.x,psarray30.poses[i].position.y,psarray30.poses[i].position.z))
            for o in psarray10.poses:
                ps10 = numpy.array((o.position.x,o.position.y,o.position.z))
                dist = numpy.linalg.norm(ps10-ps30)
                if dist < dist_clusters and not trobat:
                    trobat = True
            if (trobat):
                psarr.poses.append(psarray30.poses[i])
                Trobat = False
            i+=1
    else:
        print("do not know how to proceed")
        psarr = psarray30
        """while(i<l):
            ps = numpy.array((psarray10.poses[i].position.y,psarray.poses[i].position.z))
            for o in psarray.poses:
                psa = numpy.array((o.position.y,o.position.z))
                dist = numpy.linalg.norm(psa-ps)
                #print(psarray.poses[i])
                #print("dist",dist)
                #print("psa",psa)
                #print("ps",ps)
                if dist > 0.8:
                    psarr.poses.append(psarray.poses[i])
                    print(psarray.poses[i])
                    #print(psarr.header)
                    print(i)
            i+=1"""
    #print("psarr",psarr)
    return psarr

def check_waypoint_v2(pstamped,psarray,dist_cluster):
    #print(pstamped)
    ps = numpy.array((pstamped.pose.position.y,pstamped.pose.position.z))
    #ps_temp = numpy.array((0,0,0))
    #print(colour)
    #print("ps",ps)
    if (len(psarray.poses) == 0): #there isn't any pose defined in psarray
        #print("first iteration, adding psarray")
        psarray.header = pstamped.header
        psarray.poses.append(pstamped.pose)
    else:
        l = len(psarray.poses)
        i=0
        trobat = False #there isn't any marke in posearray that is the same as this one
        #check2punts = False
        while (i<l):
            psa = numpy.array((psarray.poses[i].position.y,psarray.poses[i].position.z))
            dist_y = psarray.poses[i].position.y - pstamped.pose.position.y
            dist_z = numpy.subtract(psarray.poses[i].position.z,pstamped.pose.position.z)
            dist = numpy.linalg.norm(psa-ps)
            #print("dist_y",dist_y)
            #print("dist_z",dist_z)
            if abs(dist) < dist_cluster:
            #if abs(dist_z) < self.offset_z_max and abs(dist_y) < self.offset_y_max:
                trobat = True #ja tenim la persona
                #print("dist",dist)
            i+=1
        if not trobat:
            psarray.header = pstamped.header
            psarray.poses.append(pstamped.pose)
        #psarray = double_check(psarray,self.psarr)

            dist_ps = numpy.linalg.norm(ps)
            dist_temp = numpy.linalg.norm(self.ps_temp)
            print("Distancia ps",dist_ps)
            print("Distancia temp",dist_temp)
            if ((abs(ps[0]-self.ps_temp[0])>abs(0.2) or abs(ps[1]-self.ps_temp[1])>abs(0.2))and abs(dist_ps-dist_temp) > abs(0.08)):
                self.ps_temp = ps
            else:
                psarray.poses.append(pstamped.pose)
                self.ps_temp = numpy.array((0,0,0))
                print("S'ha afegit punt!!!")


    return psarray


def check_waypoint_v3(pstamped,psarray,dist_cluster):
    #print(pstamped)
    ps = numpy.array((pstamped.pose.position.x,pstamped.pose.position.y,pstamped.pose.position.z))
    #ps_temp = numpy.array((0,0,0))
    #print(colour)
    #print("ps",ps)
    if (len(psarray.poses) == 0): #there isn't any pose defined in psarray
        #print("first iteration, adding psarray")
        psarray.header = pstamped.header
        psarray.poses.append(pstamped.pose)
    else:
        l = len(psarray.poses)
        i=0
        trobat = False #there isn't any marke in posearray that is the same as this one
        #check2punts = False
        while (i<l):
            psa = numpy.array((psarray.poses[i].position.x,psarray.poses[i].position.y,psarray.poses[i].position.z))
            #dist_y = psarray.poses[i].position.y - pstamped.pose.position.y
            #dist_z = numpy.subtract(psarray.poses[i].position.z,pstamped.pose.position.z)
            dist = numpy.linalg.norm(psa-ps)
            #print("dist_y",dist_y)
            #print("dist_z",dist_z)
            if abs(dist) < dist_cluster:
            #if abs(dist_z) < self.offset_z_max and abs(dist_y) < self.offset_y_max:
                trobat = True #ja tenim la persona
                #print("dist",dist)
            i+=1
        if not trobat:
            psarray.header = pstamped.header
            psarray.poses.append(pstamped.pose)
        #psarray = double_check(psarray,self.psarr)

    return psarray



def obtain_people_danger_zones(psarray):
    people = People()
    l = len(psarray.poses)
    #print("l",l)
    i=0
    #print("i",i)
    people.header = psarray.header
    while (i <l):
        person = Person()
        person.name = str(i)
        person.position = psarray.poses[i].position
        people.people.append(person)
        i+=1
        #print("i",i)
    return people



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
        #Marker_.type = Marker_.CUBE
        Marker_.type = Marker_.TEXT_VIEW_FACING;
        Marker_.action = 0
        Marker_.pose.position = psarray.poses[i].position
        Marker_.pose.orientation = psarray.poses[i].orientation
        #Marker_.scale.x = 0.1;
        #Marker_.scale.y = 0.2;
        #Marker_.scale.z = 0.5;
        #Marker_.scale.x = 0.1;
        #Marker_.scale.y = 0.2;
        Marker_.scale.z = 0.3;
        if cluster == "person":
            Marker_.color.r = 0
            Marker_.color.g = 0
            #Marker_.color.b = 255
            Marker_.color.b = 1
            Marker_.color.a = 1
        elif   cluster == "danger_zone":
            #Marker_.color.r = 255
            Marker_.color.r = 1
            Marker_.color.g = 0
            Marker_.color.b = 0
            Marker_.color.a = 1
        elif   cluster == "wayout":
            Marker_.color.r = 0
            #Marker_.color.g = 255
            Marker_.color.g = 1
            Marker_.color.b = 0
            Marker_.color.a = 1
        else:
            Marker_.color.r = 0
            Marker_.color.g = 0
            Marker_.color.b = 0
            Marker_.color.a = 1
        #print("Marker Color", Marker_.color)
        Marker_.text = cluster + ": " + str(i)
        Marker_.lifetime = rospy.Duration()

        mkrarray.markers.append(Marker_)
        i+=1
    return mkrarray
