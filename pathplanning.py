#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import *
import sys
from IPython.terminal.embed import InteractiveShellEmbed
import geopy
import copy
from geopy.units import meters
from numpy import Inf, angle
from pygeodesy.units import Lat
from pygeodesy.sphericalTrigonometry import LatLon
from pygeodesy.sphericalNvector import LatLon as LatLonS
from pygeodesy.sphericalNvector import intersection
from geopy.distance import geodesic
from geopy import Point, distance
from pygeodesy.points import *
from pygeodesy.points import isclockwise, isconvex, centroidOf
from itertools import cycle

import waypoint
from utils import *
from waypoint import WayPoint
from waypointsmap import WaypointMap
from collections import deque
from copy import copy

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones
# Faire des schémas
# https://www.math10.com/en/geometry/geogebra/fullscreen.html


debug = False
#debug = True


class PathPlanning:
    "PathPlanning class"

    def __init__(self, points,  bearing = 0, lateral_footprint=0, longitudinal_footprint=0, percent_recouvrement_lat=0., percent_recouvrement_lon=0., orientation=None, start_point=None):
        """Pathplanning generates waypoints """

        # a voir comment adapter normal_plus dans le cas concav avec 4 points par 
        if not isconvex(points):
            print("points must form a convex shape")
            sys.exit(-1)

        self.points = points
        self.centroid =centroidOf(self.points,LatLon=LatLon)
        self.centroid.text='Centroid'

        self.nb_points = len(self.points)
        self.bearing=bearing
#        if debug: print('Les points de ce pathplanning sont {}'.format(self.points))
        self.waypoint_list = []
        self.paired_waypoints_list = []
        self.extra_point = []
        
        self.distances = []
        self.bearings = []
        if isclockwise(points):
            self.othogonal_increment = 90
            self.isclockwise = True
        else:
            self.othogonal_increment = -90
            self.isclockwise = False

        self.orientation = orientation
        self.start_point = start_point
        self.lateral_footprint = lateral_footprint  # en mètres
        self.longitudinal_footprint = longitudinal_footprint  # en mètres
        self.recouvrement_lat = percent_recouvrement_lat  # pourcentage
        self.recouvrement_lon = percent_recouvrement_lon  # pourcentage
        self.increment_lon = self.longitudinal_footprint*(1-percent_recouvrement_lon)
        self.increment_lat = self.lateral_footprint*(1-percent_recouvrement_lat)
        self.compute_distances_and_bearings(self.points)


 

    def compute_distances_and_bearings(self,input_points):
        """ compute distances between all points"""
        i=0
        point_cycle = cycle(input_points)
        next_point = next(point_cycle)
        preceding_point=input_points[-1]
        while i<len(input_points):
            this_point, next_point = next_point, next(point_cycle)
            #print(this_point)
            #if debug: print('point lat {} lon {}'.format(this_point.lat, this_point.lon))
            #self.distances.append(this_point.distanceTo(next_point))
            #self.bearings.append(this_point.compassAngleTo(next_point))
            this_point.angle=getAnglelatlon(preceding_point,this_point,next_point)
            this_point.bearing=this_point.compassAngleTo(next_point)
            this_point.l=this_point.distanceTo(next_point)
            preceding_point = this_point
            i +=1
        


    def generate_path(self, style):
        """choix du syle du path"""
        self.style=style
        if style == "snail":
            self.generate_path_snail()
        elif style == "normal":
            self.generate_path_normal()
        elif style == "espiral":
            self.generate_path_espiral()
        elif style == "normal_plus":
            self.generate_path_normal_plus()
            

    def generate_path_normal_plus(self):
        # find the closest and the raest summit  to the start point
        min_dist=Inf
        max_dist = 0
        self.closest_summit = None
        self.farest_summit = None
        
        for point in self.points :
            tmp_distance = self.start_point.distanceTo(point)
            if tmp_distance > max_dist:
                max_dist=tmp_distance
                self.farest_summit = point
            if tmp_distance < min_dist:
                min_dist=tmp_distance
                self.closest_summit = point
        
        self.closest_summit.text='closest summit'
        self.farest_summit.text = 'farest summit'
        #self.extra_point.append(self.closest_summit)
        #self.extra_point.append(self.farest_summit)

        # Find the best starting point for self.bearing
        # => if we start W to E, we need to find the Western summit 

        ref_point = self.farest_summit
        distance_max_to_centroid = 0
        projection_point = None

        for point in self.points:
            intersection_point = intersection(LatLonS(point.lat,point.lon),self.bearing,LatLonS(self.centroid.lat,self.centroid.lon),self.bearing+90)
            distance_to_centroid = intersection_point.distanceTo(LatLonS(self.centroid.lat,self.centroid.lon))
            if (distance_to_centroid > 10000):
                intersection_point= intersection_point.antipode()
                distance_to_centroid = intersection_point.distanceTo(LatLonS(self.centroid.lat,self.centroid.lon))
            #intersection_point.text='intersection'
            #self.extra_point.append(intersection_point)

            if distance_to_centroid > distance_max_to_centroid:
                distance_max_to_centroid = distance_to_centroid
                ref_point = point
                projection_point = LatLon(intersection_point.lat,intersection_point.lon)

        ref_point.text='ref point'
        #self.extra_point.append(ref_point)
        #self.extra_point.append(self.centroid)

        # find direction from ref_point to centroid
        direction_to_centroid = projection_point.compassAngleTo(self.centroid)
        #print('direction_to_centroid {} self.angle {}'.format(direction_to_centroid,self.bearing))
        
        # Create all the segments of the area
        self.area_segments =[]
        i=0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)
        while i<self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            self.area_segments.append([this_point, next_point])
            i+=1
        

        # If we start from self.farest_summit
        intersection_list = []
        intersection_exist = True
        i=1

        # We search until we do not have intersections points
        while intersection_exist == True :
            #new_point = ref_point.destination(i*self.increment_lat,self.bearing+self.othogonal_increment)
            new_point = ref_point.destination(i*self.increment_lat,direction_to_centroid)
            

           # new_point.text='new point'
            #self.extra_point.append(new_point)

            intersection_exist = False

            # find intersections points
            for segment in self.area_segments:

                # calcul avec les LatLonS de pygeodesy.sphericalNvector
                intersection_point_A = intersection(LatLonS(new_point.lat,new_point.lon),self.bearing,LatLonS(segment[0].lat,segment[0].lon),LatLonS(segment[1].lat,segment[1].lon))
                intersection_point_B = intersection(LatLonS(new_point.lat,new_point.lon),self.bearing+180,LatLonS(segment[0].lat,segment[0].lon),LatLonS(segment[1].lat,segment[1].lon))
                
                # the intersection point shoumd be in the segment
                if intersection_point_A  and  intersection_point_A.iswithin(LatLonS(segment[0].lat,segment[0].lon),LatLonS(segment[1].lat,segment[1].lon)) :
                    intersection_list.append(intersection_point_A)
                    intersection_exist = True
                elif intersection_point_B  and  intersection_point_B.iswithin(LatLonS(segment[0].lat,segment[0].lon),LatLonS(segment[1].lat,segment[1].lon)) :
                    intersection_list.append(intersection_point_B)
                    intersection_exist = True
            
            i+=1
            
        # we need to reorder the pairs of point
        flip_left_right = False

        # the summit will be the first WP
        self.waypoint_list.append(WayPoint(ref_point,self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
       
        self.paired_waypoints_list.append([[ref_point.lat, ref_point.lon],[]])


   
        
        for i in range(0,len(intersection_list)-1,2):
            if flip_left_right :
                self.waypoint_list.append(WayPoint(intersection_list[i],self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(intersection_list[i+1],self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.paired_waypoints_list.append([[intersection_list[i].lat,intersection_list[i].lon],[intersection_list[i+1].lat,intersection_list[i+1].lon]])
            else :
                self.waypoint_list.append(WayPoint(intersection_list[i+1],self.bearing+180, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(intersection_list[i],self.bearing+180, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.paired_waypoints_list.append([[intersection_list[i+1].lat,intersection_list[i+1].lon],[intersection_list[i].lat,intersection_list[i].lon]])


            flip_left_right = not flip_left_right


    def generate_path_espiral(self):
        """ Crée un parcours de type escargot """
        
        #   for point in self.points:
        #    print('Point {} {} Angle {} Distance au suivant {}'.format(point.lat ,point.lon,point.angle,point.l))
        
 
        #self.waypoint_list.append(WayPoint(self.centroid,0, lateral_footprint=0, longitudinal_footprint=0, wp_text="Centroid"))
                            
        # Find shortest distance between self.centroid and each segments
        distance_cp_min = float('inf')
        i=0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)

        while i<self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            tmp_distance_point =(self.centroid.nearestOn(this_point, next_point))
            tmp_distance=tmp_distance_point.distanceTo(self.centroid)
            #print('distance {} {} to {} {} is {}'.format(this_point.lat,this_point.lon,next_point.lat,next_point.lon,tmp_distance))
            if tmp_distance <distance_cp_min : distance_cp_min = tmp_distance
            i +=1
        print('Centroid to each segment minimum distance is {}'.format(distance_cp_min))

        ovx=self.lateral_footprint*self.recouvrement_lat
        #### NB of Rings
        distance_between_rings = self.lateral_footprint*(1-self.recouvrement_lat)
        print('distance_cp_min {} distance_between_rings/2 {} ovx {} self.lateral_footprint {}'.format(distance_cp_min,distance_between_rings/2,ovx,self.lateral_footprint))
        if distance_cp_min < distance_between_rings/2 :
            nb_of_rings =0
            print("0 ring ! => exit")
            sys.exit()

        else:
            nb_of_rings = ceil((distance_cp_min - self.recouvrement_lat*self.lateral_footprint)/(distance_between_rings))

        if nb_of_rings == 1:
            print("un seul ring ! => exit")
            sys.exit()

        print('nb_of_rings is {}'.format(nb_of_rings))

        distance_between_rings = (distance_cp_min -self.lateral_footprint)/(nb_of_rings -1)

        #print('old ovx {} old dr {}'.format(self.increment_lat,distance_between_rings))
        #self.new_ovx = (nb_of_rings*self.lateral_footprint-distance_cp_min)/(nb_of_rings-1)
        self.new_ovx = self.lateral_footprint-distance_between_rings
        
        #new_dr = (distance_cp_min-self.lateral_footprint)/(nb_of_rings-1)
        
        #print('new ovx {} new dr {}'.format(ovx,dr))

        self.first_point = True
        new_point = None
        vertices = self.points

        for ring_nb in range(nb_of_rings):
            print('Ring nb {} nb of vertices {}'.format(ring_nb,len(vertices)))
            vertices = self.compute_ring_waypoints(vertices,ring_nb)
            self.compute_distances_and_bearings(vertices)
            if ring_nb == 0:
                break

                
    def compute_ring_waypoints(self,vertices,ring_nb):
        last_waypoints=[]
        first_waypoints=[]
        new_points=[]

        for point in vertices:
            if self.first_point :
            #if first_point :
                # print('max is {}'.format(max([0,-self.lateral_footprint/tan(radians(point.angle))])))
                d=point.l+max([0,-self.lateral_footprint/tan(radians(point.angle))])
                print('premier point avec d={} sinon d aurait été d={}'.format(d,point.l))
            else:
                d=point.l

            
            nb_waypoints_per_segment=ceil((d-self.recouvrement_lon)/(self.longitudinal_footprint-self.recouvrement_lon))
            #print('Nb de waypoints {} d {} ovy {}'.format(nb_waypoints_per_segment,d,ovy))

            new_ovy = (nb_waypoints_per_segment*self.longitudinal_footprint-point.l)/(nb_waypoints_per_segment-1)
            new_dw = ((d - self.longitudinal_footprint)/(nb_waypoints_per_segment-1))

            #print('Nb de waypoints {} new_dw is {}'.format(nb_waypoints_per_segment,new_dw))

            #print('Ly {} new_ovy{}'.format(self.longitudinal_footprint,new_ovy))

            for i in range(nb_waypoints_per_segment):
                
                if i == 0 :
                    if self.first_point:
                        self.first_point = False
                        new_point= point.destination((self.lateral_footprint/2), point.bearing+self.othogonal_increment)
                        # on rajoute le 1er nouveau point, intersection de l'emprise de new_point avec le segment vertices[-1] et point
                        waypoint_tmp = WayPoint(new_point,point.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint)
 
                        if self.isclockwise:
                        # print( ' self.X0 {} self.X1 {} waypoint_2.X0 {} waypoint_2.X1 {}'.format(self.X0,self.X1,waypoint_2.X0,waypoint_2.X1))
                            new_first_point= doIntersect(waypoint_tmp.X2_latlon,waypoint_tmp.X1_latlon,vertices[-1],point)
                        else:
                        # print( ' self.X2 {} self.X3 {} waypoint_2.X2 {} waypoint_2.X3 {}'.format(self.X2,self.X3,waypoint_2.X2,waypoint_2.X3))
                            new_first_point=  doIntersect(waypoint_tmp.X2_latlon,waypoint_tmp.X1_latlon,vertices[-1],point)
                        new_points.append(new_first_point)
                        new_first_point.text='First New point'
                        self.extra_point.append(new_first_point)

                    else:    
                        new_point_tmp= point.destination((self.lateral_footprint/2), point.bearing+self.othogonal_increment)
                        new_point= new_point_tmp.destination((self.longitudinal_footprint/2), point.bearing)
                    first_waypoints.append(WayPoint(new_point,point.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                else:
                    new_point= last_point.destination(new_dw, point.bearing)

                if i == nb_waypoints_per_segment-1 :
                    last_waypoints.append(WayPoint(new_point,point.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))

                self.waypoint_list.append(WayPoint(new_point,point.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint,wp_text='Ring nb='+str(ring_nb)))
                last_point=new_point
        
        # On retire le dernier point ? 
        #self.waypoint_list.pop()

        ### Calcul des nouveaux vertices :
        i=0
        last_cycle = cycle(last_waypoints)
        first_cycle = cycle(first_waypoints)
        first= next(first_cycle)

        #print('len(last_waypoints)) {}'.format(len(last_waypoints)))
        while i<len(last_waypoints):
            last= next(last_cycle)
            first= next(first_cycle)
            
            inter = last.footprint_intersection(first,self.isclockwise)
            print(inter)
            if inter:
                # self.new_ovx shift
                #print('angle {}'.format(vertices[i].angle))
                new_vertice = inter.destination(self.new_ovx,vertices[i].bearing+self.othogonal_increment+180)
                inter.text='intersection'
                new_vertice.text='intersecti_D'
                new_points.append(new_vertice)
                #print('New inter is {} {}'.format(inter.lat,inter.lon))
                #print('New vertice is {} {}'.format(new_vertice.lat,new_vertice.lon))
                self.extra_point.append(inter)
                self.extra_point.append(new_vertice)
                
            else:
                print('probleme pas d intersection')
            i +=1
        return new_points


    def generate_path_snail(self):
        """ Crée un parcours de type escargot """

        # le premier point est celui qui est donné dans la liste
        tmp_point = self.points[0]

        round_nb = 0
        j = 0
        # keep the created segments with their offset
        segments_list = []
        CD_segments_list = []

        finish=False

        # Each time we create a new waypoint,  lock_points is initialise whith False
        # the stop condition
        lock_points = [False  for i in range(self.nb_points)]

        # Si finish 
        # Any fait un OR sur toute la liste
        # OR operation - Using any()
        # AND Operation - Using all()
        finish_condition=all(lock_points)

        # on parcours tous les points
#        while not finish_condition:
        while not finish:
            # i va décrire tous les points
            i = 0
            for point in self.points:
                print(i)
                if i == self.nb_points-1:
                    next_point = self.points[0]
                else:
                    next_point = self.points[i+1]
                
                # On ne fait rien pour le premier point
                j += 1
                if round_nb == 0:
                    new_point = point
                    #print('new_point is {} points[i] is {}'.format(new_point,self.points[i]))

                    # on change pour le dernier point
                    if i == self.nb_points:
                        new_point = next_point.destination(50, self.bearings[i]+180)

                    #from IPython import embed; embed()

                    self.waypoint_list.append(WayPoint(new_point, self.bearings[i],
                                                       lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text=i))

                    C = point.destination(self.increment_lat*round_nb, self.bearings[i]+self.othogonal_increment)
                    D = C.destination( 50, self.bearings[i])
                    #print('distance entre C et tmp_point {}'.format(distance_to_line(tmp_point, C, D)))

                # After first rounf of all points
                else:
                    # middle_of_base_segment is on the middle of [point next_point] segment
                    middle_of_base_segment = point.destination( point.l/2, self.bearings[i])

                    # C is on the parrall line to [point next_point] shifted of self.increment_lat*round_nb
                    C = middle_of_base_segment.destination( self.increment_lat*round_nb, self.bearings[i]+self.othogonal_increment)

                    # D is on the "C line". 50 meters has no real meaning except for creating a second point on the line
                    D = C.destination( 50, self.bearings[i])

                    #print('distance entre C et tmp_point {}'.format(distance_to_line(tmp_point, C, D)))
                    # new point is a potential waypoint, and the intersection of [CD] and last created waypoint with bearing of current point 
                    new_point = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], C, self.bearings[i])


                    # E is  on the parrall line to [next_point new_next_point] shifted of self.increment_lat*round_nb
                    # F is on the same "E line"'
                    # new_point_bis is the intersection point if it exists of [EF] and last created waypoint with bearing of next point 
                    if i<self.nb_points-1:
                        E = next_point.destination( self.increment_lat*round_nb, self.bearings[i+1]+self.othogonal_increment)
                        # 50m est une valeur completement aléatoire !
                        F = E.destination( 50, self.bearings[i+1])
                        new_point_bis = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], E, self.bearings[i+1])

                    #last_point
                    else:
                        E = next_point.destination( self.increment_lat*(round_nb+1), self.bearings[0]+self.othogonal_increment)
                        # 50m est une valeur completement aléatoire !
                        F = E.destination( 50, self.bearings[0])
                        new_point_bis = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], E, self.bearings[0])

                    #on croise la parralle au segment[i+1] avant la parralle au segment[i] donc le new_point doit etre le new_point_bis.
                    # il faut verifier que new_point et new_point_bis ont le meme bearing sinon les points ne sont pas dans le bon ordre 
                    #if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis) and tmp_point.compassAngleTo(new_point) == tmp_point.compassAngleTo((new_point_bis)):
                        #pass

                    #print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))

                    # If new_point_bis is closer to tmp_point and inside [tmp_point new_point] segmnet, then the new point is new_point_bis
                    if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis)  and abs(tmp_point.compassAngleTo(new_point) - tmp_point.compassAngleTo((new_point_bis))<2):
                    #if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis) :
                        #print('tmp_point.compassAngleTo(new_point) {} tmp_point.compassAngleTo((new_point_bis)) {}'.format(tmp_point.compassAngleTo(new_point) , tmp_point.compassAngleTo((new_point_bis))))
                       # print('tmp_point.distanceTo(new_point) {} tmp_point.distanceTo(new_point_bis){}'.format(tmp_point.distanceTo(new_point),tmp_point.distanceTo(new_point_bis)))
                       # print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))
                       # self.waypoint_list.append(WayPoint(C, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="C"))
                       # self.waypoint_list.append(WayPoint(D, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="D"))
                       # self.waypoint_list.append(WayPoint(E, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="E"))
                       # self.waypoint_list.append(WayPoint(F, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="F"))
                       # self.waypoint_list.append(WayPoint(tmp_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="tmp_point"))
                       # self.waypoint_list.append(WayPoint(new_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="new_point"))
                       # self.waypoint_list.append(WayPoint(new_point_bis, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="new_point_bis"))
                        #from IPython import embed; embed()
                        #finish = True
                        #break
                        new_point=new_point_bis


                        # We have a lock with current point
                        lock_points[i]=True

                    


                    # Si le nouveau point n'est pas dirigé dans le sens de angle_bearing, c'est que l'on est allé trop loin
                    new_bearing = tmp_point.compassAngleTo(new_point)
                    angle_bearing=self.bearings[i-1]

                    print('bearing tmp to new {}  bearing angle {}'.format(new_bearing,angle_bearing))
                    
                    if abs(new_bearing-angle_bearing) >1:
                        print('changement de sens donc on arrete ici')
                        print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))

                        finish = True
                        break

                    if j >  78 :
                        print('tmp_point.compassAngleTo(new_point) {} tmp_point.compassAngleTo((new_point_bis)) {}'.format(tmp_point.compassAngleTo(new_point) , tmp_point.compassAngleTo((new_point_bis))))
                        print('tmp_point.distanceTo(new_point) {} tmp_point.distanceTo(new_point_bis){}'.format(tmp_point.distanceTo(new_point),tmp_point.distanceTo(new_point_bis)))
                        print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))
                        self.waypoint_list.append(WayPoint(C, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="C"))
                        self.waypoint_list.append(WayPoint(D, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="D"))
                        self.waypoint_list.append(WayPoint(E, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="E"))
                        self.waypoint_list.append(WayPoint(F, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="F"))
                        self.waypoint_list.append(WayPoint(tmp_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="tmp_point"))
                        self.waypoint_list.append(WayPoint(new_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="new_point"))
                        self.waypoint_list.append(WayPoint(new_point_bis, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="new_point_bis"))
                        #from IPython import embed; embed()
                        #finish = True
                        break 

                    # on cherche si le nouveau segment coupe un des précédants sans que ce soit le dernier et en partant en arrière
#                    intersec_point, segment = intersect_segments_list([tmp_point, new_point], segments_list[:-1])
                    intersec_point, segment = intersect_segments_list([tmp_point, new_point], segments_list[:-1])
                    intersec_point_CD, segment_CD = intersect_segments_list([tmp_point, new_point], CD_segments_list)
                    
                    if intersec_point_CD:
                        print('intersection CD ici {} {}'.format(intersec_point_CD.lat,intersec_point_CD.lon))

                    ## probleme ici. new_point a peu de chance d'etre sur un ancien CD...
#                    if intersec_point and ( onSegment(segment[0],new_point,segment[1] )):
                    if intersec_point :
                        print('Fini intersection avec segment déja créé')
                        print('Point {} {} C {} {}'.format(point.lat, point.lon, C.lat, C.lon))
                        print('intersc {} {} tmp_point {} {} new_point {} {}'.format(intersec_point.lat, intersec_point.lon, tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon))
                        print('intersc_segment {} {} {} {}'.format(segment[0].lat, segment[0].lon, segment[1].lat, segment[1].lon))
                        

                        # il faut trouver le dernier point

                        #le nouveau point coupe la parralle a un vieux segment. Il faut s'arretera l'intersection 
                        if not onSegment(segment[0],new_point,segment[1]):
                            print('le nouveau point coupe la parralle a un vieux segment')
                            self.waypoint_list.append(WayPoint(intersec_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="Last"))
                            finish = True
                            break

                        # on teste si la droite parralelle au segment coupé passant par tmp_point est plus éloigné de self.increment_lat. si ce n'est pas le cas alors
                        # tmp est le dernier point
                        # on trouve l'angle entre le segment coupé et [tmp,new]

                        angle = getAnglelatlon(segment[1], intersec_point, tmp_point)
                        H = tmp_point.distanceTo(intersec_point)*sin(radians(angle))
                        #print('H {} self.increment_lat {}'.format(H, self.increment_lat))

                        if H <= self.increment_lat:
                            print('tmp est le dernier point possible')
                            finish = True
                            break
                        # sinon, on trouve l'intersection avec la parrallele au segment décalé de self.increment_lat
                        distance = (H-self.increment_lat)/sin(radians(angle))
                        # Le +90 depend si on est clockwise !!!!
                        last_point = tmp_point.destination(distance, tmp_point.compassAngleTo(new_point))
                        self.waypoint_list.append(WayPoint(last_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint, wp_text="Last"))
                        finish = True
                        break
                    self.waypoint_list.append(WayPoint(
                        new_point, self.bearings[i], lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                segments_list.append([tmp_point, new_point])
                CD_segments_list.append([C, D])
                tmp_point = new_point
                i += 1

            round_nb += 1
            finish_condition=all(lock_points)


    def generate_path_normal(self):
        """ path type allez-retour
        avec des parrallelles"""
        # point de départ
        indice_gauche = self.nb_points
        indice_droit = 1
        flip_left_right = True
        limite_AB = .5
        finish = False
        not_in_right = False
        not_in_left = False

        tmp_A = self.points[0]
        tmp_B = self.points[1]
        # la direction initiale est celle de points[0] vers points[1]
        direction_right = tmp_A.compassAngleTo(tmp_B)
        direction_left = tmp_B.compassAngleTo(tmp_A)

        tmp = None
        # premiers point
        self.waypoint_list.append(WayPoint(
            tmp_A, direction_right, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))

        self.waypoint_list.append(WayPoint(tmp_B, tmp_B.compassAngleTo(self.points[indice_droit+1]),
                                           lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
        i = 0

        # on s'arrete quand cette distance est très petite
        while (tmp_A.distanceTo(tmp_B) > limite_AB) and not finish:
            #print('i {}'.format(i))
            # if i == 15:
            #    break
            direction_ext_right = tmp_B.compassAngleTo(self.points[indice_droit+1])
            direction_ext_left = tmp_A.compassAngleTo(self.points[indice_gauche-1])
            #print('direction_ext_right {} direction_ext_left {}'.format(direction_ext_right,direction_ext_left))


            # Le point C est  sur la parralle à [tmp_A tmp_B]
            C = tmp_A.destination(self.increment_lat, direction_right+self.othogonal_increment)
            # le point D est sur la ligne C avec son bearing et à distance d=10m sans importance
            D = C.destination(100, direction_left)
 
            # on trouve les points d'intersection de C avec son bearing et les segments latéraux


            intersect_right = intersect_four_points_latlon(C, D, tmp_B, self.points[indice_droit+1])

            # verification que les points son bien sur les segments
            if not iswithinLatLontTri(intersect_right, tmp_B, self.points[indice_droit+1]):
                #not_in_right = True
                if debug: print('not in segment right')
                indice_droit += 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche) or (indice_droit > self.nb_points - 2):
                    if debug:
                        print('finish')
                    finish = True
                    break
                direction_ext_right = self.points[indice_droit].compassAngleTo(self.points[indice_droit+1])

                intersect_right = intersect_four_points_latlon(C, D, self.points[indice_droit], self.points[indice_droit+1])

                if not iswithinLatLontTri(intersect_right, self.points[indice_droit], self.points[indice_droit+1]):
                    if debug:
                        print('Really not in segment right !')
                    not_in_right = True

            intersect_left = intersect_four_points_latlon(C, D, tmp_A, self.points[indice_gauche-1])

            # si il n'y a pas d'intersection dans ce segment, on prend le suivant
            if not iswithinLatLontTri(intersect_left, tmp_A, self.points[indice_gauche-1]):
                if debug:
                    print('not in segment left')
                indice_gauche -= 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche):
                    if debug:
                        print('finish')
                    finish = True
                    break
                direction_ext_left = self.points[indice_gauche].compassAngleTo(self.points[indice_gauche-1])

                intersect_left = intersect_four_points_latlon(
                    C, D, self.points[indice_gauche], self.points[indice_gauche-1])

                if not iswithinLatLontTri(intersect_left, self.points[indice_gauche], self.points[indice_gauche-1]):
                    if debug:
                        print('Really not in segment left !')
                    not_in_left = True

            if not_in_left and not_in_right:
                if debug:
                    print('Finish !!!! ')
                finish = True

                break

            # on ajoute les points dans le bon ordre
            if flip_left_right:
                self.waypoint_list.append(WayPoint(
                    intersect_right, direction_ext_right, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(
                    intersect_left, direction_ext_left, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
            else:
                self.waypoint_list.append(WayPoint(
                    intersect_left, direction_ext_left, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(
                    intersect_right, direction_ext_right, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))

#            print('intersect_right {} intersect_left {}'.format(                intersect_right, intersect_left))
            tmp_A = intersect_left
            tmp_B = intersect_right
            flip_left_right = not flip_left_right
            i += 1
            not_in_left = False
            not_in_right = False

    def compute_length_and_turns(self):
        """ return the path length and turn numbers from and to the start point"""

        total_distance = 0.0
        nb_turns = 0

        # start point
        tmp_point = self.start_point

        for waypoint in self.waypoint_list:
            #print('type(waypoint) {} type(tmp_point) {}'.format(waypoint,type(tmp_point)))
            total_distance += tmp_point.distanceTo(LatLon(waypoint.point.lat,waypoint.point.lon))
            tmp_point= LatLon(waypoint.point.lat,waypoint.point.lon)
            nb_turns +=1

        total_distance += tmp_point.distanceTo(self.start_point)

      #  print('############################################################')
     #   print('{} Total distance is {}m with {} turns'.format(self.style, total_distance, nb_turns))
      #  print('############################################################')

        return total_distance, nb_turns

    def export_to_list(self):
        """ Export waypoints to a list"""
        wp_list =[]
        for wp in self.waypoint_list:
            wp_list.append(wp.latlon())
        return wp_list
    def export_to_paired_wp(self):
        """ Export waypoints in pairs"""
        return self.paired_waypoints_list


def main(args):
    """la fonction main"""

    """ IPGP
    start_point_list = []
    start_point_list.append(LatLon(48.846383084057315, 2.356424447320463))
    start_point_list.append(LatLon(48.84415899803569, 2.353495475053588))
    start_point_list.append(LatLon(48.844599153918324, 2.355340339361402))
    start_point_list.append(LatLon(48.84508549389749, 2.356311190101862))

    a = LatLon(48.844781966005414, 2.354806246580006)
    b = LatLon(48.845476490908986, 2.3559582742434224)
    c = LatLon(48.844800522139515, 2.356945151087957)
    d = LatLon(48.84415592294359, 2.3565687535257593)
    e = LatLon(48.84395753653702, 2.355015706155173)
    f = LatLon(48.844565798460536, 2.3552507666007094)
    """

    """
    Martinique
    """
    start_point = LatLon(14.810431373395028, -61.176753253004485)
    start_point.text = 'Start'
    a = LatLon(14.801717517497558, -61.17830598375636)
    b= LatLon(14.803184637915658, -61.177470922032136)
    c= LatLon(14.804110982101069, -61.176259107347896)
    d= LatLon(14.80476281073443, -61.175343978459765)
    e= LatLon(14.804147551878703, -61.17414211429372)
    f= LatLon(14.802389075700889, -61.175630772903205)
    g= LatLon(14.801758424759862, -61.176496729696545)

#    points = deque([f, e, d, c, b, a])
#    points = deque([a,f, e, d, c, b ])
#    points = deque([a, b, c])
#    points = deque([a, b, c, d])
    points = deque([a, b, c, d, e,f,g])


    lateral_footprint = 80
    longitudinal_footprint = 50



    Path_generator= PathPlanning(points=points,  bearing = 140,lateral_footprint=lateral_footprint,
                                    longitudinal_footprint=longitudinal_footprint, start_point=start_point, percent_recouvrement_lat=0.6, percent_recouvrement_lon=0.80)
    
    Path_generator.extra_point.append(start_point)
    Path_generator.generate_path("normal_plus")


    ## export map
    the_map = WaypointMap(start_point)
    # on place les limites de la zone
    the_map.add_polygon(points=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")
    
    
    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp, direction=False, footprint=False,footprint_markers=False)
    

    the_map.add_colored_waypoint_path(Path_generator.waypoint_list)

    # Add extra points (for debug)
    for extra in Path_generator.extra_point:
        the_map.add_extra(extra, text=extra.text)
        #print('extra text '+extra.text + '\t\tExtra point\t' + str(extra.lat)+ '\t'+str(extra.lon))

    # Exportation de la carte
    the_map.export_to_file('normal_plus')
    # Path_generator.export_to_kml()




    wp_list =[]
    for wp in Path_generator.waypoint_list:
        wp_list.append(wp.latlon())

   # from IPython import embed; embed()

if __name__ == '__main__':
    main(sys.argv)
