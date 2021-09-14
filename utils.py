#!/usr/bin/python
# -*- coding:utf-8 -*-
from geopy.distance import geodesic
from math import degrees, acos, radians
from geographiclib.geodesic import Geodesic
from geopy import Point, distance, point
import geopy
from math import degrees, sqrt, sin, cos
import sys

from numpy import result_type
from pygeodesy.sphericalTrigonometry import LatLon as latlontri
from pygeodesy.sphericalNvector import *

debug = False
debug = True



def getAngle(A, B, C):
    "return angle en degrees between A,B and C in degrees"
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CA = geodesic(A, C).m

    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))


def getDistance(A, B):
    return geodesic(A, B).m


def getBearing(A, B):
    "return bearing en degrees between A and B"
    return Geodesic.WGS84.Inverse(A[0], A[1], B[0], B[1])['azi1']


def point_distance_bearing_to_new_point(point, distance, bearing):
    """return new coordinates of the input point with distance and bearing
        distance in meters
    """

    dist = geopy.distance.distance(meters=(distance))
    tmp = dist.destination(point=Point(point[0], point[1]), bearing=bearing)
#
    # print("angle "+str(angle + self.orientation))
    return [tmp.latitude, tmp.longitude]

def middlepoint(A,B):
    """ return the point in the middle of AB segment"""
    a = LatLon(A[0], A[1])
    b = LatLon(B[0], B[1])
    c=a.intermediateTo(b,.5)
    return [c.lat,c.lon ] 


def iswithin(A, point1, point2):
    """
    Check whether this point is between two other points.
    If this point is not on the great circle arc defined by both points,
    return whether it is within the area bound by perpendiculars to the great
    circle at each point (in the same hemispere).
    """
    a = LatLon(A[0], A[1])
    b = LatLon(point1[0], point1[1])
    c = LatLon(point2[0], point2[1])
    list = b, c
#    return a.isenclosedBy(list)
    return a.iswithin(b, c)

def issegmentsintersects(A,B,C,D):
    """
    return intersection of segment AB and segment CD
    if it exists otherwise return None
    https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    """


def onSegment(p, q, r):
# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
    if ( (q.lon <= max(p.lon, r.lon)) and (q.lon >= min(p.lon, r.lon)) and
           (q.lat <= max(p.lat, r.lat)) and (q.lat >= min(p.lat, r.lat))):
        return True
    return False
 

def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
     
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.
     
    val = (float(q.lat - p.lat) * (r.lon - q.lon)) - (float(q.lon - p.lon) * (r.lat - q.lat))
    if (val > 0):
         
        # Clockwise orientation
        return 1
    elif (val < 0):
         
        # Counterclockwise orientation
        return 2
    else:
         
        # Collinear orientation
        return 0
 
 
 # The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):
     
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
 
    # Special Cases
 
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
 
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
 
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
 
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
 
    # If none of the cases
    return False

def intersect_four_points_(A,B,C,D):
    """
    return the intersection point of the line AB and line CD
    """

    a = latlontri(A[0], A[1])
    b = latlontri(B[0], B[1])
    c = latlontri(C[0], C[1])
    d = latlontri(D[0], D[1])

    c = a.intersection( b, c,d)
    # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (c.distanceTo(a) > 10000)and (c.distanceTo(a) > 10000):
        c= c.antipode()

    return [c.lat, c.lon]


def intersect_points_bearings(A, bearing_A, B, bearing_B):
    """ return the intersection point C of the two lines
    from A with bearing_A to B with bearing_B
    """
 ##   print('tmp {} bearing_tmp {} C {} bearing_C {}'.format(A, bearing_A, B, bearing_B))
    a = LatLon(A[0], A[1])
    b = LatLon(B[0], B[1])

    c = a.intersection(bearing_A, b, bearing_B)

          # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (c.distanceTo(a) > 10000)and (c.distanceTo(a) > 10000):
        c= c.antipode()
    return [c.lat, c.lon]


def main():
    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

#    print(getAngle(A, B, C))
    #   print(getBearing(A, B))
  #  print(getBearing(B, C))
    A_to_B_bearing = getBearing(A, B)
    C_to_B_bearing = getBearing(C, B)

    inter = intersect_points_bearings(A, A_to_B_bearing, C, C_to_B_bearing)
    print('B is {}, B intersect is {} difference {}'.format(
        B, inter, getDistance(B, inter)))
    iswithin(A, B, C)

    p1= latlontri(1, 1)
    q1= latlontri(10, 1)
    p2= latlontri(1, 2)
    q2= latlontri(10, 2)
    doIntersect(p1,q1,p2,q2)
    
    p1= latlontri(10, 0)
    q1= latlontri(0, 10)
    p2= latlontri(0, 0)
    q2= latlontri(10, 10)
    doIntersect(p1,q1,p2,q2)
    
    p1= latlontri(-5, -5)
    q1= latlontri(0, 0)
    p2= latlontri(1, 1)
    q2= latlontri(10, 10)
    doIntersect(p1,q1,p2,q2)
    

if __name__ == '__main__':
    main()
