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
from pygeodesy.sphericalTrigonometry import LatLon as LatLontri
from pygeodesy.sphericalNvector import LatLon as LatLonsphericalNvector


debug = False
debug = True



def LatLontrigo2nvector(A):
    return(LatLonsphericalNvector(A.lat,A.lon))

def test_LatLontrigo2nvector():
    a = LatLontri(2,4)
    if type(LatLontrigo2nvector(a))==LatLonsphericalNvector:
        return True
    else:
        return False

def getAngle(A, B, C):
    "return angle en degrees between A,B and C in degrees"
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CA = geodesic(A, C).m

    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))

def getAnglelatlon(a, b,c):
    "return angle en degrees between Latlon A,B and C in degrees"
    A=[a.lat,a.lon]
    B=[b.lat,b.lon]
    C=[c.lat,c.lon]
    
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CA = geodesic(A, C).m

    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))


def getDistance(A, B):
    return geodesic(A, B).m

def distance_to_line(A,B,C):
    """ return the distance of A to line BC"""
    CA=C.distanceTo(A)
    CB=C.distanceTo(B)
    BA=B.distanceTo(A)

    BH = (CA*CA-CB*CB-BA*BA)/(2*CB)
    return sqrt(BA*BA-BH*BH)


def point_distance_bearing_to_new_point_latlon(point, distance, bearing):
    """return new coordinates of the input point with distance and bearing
        distance in meters
    """

    dist = geopy.distance.distance(meters=(distance))
    tmp = dist.destination(point=Point(point.lat, point.lon), bearing=bearing)
#
    # print("angle "+str(angle + self.orientation))
    return LatLontri(tmp.latitude, tmp.longitude)


def iswithinLatLontTri(A, point1, point2):
    """
    Check whether this point is between two other points.
    If this point is not on the great circle arc defined by both points,
    return whether it is within the area bound by perpendiculars to the great
    circle at each point (in the same hemispere).
    """
    a = LatLonsphericalNvector(A.lat,A.lon)
    b = LatLonsphericalNvector(point1.lat,point1.lon)
    c = LatLonsphericalNvector(point2.lat, point2.lon)
    list = b, c
#    return a.isenclosedBy(list)
    return a.iswithin(b, c)



def onSegment(p, q, r):
# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
    if ( (q.lat <= max(p.lat, r.lat)) and (q.lat >= min(p.lat, r.lat)) and
           (q.lon <= max(p.lon, r.lon)) and (q.lon >= min(p.lon, r.lon))):
        return True
    return False
 

def orientation_value(p, q, r):
    val = (float(q.lon - p.lon) * (r.lat - q.lat)) - (float(q.lat - p.lat) * (r.lon - q.lon))
    return val
def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
     
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.
     
    val = (float(q.lon - p.lon) * (r.lat - q.lat)) - (float(q.lat - p.lat) * (r.lon - q.lon))
    if (val > 0):
         
        # Clockwise orientation
        return 1
    elif (val < 0):
         
        # Counterclockwise orientation
        return 2
    else:
         
        # Collinear orientation
       # print('Collinear orientation ')
        return 0
 
 
# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
# false if Collinear
def doIntersect(p1,q1,p2,q2):
     
    # Find the 4 orientations required for
    # the general and special cases
    # print("#############################")
    # print('type(p1) {}'.format(type(p1)))
    # print('type(q1) {}'.format(type(q1)))
    # print('type(p2) {}'.format(type(p2)))
    # print('type(q2) {}'.format(type(q2)))
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    # General case
    if ((o1 != o2) and (o3 != o4)):
        inter = p1.intersection( q1,p2,q2)
        # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
        #if (inter.distanceTo(p1) > 10000)and (inter.distanceTo(p1) > 10000):
            #inter= inter.antipode()
        #return inter
        return inter,(p2,q2)
 
    # Special Cases
    """ 
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
    """
    # If none of the cases
    return False,False



def intersect_segments_list(new_vector, segments_list):
    # Returns true if new_vector intersec with one at least one of the segement 
    # of the segment list
    p1=new_vector[0]
    q1=new_vector[1]
    for segment in reversed(segments_list):
        p2=segment[0]
        q2=segment[1]
        inter,segment = doIntersect(p1,q1,p2,q2)
        if inter:
            #print('intersection avec {} {} {} {} et {} {} {} {}'.format(p1.lat,p1.lon,q1.lat,q1.lon,p2.lat,p2.lon,q2.lat,q2.lon))
            return inter,segment
        
    return False,False





def intersect_four_points_latlon(A,B,C,D):
    """
    return the intersection point of the line AB and line CD
    """

    f = A.intersection( B, C,D)
    # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (f.distanceTo(A) > 10000)and (f.distanceTo(A) > 10000):
        f= f.antipode()

    return f

def intersect_points_bearings(A, bearing_A, B, bearing_B):
    """ return the intersection point C of the two lines
    from A with bearing_A to B with bearing_B
    """
 ##   print('tmp {} bearing_tmp {} C {} bearing_C {}'.format(A, bearing_A, B, bearing_B))
    a = LatLontri(A[0], A[1])
    b = LatLontri(B[0], B[1])

    c = a.intersection(bearing_A, b, bearing_B)

          # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (c.distanceTo(a) > 10000)and (c.distanceTo(a) > 10000):
        c= c.antipode()
    return [c.lat, c.lon]

def intersect_points_bearings_latlon(A, bearing_A, B, bearing_B):
    """ return the intersection point C of the two lines
    from A with bearing_A to B with bearing_B
    """
 ##   print('tmp {} bearing_tmp {} C {} bearing_C {}'.format(A, bearing_A, B, bearing_B))
    c = A.intersection(bearing_A, B, bearing_B)

          # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (c.distanceTo(A) > 10000)and (c.distanceTo(A) > 10000):
        c= c.antipode()
    return c

def main():
    print('test_LatLontrigo2nvector() {}'.format(test_LatLontrigo2nvector()))

    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

    A =LatLontri(48.844781966005414, 2.354806246580006)
    B = LatLontri(48.845476490908986, 2.3559582742434224)
    C = LatLontri(48.844800522139515, 2.356945151087957)
    D = LatLontri(48.84415592294359, 2.3565687535257593)
    E = LatLontri(48.84395753653702, 2.355015706155173)


    p1= LatLontri(1, 1)
    q1= LatLontri(10, 1)
    p2= LatLontri(1, 2)
    q2= LatLontri(10, 2)
    doIntersect(p1,q1,p2,q2)
    
    p1= LatLontri(10, 0)
    q1= LatLontri(0, 10)
    p2= LatLontri(0, 0)
    q2= LatLontri(10, 10)
    doIntersect(p1,q1,p2,q2)
    
    p1= LatLontri(-5, -5)
    q1= LatLontri(0, 0)
    p2= LatLontri(1, 1)
    q2= LatLontri(10, 10)
    doIntersect(p1,q1,p2,q2)
    

if __name__ == '__main__':
    main()
