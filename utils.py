#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from math import degrees, acos,  sqrt
from geopy.distance import geodesic

DEBUG = False
DEBUG = True

# To print in color
# ex  print(bg("text", 160))
# ex  print(fg("text", 160))
# https://stackoverflow.com/questions/287871/how-to-print-colored-text-to-the-terminal
# https://code.labstack.com/VfphHQ14


def foreground_color(text, color):
    return "\33[38;5;" + str(color) + "m" + text + "\33[0m"
def background_color(text, color):
    return "\33[48;5;" + str(color) + "m" + text + "\33[0m"


def background_foreground_color(text, color_bg, color_fg):
    return "\33[38;5;" + str(color_fg) + "m"+"\33[48;5;" + str(color_bg) + "m" + text + "\33[0m"



def get_angle(point_a, point_b, point_c):
    "return angle en degrees between A,B and C in degrees"
    distance_ab = geodesic(point_a, point_b).m
    distance_bc = geodesic(point_b, point_c).m
    distance_ca = geodesic(point_a, point_c).m

    return degrees(acos((distance_ab*distance_ab+distance_bc*distance_bc-distance_ca*distance_ca)/(2*distance_ab*distance_bc)))


def get_angle_latlon(point_a, point_b, point_c):
    "return angle en degrees between Latlon A,B and C in degrees"

    distance_ab = point_a.distanceTo(point_b)
    distance_bc = point_b.distanceTo(point_c)
    distance_ca = point_c.distanceTo(point_a)

    return degrees(acos((distance_ab*distance_ab+distance_bc*distance_bc-distance_ca*distance_ca)/(2*distance_ab*distance_bc)))


def distance_to_line(point_a, point_b, point_c):
    """ return the distance of a to line bc"""
    distance_ca = point_c.distanceTo(point_a)
    distance_cb = point_c.distanceTo(point_b)
    distance_ba = point_b.distanceTo(point_a)

    distance_bh = (distance_ca*distance_ca-distance_cb*distance_cb-distance_ba*distance_ba)/(2*distance_cb)
    return sqrt(distance_ba*distance_ba-distance_bh*distance_bh)


def on_segment(point_p, point_q, point_r):
    """ Given three collinear points p, q, r, the function checks if
    point q lies on line segment 'pr'
    """
    if ((point_q.lat <= max(point_p.lat, point_r.lat)) and (point_q.lat >= min(point_p.lat, point_r.lat)) and
            (point_q.lon <= max(point_p.lon, point_r.lon)) and (point_q.lon >= min(point_p.lon, point_r.lon))):
        return True
    return False

def orientation(point_p, point_q, point_r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.

    val = (float(point_q.lon - point_p.lon) * (point_r.lat - point_q.lat)) - \
        (float(point_q.lat - point_p.lat) * (point_r.lon - point_q.lon))
    if val > 0:
        # Clockwise orientation
        return 1
    if val < 0:
        # Counterclockwise orientation
        return 2
    # Collinear orientation
    # print('Collinear orientation ')
    return 0

# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
# false if Collinear


def do_intersect(point_p1, point_q1, point_p2, point_q2):
    """ Return intersection point of segments [p1 q1] and [p2 q2]
        if points are coolinear return False
        Antipode correction if needed
    """

    if point_p1.isequalTo(point_p2, eps=0.000001) or point_p1.isequalTo(point_q2, eps=0.000001)\
    or point_q1.isequalTo(point_p2, eps=0.000001) or point_q1.isequalTo(point_q2, eps=0.000001):
        print("Same points")
        return False

    # Find the 4 orientations required for
    # the general and special cases
    # print("#############################")
    # print('type(p1) {}'.format(type(p1)))
    # print('type(q1) {}'.format(type(q1)))
    # print('type(p2) {}'.format(type(p2)))
    # print('type(q2) {}'.format(type(q2)))
    point_o1 = orientation(point_p1, point_q1, point_p2)
    point_o2 = orientation(point_p1, point_q1, point_q2)
    point_o3 = orientation(point_p2, point_q2, point_p1)
    point_o4 = orientation(point_p2, point_q2, point_q1)

    # General case
    if ((point_o1 != point_o2) and (point_o3 != point_o4)):
        inter = point_p1.intersection(point_q1, point_p2, point_q2)
        # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
        # if (inter.distanceTo(p1) > 10000)and (inter.distanceTo(p1) > 10000):
        #inter= inter.antipode()
        # return inter
        if (inter.distanceTo(point_p1) > 10000) and (inter.distanceTo(point_q2) > 10000):
            inter = inter.antipode()
        return inter

    # Special Cases
    """
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and on_segment(p1, p2, q1)):
        return True
 
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and on_segment(p1, q2, q1)):
        return True
 
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and on_segment(p2, p1, q2)):
        return True
 
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and on_segment(p2, q1, q2)):
        return True
    """
    # If none of the cases
    return False, False
