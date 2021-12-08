#!/usr/bin/python
# -*- coding:utf-8 -*-
import folium
from geopy import point
from gpxplotter import create_folium_map
from numpy.lib.function_base import append
from waypoint import WayPoint
from utils import getAnglelatlon
from pygeodesy.sphericalTrigonometry import LatLon 
from collections import deque

# https://fontawesome.com/v4.7/icons/



class WaypointMap:
    "WaypointMap class to plot waypoints of a mission in a nice map"

    def __init__(self,startpoint=None):
        self.start_icon_path =r"waypoints_generator/icons/star_yellow.png"
        self.WP_icon_path_black_path=r"waypoints_generator/icons/circle_black.png"
        self.WP_icon_path_red_path=r"waypoints_generator/icons/circle_red.png"
        self.startpoint=startpoint
        self.waypoint_nb = 0
        self.the_map = create_folium_map(
            zoom_min=0, max_zoom=50, zoom_start=50)

        # Add custom base maps to folium
        basemaps = {
            'Google Maps': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Maps',
                overlay=False,
                control=True
            ),
            'Google Satellite': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Satellite',
                overlay=True,
                control=True
            ),
            'Google Terrain': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=p&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Terrain',
                overlay=True,
                control=True
            ),
            'Google Satellite Hybrid': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Satellite',
                overlay=True,
                control=True
            ),
            'Esri Satellite': folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='Esri Satellite',
                overlay=True,
                control=True
            )
        }

        # Add custom basemaps
        basemaps['Google Maps'].add_to(self.the_map)
        basemaps['Google Satellite'].add_to(self.the_map)
       # basemaps['Google Terrain'].add_to(self.the_map)
        basemaps['Google Satellite Hybrid'].add_to(self.the_map)
       # basemaps['Esri Satellite'].add_to(self.the_map)

        tiles_maps = ['openstreetmap', 'Stamen Terrain']
        # tiles_maps = ['openstreetmap']
        # tiles_maps = []
        # tiles_maps=[ 'openstreetmap',''Cartodb Positron',
        # 'Stamen Terrain','Stamen Toner','Stamen Watercolor']

        for tile in tiles_maps:
            folium.TileLayer(tile).add_to(self.the_map)

    def add_waypoint(self, waypoint, footprint_markers=False, direction=False, footprint=False, color='blue', popup_text=None):
        # Le point central du waypoint avec une fleche pour sa direction
        self.waypoint_nb += 1

        if direction:
            if waypoint.orientation < 0:
                orientation = waypoint.orientation+360
            else:
                orientation = waypoint.orientation

            # marqueur central avec fleche
            folium.Marker(location=waypoint.location, popup=waypoint.text, icon=folium.Icon(
                color='blue', angle=int(orientation), icon='arrow-up', prefix='fa')).add_to(self.the_map)

        if footprint:
            self.the_map.add_child(folium.vector_layers.Polygon(locations=[waypoint.X0, waypoint.X1, waypoint.X2, waypoint.X3],
                                                                color='blue', fill=True,
                                                                fill_color='blue', fill_opacity=0.3, weight=2, popup=""))

        icon_black = folium.features.CustomIcon(
            icon_image=self.WP_icon_path_black_path, icon_size=(25, 25))

        folium.Marker(location=waypoint.location, tooltip=str(waypoint.text)+ str( 
            self.waypoint_nb-1)+"<br>"+str(waypoint.location[0])+"<br>"+str(waypoint.location[1]), icon=icon_black).add_to(self.the_map)

        # Les markers de l'emprise
        if footprint_markers:

            folium.Marker(location=waypoint.X0, tooltip='X0'+"<br>"+str(waypoint.X0), icon=folium.Icon(
                color='red', icon='fa-map-pin')).add_to(self.the_map)

            folium.Marker(location=waypoint.X1,  tooltip='X1'+"<br>"+str(waypoint.X1),icon=folium.Icon(
                color='green', icon='fa-map-pin')).add_to(self.the_map)

            folium.Marker(location=waypoint.X2,tooltip='X2'+"<br>"+str(waypoint.X2), icon=folium.Icon(
                color='green', icon='fa-map-pin')).add_to(self.the_map)

            folium.Marker(location=waypoint.X3, tooltip='X3'+"<br>"+str(waypoint.X3),icon=folium.Icon(
                color='red', icon='fa-map-pin')).add_to(self.the_map)

    def add_extra(self, waypoint, text=None):
        """ add extra points to the map. for debug purpose"""
        
        icon_red = folium.features.CustomIcon(icon_image=self.WP_icon_path_red_path, icon_size=(15, 15))

        folium.Marker(location=[waypoint.lat,waypoint.lon], 
        tooltip=text+"<br>"+str(waypoint.lat)+"<br>"+str(waypoint.lon), icon=icon_red).add_to(self.the_map)

       
    def add_polygon(self, points, color, fill_color, fill_opacity, weight, popup, fill=True):
        """Plot
         Plot shape of the mapping area with markers
         """
        locs=[]
        for point in points:
          #  print('point {} nb {}'.format( point, nb))
            locs.append([point.lat,point.lon])
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locs, color=color, fill=True,
                                                            fill_color=fill_color, fill_opacity=fill_opacity, weight=2, popup=popup))
        nb = 0
        for point in points:
          #  print('point {} nb {}'.format( point, nb))

            folium.Marker(location=[point.lat,point.lon], tooltip=str(nb)+"<br>"+str(point.lat)+"<br>"+str(point.lon),
                          icon=folium.Icon(color='blue', icon="circle", prefix='fa')).add_to(self.the_map)
            nb += 1

    def add_colored_waypoint_path(self, waypoints_list):
        """Draw waypoints path on the map"""

        # If we have only one waypoint exit 
        if len(waypoints_list) <= 1:
            return

        # FF0000 Red
        # 00FF00 Green

        if self.startpoint:
            self.waypoint_nb +=1
            list_waypoints = [[self.startpoint.lat,self.startpoint.lon]]
            icon_star = folium.features.CustomIcon(icon_image=self.start_icon_path, icon_size=(40, 40))
            folium.Marker(location=(self.startpoint.lat,self.startpoint.lon), tooltip='Start'+"<br>"+str(self.startpoint.lat)+"<br>"+str(self.startpoint.lon), icon=icon_star).add_to(self.the_map)
        else:
            list_waypoints = []
        for waypoint in waypoints_list:
            list_waypoints.append(waypoint.location)
            #print(waypoint.location)

        color_list = []
        for i in range(self.waypoint_nb):
            color_list.append(0xFF00+i*(0xFF0000-0xFF00)/(self.waypoint_nb))

        
        line = folium.features.ColorLine(list_waypoints,  colors=color_list, nb_steps=12, weight=10, opacity=1,name='Colored Waypoint Path')
        self.the_map.add_child(line)


    def add_area_of_interest(self, locations):
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locations, color='#348feb', fill=True,
                                                            fill_color='#34e5eb', fill_opacity=0.3, weight=2, popup=""))

    def export_to_file(self, filename):
        """Export waypoint map to file"""
        if filename:filename+='.html'
        #print('{} waypoint exportés'.format(self.waypoint_nb))

        self.the_map.fit_bounds(self.the_map.get_bounds(), padding=(5, 5))
        folium.LayerControl(sortLayers=True).add_to(self.the_map)
        self.the_map.save(filename)

    def add_path(self, pathplanning):
        pass


def main():

    a = LatLon(48.844781966005414, 2.354806246580006)
    b = LatLon(48.845476490908986, 2.3559582742434224)
    c = LatLon(48.844800522139515, 2.356945151087957)
    d = LatLon(48.84415592294359, 2.3565687535257593)
    e = LatLon(48.84395753653702, 2.355015706155173)
    f = LatLon(48.844565798460536, 2.3552507666007094)

    zone = deque([a, b, c, d,e])
    

    emprise_laterale = 50
    emprise_longitudinale = 25

    the_map = WaypointMap()

    the_map.add_polygon(points=zone, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    lat = 48.84482270388685
    lon = 2.3562098704389163

    orientation = getAnglelatlon(a, b,c)
    loc_IPGP = LatLon(lat,lon)
    IPGP = WayPoint(loc_IPGP, orientation, emprise_laterale,
                    emprise_longitudinale)

    the_map.add_waypoint(IPGP)
    the_map.export_to_file('test.html')


if __name__ == '__main__':
    main()
