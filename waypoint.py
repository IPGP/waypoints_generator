from math import atan, degrees, sqrt
import folium
from gpxplotter import create_folium_map
from geopy import Point, distance
import geopy


class Waypoint:
    "Waypoint class"

    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude_aboslue_sol = None
        self.altitude_relative_drone = None
        self.hauteur_sol = None
        self.emprise_laterale = 100  # en mètres
        self.emprise_longitudinale = 50  # en mètres
        self.direction = 30
        self.emprise = None
        self.name = "IPGP"

    def emprise_coordinates(self):
        """Détermine les coordonnées de l'emprise au sol à partir du point central
                   5 | 5
        x3,y3 ---------------- x0,y0
            |                 |
         10 |        ^        |
          – |        |        |
         10 |        X        |
            |                 |
            |                 |
        x2,y2 ---------------- x1,y1

        """
        delta_lat = self.emprise_laterale / 2
        delta_long = self.emprise_longitudinale / 2

        angle = degrees(atan(delta_lat/delta_long))

        dist = geopy.distance.distance(kilometers=(
            sqrt(delta_lat * delta_lat+delta_long * delta_long)/1000))

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=angle + self.direction)
        #        print("angle "+str(angle + self.direction))
        x0, y0, *_ = tmp

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180-angle + self.direction)
        #print("angle "+str(180-angle + self.direction))
        x1, y1, *_ = tmp

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180+angle + self.direction)
        #print("angle "+str(180+angle + self.direction))

        x2, y2, *_ = tmp
        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=360-angle + self.direction)

        x3, y3, *_ = tmp

        print(self.latitude, " ", self.longitude)
        print(x0, " ", y0)
        print(x1, " ", y1)
        print(x2, " ", y2)
        print(x3, " ", y3)

        self.emprise = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
        return self.emprise


LAT = 48.84482270388685
LON = 2.3562098704389163

IPGP = Waypoint(LAT, LON)
emprise_IPGP = IPGP.emprise_coordinates()
the_map = create_folium_map(zoom_min=0, max_zoom=18, zoom_start=10)

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
basemaps['Google Maps'].add_to(the_map)
basemaps['Google Satellite'].add_to(the_map)
basemaps['Google Terrain'].add_to(the_map)
basemaps['Google Satellite Hybrid'].add_to(the_map)
basemaps['Esri Satellite'].add_to(the_map)

tiles_maps = ['openstreetmap', 'Stamen Terrain']
#tiles_maps = ['openstreetmap']
#tiles_maps = []
# tiles_maps=[ 'openstreetmap',''Cartodb Positron',
# 'Stamen Terrain','Stamen Toner','Stamen Watercolor']

for tile in tiles_maps:
    folium.TileLayer(tile).add_to(the_map)
folium.Marker(
    location=[LAT, LON],
    # popup=image.filename,
    # color=color_image,
    #                icon=folium.Icon(color=color_image,icon='fas fa-camera')
    icon=folium.Icon(color='blue', icon='fa-map-pin')).add_to(the_map)

folium.Marker(location=emprise_IPGP[0], popup='X0', icon=folium.Icon(
    color='red', icon='fa-map-pin')).add_to(the_map)
folium.Marker(location=emprise_IPGP[1], popup='X1', icon=folium.Icon(
    color='green', icon='fa-map-pin')).add_to(the_map)
folium.Marker(location=emprise_IPGP[2], popup='X2', icon=folium.Icon(
    color='green', icon='fa-map-pin')).add_to(the_map)
folium.Marker(location=emprise_IPGP[3], popup='X3', icon=folium.Icon(
    color='red', icon='fa-map-pin')).add_to(the_map)

the_map.add_child(folium.vector_layers.Polygon(locations=emprise_IPGP, color='#ff7800', fill=True, fill_color='#ffff00', fill_opacity=0.2,
                                               weight=2, popup=(folium.Popup(IPGP.name))))

boundary = the_map.get_bounds()
the_map.fit_bounds(boundary, padding=(5, 5))
folium.LayerControl(sortLayers=True).add_to(the_map)


the_map.save('./carte.html')
