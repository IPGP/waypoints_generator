import matplotlib.pyplot as plt
from math import pi, cos, atan2, asin, acos, sin, radians, degrees
import pandas as pd
import plotly.graph_objects as go
import geopy
import geopy.distance


class Waypoint:
    "Waypoint class"

    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude_aboslue_sol = None
        self.altitude_relative_drone = None
        self.hauteur_sol = None
        self.emprise_laterale = 10  # en mètres
        self.emprise_longitudinale = 50  # en mètres
        self.direction = 0

    def emprise_coordinates(self):
        """Détermine les coordonnées de l'emprise au sol à partir du point central
                   5 | 5
        x0,y0 ---------------- x1,y1
            |                 |
         10 |        ^        |
          – |        |        |
         10 |        X        |
            |                 |
            |                 |
        x2,y2 ---------------- x3,y3

        """
        delta_lat = self.emprise_laterale / 2
        delta_long = self.emprise_longitudinale / 2

        x0, y0 = latitude_longitude_plus_distance(
            self.latitude,
            self.longitude,
            -delta_lat * cos(radians(self.direction))
            + delta_long * sin(radians(self.direction)),
            delta_lat * sin(radians(self.direction))
            + delta_long * cos(radians(self.direction)),
        )

        x1, y1 = latitude_longitude_plus_distance(
            self.latitude,
            self.longitude,
            delta_lat * cos(radians(self.direction))
            + delta_long * sin(radians(self.direction)),
            -delta_lat * sin(radians(self.direction))
            + delta_long * cos(radians(self.direction)),
        )

        x2, y2 = latitude_longitude_plus_distance(
            self.latitude,
            self.longitude,
            delta_lat * cos(radians(self.direction))
            - delta_long * sin(radians(self.direction)),
            -delta_lat * sin(radians(self.direction))
            - delta_long * cos(radians(self.direction)),
        )

        x3, y3 = latitude_longitude_plus_distance(
            self.latitude,
            self.longitude,
            -delta_lat * cos(radians(self.direction))
            - delta_long * sin(radians(self.direction)),
            delta_lat * sin(radians(self.direction))
            - delta_long * cos(radians(self.direction)),
        )

        #        return x0, y0, x1, y1, x2, y2, x3, y3
        data = [['Center', self.latitude, self.longitude], [
            'X0', x0, y0], ['X1', x1, y1], ['X2', x2, y2], ['X3', x3, y3]]
        # Create the pandas DataFrame
        df = pd.DataFrame(data, columns=['Name', 'X', 'Y'])
        return df


def latitude_longitude_distance_bearing(latitude, longitude, distance, bearing):
    print(latitude, longitude, distance_x, distance_y)
    "Return latitude + distance_x and longitude + distance_y"
    new_point = geopy.distance.distance(
        kilometers=distance/1000).destination((latitude, longitude), bearing=bearing)
    return new_point.latitude, new_point.longitude


IPGP = Waypoint(2.3562098704389163, 48.84482270388685)
# IPGP = Waypoint(0, 0)
df = IPGP.emprise_coordinates()
print(df)
fig = go.Figure(data=go.Scattergeo(
    lon=df['X'],
    lat=df['Y'],
    text=df['Name'],
    mode='markers'
))

fig.update_layout(
    title='',
    geo_scope='europe',
)
fig.show()


IPGP.direction = 90
df = IPGP.emprise_coordinates
