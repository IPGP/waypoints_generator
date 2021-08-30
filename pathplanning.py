import waypoint
from geopy import Point, distance
import geopy
import sys
from geopy.distance import geodesic
import pyproj
from math import degrees, acos


class PathPlanning:
    "PathPlanning class"

    def __init__(self, A,B,C,D ,emprise_laterale, emprise_longitudinale):
        """
        A(Xa, Ya) ----------------- B(Xb, Yb)
        |                        |
        |                        |
        |                        |
        |                        |
        |                        |
        |                        |
        D(Xd, Yd) ----------------- C(Xc, Yc)
        """
        #c^2=a^2+b^2-2*ab*cos(C)

       #self.AOI = [(X0, Y0),  (X0, Y1), (X1, Y1), (X1, Y0)]
        self.A=A
        self.B=B
        self.C=C
        self.D=D

        self.best_orientation = None

        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres

       # vertical_distance = geodesic(self.AOI[0], self.AOI[1]).km
       # horizontal_distance = geodesic(self.AOI[0], self.AOI[2]).km

    def GeneratePath(self,style):
        if style == "snail":
            self.GeneratePathSnail()
        elif style == "normal":
            self.GeneratePath()

    def GeneratePathSnail(self):
        pass
    
    def GeneratePathNormal(self):
        pass


def getAngle(A,B,C):
    "return angle between A,B and C in degrees"
    AB=geodesic(A,B).m
    BC=geodesic(B,C).m
    CA=geodesic(A,C).m
    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))
        


    
def main(args):

    LAT = 48.84482270388685
    LON = 2.3562098704389163

#    X0 = 48.84643250706535
#    Y0 = 2.3527444567404943

    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84395753653702, 2.355015706155173)
#    X1 = 48.84350234422499


    # Calcul des distances
    AB=geodesic(A,B).m
    BC=geodesic(B,C).m
    CD=geodesic(C,D).m
    DA=geodesic(D,A).m
    BD=geodesic(B,D).m
    AC=geodesic(A,C).m

    # Calcul des angles
    angle_ABC=getAngle(A,B,C)
    angle_BCD=getAngle(B,C,D)
    angle_CDA=getAngle(C,D,A)
    angle_DAB=getAngle(D,A,B)

    pp = PathPlanning(A,B, 100, 50)
    pp.find_best_orientation()
    pp.GeneratePath("snail")
#    pp.GeneratePath("snail")


if __name__ == '__main__':
    main(sys.argv)
