import waypoint
from geopy import Point, distance
import geopy
import sys
from geopy.distance import geodesic


class PathPlanning:
    "PathPlanning class"

    def __init__(self, A,B, emprise_laterale, emprise_longitudinale):
        """
        A(X0, Y0) ----------------- X1, Y0
        |                        |
        |                        |
        |                        |
        |                        |
        |                        |
        |                        |
        X0, Y1 ----------------- B(X1, Y1)
        """

       #self.AOI = [(X0, Y0),  (X0, Y1), (X1, Y1), (X1, Y0)]
        self.A=A
        self.B=B
        self.AOI = [(A[0], A[1]),  (A[0], B[1]), (B[0], B[1]), (B[1], A[1])]

        self.best_orientation = None

        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres

    def create_back_foth_route(self):
        pass

    def find_best_orientation(self):
        # X1-X0 ou Y1-Y0 ?

        # Print the distance calculated in km
        #print(geodesic(self.AOI[0], self.AOI[1]).km)
        vertical_distance = geodesic(self.AOI[0], self.AOI[1]).km
        horizontal_distance = geodesic(self.AOI[0], self.AOI[2]).km

        print("Vertical "+str(vertical_distance))
        print("Horizontal "+str(horizontal_distance))

        if(vertical_distance > horizontal_distance):
            self.best_orientation = 0
        else:
            self.best_orientation = 90

        print("Best Orientation "+str(self.best_orientation))
    
    def GeneratePath(self,style):
        if style == "snail":
            self.GeneratePathSnail()
        elif style == "normal":
            self.GeneratePath()

    def GeneratePathSnail(self):
        pass
    
    def GeneratePathNormal(self):
        pass




    
def main(args):

    LAT = 48.84482270388685
    LON = 2.3562098704389163

#    X0 = 48.84643250706535
#    Y0 = 2.3527444567404943

    A = (48.84643250706535,2.3527444567404943)
    B = (48.84350234422499,2.358956452357875 )
#    X1 = 48.84350234422499
#    Y1 = 2.358956452357875

    pp = PathPlanning(A,B, 100, 50)
    pp.find_best_orientation()
    pp.GeneratePath("snail")
#    pp.GeneratePath("snail")


if __name__ == '__main__':
    main(sys.argv)
