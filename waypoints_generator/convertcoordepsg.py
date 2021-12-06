#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import ogr, osr

def convert(coord, epsg_in, epsg_out):
    """Convertit des coordonnées d'un système de coordonnées à un autre
    (avril 2021 - delorme@ipgp.fr)
    
    Entrées :
        - coord           : liste contenant les coordonnées [x, y]
        - epsg_in         : code EPSG du système de coordonnées en entrée
        - epsg_out        : code EPSG du système de coordonnées en sortie
    
    Sorties :
        - coordonnées converties
    """
    spat_ref_src = osr.SpatialReference()
    spat_ref_src.ImportFromEPSG(epsg_in)
    spat_ref_tgt = osr.SpatialReference()
    spat_ref_tgt.ImportFromEPSG(epsg_out)
    conversion = osr.CoordinateTransformation(spat_ref_src, spat_ref_tgt)
    point = ogr.Geometry(ogr.wkbPoint)
    point.AddPoint(coord[0], coord[1])
    point.Transform(conversion)
    x = point.GetX()
    y = point.GetY()
    return [x, y]

def make_float(x):
    try:
        x = float(x)
    except ValueError:
        return False
    return x

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=('Convert coordinates from one coordinate system to '
            'another'),
    )
    parser.add_argument('--epsg_in', help="input EPSG code (default: 4326)",
        type=int, default=4326, metavar='CODE')
    parser.add_argument('--epsg_out', help="output EPSG code (default: 4326)",
        type=int, default= 4326, metavar='CODE')
    parser.add_argument('-x', help="x input coordinate", type=float)
    parser.add_argument('-y', help="y input coordinate", type=float)
    parser.add_argument('--file', help=("a list of [x,y] input coordinates, one"
        " per line, separated by a whitespace"),
        type=str)
    args = parser.parse_args()
    
    if args.x is None and args.y is None and args.file is None or \
            args.file is not None and \
                (args.x is not None or args.y is not None) or \
            args.x is not None and args.y is not None and args.file is not None:
        sys.exit(("You must use -x and -y OR --file (and you cannot use both)."
            " (See --help)"))
    
    if args.x and args.y:
        x, y = convert((args.x, args.y), args.epsg_in, args.epsg_out)
        print(x, y)
    else:
        with open(args.file) as f:
            for i, l in enumerate(f):
                if ' ' not in l:
                    sys.exit(("Error in {} (line {}): missing separator (i.e."
                        " whitespace)").format(args.file, i+1))
                x, y = l.strip().split(' ')
                
                x = make_float(x)
                y = make_float(y)
                
                if x is False or y is False:
                    sys.exit(("Error in {} (line {}): coordinates should be "
                        "either int or float").format(args.file, i+1))
                x, y = convert((x, y), args.epsg_in, args.epsg_out)
                print(x, y)
