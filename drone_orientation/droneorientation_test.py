#!/usr/bin/python
# -*- coding:utf-8 -*-

from PIL import Image as ImagePil
import numpy as np

from classes.droneorientation import DroneOri

# This script is to test droneorientation.py
# Test data are RGE ALTI data, from IGN (https://geoservices.ign.fr/rgealti)

np_dsm = np.array(ImagePil.open('mosaic.tif'))

prof1 = DroneOri('prof1', np_dsm, 'mosaic.tfw', 365649.5, 6397299.5, 366205.5,
    6396986.5, 20, 84, 0.55)

prof1.dsm_profile()
prof1.drone_orientations()
prof1.draw_orientations(disp_linereg=True, disp_footp=False, disp_fov=True)

# Test unitaire fonction find_orientation
# Fichiers test : test.tif et test2.tif, qui utilisent tous les deux test.tfw
#np_dsm = np.array(ImagePil.open('test2.tif'))
#prof1 = DroneOri(np_dsm, 'test.tfw', 500, 500, 700, 500, 15, 84, 75)
#prof1.dsm_profile()
#prof1.drone_orientations()
