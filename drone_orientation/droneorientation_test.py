#!/usr/bin/python
# -*- coding:utf-8 -*-

from PIL import Image as ImagePil
import numpy as np

from classes.droneorientation import DroneOri

# This script is to test droneorientation.py
# Test data are RGE ALTI data, from IGN (https://geoservices.ign.fr/rgealti)

np_dsm = np.array(ImagePil.open('mosaic.tif'))

prof1 = DroneOri(
        name='prof1', np_dsm=np_dsm, tfw='mosaic.tfw',
        a_east=365649.5, a_north=6397299.5, b_east=366205.5, b_north=6396986.5,
        h=20, sensor_size=(23.5,15.7), img_size=(6016,3376), focal=24, ovlp=0.1
    )

prof1.dsm_profile()
prof1.drone_orientations()
prof1.draw_orientations(disp_linereg=True, disp_footp=False, disp_fov=True)

# Test unitaire fonction find_orientation
# Fichiers test : test.tif et test2.tif, qui utilisent tous les deux test.tfw
#np_dsm = np.array(ImagePil.open('test2.tif'))
#prof1 = DroneOri(np_dsm, 'test.tfw', 500, 500, 700, 500, 15, 84, 75)
#prof1.dsm_profile()
#prof1.drone_orientations()
