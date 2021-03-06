#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from classes.droneorientation import DroneOri

# This script is to test droneorientation.py
# Test data are RGE ALTI data, from IGN (https://geoservices.ign.fr/rgealti)
# (Be careful: those are DTMs, not DSMs, so trees do not appear, for example.
# Also, it might need to be corrected for the difference between the geoid and
# the ellipsoid, depending on the situation.)

# Example1: fixed pitch, with the drone shooting backwards (fixed_pitch
# positive)
# The calculated z coordinates of the drone relate to the altitude at the
# takeoff point, this altitude being taken from the DSM
prof1 = DroneOri(
        name='prof1', dsm='rge_alti_1m_1.tif', tfw='rge_alti_1m_1.tfw',
        a_east=365649.5, a_north=6397299.5, b_east=366205.5, b_north=6396986.5,
        h=20, sensor_size=(23.5,15.7), img_size=(6016,3376), focal=24, ovlp=0.1,
        fixed_pitch=75, takeoff_pt=(366151.5, 6397132.5)
    )
prof1.dsm_profile()
prof1.drone_orientations()
prof1.draw_orientations(disp_linereg=True, disp_footp=True, disp_fov=True)
prof1.draw_map(shaded_dsm='rge_alti_1m_1_shaded.tif')
prof1_export_ori = prof1.export_ori()

# Example2: the pitch angle is free, i.e. it is estimated for each orientation
# A reference altitude is provided, so the z coordinates of the drone relate to
# that altitude (cannot be used in conjunction with takeoff_pt)
prof2 = DroneOri(
        name='prof2', dsm='rge_alti_1m_2.tif', tfw='rge_alti_1m_2.tfw',
        a_east=764122.5, a_north=6362635.5, b_east=764950.5, b_north=6362688.5,
        ref_alti=1000, h=80, sensor_size=(23.5,15.7), img_size=(6016,3376),
        focal=24, ovlp=0.6
   )
prof2.dsm_profile()
prof2.drone_orientations()
prof2.draw_orientations(disp_linereg=True, disp_footp=True, disp_fov=True)
prof2.draw_map(shaded_dsm='rge_alti_1m_2_shaded.tif')
prof2_export_ori = prof2.export_ori()

# Test unitaire fonction find_orientation
# Fichiers test : test.tif et test2.tif, qui utilisent tous les deux test.tfw
# np_dsm = np.array(ImagePil.open('test2.tif'))
# prof1 = DroneOri(np_dsm, 'test.tfw', 500, 500, 700, 500, 15, 84, 75)
# prof1.dsm_profile()
# prof1.drone_orientations()
