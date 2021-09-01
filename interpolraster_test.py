#!/usr/bin/python
# -*- coding:utf-8 -*-

from PIL import Image as ImagePil
import numpy as np

from classes.interpolraster import Interpolate

np_raster = np.array(ImagePil.open('dsm_post.tif'))
interpol_raster = Interpolate(np_raster)
print(interpol_raster.interpolate(4,4))
