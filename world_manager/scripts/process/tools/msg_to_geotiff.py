#!/usr/bin/env python3.5
# from https://github.com/locusrobotics/qgis_ros/blob/cd3bf3ef2e50d5ae0e3a681f81302f90c8e19768/scripts/msg_to_geotiff.py
import argparse
from pathlib import Path
import numpy
import gdal
import osr
import json

import os
script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
#Get relative file path
#abs_file_path = os.path.join(script_dir, CONFIG_PATH + "/"+CONFIG_FILE_NAME)

class msg_to_geotiff():
    DEFAULT_GTiff_FILE_PATH="../../tmp"
    DEFAULT_GTiff_FILE_NAME="map.gtiff"

    def __init__(self):
        self.PROJ4_SIMPLE = '+proj=tmerc \
                        +lat_0=0 \
                        +lon_0=0 \
                        +k=1 \
                        +x_0=0 \
                        +y_0=0 \
                        +ellps=WGS84 \
                        +towgs84=0,0,0,0,0,0,0 \
                        +units=m \
                        +no_defs'

    def process_file(self, input_file, output_file):
        with input_file.open('r') as f:
            data = json.loads(json.load(f))
        self._convert(data,output_file)

    def _convert(self, json_obj,target_file):
        data = json_obj
        rows = data['info']['height']
        cols = data['info']['width']
        resolution = data['info']['resolution']
        bands = 1
        originX = data['info']['origin']['position']['x']
        originY = data['info']['origin']['position']['y']
    
        npData = numpy.reshape(data['data'], (rows, cols))
    
        npData[-1]=50
        npData[0]=0
        npData[100]=100
        driver = gdal.GetDriverByName('GTiff')
        outRaster = driver.Create(str(target_file), cols, rows, bands, gdal.GDT_Byte)
        outRaster.SetGeoTransform((originX, resolution, 0, originY, 0, resolution))  # TODO Explain magic numbers.
        outBand = outRaster.GetRasterBand(1)
        outBand.WriteArray(npData)
        outBand.ComputeRasterMinMax(True)
        outRasterSRS = osr.SpatialReference()
        outRasterSRS.ImportFromProj4(self.PROJ4_SIMPLE)
        outRaster.SetProjection(outRasterSRS.ExportToWkt())
        outBand.FlushCache()
        outRaster = None  # Free up underlying C++ memory.

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Get message from an OccupancyGrid topic and save it as a geotiff.')
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)

    args = parser.parse_args()

    with args.input.open('r') as f:
        data = json.loads(json.load(f))
     
    msg2Geotiff = msg_to_geotiff()
    msg2Geotiff.process_file(args.input,args.output)



