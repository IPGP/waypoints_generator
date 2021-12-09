from string import Template
import sys
from pyproj import Transformer
from pygeodesy.sphericalNvector  import intersection, LatLon as LatLonS


def dict2djikml (dic,output_filename,reverse_coordonates_transformer,onfinish='hover',speed = 5,turnmode = 'Auto'):
  extra_points=[]

  if onfinish == 'hover':
      ON_FINISH = "Hover"
  elif onfinish == 'gohome':
      ON_FINISH = "GoHome"
  else:
      sys.exit('onfinish shoud be hover or gohome')

  XML_string = """<?xml version="1.0" encoding="UTF-8"?>

  <kml xmlns="http://www.opengis.net/kml/2.2">
    <Document xmlns="">
      <name>chambon_small</name>
      <open>1</open>
      <ExtendedData xmlns:mis="www.dji.com">
        <mis:type>Waypoint</mis:type>
        <mis:stationType>0</mis:stationType>
      </ExtendedData>
      <Style id="waylineGreenPoly">
        <LineStyle>
          <color>FF0AEE8B</color>
          <width>6</width>
        </LineStyle>
      </Style>
      <Style id="waypointStyle">
        <IconStyle>
          <Icon>
            <href>https://cdnen.dji-flighthub.com/static/app/images/point.png</href>
          </Icon>
        </IconStyle>
      </Style>
      <Folder>
        <name>Waypoints</name>
        <description>Waypoints in the Mission.</description>\n"""
  #name = None
  #lon = None
  #lat = None
  #height = None
  #heading = None
  ##gimbal = None
  all_coordinates = ""
  waypoint_number = 1

  waypoint_start = Template("""      <Placemark>
          <name>Waypoint$waypoint_number</name>
          <visibility>1</visibility>
          <description>Waypoint</description>
          <styleUrl>#waypointStyle</styleUrl>
          <ExtendedData xmlns:mis="www.dji.com">
            <mis:useWaylineAltitude>false</mis:useWaylineAltitude>
            <mis:heading>$heading</mis:heading>
            <mis:turnMode>$turnmode</mis:turnMode>
            <mis:gimbalPitch>$gimbal</mis:gimbalPitch>
            <mis:useWaylineSpeed>false</mis:useWaylineSpeed>
            <mis:speed>$speed</mis:speed>
            <mis:useWaylineHeadingMode>true</mis:useWaylineHeadingMode>
            <mis:useWaylinePointType>true</mis:useWaylinePointType>
            <mis:pointType>LineStop</mis:pointType>
            <mis:cornerRadius>0.2</mis:cornerRadius>""")


  waypoint_end = Template("""
          </ExtendedData>
          <Point>
            <altitudeMode>relativeToGround</altitudeMode>
            <coordinates>$lon,$lat,$height</coordinates>
          </Point>
        </Placemark>""")
  hover_template = Template("""
            <mis:actions param="$length" accuracy="0" cameraIndex="0" payloadType="0" payloadIndex="0">Hovering</mis:actions>""")
  shoot_template = Template("""
            <mis:actions param="0" accuracy="0" cameraIndex="0" payloadType="0" payloadIndex="0">ShootPhoto</mis:actions>""")

  gimbal_template = Template("""
            <mis:actions param="$gimbal_angle" accuracy="1" cameraIndex="0" payloadType="0" payloadIndex="0">GimbalPitch</mis:actions>""")
  aircraftyaw_template = Template("""
            <mis:actions param="$aircraftyaw" accuracy="0" cameraIndex="0" payloadType="0" payloadIndex="0">AircraftYaw</mis:actions>""")
  record_template = Template("""
            <mis:actions param="0" accuracy="0" cameraIndex="0" payloadType="0" payloadIndex="0">StartRecording</mis:actions>""")
  stoprecord_template = Template("""
            <mis:actions param="0" accuracy="0" cameraIndex="0" payloadType="0" payloadIndex="0">StopRecording</mis:actions>""")


  all_coordinates_template = Template("$lon,$lat,$height")
  xml_end = Template("""    </Folder>
      <Placemark>
        <name>Wayline</name>
        <description>Wayline</description>
        <visibility>1</visibility>
        <ExtendedData xmlns:mis="www.dji.com">
          <mis:altitude>50.0</mis:altitude>
          <mis:autoFlightSpeed>5.0</mis:autoFlightSpeed>
          <mis:actionOnFinish>$ON_FINISH</mis:actionOnFinish>
          <mis:headingMode>UsePointSetting</mis:headingMode>
          <mis:gimbalPitchMode>UsePointSetting</mis:gimbalPitchMode>
          <mis:powerSaveMode>false</mis:powerSaveMode>
          <mis:waypointType>LineStop</mis:waypointType>
          <mis:droneInfo>
            <mis:droneType>COMMON</mis:droneType>
            <mis:advanceSettings>false</mis:advanceSettings>
            <mis:droneCameras/>
            <mis:droneHeight>
              <mis:useAbsolute>false</mis:useAbsolute>
              <mis:hasTakeoffHeight>false</mis:hasTakeoffHeight>
              <mis:takeoffHeight>0.0</mis:takeoffHeight>
            </mis:droneHeight>
          </mis:droneInfo>
        </ExtendedData>
        <styleUrl>#waylineGreenPoly</styleUrl>
        <LineString>
          <tessellate>1</tessellate>
          <altitudeMode>relativeToGround</altitudeMode>
          <coordinates>$all_coordinates</coordinates>
        </LineString>
      </Placemark>
    </Document>
  </kml>""")

  if (speed > 15) or (speed <= 0):
      sys.exit('speed should be >0 or <=15 m/s ')
  speed=str(speed)
  if '.' not in speed:
      speed = speed+'.0'
  waypoint_nb=0
  print(dic)
  for waypoint in dic :
      #print(waypoint)
      # 'path_name': 'prof1',
      # 'path_az': 200.65033592212384,
      # 'drone_e': 764492.0107411736,
      # 'drone_n': 6362186.929337244,
      # 'drone_z': 1262.9225386835942,
      # 'drone_pitch': -75.0,
      # 'drone_fov_lat': 52.17119955656284,
      # 'drone_fov_lon': 36.22411610965541,
      # 'drone_az': False}


      name = 'WP_'+str(waypoint_nb)
      lat , lon = reverse_coordonates_transformer.transform(waypoint['drone_e'],waypoint['drone_n'])
      tmp_extra_point= LatLonS(lat, lon)
      tmp_extra_point.text=name+" "+str(waypoint['drone_z'])
      extra_points.append(tmp_extra_point)
      height = waypoint['drone_z']
      if waypoint['drone_az']:
        heading = 0
      else:
        heading = 180
      
      gimbal = waypoint['drone_pitch']
      
      actions_sequence = ON_FINISH


      gimbal=str(gimbal)
      if '.' not in gimbal:
          gimbal = gimbal+'.0'

      print(F'{name}\t{lat}\t{lon}\t{heading}\t{gimbal}')

      XML_string += waypoint_start.substitute(
          turnmode=turnmode, waypoint_number=waypoint_nb, speed=speed, heading=heading, gimbal=gimbal)
      action_list = 'SHOOT'
      XML_string += "\n" + \
          waypoint_end.substitute(lon=lon, lat=lat, height=height,)+"\n"

      all_coordinates += all_coordinates_template.substitute(
          lon=lon, lat=lat, height=height)+" "
      waypoint_nb += 1
    # remove last space from coordinates string
  all_coordinates = all_coordinates[:-1]
  XML_string += xml_end.substitute(all_coordinates=all_coordinates,
                                  ON_FINISH=ON_FINISH)

  with open(output_filename, 'w') as output_file:
      output_file.write(XML_string)
  return extra_points
