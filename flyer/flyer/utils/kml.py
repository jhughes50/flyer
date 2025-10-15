import os
import numpy as np
from xml.etree.ElementTree import parse, ParseError

def create_drone_kml(coordinates, id_num, output_file="/home/jason/data/"):
    """
    Create a KML file with drone position, image corner coordinates, and casualty location.
    
    Parameters:
    coordinates (list): List of 6 tuples in format (lat, lon):
                       [drone_pos, top_left, bottom_right, top_right, bottom_left, casualty]
    output_file (str): Output KML filename
    
    Returns:
    str: The KML content as a string
    """
    
    # Unpack coordinates
    drone_pos, top_left, bottom_right, top_right, bottom_left, casualty = coordinates
    
    # KML template
    kml_template = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Drone Camera Localization</name>
    <description>Drone position and image corner projections on ground</description>
    
    <!-- Style for drone position -->
    <Style id="droneStyle">
      <IconStyle>
        <color>ff0000ff</color>
        <scale>1.2</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/aircraft.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <color>ff0000ff</color>
        <scale>1.0</scale>
      </LabelStyle>
    </Style>
    
    <!-- Style for image corners -->
    <Style id="cornerStyle">
      <IconStyle>
        <color>ff00ff00</color>
        <scale>0.8</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <color>ff00ff00</color>
        <scale>0.9</scale>
      </LabelStyle>
    </Style>
    
    <!-- Style for casualty -->
    <Style id="casualtyStyle">
      <IconStyle>
        <color>ff0000ff</color>
        <scale>1.5</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/caution.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <color>ff0000ff</color>
        <scale>1.2</scale>
      </LabelStyle>
    </Style>
    
    <!-- Drone Position -->
    <Placemark>
      <name>Drone Position</name>
      <description>Current drone location</description>
      <styleUrl>#droneStyle</styleUrl>
      <Point>
        <coordinates>{drone_lon},{drone_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <!-- Image Corner Projections -->
    <Placemark>
      <name>Top Left Corner</name>
      <description>Projection of top-left image corner onto ground</description>
      <styleUrl>#cornerStyle</styleUrl>
      <Point>
        <coordinates>{tl_lon},{tl_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <Placemark>
      <name>Top Right Corner</name>
      <description>Projection of top-right image corner onto ground</description>
      <styleUrl>#cornerStyle</styleUrl>
      <Point>
        <coordinates>{tr_lon},{tr_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <Placemark>
      <name>Bottom Left Corner</name>
      <description>Projection of bottom-left image corner onto ground</description>
      <styleUrl>#cornerStyle</styleUrl>
      <Point>
        <coordinates>{bl_lon},{bl_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <Placemark>
      <name>Bottom Right Corner</name>
      <description>Projection of bottom-right image corner onto ground</description>
      <styleUrl>#cornerStyle</styleUrl>
      <Point>
        <coordinates>{br_lon},{br_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <!-- Casualty Location -->
    <Placemark>
      <name>Casualty</name>
      <description>Casualty location identified by drone</description>
      <styleUrl>#casualtyStyle</styleUrl>
      <Point>
        <coordinates>{casualty_lon},{casualty_lat},0</coordinates>
      </Point>
    </Placemark>
    
    <!-- Image footprint polygon -->
    <Placemark>
      <name>Image Footprint</name>
      <description>Approximate camera field of view on ground</description>
      <Style>
        <LineStyle>
          <color>7f00ffff</color>
          <width>2</width>
        </LineStyle>
        <PolyStyle>
          <color>3f00ffff</color>
        </PolyStyle>
      </Style>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              {tl_lon},{tl_lat},0
              {tr_lon},{tr_lat},0
              {br_lon},{br_lat},0
              {bl_lon},{bl_lat},0
              {tl_lon},{tl_lat},0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
    
  </Document>
</kml>'''
    
    # Format the template with coordinates
    kml_content = kml_template.format(
        drone_lat=drone_pos[0], drone_lon=drone_pos[1],
        tl_lat=top_left[0], tl_lon=top_left[1],
        tr_lat=top_right[0], tr_lon=top_right[1],
        bl_lat=bottom_left[0], bl_lon=bottom_left[1],
        br_lat=bottom_right[0], br_lon=bottom_right[1],
        casualty_lat=casualty[0], casualty_lon=casualty[1]
    )
    
    # Write to file
    output_file += "drone_localization_%i.kml" %id_num
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(kml_content)
    
    print(f"KML file saved as: {output_file}")
    return kml_content
