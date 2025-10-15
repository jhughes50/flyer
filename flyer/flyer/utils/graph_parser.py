"""
    Jason Hughes
    March 2025

    script to parse the constructed graph
"""
import utm
import json
from typing import Dict
from flyer.utils.converter import UTMtoLL

OBJECT_COLOR = '#28B463'
ROAD_COLOR = '#2980B9'

class Graph:

    def __init__(self) -> None:
        pass

    @classmethod
    def parse(cls, graph : str):
        d = json.loads(graph)
        objects = list()
        for obj in d['objects']:
            lat, lon = utm.to_latlon(obj['coords'][0], obj['coords'][1], '18S')
            label = obj['name']
            connections = list()
            for conn in d['object_connections']:
                if label in conn:
                    connections.append(conn[-1])
            objects.append({"latitude": lat, "longitude": lon, "label": label, "connections": connections})

        for reg in d['regions']:
            lat, lon = UTMtoLL(reg['coords'][0], reg['coords'][1], '18S')
            label = reg['name']
            connections = list()
            for conn in d['region_connections']:
                if label in conn:
                    connections.append(conn[-1])
            objects.append({"latitude": lat, "longitude": lon, "label": label, "connections": connections})

def parse(graph : str) -> Dict:

    d = json.loads(graph)
    oe = d['origin'][0]
    on = d['origin'][1]
    objects = list()
    for obj in d['objects']:
        lat, lon = utm.to_latlon(obj['coords'][0]+oe, obj['coords'][1]+on, 18, 'S')
        label = obj['name']
        connections = list()
        for conn in d['object_connections']:
            if label in conn:
                connections.append(conn[-1])
        objects.append({"lat": lat, "lon": lon, "label": label, "connections": connections,"color": OBJECT_COLOR})

    for reg in d['regions']:
        lat, lon = utm.to_latlon(reg['coords'][0]+oe, reg['coords'][1]+on, 18, 'S')
        label = reg['name']
        connections = list()
        for conn in d['region_connections']:
            if label in conn:
                connections.append(conn[-1])
        objects.append({"lat": lat, "lon": lon, "label": label, "connections": connections, "color": ROAD_COLOR})

    return objects
