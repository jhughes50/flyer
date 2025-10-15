"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import folium
from flask import Flask, render_template_string, render_template
from flask_socketio import SocketIO
from threading import Thread

from flyer.utils.graph_parser import parse
from flyer.utils.converter import UTMtoLL
import utm
from scipy.spatial.transform import Rotation
import numpy as np


class MapApp:

    def __init__(self) -> None:
        dire = '/home/dcist/ws/src/symbiote-ag/symbiote_ag/viz/'
        self.app_ = Flask(__name__, template_folder=dire+'templates', static_folder=dire+'static')
        self.socketio_ = SocketIO(self.app_, cors_allowed_origins="*")
        
        self.map_ = folium.Map(location=[39.941326, -75.199492],
                               zoom_start=24,
                               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                               attr='Esri',
                               name='Esri Satellite')


        self.setup_routes()
        self.points = list()

    def setup_routes(self) -> None:
        @self.app_.route('/')
        def index():
            return render_template('index.html')

    def update_map(self, lat : float, lon : float, popup : str ="New Point") -> None:
        #folium.Marker([lat, lon], popup=popup).add_to(self.map_)
        self.points.append({"lat": lat, "lon": lon, "popup": popup})
        self.socketio_.emit('gps_update', {'points': self.points})

    def update_objects(self, objects) -> None:
        self.socketio_.emit('objects_update', {'objects': objects})

    def update_orientation(self, roll : float, pitch : float, yaw : float) -> None:
        self.socketio_.emit('orientation_update', {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        })

    def run_in_thread(self) -> None:
        thread = Thread(target=self.run)
        thread.daemon = True
        thread.start()

    def run(self) -> None:
        self.app_.run(debug=False, host="127.0.0.1", port=5000)


class VizNode(Node):

    def __init__(self) -> None:
        super(VizNode, self).__init__('symbiote_map')

        gps_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        #self.dgps_sub_ = self.create_subscription(NavSatFix, "/gps", self.drone_gps_callback, 1, callback_group=gps_group)
        self.odom_sub_ = self.create_subscription(Odometry, "/odom", self.drone_odom_callback, 1, callback_group=gps_group)
        self.grph_sub_ = self.create_subscription(String, "/graph", self.graph_callback, 1, callback_group=gps_group)
        self.plotter_ = self.create_timer(0.2, self.plot)

        self.map_ = MapApp()
        self.map_.run_in_thread()

        self.enu2ned = np.array([[0,-1,0], [1,0,0],[0,0,1]])

        self.lat, self.lon = None, None
        self.rpy = None

    def plot(self) -> None:
        #pass
        #print("updating")
        if self.lat is not None:
            self.map_.update_map(self.lat, self.lon)
            self.map_.update_orientation(self.rpy[0], self.rpy[1], self.rpy[2])

    def drone_odom_callback(self, msg : Odometry) -> None:
        #lat, lon = UTMtoLL(23, msg.pose.pose.position.y, msg.pose.pose.position.x, '18N')
        self.lat, self.lon = utm.to_latlon(msg.pose.pose.position.x, msg.pose.pose.position.y, 18, 'S')
        rot = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        nrot = self.enu2ned @ rot.as_matrix()
        self.rpy = Rotation.from_matrix(nrot).as_euler('xyz', degrees=True)
        

        #print(lat, lon)
        #self.map_.update_map(self.lat, self.lon)

    def drone_gps_callback(self, msg : NavSatFix) -> None:
        
        self.lat = msg.latitude
        self.lon = msg.longitude

        self.map_.update_map(self.lat,self.lon)

    def graph_callback(self, msg : String) -> None:
        objects = parse(msg.data)
        #print(objects)
        self.map_.update_objects(objects)
