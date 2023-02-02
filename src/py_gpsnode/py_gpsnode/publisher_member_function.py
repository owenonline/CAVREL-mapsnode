# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import requests
import json
import polyline
import numpy as np
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import NavSatFix

WAYPOINT_REACHED_THRESHOLD = 0.0000000002
MAPS_PLATFORM_API_KEY = None

class maps_node(Node):

    def __init__(self, start_lat, start_long, end_lat, end_long):
        super().__init__('minimal_publisher')

        # get route
        header = {
            "X-Goog-Api-Key": MAPS_PLATFORM_API_KEY,
            "X-Goog-FieldMask": "*"
        }

        body = {
            "origin":{
                "vehicleStopover":False,
                "sideOfRoad":False,
                "location":{
                    "latLng":{
                        "latitude": start_lat,
                        "longitude": start_long
                    }
                }
            },
            "destination":{
                "vehicleStopover":False,
                "sideOfRoad":False,
                "location":{
                    "latLng":{
                        "latitude": end_lat,
                        "longitude": end_long
                    }
                }
            },
            "travelMode":"walk",
            "routingPreference":None,
            "polylineQuality":"high_quality",
            "computeAlternativeRoutes":False,
            "routeModifiers":{
                "avoidTolls":False,
                "avoidHighways":False,
                "avoidFerries":False,
                "avoidIndoor":False
            }
        }
        resp = requests.post("https://routes.googleapis.com/directions/v2:computeRoutes", json=body, headers=header)
        route = json.loads(resp.text)
        print(route)
        self.lower_left = (route['routes'][0]['viewport']['low']['latitude'], route['routes'][0]['viewport']['low']['longitude'])
        self.upper_right = (route['routes'][0]['viewport']['high']['latitude'], route['routes'][0]['viewport']['high']['longitude'])

        # store coordinate list
        self.coords = []
        for leg in route['routes'][0]['legs'][0]['steps']:
            line = polyline.decode(leg['polyline']['encodedPolyline'])
            for coord in line:
                coord = (coord[0] - self.lower_left[0], coord[1] - self.lower_left[1])
                self.coords.append(coord)

        # create subscriber for gps topic
        self.current_xy = []
        self.subscription = self.create_subscription(NavSatFix, 'current_pos', self.sub_callback, 10)

        # create publisher for maps topic
        self.publisher_ = self.create_publisher(PoseStamped, 'map_waypoint', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.i = 0

    def pub_callback(self):
        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y = self.get_closest()
        msg.pose.position.z = 0.0
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = 0.0, 0.0, 0.0, 0.0
        self.publisher_.publish(msg)
        self.i += 1

    def sub_callback(self, msg: NavSatFix):
        self.current_xy = (msg.latitude - self.lower_left[0], msg.longitude - self.lower_left[1])

    def get_closest(self):
        # route complete
        if len(self.coords) == 0:
            return (-1, -1)

        node = self.current_xy

        nodes = np.asarray(self.coords)
        dist_2 = np.sum((nodes - node)**2, axis=1)
        closest = np.argmin(dist_2)
        closest_node = nodes[closest]
        self.get_logger().info('current location: {} {} closest node: {} {}'.format(node[0], node[1], closest_node[0], closest_node[1]))
        if dist_2[closest] < WAYPOINT_REACHED_THRESHOLD:
            self.coords.pop(closest)
        return closest_node 


def main(args=None):
    rclpy.init(args=args)
    mn = maps_node(28.6027369, -81.19816929999999, 28.602240899999998, -81.1967394)

    rclpy.spin(mn)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
