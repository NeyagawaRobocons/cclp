#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from cclp.msg import Line
from cclp.msg import LineArray
from cclp.msg import MapRequest
import threading

class LineMapServerNode(Node):
    def __init__(self):
        super().__init__('line_map_server')
        self.map_topic = self.declare_parameter('map_topic', 'line_map')
        self.initial_pose_topic = self.declare_parameter('initial_pose_topic', 'initial_pose')
        self.map_request_topic = self.declare_parameter('map_request_topic', 'map_request')
        self.map_publisher = self.create_publisher(LineArray, self.map_topic.value, 10)
        self.initial_pose_publisher = self.create_publisher(Pose, self.initial_pose_topic.value, 10)
        self.subscription = self.create_subscription(MapRequest, self.map_request_topic.value, self.map_request_callback, 10)
        self.get_logger().info("line_map_server_node has been started")

    left_map_origin_left = [
        [[0.0, 0.0], [3.424, 0.0]],
        [[3.424, 0.0], [3.424, 6.924]],
        [[3.424, 6.924], [0.0, 6.924]],
        [[0.0, 6.924], [0.0, 0.0]],
        [[1.019, 0.0], [1.019, 1.0]],
        [[0.8, 2.143], [3.424, 2.143]],
        [[0.0, 3.381], [1.286, 3.381]],
        [[0.8, 4.619], [2.105, 4.619]],
        [[2.105, 2.143], [2.105, 5.924]],
        [[1.019, 5.924], [1.019, 6.924]],
    ]
    y_delta = 6.924 /2
    x_delta = 3.424+0.038/2
    
    left_map = [
        [[ -3.443,  -3.462], [ -0.019,  -3.462]],
        [[ -0.019,  -3.462], [ -0.019,   3.462]],
        [[ -0.019,   3.462], [ -3.443,   3.462]],
        [[ -3.443,   3.462], [ -3.443,  -3.462]],
        [[ -2.424,  -3.462], [ -2.424,  -2.462]],
        [[ -2.643,  -1.319], [ -0.019,  -1.319]],
        [[ -3.443,  -0.081], [ -2.157,  -0.081]],
        [[ -2.643,   1.157], [ -1.338,   1.157]],
        [[ -1.338,  -1.319], [ -1.338,   2.462]],
        [[ -2.424,   2.462], [ -2.424,   3.462]],
    ]
    right_map = [
        [[  3.443,  -3.462], [  0.019,  -3.462]],
        [[  0.019,  -3.462], [  0.019,   3.462]],
        [[  0.019,   3.462], [  3.443,   3.462]],
        [[  3.443,   3.462], [  3.443,  -3.462]],
        [[  2.424,  -3.462], [  2.424,  -2.462]],
        [[  2.643,  -1.319], [  0.019,  -1.319]],
        [[  3.443,  -0.081], [  2.157,  -0.081]],
        [[  2.643,   1.157], [  1.338,   1.157]],
        [[  1.338,  -1.319], [  1.338,   2.462]],
        [[  2.424,   2.462], [  2.424,   3.462]],
    ]

    def map_request_callback(self, msg):
        if msg.map == MapRequest.MAP_LEFT:
            line_map = LineArray()
            for line in self.left_map:
                l = Line()
                l.p1.x = line[0][0]
                l.p1.y = line[0][1]
                l.p2.x = line[1][0]
                l.p2.y = line[1][1]
                line_map.lines.append(l)
            self.map_publisher.publish(line_map)
            self.get_logger().info("published MAP_LEFT (size: %d)" % len(line_map.lines))
        elif msg.map == MapRequest.MAP_RIGHT:
            line_map = LineArray()
            for line in self.left_map:
                l = Line()
                l.p1.x = -line[0][0]
                l.p1.y = line[0][1]
                l.p2.x = -line[1][0]
                l.p2.y = line[1][1]
                line_map.lines.append(l)
            self.map_publisher.publish(line_map)
            self.get_logger().info("published MAP_RIGHT (size: %d)" % len(line_map.lines))
        else:
            self.get_logger().error("Unknown map request")

def main(args=None):
    rclpy.init(args=args)
    line_map_server_node = LineMapServerNode()
    rclpy.spin(line_map_server_node)
    line_map_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()