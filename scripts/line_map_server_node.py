#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cclp.msg import Line
from cclp.msg import LineArray
import threading

class LineMapServerNode(Node):
    def __init__(self):
        super().__init__('line_map_server')
        self.publisher = self.create_publisher(LineArray, '/line_map', 10)
        self.get_logger().info("line_map_server_node has been started")
        self.thread = threading.Thread(target=self.input_map_pub).start()

    map = [
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

    def input_map_pub(self):
        while True:
            input("Press Enter to publish line_map")
            line_map = LineArray()
            for line in self.map:
                l = Line()
                l.p1.x = line[0][0]
                l.p1.y = line[0][1]
                l.p2.x = line[1][0]
                l.p2.y = line[1][1]
                line_map.lines.append(l)
            self.publisher.publish(line_map)
            self.get_logger().info("line_map_server_node has published line_map")

def main(args=None):
    rclpy.init(args=args)
    line_map_server_node = LineMapServerNode()
    rclpy.spin(line_map_server_node)
    line_map_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()