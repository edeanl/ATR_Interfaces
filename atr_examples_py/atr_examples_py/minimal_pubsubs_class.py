from rclpy.node import Node
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from atr_interfaces.msg import ObjectStamped
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            ObjectStamped, 'Object', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ObjectStamped()
        msg.header._frame_id = "/map"
        msg.header._stamp = self.get_clock().now().to_msg()

        msg.object_id = "Object 1"
        msg.object_idx = 0

        points = np.array([[0, 0, 0], [0, 1, 0], [1, 1.5, 0],
                           [2, 1, 0], [2.5, 0.1, 0]])

        rows, cols = points.shape

        list = ['one', 'two']

        print(f'List={list}')

        for i in range(0, rows):
            msg.polygon.points.append(Point32())
            msg.polygon.points[i].x = points[i][0]
            msg.polygon.points[i].y = points[i][1]
            msg.polygon.points[i].z = points[i][2]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Points List {msg.polygon.points}')


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            ObjectStamped,
            'Object',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            PolygonStamped, 'ObjPoly', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: ' + str(msg.header.stamp._sec) +
                               "." + str(msg.header.stamp._nanosec))
        aux_poly = PolygonStamped()
        aux_poly.header = msg.header
        aux_poly.polygon = msg.polygon
        self.publisher_.publish(aux_poly)
