import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


class MyPythonNode(Node):
    def __init__(self):
        super().__init__("dummy_node")
        self.get_logger().info("This node just says 'Hello'")

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('my_int', None),
                ('my_string', None)
            ])

        # self.declare_parameter('my_string', None)

    def printParams(self):
        self.get_logger().info("My parameters: " +
                               self.get_parameter('my_string').get_parameter_value().string_value + ", " + str(self.get_parameter('my_int').get_parameter_value().integer_value))

    def timer_callback(self):
        my_param_s = self.get_parameter(
            'my_string').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param_s)


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    node.printParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
