import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints_py')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a=0, b=0):
        self.req.a = a
        self.req.b = b
        self.future = self.client_.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    argc = len(sys.argv)
    minimal_client.get_logger().info(str(argc))

    if argc == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        a = 0
        b = 0

    minimal_client.get_logger().info('a: ' + str(a) + '---> b: ' + str(b))
    minimal_client.send_request(a, b)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
