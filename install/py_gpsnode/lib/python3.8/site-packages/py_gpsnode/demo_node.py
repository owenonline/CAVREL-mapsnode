import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class demo(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.lat = 28.6027369
        self.long = -81.19816929999999
        self.latchange = -0.000010379
        self.longchange = .000015429
        self.publisher_ = self.create_publisher(NavSatFix, 'current_pos', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NavSatFix()
        if self.i < 100:
            self.lat += self.latchange
            self.long += self.longchange
        msg.latitude = self.lat
        msg.longitude = self.long
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: {} {}'.format(self.lat, self.long))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    d = demo()

    rclpy.spin(d)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()