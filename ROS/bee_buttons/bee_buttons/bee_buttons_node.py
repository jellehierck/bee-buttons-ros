import rclpy
from rclpy.node import Node

import rclpy.parameter
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class BeeButtonsNode(Node):
    def __init__(self) -> None:
        super().__init__("bee_buttons")
        self.publisher_ = self.create_publisher(String, "topic", 10)

        self.declare_parameter(
            "update_rate",
            100.0,
            ParameterDescriptor(
                description="Rate at which the button states are polled",
                read_only=True,  # Rate cannot be changed after the node is initialized
                floating_point_range=[
                    FloatingPointRange(
                        from_value=0.001,  # Allow only values >0
                        to_value=1000.0,
                    ),
                ],
            ),
        )

        update_rate_hz = (
            self.get_parameter("update_rate").get_parameter_value().double_value
        )

        self.timer = self.create_timer(1 / update_rate_hz, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bee_buttons_node = BeeButtonsNode()

    rclpy.spin(bee_buttons_node)

    bee_buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
