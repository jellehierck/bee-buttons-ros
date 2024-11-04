import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

import rclpy.qos
import std_msgs.msg

import bee_buttons_interfaces.msg

from .periodic_serial_line_reader import PeriodicSerialLineReader


class BeeButtonsNode(Node):
    """
    ROS node for handling Bee Buttons.

    This node will periodically read the Bee Buttons dongle and publish the information to ROS topics. It can also be
    used to send commands to connected buttons.

    Also see the attached README for additional information.

    Parameters:
    - `update_rate` (double): Rate (Hz) at which the buttons are polled. Defaults to `100.0`.
    - `battery_info_on_startup` (bool): Whether to send request a battery info message from all connected buttons
      on startup. Defaults to true.

    Topics:
    - `/button_press`: Publishes a message whenever a button press is detected on one of the connected buttons.
    - `/button_battery_info`: Publishes battery information messages as a response to a battery information request.

    Services (WIP):
    - TODO: ROS services will be used to send commands to connected buttons, such as changing colors or playing sounds.

    See also: https://github.com/utwente-interaction-lab/Bee-Buttons
    """

    DEFAULT_BAUD_RATE = 115200
    DONGLE_DEVICE_NAME = "USB JTAG/serial debug unit"

    COMMAND_SEPARATOR = ":"
    COMMAND_END = "`"

    def __init__(self) -> None:
        """
        Initialize a new Bee Buttons Node.

        :raises `serial.SerialException`: If the connection to the dongle failed.
        """
        super().__init__("bee_buttons")

        # Declare all parameters
        update_rate_param = self.declare_parameter(
            "update_rate",
            100.0,
            ParameterDescriptor(
                description="Rate (Hz) at which the buttons are polled",
                read_only=True,  # Rate cannot be changed after the node is initialized
                floating_point_range=[
                    FloatingPointRange(
                        from_value=0.001,  # Allow only values >0
                        to_value=1_000_000.0,  # Some large number, required for the FloatingPointRange
                    ),
                ],
            ),
        )

        battery_info_on_startup_param = self.declare_parameter(
            "battery_info_on_startup",
            True,
            ParameterDescriptor(
                description="Whether to send request a battery info message from all connected buttons on startup",
                read_only=True,  # Only takes effect when the node is started
            ),
        )

        # Serial connection to the dongle
        try:
            self.serial_socket = self.open_serial_to_dongle()  # Raises if the connection cannot be opened
        except serial.SerialException as err:
            self.get_logger().error(f"Failed to connect to dongle: {err}")
            raise err
        self.get_logger().info(f"Connected to dongle on COM port {self.serial_socket.port}")
        self.serial_line_reader = PeriodicSerialLineReader(self.serial_socket)

        # Create publishers
        self.button_press_publisher = self.create_publisher(
            bee_buttons_interfaces.msg.BeeButtonPress, "button_press", rclpy.qos.qos_profile_system_default
        )
        self.button_battery_info_publisher = self.create_publisher(
            bee_buttons_interfaces.msg.BeeButtonBatteryInfo, "button_battery_info", rclpy.qos.qos_profile_system_default
        )

        # Start timer to poll the serial port
        update_rate_hz = self.get_parameter(update_rate_param.name).get_parameter_value().double_value
        self.timer = self.create_timer(1 / update_rate_hz, self.poll_buttons)

        # If specified, all buttons are requested to send their battery information when the node starts
        if self.get_parameter(battery_info_on_startup_param.name).get_parameter_value().bool_value:
            self.command_broadcast("Battery:Show")

    @classmethod
    def open_serial_to_dongle(cls, baudrate: int = DEFAULT_BAUD_RATE) -> serial.Serial:
        """
        Open a serial socket to the interaction button dongle.

        :return: An open PySerial `Serial` socket which is connected to the interaction button dongle.
        :raises `serial.SerialException`: If the dongle cannot be found or the serial port cannot be opened.
        """
        # Find the first occurence of the dongle serial port
        try:
            dongle_com_port = next(
                port.device for port in serial.tools.list_ports.comports() if cls.DONGLE_DEVICE_NAME in port.description
            )
        except StopIteration:
            raise serial.SerialException("Could not find dongle serial COM port")

        # Open the serial connection
        return serial.Serial(port=dongle_com_port, baudrate=baudrate)

    def poll_buttons(self) -> None:
        """
        Poll the serial connection for any updates the buttons may have.

        This function is meant to be run periodically. It will read all available messages from the dongle and handle
        each message separately.
        """
        # Trigger the reader to read all lines currently in the serial socket buffer
        self.serial_line_reader.update()

        # Handle all messages that are received
        while self.serial_line_reader.has_next_line():
            # Read the next message as a string and strip any whitespaces (including the newline)
            button_message = self.serial_line_reader.read_next_line().decode().strip()
            self.get_logger().debug(f'Received button message: "{button_message}"')
            self.handle_button_message(button_message)

    def handle_button_message(self, button_message: str) -> None:
        """
        Handle a button message.

        If the message is for a button press, it is published to the button press ROS topic.

        If the message is a battery information message, it is published to the battery information ROS topic.

        :param button_message: Raw button message as obtained from the dongle, without a newline character.
        """
        button_msg_split = button_message.split(self.COMMAND_SEPARATOR, 1)
        node_id = button_msg_split[0]
        button_message_contents = button_msg_split[1]

        header = std_msgs.msg.Header(stamp=self.get_clock().now().to_msg())

        if button_message_contents == "Pressed":
            # Single button press
            button_press_ros_msg = bee_buttons_interfaces.msg.BeeButtonPress(
                header=header,
                node_id=node_id,
                press_type=bee_buttons_interfaces.msg.BeeButtonPress.PRESSED,
            )
            self.button_press_publisher.publish(button_press_ros_msg)

        elif button_message_contents == "Double Pressed":
            # Double button press
            button_press_ros_msg = bee_buttons_interfaces.msg.BeeButtonPress(
                header=header,
                node_id=node_id,
                press_type=bee_buttons_interfaces.msg.BeeButtonPress.DOUBLE_PRESSED,
            )
            self.button_press_publisher.publish(button_press_ros_msg)

        elif button_message_contents == "Long Press":
            # Long button press
            button_press_ros_msg = bee_buttons_interfaces.msg.BeeButtonPress(
                header=header,
                node_id=node_id,
                press_type=bee_buttons_interfaces.msg.BeeButtonPress.LONG_PRESS,
            )
            self.button_press_publisher.publish(button_press_ros_msg)

        elif button_message_contents.startswith("Battery Percentage"):
            # Battery information message, which requires some additional parsing
            # Remove all descriptions from the string to leave only the numbers
            button_message_contents = button_message_contents.replace("Battery Percentage:", "", 1)
            button_message_contents = button_message_contents.replace("Battery Voltage:", "", 1)
            button_message_contents = button_message_contents.replace("leds on:", "", 1)

            # Split the message (which automatically handles multiple spaces in a row)
            battery_values_split = button_message_contents.split()

            battery_info_ros_msg = bee_buttons_interfaces.msg.BeeButtonBatteryInfo(
                header=header,
                node_id=node_id,
                battery_percentage=int(battery_values_split[0]),
                battery_voltage=float(battery_values_split[1]),
                battery_indicator_leds_on=int(battery_values_split[2]),
            )
            self.button_battery_info_publisher.publish(battery_info_ros_msg)

        else:
            # Unknown message contents
            self.get_logger().warn(
                f'Cannot handle button message contents: "{button_message_contents}" (from node {node_id})'
            )

    def command_specific_button(self, node_id: str, command: str) -> None:
        """
        Push a message to a specific button.

        :param node_id: Node ID of the button to push to.
        :param command: Command to send.
        """
        command = f"{node_id}{self.COMMAND_SEPARATOR}{command}{self.COMMAND_END}"
        self.get_logger().info(f'Sending command: "{command}" to button {node_id}')
        self.serial_socket.write(command.encode())

    def command_broadcast(self, command: str) -> None:
        """
        Push a message to all connected buttons.

        :param command: Command to send.
        """
        command = f"Broadcast{self.COMMAND_SEPARATOR}{command}{self.COMMAND_END}"
        self.get_logger().info(f'Sending command: "{command}" to all nodes')
        self.serial_socket.write(command.encode())


def main(args=None):
    """Start the node."""
    rclpy.init(args=args)

    bee_buttons_node = BeeButtonsNode()

    rclpy.spin(bee_buttons_node)

    bee_buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
