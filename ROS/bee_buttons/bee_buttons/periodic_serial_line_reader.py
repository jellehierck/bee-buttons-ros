import serial


class PeriodicSerialLineReader:
    """
    Serial line reader which can periodically read new lines.

    This class is meant to be used in a loop where you want to periodically poll the serial socket for all
    available data up to now and consume the read data later.
    """

    def __init__(self, serial_socket: serial.Serial):
        """
        Instantiate the PeriodicSerialLineReader.

        :param serial_socket: Serial socket to read. The socket should be configured and opened.
        :type serial_socket: PySerial `Serial` object
        """
        self.serial_socket = serial_socket
        self.lines_buffer: list[bytearray] = []
        self.partial_line_buffer = bytearray()

    def update(self) -> None:
        """
        Read all contents from the serial socket's internal buffer and store read lines internally.

        This function is meant to be called periodically.
        """
        # Read all available data on the socket
        read_data = self.serial_socket.read(self.serial_socket.in_waiting)

        # We might have a partial line left from the last iteration, so we need to prepend the read data with that
        partial_line_buffer_extended_with_read_data = self.partial_line_buffer + read_data

        # Split all data on the newline byte
        read_lines = partial_line_buffer_extended_with_read_data.split(b"\n")

        # If the last element in the read data was a newline, the last element of the returned read_lines list is empty.
        if read_lines[-1] == b"":
            # We do not want to store empty last element since it is not a read line
            read_lines.pop(-1)
        else:
            # If the last element of the returned read_lines list is not empty, it contains a partial message.
            # Remove the partial message from the read_lines list and store in the partial line buffer
            self.partial_line_buffer = read_lines.pop(-1)

        # Add all full lines to the lines buffer
        self.lines_buffer += read_lines

    def has_next_line(self) -> bool:
        """
        Return whether there are full lines in the buffer.
        """
        return bool(self.lines_buffer)

    def read_next_line(self) -> bytearray:
        """
        Returns the next line from the buffer. The line is also removed from the buffer.

        You should first check with `has_next_line()` if there is a next line.
        """
        return self.lines_buffer.pop(0)
