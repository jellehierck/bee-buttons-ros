import serial


class ReadLine:
    """
    Serial readline() helper class.

    This class attempts to read a line from the serial array more efficiently than the `realine()` implementation of
    pySerial.

    Based on: https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522
    Author: skoehler (<https://github.com/skoehler>)
    Changes made by Jelle Hierck (2024):
    - Made max bytes to get configurable
    - Added has_new_data() function to check if more data is available
    - Added exception handling in case of timeout
    - Added type hints
    - Renamed variables to be more descriptive
    - Added comments
    """

    DEFAULT_MAX_BYTES_TO_GET: int = 2048

    def __init__(self, serial_socket: serial.Serial, max_bytes_to_get: int = DEFAULT_MAX_BYTES_TO_GET):
        self.serial_socket: serial.Serial = serial_socket
        self.max_bytes_to_get = max_bytes_to_get

        self.buffer = bytearray()

    def has_new_data(self) -> bool:
        # TODO: Check if the newline is always present in the buffer
        return self.buffer or self.serial_socket.in_waiting != 0

    def readline(self) -> bytearray:
        # Check if the newline character is present in the current buffer. If so, we can read the next line from the
        # buffer without needing to read the serial socket
        next_line_size = self.buffer.find(b"\n")
        if next_line_size >= 0:
            # Get the contents of the buffer up to and including the newline
            result = self.buffer[: next_line_size + 1]

            # Remove the result contents (including the newline) from the buffer
            self.buffer = self.buffer[next_line_size + 1 :]

            # Return the result
            return result

        # There currently is no newline character in the buffer. This means we have to read from the serial socket
        while True:
            # Determine the number of bytes to get, and read those bytes from the serial socket
            # Number of bytes read is chunked using self.max_bytes_to_get
            bytes_to_get = max(1, min(self.serial_socket.in_waiting, self.max_bytes_to_get))
            read_data = self.serial_socket.read(bytes_to_get)

            # No data read means that a timeout has occurred
            if not read_data:
                raise serial.SerialException("Timeout on reading data")

            # Check if there is a newline in the current buffer
            next_line_size = read_data.find(b"\n")
            if next_line_size >= 0:
                # We have a newline character inside the buffer
                # Get the contents of the buffer up to and including the newline
                result = self.buffer + read_data[: next_line_size + 1]

                # Remove the result contents (including the newline) from the buffer
                self.buffer[0:] = read_data[next_line_size + 1 :]

                # Return the result
                return result

            else:
                # There is no newline character inside the buffer. The buffer needs to be extended to hold the data we
                # just read and the loop is started again.
                self.buffer.extend(read_data)
