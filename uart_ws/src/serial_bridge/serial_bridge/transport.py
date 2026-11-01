"""Linux nonblocking UART, 8N1, no flow control. Standard library only."""
import errno
import fcntl
import os
import termios
import tty


class SerialPort:
    def __init__(self, path, baudrate):
        speed = getattr(termios, 'B' + str(baudrate), None)
        if speed is None:
            raise ValueError(f'unsupported baud rate: {baudrate}')
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            tty.setraw(self.fd, termios.TCSANOW)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
            attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = speed
            # VMIN=1 + O_NONBLOCK distinguishes no data (EAGAIN) from EOF.
            attrs[6][termios.VMIN] = 1
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except BaseException:
            self.close()
            raise

    def read(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return b''
        if not data:
            raise OSError(errno.EIO, 'serial device disconnected')
        return data

    def write(self, data):
        try:
            return os.write(self.fd, data)
        except BlockingIOError:
            return 0

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
