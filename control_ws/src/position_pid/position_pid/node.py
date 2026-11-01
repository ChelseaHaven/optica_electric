from control_common.node import PlanarControlNode, yaw_from_quaternion
from .controller import Config, PositionPID
import signal
import rclpy
from rclpy.executors import ExternalShutdownException


class PositionControl(PlanarControlNode):
    def __init__(self):
        super().__init__('position_pid', vars(Config()), lambda cfg: PositionPID(Config(**cfg)))
        self.pid = self.controller


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PositionControl()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
