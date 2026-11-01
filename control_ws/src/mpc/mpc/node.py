import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from control_common.node import PlanarControlNode
from .controller import Config,MPC


class MPCNode(PlanarControlNode):
    def __init__(self):
        super().__init__('mpc',vars(Config()),lambda cfg:MPC(Config(**cfg)),control_rate_hz=20.0)
        self.mpc=self.controller


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=MPCNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
