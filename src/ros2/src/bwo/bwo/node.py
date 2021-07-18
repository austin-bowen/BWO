import rclpy.node

from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter


ENABLED_PARAMETER_NAME = 'enabled'


class Node(rclpy.node.Node):
    def __init__(self, node_name: str, logger_level: LoggingSeverity = None, **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        # Setup logger
        if logger_level is not None:
            logger = self.get_logger()
            logger.set_level(logger_level)

        # Declare parameters
        self.declare_parameter(ENABLED_PARAMETER_NAME, value=True)

    @property
    def is_enabled(self) -> bool:
        """True if the node is enabled. If not, then the node should do as little as possible."""
        return self.get_parameter(ENABLED_PARAMETER_NAME).value

    @is_enabled.setter
    def is_enabled(self, enabled: bool) -> None:
        self.get_logger().debug(f'Setting "enabled" parameter to: {enabled}')

        self.set_parameters([Parameter(ENABLED_PARAMETER_NAME, value=enabled)])

