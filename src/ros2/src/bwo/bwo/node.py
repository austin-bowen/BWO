import rclpy.node

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter


ENABLED_PARAMETER_NAME = 'enabled'


class Node(rclpy.node.Node):
    """Subclass of rclpy.node.Node with additional functionality."""

    def __init__(self, node_name: str, logger_level: LoggingSeverity = None, **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        # Setup logger
        if logger_level is not None:
            self.logger.set_level(logger_level)

        # Declare parameters
        self.declare_parameter(
            ENABLED_PARAMETER_NAME,
            value=True,
            descriptor=ParameterDescriptor(
                description='If True (default), the node should operate like normal. ' + \
                    'If False, the node should cease as much operation as possible.'
            )
        )

    def __str__(self) -> str:
        """Returns "[/namespace]/<name>"."""

        result = self.get_namespace()

        if not result.endswith('/'):
            result += '/'

        result += self.get_name()

        return result  

    @property
    def is_enabled(self) -> bool:
        """True if the node is enabled. If not, then the node should do as little as possible."""
        return self.get_parameter(ENABLED_PARAMETER_NAME).value

    @is_enabled.setter
    def is_enabled(self, enabled: bool) -> None:
        self.get_logger().debug('Node enabled' if enabled else 'Node disabled')
        self.set_parameter(ENABLED_PARAMETER_NAME, enabled)

    @property
    def logger(self):
        """The result of self.get_logger()."""
        return self.get_logger()

    def destroy_node(self) -> bool:
        self.logger.info('Destroying...')
        return super().destroy_node()

    def set_parameter(name, value) -> SetParametersResult:
        """Sets a single parameter and returns the result."""

        param = Parameter(name, value=value)
        return self.set_parameters([param])[0]

