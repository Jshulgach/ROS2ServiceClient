# Import ROS2.
import rclpy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def is_boolean_array(variable):
    return isinstance(variable, list) and all(isinstance(item, bool) for item in variable)


def is_integer_array(variable):
    return isinstance(variable, list) and all(isinstance(item, int) for item in variable)


def is_double_array(variable):
    return isinstance(variable, list) and all(isinstance(item, float) for item in variable)


def is_string_array(variable):
    return isinstance(variable, list) and all(isinstance(item, str) for item in variable)


def extract_parameter_value(parameter_value_msg):
    """ Helper function that picks out the correct ROS2 parameter message field based
        on the message type
    """
    if parameter_value_msg.type == ParameterType.PARAMETER_BOOL:
        return parameter_value_msg.bool_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_INTEGER:
        return parameter_value_msg.integer_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_DOUBLE:
        return parameter_value_msg.double_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_STRING:
        return parameter_value_msg.string_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return parameter_value_msg.byte_array_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return parameter_value_msg.bool_array_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return parameter_value_msg.integer_array_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return parameter_value_msg.double_array_value
    elif parameter_value_msg.type == ParameterType.PARAMETER_STRING_ARRAY:
        return parameter_value_msg.string_array_value
    else:
        return None  # Unknown type, return None


class ROS2ServiceClient:
    """ Initializes a ROS2ServiceClient object to interact with parameter services from an external node in ROS2 like getting/setting values.

        Parameters:
        -----------
        - node              : A ROS2 node to which the service client is associated.
        - node_name         : The external ROS2 node a client connection to the service is made.
        - tag     (optional): Custom string tag for identifying the ROS2ServiceClient object (default:
                              None).
        - verbose (optional): Enables verbose text output if set to True (default: False).

        Example:
        -----------
        ```
        rclpy.init()
        my_node = rclpy.create_node('my_node')
        service_client = ROS2ServiceClient(my_node, name='param_client', verbose=True)
        ```
        """

    def __init__(self, node, node_name, tag=None, verbose=False):
        self.node = node
        self.node_name = node_name
        self.tag = tag
        self.verbose = verbose
        self.set_client = self.node.create_client(SetParameters, '/' + str(self.node_name) + '/set_parameters')
        while not self.set_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.get_client = self.node.create_client(GetParameters, '/' + str(self.node_name) + '/get_parameters')
        while not self.get_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info('service not available, waiting again...')

    def set_parameter(self, param_name, param_value):
        """ Set new value for parameter to service node from external client node

        Parameters:
        -----------
        param_name  - (str) Name of the parameter to adjust
        param_value - (float, int, str, bool) New parameter value, accepts multiple datatypes
        """
        if self.set_client:
            request = SetParameters.Request()
            if isinstance(param_value, bool):
                val = ParameterValue(bool_value=param_value,
                                     type=ParameterType.PARAMETER_BOOL)
            elif isinstance(param_value, float):
                val = ParameterValue(double_value=param_value,
                                     type=ParameterType.PARAMETER_DOUBLE)
            elif isinstance(param_value, int):
                val = ParameterValue(integer_value=param_value,
                                     type=ParameterType.PARAMETER_INTEGER)
            elif isinstance(param_value, str):
                val = ParameterValue(string_value=param_value,
                                     type=ParameterType.PARAMETER_STRING)
            elif isinstance(param_value, bytearray):
                val = ParameterValue(byte_array_value=param_value,
                                     type=ParameterType.PARAMETER_BYTE_ARRAY)
            elif is_boolean_array(param_value):
                val = ParameterValue(bool_array_value=param_value,
                                     type=ParameterType.PARAMETER_BOOL_ARRAY)
            elif is_integer_array(param_value):
                val = ParameterValue(integer_array_value=param_value,
                                     type=ParameterType.PARAMETER_INTEGER_ARRAY)
            elif is_double_array(param_value):
                val = ParameterValue(double_array_value=param_value,
                                     type=ParameterType.PARAMETER_DOUBLE_ARRAY)
            elif is_string_array(param_value):
                val = ParameterValue(string_array_value=param_value,
                                     type=ParameterType.PARAMETER_STRING_ARRAY)

            request.parameters = [Parameter(name=param_name, value=val)]

            future = self.set_client.call_async(request)
            # rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                if self.verbose:
                    self.logger(f'Successfully set parameter {param_name} to {param_value}')
                return True
            else:
                if self.verbose:
                    self.logger(f'Failed to set parameter {param_name}', warning=True)
                return False

    def get_parameter(self, param_name):
        """ Get parameter from external node

        Parameters:
        -----------
        param_name  - (str) Name of the parameter to adjust
        """
        if self.get_client:
            request = GetParameters.Request()
            request.names = [param_name]

            future = self.get_client.call_async(request)
            while not future.done():
                pass
            # rclpy.spin_once(self.node, timeout_sec=0.1)
            # rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None and future.result().values:
                value = extract_parameter_value(future.result().values[0])
                if self.verbose:
                    self.node.get_logger().info(f'Value of parameter {param_name}: {value}')
                return value
            else:
                if self.verbose:
                    self.node.get_logger().warning(f'Failed to get parameter {param_name}')
                return None

    def logger(self, msg="", warning=False):
        if warning:
            self.node.get_logger().warning(msg)
        else:
            self.node.get_logger().info(msg)
