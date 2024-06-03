# Import ROS2.
import rclpy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import math
import numpy as np

# Function to calculate the Euler angles from a quaternion
# https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation
def quaternion_to_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2 > +1.0, +1.0, t2)
    # t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2 < -1.0, -1.0, t2)
    # t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

# Functions to calculate the new cartesian coordinates after a rotation around the x, y, or z axis
def rotate_around_x_axis(x, y, z, alpha):
    """
    Rotate a point (x, y, z) around the x-axis (sagittal axis) by angle alpha (in radians).

    Args:
        x (float): The x-coordinate of the point.
        y (float): The y-coordinate of the point.
        z (float): The z-coordinate of the point.
        alpha (float): The angle of rotation around the x-axis in radians.

    Returns:
        tuple: A tuple containing the new coordinates (x', y', z').
    """
    # Calculate the new y and z coordinates using the rotation matrix around the x-axis
    y_prime = y * math.cos(alpha) - z * math.sin(alpha)
    z_prime = y * math.sin(alpha) + z * math.cos(alpha)

    # x coordinate remains unchanged
    x_prime = x

    return (x_prime, y_prime, z_prime)

def rotate_around_y_axis(x, y, z, alpha):
    """
    Rotate a point (x, y, z) around the y-axis (posterior/anterior axis) by angle alpha (in radians).

    Args:
        x (float): The x-coordinate of the point.
        y (float): The y-coordinate of the point.
        z (float): The z-coordinate of the point.
        alpha (float): The angle of rotation around the y-axis in radians.

    Returns:
        tuple: A tuple containing the new coordinates (x', y', z').
    """
    # Calculate the new x and z coordinates using the rotation matrix around the y-axis
    x_prime = x * math.cos(alpha) + z * math.sin(alpha)
    z_prime = -x * math.sin(alpha) + z * math.cos(alpha)

    # y coordinate remains unchanged
    y_prime = y

    return (x_prime, y_prime, z_prime)

def rotate_around_z_axis(x, y, z, alpha):
    """
    Rotate a point (x, y, z) around the z-axis (vertical axis) by angle alpha (in radians).

    Args:
        x (float): The x-coordinate of the point.
        y (float): The y-coordinate of the point.
        z (float): The z-coordinate of the point.
        alpha (float): The angle of rotation around the z-axis in radians.

    Returns:
        tuple: A tuple containing the new coordinates (x', y', z').
    """
    # Calculate the new x and y coordinates using the rotation matrix around the z-axis
    x_prime = x * math.cos(alpha) - y * math.sin(alpha)
    y_prime = x * math.sin(alpha) + y * math.cos(alpha)

    # z coordinate remains unchanged
    z_prime = z

    return (x_prime, y_prime, z_prime)

# Define a function to compute rotation matrices.
def get_rotation_matrix(axis, angle):
    """ Return the rotation matrix for a given axis and angle. """
    if axis == 'x':
        return np.array([
                [1, 0, 0],
                [0, np.cos(angle), -np.sin(angle)],
                [0, np.sin(angle), np.cos(angle)]
            ])
    elif axis == 'y':
        return np.array([
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)]
            ])
    elif axis == 'z':
        return np.array([
                [np.cos(angle), -np.cos(angle)],
                [np.sin(angle), np.cos(angle)],
                [0, 0, 1]
            ])
    else:
        raise ValueError("Invalid rotation axis. Choose 'x', 'y', or 'z'.")


# Function to calculate the rotation matrix from a quaternion using scipy
def quaternion_to_rotation_matrix(quaternion):
    # Convert the quaternion to a scipy Rotation object
    rotation = R.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
    # Get the rotation matrix from the Rotation object
    return rotation.as_matrix()


# Function to calculate the quaternion from Euler angles
def euler_to_quaternion(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


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
        - parent_node        : A ROS2 node to which the service client is associated.
        - external_node_name : The external ROS2 node a client connection to the service is made.
        - tag     (optional) : Custom string tag for identifying the ROS2ServiceClient object (default:
                               None).
        - verbose (optional) : Enables verbose text output if set to True (default: False).

        Example:
        -----------
        ```
        rclpy.init()
        my_node = rclpy.create_node('my_node')
        service_client = ROS2ServiceClient(my_node, 'external_node', verbose=True)
        ```
        """

    def __init__(self, parent_node, external_node_name, tag=None, verbose=False):
        self.node = parent_node
        self.external_node_name = external_node_name
        self.tag = tag
        self.verbose = verbose
        self.logger("Creating ROS2ServiceClient object...")
        self.set_client = self.node.create_client(SetParameters, '/' + str(self.external_node_name) + '/set_parameters')
        while not self.set_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info('{} service not available, waiting again...'.format(str(self.external_node_name)))

        self.get_client = self.node.create_client(GetParameters, '/' + str(self.external_node_name) + '/get_parameters')
        while not self.get_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info('{} service not available, waiting again...'.format(str(self.external_node_name)))

        self.logger("ROS2ServiceClient object for {} created successfully!".format(self.external_node_name))

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
