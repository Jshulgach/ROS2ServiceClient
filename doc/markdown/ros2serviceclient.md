
## ROS2 Service Client

Have you even wanted to programmatically acquire or change the parameters from another ROS2 node? 

If the answer is yes, then this example is for you!

`RO2ServiceClient` is a Python-based object that acts as a client for a ROS2 service with another ROS2 node in your network. Basic usage is as follows:

In a new terminal (terminal #1), let's create a node that echoes a message every second:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

node = Node('talker')
node.declare_parameter('message', 'Hello, world!')
pub = node.create_publisher(String, 'chatter', 10)
timer = node.create_timer(1, lambda: pub.publish(String(data=node.get_parameter('message').value)))

```

Now let's open another new terminal (terminal #2) and create a ROS2ServiceClient object with some unique name:
```python
from ros2_utilities_py import ROS2ServiceClient
talker_client = ROS2ServiceClient('talker', 'message-param-client')
```
If we want to get the current value of the 'message' parameter, we can use the `get_parameter` method:
```python
talker_client.get_parameter('message')
>> 'Hello, world!'
```

If we want to change the value of the 'message' parameter, we can use the `set_parameter` method:
```python
talker_client.set_parameter('message', 'Goodbye, world!')
```
The next time you check the output message from the talker node, you should see that it has changed to 'Goodbye, world!'
```python
talker_client.get_parameter('message')
>> 'Goodbye, world!'
```
In addition to the ```str()``` data type, the following python types are also supported:
* ```int()```
* ```float()```
* ```bool()```
* ```bytearray()```
* ```list()``` containing only ```bool()```
* ```list()``` containing only ```int()```
* ```list()``` containing only ```float()```
* ```list()``` containing only ```str()```


