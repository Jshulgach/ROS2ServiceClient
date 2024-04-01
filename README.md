
Various utilities for ROS2 projects

## General Usage:

The following utilities are available in this package:

* [ROS2ServiceClient](doc/markdown/ros2serviceclient.md) - A Python-based ROS2 Service Client for external node parameters


---
## Installation

This package can be placed inside a ROS2 workspace and built like any other [ROS2 package].
1. Navigate to your ROS2 workspace directory. [Create the workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) if you haven't already.
    ```
    cd /path/to/your/ros2/workspace
    ```
2. Clone this repository into the `src` directory of your workspace.
    ```
    git clone https://github.com/Jshulgach/ros2_utilities_py.git src/ros2_utilities_py
    ``` 
3. Build the package
    ```
    colcon build --packages-select ros2_utilities_py
    ```
4. Source the workspace

    * For Linux users:
    ```
    source install/setup.bash
    ```
    * For Windows users:
    ```
    call install/setup.bat
    ```

#### Note: if you have a `CMakeLists.txt` file in your package, modify it to include the following:
```
find_package(ros2_utilities_py REQUIRED)

ament_target_dependencies(my_executable <-- Change this to whatever executable name you have
  # Dependencies
  rclcpp
  ros2_utilities_py # <-- Include this dependency
  # Include all other dependencies you have for your executable
  # ...
)
```
* Open the `package.xml` file and modify it to include the following:
```xml
  <depend>ros2_utilities_py</depend>
```
You can now use the utilities in this package in your ROS2 projects!

<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[ROS2 package]: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html


