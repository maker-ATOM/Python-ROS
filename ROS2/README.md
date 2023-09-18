# ROS2

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Concepts">Concepts</a></li>
    <ol>
        <li><a href="#Environment">Environment</a></li>
        <li><a href="#RQT-and-ROS2-tools">RQT and ROS2 tools</a></li>
        <li><a href="#Nodes">Nodes</a></li>
        <li><a href="#Topics">Topics</a></li>
        <li><a href="#Services">Services</a></li>
        <li><a href="#Parameters">Parameters</a></li>
        <li><a href="#Actions">Actions</a></li>
        <li><a href="#Launch">Launch</a></li>
        <li><a href="#ROSbag">ROSbag</a></li>
        <li><a href="#Transforms">Transforms</a></li>
        <li><a href="#Script-Node-Executable-Name">Script vs Node vs Executable Name</a></li>
        <li><a href="#Custom-ROS-message">Custom ROS message</a></li>
        <li><a href="#ros2doctor">ros2doctor</a></li>
    </ol>
    <li><a href="#Projects">Projects</a></li>
        <ol>
        <li><a href="#On-Going">On-Going</a></li>
        <li><a href="#Completed">Completed</a></li>
    </ol>
    <li><a href="#Resources">Resources</a></li>
    <!-- <li><a href="#Description-of-src-files">Description of src files</a></li>
    <li><a href="#Project-Status">Project Status</a></li>
    <li><a href="#Project-Upgradation">Project Upgradation</a></li> -->
  </ol>
</details>



**Learning Status**

- [x] ROS - Mecharithm
- [ ] ROS wiki
  - [ ] Tutorials
    - [x] CLI Tools
      -  Configuring environment
      -  Using turtlesim, ros2, and rqt
      -  Understanding nodes
      -  Understanding topics
      -  Understanding services
      -  Understanding parameters
      -  Understanding actions
      -  Using rqt_console to view logs
      -  Launching nodes
      -  Recording and playing back data
    - [x] Client Libraries
      -  Using colcon to build packages
      -  Creating a workspace
      -  Creating a package
      -  Writing a simple publisher and subscriber (Python)
      -  Writing a simple service and client (Python)
      -  Creating custom msg and srv files
      -  Implementing custom interfaces
      -  Using parameters in a class (Python)
      -  Using ros2doctor to identify issues
      -  Creating and using plugins (C++)
    - [ ] Intermediate
      -  Managing Dependencies with rosdep
      -  Creating an action
      -  Writing an action server and client (C++)
      -  Writing an action server and client (Python)
      -  Composing multiple nodes in a single process
      -  Monitoring for parameter changes (C++)
      -  Launch
      -  tf2
      -  Testing
      -  URDF
    - [ ] Advance
    - [ ] Demos
    - [ ] ~~Miscellaneous~~
  - [ ] How to Guides
  - [ ] Concepts
- [ ] Robotics Back-End
- [ ] The Construct
- [ ] Raspberry Pi
- [ ] Linux


## Concepts

### Environment

**Underlays and Overlays**

The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays.<br>
In general, it is recommended to use an overlay when we plan to iterate on a small number of packages, rather than putting all of our packages into the same workspace.<br>
Packages in our overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.<br>
Usually main ROS 2 installation will be the underlay which does not has to be every time.<br>

A particular package may exist in an overlay and underlay for example, turtlesim package if installed from apt methods exists in main ROS2 environment, and is it is cloned from github into the workspace it exists in the overlay. If package exists in both lays overlay takes precedence over the underlay.<br>
Modifications in the overlay do not affect packages in underlay.  

**Sourcing**
Without sourcing the setup files, we won’t be able to access ROS 2 commands, or find or use ROS 2 packages.

Add this command to `.bashrc` file.
```python
source /opt/ros/humble/setup.bash
```
So that every time a new terminal session is initiated this command is executed and ROS 2 commands can be accessed.<br>
What we are actually doing is sourcing our main ROS2 environment as the “underlay”, so we can build the overlay “on top of” it.
```python
source ~/ros2_ws/install/local_setup.bash
```
This sources our overlay workspace.

Sourcing the local_setup of the overlay will only add the packages available in the overlay to our environment. setup sources the overlay as well as the underlay it was created in, allowing us to utilize both workspaces.<br>
So, sourcing our main ROS 2 installation’s setup and then the ros2_ws overlay’s local_setup, like we just did, is the same as just sourcing ros2_ws’s setup, because that includes the environment of its underlay.

**Domain**
ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot.<br>
To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.<br>

The domain ID can be changed using
```python
export ROS_DOMAIN_ID=<our_domain_id>
 ```

**Local Host**
By default, ROS 2 communication is not limited to localhost. ROS_LOCALHOST_ONLY environment variable allows us to limit ROS 2 communication to localhost only. <br>
This means our ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network.<br>

```python
export ROS_LOCALHOST_ONLY=1
```

**Colcon**

We won’t be able to use the `sudo apt install ros-<distro>-<package>` command if we install from source.

Colcon is an iteration on the ROS build tools catkin_make.

When colcon has completed building successfully, the output will be in the install directory.<br>
Before we can use any of the installed executables or libraries, we will need to add them to our path and library paths. Colcon will have generated bash/bat files in the install directory to help set up the environment. These files will add all of the required elements to our path and library paths as well as provide any bash or shell commands exported by packages.

Colcon supports multiple build types. The recommended build types are ament_cmake and ament_python.<br>
`ament_cmake` is a build system package used for building packages written in C++ and using the CMake build system. 
`ament_python` is a build system package used for building Python packages in the ROS 2 ecosystem.<br>
In python packages, setup.py is the primary entry point for building.

Other useful arguments for colcon build:

- `--packages-up-to` builds the package we want, plus all its dependencies, but not the whole workspace (saves time)
- `--symlink-install` saves us from having to rebuild every time we tweak python scripts
- `--event-handlers` console_direct+ shows console output while building (can otherwise be found in the log directory)

The install directory is where our workspace’s setup files are, which we can use to source our overlay.

**ROS dependencies**
Packages developed may depend on other package for its operation, this can be verified using,
From the root of our workspace,

```python
rosdep install -i --from-path src --rosdistro humble -y
```
`rosdep` is a dependency management utility. It is a command-line utility for identifying and installing dependencies to build or install a package. rosdep is not a package manager in its own right; it is a meta-package manager that uses its own knowledge of the system and the dependencies to find the appropriate package to install on a particular platform. The actual installation is done using the system package manager (e.g. apt on Debian/Ubuntu, etc).

rosdep check for keys(entity we mention within the tags) within the package.xml file. These keys are then cross-referenced against a central index to find the appropriate ROS package or software library in various package managers.

Packages declare their dependencies in the package.xml file. This command walks through those declarations and installs the ones that are missing.

Various tag are to define the level of dependencies within the package.xml file.

<b>Depend</b>
These are dependencies that should be provided at both build time and run time for our package.

<b>build_depend</b>
If we only use a particular dependency for building our package, and not at execution time.
With this type of dependency, an installed binary of our package does not require that particular package to be installed.


<b>build_export_depend</b>
If we export a header that includes a header from a dependency, it will be needed by other packages that <build_depend> on ours.

<b>exec_depend</b>
This tag declares dependencies for shared libraries, executables, Python modules, launch scripts and other files required when running our package.

<b>test_depend</b>
This tag declares dependencies needed only by tests. Dependencies here should not be duplicated with keys specified by <build_depend>, <exec_depend>, or <depend>.

**Package**

Package creation in ROS 2 uses ament as its build system and colcon as its build tool.

Minimal package contents:

- package.xml file containing meta information about the package
- resource/<package_name> marker file for the package
- setup.cfg is required when a package has executables, so ros2 run can find them
- setup.py containing instructions for how to install the package
- <package_name> - a directory with the same name as our package, used by ROS 2 tools to find our package, contains __init__.py

Nested packages are not allowed.

```python
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```

To build a specific package,

```python
colcon build --packages-select <package_name>x
```

**Others**

IF any ROS2 package is not being located, the first thing we should do is check our environment variables and ensure they are set to the version and distro we intended.

### RQT and ROS2 tools

rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.

Use `rqt_graph` to visualize the changing nodes and topics, as well as the connections between them.

Use `rqt` to have a graphical user interface to publish to topic without creating a publisher node.  
 

To find executables within a package run,
```python
ros2 pkg executables <package_name>
```

List subcommand can be used to view all the available elements of the corresponding command.
```python
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```


The ROS graph is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if we were to map them all out and visualize them.

`rqt_console` is a GUI tool used to introspect log messages in ROS 2. Typically, log messages show up in our terminal.
```python
ros2 run rqt_console rqt_console
```
ROS 2’s logger levels are ordered by severity:

```python
Fatal
Error
Warn
Info
Debug
```
There is no exact standard for what each level indicates, but it’s safe to assume that:

- **Fatal** messages indicate the system is going to terminate to try to protect itself from detriment.

- **Error** messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.

- **Warn** messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.

- **Info** messages indicate event and status updates that serve as a visual verification that the system is running as expected.

- **Debug** messages detail the entire step-by-step process of the system execution.

The default level is Info. We will only see messages of the default severity level and more-severe levels.


### Nodes
Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

 <p align="center">
	<img src="Images/Nodes-TopicandService.gif"/>
</p>


The ROS system architecture must be designed in a way so that each individual task of the robot can be executed by individual nodes.

To run a executable,
```python
ros2 run <package_name> <executable_name>
```

The `node_name` and `executable_name` need not necessarily be same always.

**Remapping** allows us to reassign default node properties, like node name, topic names, service names, etc., to custom values.

To run a node while setting a specific parameter,
```python
ros2 run <package_name> <executable_name> --ros-args -p <parameter_nmae>:=<parameter_value>
```

### Topics
Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.
A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.
Topics don't have be a point-point communication they can one-many, many-one and many-many.

 <p align="center">
	<img src="Images/Topic-SinglePublisherandSingleSubscriber.gif"/>
</p>

```python
ros2 topic list -t
``` 
will return the same list of topics, this time with the topic type appended in brackets.

```python
ros2 topic ...
```
Double tab to view all available introspection tools of ROS2.


### Services
Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.
There can be many clients requesting a service from a server. But only one server exists serving the request.

 <p align="center">
	<img src="Images/
Service-SingleServiceClient.gif"/>
</p>


Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

We generally don’t want to use a service for continuous calls; topics or even actions would be better suited.

Running the `ros2 service list` command in a new terminal will return a list of all the services currently active in the system:

To find out the type of a service, use the command:
```python
ros2 service type <service_name>
```

To find all the services of a specific type, we can use the command: 
```python
ros2 service find <type_name>
```

If run,
```python
ros2 interface show turtlesim/srv/Spawn
```
a service within turtlesim package, we will receive,

```python
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

The information above the --- line tells us the arguments needed to call /spawn. x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional. The information below the line indicates the information received after the server serves the requests. 

To call a service,
```python
ros2 service call <service_name> <service_type> <arguments>
```

Request and response may or may not have same rosmsg, this structure is defined within a `.srv` file.

### Parameters
A parameter is a configuration value of a node. We can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.

To see the parameters belonging to our nodes, open a new terminal and enter the command:
```python
ros2 param list
```

To display the type and current value of a parameter, use the command:

```python
ros2 param get <node_name> <parameter_name>
```

To change a parameter’s value at runtime, use the command:
```python
ros2 param set <node_name> <parameter_name> <value>
```

We can view all of a node’s current parameter values by using the command:
```python
ros2 param dump <node_name>
```
The command prints to the standard output (stdout) by default but we can also redirect the parameter values into a file to save them for later.

```python
ros2 param dump /<node_name> > <file_name>
```

To start the same node using our saved parameter values, use:
```python
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

To see the description of parameters,
```python
ros2 param describe /node_name parameter_name
```

Often a node needs to respond to changes to its own parameters or another node’s parameters. The ParameterEventHandler class makes it easy to listen for parameter changes so that our code can respond to them.

### Actions
Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.


Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.


Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

 <p align="center">
	<img src="Images/Action-SingleActionClient.gif"/>
</p>

The action client send a request to the action server stating that a goal(task) needs to be achieved. The task can be something like to get a robot joint reach a particular position.

The server keeps the client updating over the feedback topic.
When the server-side chooses to stop processing a goal, it is said to “abort” the goal.
Don’t assume every action server will choose to abort the current goal when it gets a new one.

Clients and servers are two different nodes.

To identify all the actions in the ROS graph, run the command:

```python
ros2 action list
```

Each action is encompassed in a rosmsg.
If we check the action provided within turtlesim,
```python
ros2 interface show turtlesim/action/RotateAbsolute

# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

An example of action could be A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

### Launch
Launch files allow us to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Running a single launch file with the ros2 launch command will start up our entire system - all nodes and their configurations - at once.

Launch files can be written in Python, XML and YAML.

The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

Each node launched through a launch files required package name, executable name and identification name at the minimal.

**Substitutions**
Substitutions are variables that are only evaluated during execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.

**Event Handles**
Launch files are also responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes. These changes are called events and can be handled by registering an event handler with the launch system. Event handlers can be registered for specific events and can be useful for monitoring the state of processes. Additionally, they can be used to define a complex set of rules which can be used to dynamically modify the launch file.

**Launch File architecture**
The system must be so designed that is can be reusable as possible.  This could be done by clustering related nodes and configurations into separate launch files. Afterwards, a top-level launch file dedicated to a specific configuration could be written. This would allow moving between identical robots to be done without changing the launch files at all. Even a change such as moving from a real robot to a simulated one can be done with only a few changes. Top-level launch files should be short, consist of includes to other files corresponding to subcomponents of the application, and commonly changed parameters. Writing launch files in the following manner makes it easy to swap out one piece of the system. However, there are cases when some nodes or launch files have to be launched separately due to performance and usage reasons.

Unique namespaces allow the system to start two similar nodes without node name or topic name conflicts.

### ROSbag
ros2 bag is a command line tool for recording data published on topics in our system. It accumulates the data passed on any number of topics and saves it in a database. We can then replay the data to reproduce the results of our tests and experiments. Recording topics is also a great way to share our work and allow others to recreate it.

ros2 bag can only record data from published messages in topics. ROSbag file will save in the directory where we run it.

To record the data published to a topic use the command syntax:
```python
ros2 bag record <topic_name>
```
We can record multiple topics and also can rename the bag file.
```python
ros2 bag record -o <file_name> <topic_1> <topic_2>
```

We also republish the data stored in the bag for introspection.
```python
ros2 bag play <file_name>
```

## Transforms
Publishing static transforms is useful to define the relationship between a robot base and its sensors or non-moving parts. When there no relative motion between the parts through operations of robot.

Ideally to broadcast transforms we need not to create a script to do so, either we can use direct terminal to publish static transform,

```python
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id
```

or we use the `static_transform_publisher` executable provided by `tf2_ros` and run it through a launch file and providing the transform data through arguments,

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
        ),
    ])
```

To visualize transform between frame_id and child_frame_id.

```python
ros2 run tf2_ros tf2_echo <frame_id> <child_frame_id>
```
When transforming from one frame to another, tf2 will take care of all the hidden intermediate frame transformations that are introduced.

tf2 builds up a tree structure of frames and, thus, does not allow a closed loop in the frame structure. This means that a frame only has one single parent, but it can have multiple children.

timeout argument which makes the lookup_transform wait for the specified transform for up to the specified duration before throwing an exception. This tool can be useful to listen for transforms that are published at varying rates or those incoming source with unreliable networking and non negligible latency.

```python
ros2 run tf2_tools view_frames
```

To get a graphical representation of tf tree

```python
ros2 run tf2_ros tf2_monitor turtle2 turtle1
```

Another tool to monitor transform between two frames.

**Quaternions**
ROS 2 uses quaternions to track and apply rotations. A quaternion has 4 components (x, y, z, w). In ROS 2, w is last.

The magnitude of a quaternion should always be one and hence should be normalized, using `normalize()`

To apply the rotation of one quaternion to a pose, simply multiply the previous quaternion of the pose by the quaternion representing the desired rotation. The order of this multiplication matters.

An easy way to invert a quaternion is to negate the w-component.
### Script Node Executable Name

**Python Script Name**

This refers to the name of the Python script file that contains the code for a ROS 2 node.
It's simply the filename with the ".py" extension.
This script typically defines a ROS 2 node, its functionality, and how it interacts with other nodes in a ROS 2 system.

**Node Name**

The node name is a unique identifier given to a ROS 2 node when it is launched.
It is defined within the code of the Python script that creates the ROS 2 node.
The node name is used to distinguish one node from another within a ROS 2 system.
It's essential to ensure that node names are unique within a ROS 2 graph because it helps in identifying and communicating with specific nodes.

**Executable Name**

The executable name refers to the name of the binary file that is created when we build our ROS 2 package.
In ROS 2, nodes are typically organized into packages, and each package can contain one or more nodes.
When we build a ROS 2 package, it generates one or more executable files (binaries) that correspond to the nodes defined in that package.
The executable name for a node is specified in the package's setup.py and should match the name of the node's source file without the file extension.

These three names may or may not be the same, but they serve different purposes in ROS 2 development.

### Custom ROS message

Custom ROS messages for topics and services can be created.

It seems to be convenient to create teh custom message in the package it is going to be utilized but if in case the message is being is used by other packages and the latter package is dependent on the former package this can lead to cyclic dependencies which is not good.
<br>Moreover having a single package to hold all the custom message for the robot is connivent that to store in individual packages that use them.

As a conventional cmake packages are used to encapsulate custom messages.

The process to do so to create a msg directory within the package and create a new file with .msg or .srv extension based upon usage for topic or services respectively. Build the project and use the message as we use the build in ones.

```python
from package_name.msg import message_name
```

### ros2doctor

We can check ROS2 settings with the ros2doctor tool. ros2doctor checks all aspects of ROS 2, including platform, version, network, environment, running systems and more, and warns us about possible errors and reasons for issues.

```python
ros2 doctor --report
```
for full report.

We can also examine running ROS2 system using doctor.

## Projects

### On going

- Generation of custom ROS Services and Actions
- Visualization of Path-Planning algorithms in Rviz
 
### Completed

- [Mapping-and-Navigation-with-TurtleBot3](https://github.com/maker-ATOM/Mapping-and-Navigation-with-TurtleBot3)
Implementation of robotics concepts like mapping, localization, path planning, navigation and much more..

- [SPARK-A-4DOF-ROS-ROBOTIC-ARM](https://github.com/maker-ATOM/SPARK-A-4DOF-ROS-ROBOTIC-ARM)
Software platform for implementation of mathematical concepts like Forward Kinematics(FK), Inverse Kinematics(IK), Trajectory planning using a Robotic Arm in ROS.

## Resources

- [ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/index.html)
- [ROS2 Tutorials - ROS2 Humble For Beginners](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy) - YouTube
- [UNIX / Linux Tutorial for Beginners](http://www.ee.surrey.ac.uk/Teaching/Unix/)