# rws2019_tmadeira
ROS WORKSHOP 2019\
Contains work developed in the scope of a crash course of ROS in LAR at University of Aveiro.

---

## ROS Crash Course Cheat Sheet (Notebook):

### Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
```

### Navigate

* rospack = ros + pack(age)
* roscd = ros + cd
* rosls = ros + ls 

---

### Create a catkin package
Run this within the workspace (e.g. ```~/catkin_ws/src```):
```
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

### Build the packages in the catkin workspace
```
cd ~/catkin_ws
catkin_make
```

### Source the generated file
```
. ~/catkin_ws/devel/setup.bash
```

### Check first order dependencies
```
rospack depends1 <package_name> 
```

### Customize Your Packages
Change items in ```package.xml``` and ```CMakeLists.txt``` 

---

### ROS nodes
* roscore = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server (parameter server will be introduced later)
    * ```roscore``` to start service
* rosnode = ros+node : ROS tool to get information about a node.
    * ```rosnode list``` to show nodes currently running
    * ```rosnode info /<node_name>``` to show info about  a specific node
* rosrun = ros+run : runs a node from a given package.
    * ```rosrun [package_name] [node_name]```

---

### Visualize ROS topics
Dynamic graph showcasing the inner workings of the system:
```
rosrun rqt_graph rqt_graph
```
Scrolling time plot of the data published on topics:
```
rosrun rqt_plot rqt_plot
```

### Get information about and interact with **ROS topics**
**publish–subscribe** pattern	
```
rostopic -h
```

---

### Get information about and interact with **ROS services**
**request-response** pattern
```
rosservice -h
```

---

### Store and manipulate data on the ROS Parameter Server
The Parameter Server can store integers, floats, boolean, dictionaries, and lists. It uses the YAML markup language for syntax.
```
rosparam -h
```

---

### Use rqt_console and rqt_logger_level
rqt_console attaches to ROS's logging framework to display output from nodes.\
rqt_logger_level allows us to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run.

```
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```

---

### Start nodes as defined in launch file
File should be created at ```<package_name>/lauch/```
```
roslaunch [package] [filename.launch]
```

---

### Use rosed
Edit a file within a package without the use of the entire path.
```
rosed [package_name] [filename]
```

---

### Create messages and services
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv


### Get information about message types
```
rosmsg show [message type]
```
Where ```[message type]``` is ```<package_name/msg_file_name>```.\
Works without package name, but this way it's less ambiguous.

### Get information about service types
```
rossrv show [service type]
```
Where ```[service type]``` is ```<package_name/service_file_name>```.\
Works without package name, but this way it's less ambiguous.

---

### Use roswtf
roswtf examines your system to try and find problems. It uses whatever your current directory is to determine what checks it does.\
```
roswtf
```