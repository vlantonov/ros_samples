# Writing a Simple Service and Client (Python)

## Writing a Service Node

Initial setup

```bash
$ mkdir src
$ cd src
$ catkin_create_pkg client_service_python std_msgs rospy
$ catkin_make
$ source devel/setup.bash
$ catkin_make
```

Create service

```bash
$ roscd client_service_python
$ mkdir srv
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

Add to `package.xml`

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Update `client_service_python/CMakeLists.txt`:

```bash
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

Check message is generated

```bash
$ rossrv show client_service_python/AddTwoInts
```

Created `scripts/add_two_ints_server.py`

## Writing the Client Node

Created `scripts/add_two_ints_client.py`

## Building your nodes

Add to `client_service_python/CMakeLists.txt`:

```bash
catkin_install_python(PROGRAMS 
    scripts/add_two_ints_server.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client client_service_python_gencpp)
```

Run `catkin_make`

## Running the nodes

```bash
$ roscore
```

Run the server in new window

```bash
$ rosrun client_service_python add_two_ints_server.py
```

Run the client in new window after `source devel/setup.sh`

```bash
$ rosrun client_service_python add_two_ints_client.py 1 3
```

## References
* [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
* [Creating the AddTwoInts.srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)
* [Examining the Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)
