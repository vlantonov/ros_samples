# Writing a Simple Service and Client (C++)

## Writing a Service Node

Initial setup

```bash
$ mkdir src
$ cd src
$ catkin_create_pkg client_service_cpp std_msgs roscpp
$ catkin_make
$ source devel/setup.bash
$ catkin_make
```

Create service

```bash
$ roscd client_service_cpp
$ mkdir srv
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

Add tp `package.xml`

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Update `client_service_cpp/CMakeLists.txt`:

```bash
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
$ rossrv show client_service_cpp/AddTwoInts
```

Created `src/add_two_ints_server.cpp`

## Writing the Client Node

Create the `src/add_two_ints_client.cpp`

## Building your nodes

Add to `client_service_cpp/CMakeLists.txt`:

```bash
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server client_service_cpp_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client client_service_cpp_gencpp)
```

Run `catkin_make`

## Running the nodes

```bash
$ roscore
```

Run the server in new window

```bash
$ rosrun client_service_cpp add_two_ints_server
```

Run the client in new window after `source devel/setup.sh`

```bash
$ rosrun client_service_cpp add_two_ints_client 1 3
```

## References
* [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
* [Creating the AddTwoInts.srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)