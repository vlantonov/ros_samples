# Writing a Simple Action Server using the Execute Callback (C++)

## Init

```bash
$ cd %YOUR_CATKIN_WORKSPACE%/src
$ catkin_create_pkg actionlib_callback_based actionlib message_generation roscpp std_msgs actionlib_msgs
```

The action messages are generated automatically from the `.action` file

## Creating the Action Messages

```bash
$ source devel/setup.sh
$ roscd actionlib_callback_based
$ mkdir action
```

Create `Fibonacci.action`

```txt
#goal definition
int32 order
---
#result definition
int32[] sequence
---
#feedback
int32[] sequence
```

Update `CMakeLists.txt`
```bash
add_action_files(
  DIRECTORY 
    action
  FILES
    Fibonacci.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs   
    std_msgs
)
```

Add to `CMakeLists.txt`
```bash
catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
)
```

```bash
# Go back to the top level of your catkin workspace
$ catkin_make
$ ls devel/share/actionlib_callback_based/msg/
$ ls devel/include/actionlib_callback_based/
```

### Writing a Simple Server

Create `actionlib_callback_based/src/fibonacci_server.cpp`

Add to `CMakeLists.txt`

```bash
add_executable(
  fibonacci_server 
    src/fibonacci_server.cpp
)

target_link_libraries(
  fibonacci_server
    ${catkin_LIBRARIES}
)
```

### Running the Action server

Run the action server

Start a roscore in a new terminal

```bash
$ roscore
```

Run the action server

```bash
$ rosrun actionlib_callback_based fibonacci_server
```

Check that action is running properly list topics being published

```bash
$ rostopic list -v
```

## Writing a Simple Action Client (C++)

Create `actionlib_callback_based/src/fibonacci_callback_client.cpp`

Add to `CMakeLists.txt`

```bash
add_executable(
  fibonacci_callback_client 
    src/fibonacci_callback_client.cpp
)

target_link_libraries( 
  fibonacci_callback_client
    ${catkin_LIBRARIES}
)
```

Run the action client

```bash
$ rosrun actionlib_callback_based fibonacci_callback_client
```

## Writing a Class Client (C++)

Create `actionlib_callback_based/src/fibonacci_class_client.cpp`

Add to `CMakeLists.txt`

```bash
add_executable(
  fibonacci_class_client 
    src/fibonacci_class_client.cpp
)

target_link_libraries( 
  fibonacci_class_client
    ${catkin_LIBRARIES}
)

```

Run the action client

```bash
$ rosrun actionlib_callback_based fibonacci_class_client
```

## Running an Action Client and Server

In a new terminal, rostopic the feedback channel to see the feedback from the action server

```bash
$ rostopic echo /fibonacci/feedback
```

In a new terminal, rostopic the feedback channel to see the feedback from the action server:
```bash
$ rostopic echo /fibonacci/result
```

## References

* <http://wiki.ros.org/actionlib_tutorials/Tutorials>
* [Writing a Simple Action Server using the Execute Callback in C++](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
* [Writing a Simple Action Client in C++](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient)
* [Running an Action Client and Server](http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClient)
* [Writing a Callback Based SimpleActionClient](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client)
