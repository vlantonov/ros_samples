# Writing a Simple Action Server using the Execute Callback (Python)

## Init

```bash
$ cd %YOUR_CATKIN_WORKSPACE%/src
$ catkin_create_pkg client_server_actionlib_python actionlib message_generation roscpp rospy std_msgs actionlib_msgs
```

The action messages are generated automatically from the `.action` file

## Creating the Action Messages

```bash
$ source devel/setup.sh
$ roscd client_server_actionlib_python
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
$ ls devel/share/client_server_actionlib_python/msg/
$ ls devel/include/client_server_actionlib_python/
```

## Writing a Simple Server

Create `client_server_actionlib_python/fibonacci_server.py`

## Running the Action server

Run the action server

Start a roscore in a new terminal

```bash
$ roscore
```

Run the action server

```bash
$ rosrun actionlib_tutorials fibonacci_server.py
```

Check that action is running properly list topics being published

```bash
$ rostopic list -v
```

## Writing a Simple Action Client (Python)

Create `client_server_actionlib_python/fibonacci_client.py`

Run the action client

```bash
$ rosrun actionlib_tutorials fibonacci_client.py
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
* [Writing a Simple Action Server using the Execute Callback in Python](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29)
* [Writing a Simple Action Client in Python](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29)
* [Running an Action Client and Server](http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClient)