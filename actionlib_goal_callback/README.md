# Writing a Simple Action Server using the Goal Callback Method

## Init

```bash
$ cd %YOUR_CATKIN_WORKSPACE%/src
$ catkin_create_pkg actionlib_goal_callback actionlib message_generation roscpp rospy std_msgs actionlib_msgs
```

The action messages are generated automatically from the `.action` file

## Creating the Action Messages

```bash
$ catkin_make
$ source devel/setup.sh
$ roscd actionlib_goal_callback
$ mkdir action
```

Create `Averaging.action`

```txt
#goal definition
int32 samples
---
#result definition
float32 mean
float32 std_dev
---
#feedback
int32 sample
float32 data
float32 mean
float32 std_dev
```

## Manually generate message files

```bash
$ roscd actionlib_goal_callback
$ rosrun actionlib_msgs genaction.py -o msg/ action/Averaging.action
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
$ ls devel/share/actionlib_goal_callback/msg/
$ ls devel/include/actionlib_goal_callback/
```

### Writing a Simple Server

Create `actionlib_goal_callback/src/averaging_server.cpp`

Add to `CMakeLists.txt`

```bash
add_executable(
  averaging_server 
    src/averaging_server.cpp
)

target_link_libraries(
  averaging_server
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
$ rosrun actionlib_goal_callback averaging_server
```

Check that action is running properly list topics being published

```bash
$ rostopic list -v
```

Alternatively

```bash
$ rosrun rqt_graph rqt_graph
```

```bash
$ rqt_graph
```

## Writing a Threaded Simple Action Client

Create `actionlib_goal_callback/src/averaging_client.cpp`

Add to `CMakeLists.txt`

```bash
add_executable(
  averaging_client 
    src/averaging_client.cpp
)

target_link_libraries( 
  averaging_client
    ${catkin_LIBRARIES}
)
```

Run the action client

```bash
$ rosrun actionlib_goal_callback averaging_client
```

## Running an Action Client and Server

In a new terminal, rostopic the feedback channel to see the feedback from the action server

```bash
$ rostopic echo /averaging/feedback
```

In a new terminal, rostopic the feedback channel to see the result from the action server:
```bash
$ rostopic echo /averaging/result
```

## Running an Action Server and Client with Other Nodes

Create `actionlib_goal_callback/gen_numbers.py` and make it executable

```bash
$ chmod +x gen_numbers.py
```

Start the node

```bash
$ rosrun actionlib_goal_callback gen_numbers.py
```

## References

* <http://wiki.ros.org/actionlib_tutorials/Tutorials>
* [Writing a Simple Action Server using the Goal Callback Method](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29)
* [Writing a Threaded Simple Action Client](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29)
* [Running an Action Server and Client with Other Nodes](http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClientWithNodes)
