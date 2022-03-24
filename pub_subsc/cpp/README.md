
# Writing a Simple Publisher and Subscriber (C++)

## Writing the Publisher Node

```bash
$ mkdir src
$ cd src
$ catkin_create_pkg pub_subsc_cpp std_msgs roscpp
$ source devel/setup.bash
$ catkin_make
```

Create `src\talker.cpp` - code from
<https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp>

Create the `src\listener.cpp` - code from <https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp>

Update `pub_subsc_cpp/CMakeLists.txt`:

```bash
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

Add at end of `pub_subsc_cpp/CMakeLists.txt`:

```bash
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker pub_subsc_cpp_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener pub_subsc_cpp_generate_messages_cpp)
```

Make the project with

```bash
$ catkin_make
```

Add `--force-cmake` if needed.

Run `talker`:

```bash
$ rosrun pub_subsc_cpp talker
```

Run `listener` in separate window:

```bash
rosrun pub_subsc_cpp listener
```

## References

* [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
* [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
