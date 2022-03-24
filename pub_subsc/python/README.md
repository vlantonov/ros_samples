
# Writing a Simple Publisher and Subscriber (Python)

## Writing the Publisher Node

```bash
$ mkdir src
$ cd src
$ catkin_create_pkg pub_subsc_python std_msgs roscpp
$ source devel/setup.bash
$ catkin_make
```

Change directory to created package:

```bash
$ roscd pub_subsc_python
```

Create folder to store scripts:

```bash
$ mkdir scripts
$ cd scripts
```

Create `talker.py` - code from
<https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py>

Create the `listener.py` - code from <https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py>

Add at end of `pub_subsc_python/CMakeLists.txt`:

```bash
catkin_install_python(
  PROGRAMS 
    scripts/talker.py
    scripts/listener.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Make the project with

```bash
$ catkin_make
```

Run `talker`:

```bash
$ rosrun pub_subsc_python talker.py 
```

Run `listener` in separate window:

```bash
rosrun pub_subsc_python listener.py
```

## References

* [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
