# Convert ROS package to Debian in Docker

## Build image

```bash
$ docker build -t docker_ros_test .
```

## Build Debian package

```bash
$ docker run -it --name docker_ros_test --volume $PWD/client_server_actionlib_python:/ros_ws/src --rm docker_ros_test
```

Generated Debian package is inside `client_server_actionlib_python`.

## Remove Docker image

```bash
$ docker image rm docker_ros_test
```

## Reference

* [Generate deb installable package from ROS Package
](https://medium.com/@freeleons/generate-deb-installable-package-from-ros-package-b095f319b825)
