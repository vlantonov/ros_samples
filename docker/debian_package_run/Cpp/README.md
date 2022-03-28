# Run ROS package in Docker

## Build image

The ROS package is installed from `ros-noetic-client-server-actionlib-cpp_0.0.0-0focal_amd64.deb` Debian package

```bash
$ docker build --build-arg deb_name=ros-noetic-client-server-actionlib-cpp_0.0.0-0focal_amd64.deb -t docker_ros_test .
```

## Start server

Original command
`rosrun client_server_actionlib_cpp fibonacci_server`
is executed using prepared as entrypoint bash script:

```bash
$ docker run -it --name docker_ros_test --rm docker_ros_test ./run_server.sh
```

## Start client

The Docker container with surver must be running.

Original command
`rosrun client_server_actionlib_cpp fibonacci_client`
is executed using prepared as entrypoint bash script:

```bash
docker exec -it docker_ros_test ./run_client.sh
```

## Stop server

```bash
docker stop docker_ros_test
```

## Remove Docker image

```bash
$ docker image rm docker_ros_test
```

## Reference

* [Generate deb installable package from ROS Package
](https://medium.com/@freeleons/generate-deb-installable-package-from-ros-package-b095f319b825)
