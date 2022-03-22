FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y

# Current package dependencies

# Package is in the current folder
COPY . /ros_ws/src/

WORKDIR /ros_ws/

# Setup the CLI entrypoint
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Script to run server
RUN echo "#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
catkin_make\n\
source ./devel/setup.bash\n\
catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic\n\
roscore&\
rosrun client_server_actionlib_python fibonacci_server.py" >> /ros_ws/run_server.sh\
   && chmod +x run_server.sh

# Scripts to run client
RUN echo "#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
rosrun client_server_actionlib_python fibonacci_client.py" >> /ros_ws/run_client.sh\
   && chmod +x run_client.sh

# Enterpoints to run server and client
ENTRYPOINT ["/bin/bash"]
CMD ["-s"]
