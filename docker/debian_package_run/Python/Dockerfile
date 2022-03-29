FROM ros:noetic

ARG deb_name=ros-noetic-client-server-actionlib-python_0.0.0-0focal_amd64.deb

ENV DEBIAN_FRONTEND=noninteractive

# RUN apt-get update && apt-get install -y

COPY . /tmp

WORKDIR /tmp

# Install dependent packages
RUN apt-get install -f -y /tmp/${deb_name}

# Scripts to run server and client
RUN echo "#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
roscore&\
rosrun client_server_actionlib_python fibonacci_server.py" >> /tmp/run_server.sh\
   && chmod +x run_server.sh

RUN echo "#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
rosrun client_server_actionlib_python fibonacci_client.py" >> /tmp/run_client.sh\
   && chmod +x run_client.sh

# Enterpoints to run server and client
ENTRYPOINT ["/bin/bash"]
CMD ["-s"]
