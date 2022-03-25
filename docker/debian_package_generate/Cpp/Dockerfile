FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y

# Current package dependencies

# Dependencies to create Debian package
RUN apt-get install python3-bloom -y\
    && apt-get install fakeroot -y\
    && apt-get install dh-make -y

COPY . /ros_ws/src/

WORKDIR /ros_ws/

# Script to create Debian package
RUN echo "#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
bloom-generate rosdebian --ros-distro noetic\n\
cd src\n\
fakeroot debian/rules binary\n\
rm -rf obj-x86_64-linux-gnu/\n\
rm -rf debian/\n\
cp ../ros-noetic-client-server-actionlib-cpp_0.0.0-0focal_amd64.deb ." >> /ros_ws/create_package.sh\
   && chmod +x create_package.sh

# Create Debian package on container run
ENTRYPOINT ["./create_package.sh"]
CMD ["-s"]
