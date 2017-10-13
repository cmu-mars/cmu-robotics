# The base Dockerfile used by all of the (executable) components of the Phase
# 1.5 implementation of MARS.
FROM ros:kinetic

# create a "mars" user with sudo privileges
# TODO: add tidying commands to shrink image size
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y sudo && \
    useradd -ms /bin/bash mars && \
    usermod -a -G sudo mars && \
    sed -i "s/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/" "/etc/sudoers" && \
    mkdir -p /home/mars
USER mars
WORKDIR /home/mars

RUN git config --global url.https://github.com/.insteadOf git://github.com/

# create an empty catkin workspace
RUN . /opt/ros/kinetic/setup.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    catkin_make
WORKDIR /home/mars/catkin_ws

# install some basic utilities
RUN sudo apt-get update && \
    sudo apt-get install -y vim wget curl

# install Kobuki (does this really belong here?)
RUN sudo apt-get install -y ros-kinetic-kobuki ros-kinetic-kobuki-core

# install the maps
ENV CP_MAPS_REV 80afb8f
RUN git clone https://github.com/cmu-mars/cp-maps-p15 src/maps && \
    cd src/maps && \
    git reset --hard "${CP_MAPS_REV}"
    
# install shared "models" module
ENV MARS_MODELS_REVISION 5dd3542
RUN git clone https://github.com/cmu-mars/cp-models-p15 \
              src/cp-models-p15 && \
    cd src/cp-models-p15 && \
    git checkout "${MARS_MODELS_REVISION}"

# add the entrypoint script
ADD entrypoint.sh entrypoint.sh
ENTRYPOINT ["/home/mars/catkin_ws/entrypoint.sh"]

CMD ["/bin/bash"]
