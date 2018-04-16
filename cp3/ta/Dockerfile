FROM cmu-mars/cp3_base

RUN . ~/catkin_ws/devel/setup.sh

RUN echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo ". ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN sudo apt-get install ca-certificates
ENV GPG_KEY=0D96DF4D4110E5C43FBFB17F2D347EA6AA65421D
ENV PYTHON_VERSION=3.6.3

RUN sudo apt-get update
#    runDeps="$(   scanelf --needed --nobanner --recursive /usr/local    | awk '{ gsub(/,/, "\nso:", $2); print "so:" $2 }'    | sort -u    | xargs -r apt info --installed    | sort -u  )"  && \
#    sudo apt-get install -y $runDeps  && \
# install Gazebo

# Do we need all this below?
#RUN sudo apt-get install -y ros-kinetic-gazebo-ros-pkgs \
#                            ros-kinetic-gazebo-ros-control \
#                            ros-kinetic-kobuki-gazebo \
#                            apt-utils

RUN sudo apt-get install ros-kinetic-gazebo-msgs

ENV ROS_NAVIGATION_MSGS_VERSION 1.13.0
RUN wget -q "https://github.com/ros-planning/navigation_msgs/archive/${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    tar -xvf "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    rm "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    mv "navigation_msgs-${ROS_NAVIGATION_MSGS_VERSION}" navigation_msgs && \
    rm navigation_msgs/README.md && \
    mv navigation_msgs/* src && \
    rm -rf navigation_msgs

RUN sudo apt-get install -y python3

RUN sudo apt-get install -y python3-pip

RUN sudo mkdir -p /usr/src/app
RUN sudo chown mars /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN sudo pip3 install --no-cache-dir -r requirements.txt

RUN sudo pip3 install catkin_pkg rospkg numpy

COPY . /usr/src/app

RUN sudo chown -R mars /usr/src/app

ENV PYTHONUNBUFFERED 0

EXPOSE 8080

CMD ["python3", "-m", "swagger_server"]
