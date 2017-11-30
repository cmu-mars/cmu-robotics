FROM cmu-mars/base


RUN sudo apt-get update
RUN sudo apt-get install ros-kinetic-gazebo-msgs

ENV ROS_NAVIGATION_MSGS_VERSION 1.13.0
RUN wget -q "https://github.com/ros-planning/navigation_msgs/archive/${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    tar -xvf "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    rm "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    mv "navigation_msgs-${ROS_NAVIGATION_MSGS_VERSION}" navigation_msgs && \
    rm navigation_msgs/README.md && \
    mv navigation_msgs/* src && \
    rm -rf navigation_msgs
    
RUN sudo apt-get install -y python3 python3-pip

RUN sudo mkdir -p /usr/src/app
RUN sudo chown mars /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN sudo pip3 install --no-cache-dir -r requirements.txt

RUN sudo pip3 install catkin_pkg rospkg


COPY . /usr/src/app

RUN sudo chown -R mars /usr/src/app

ENV PYTHONUNBUFFERED 0

EXPOSE 5000

CMD ["python3", "-m", "swagger_server"]
