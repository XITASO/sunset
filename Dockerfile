FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ARG USERID=1001
ENV ROS_DISTRO=humble

USER root

# Add ROS repository and update the GPG keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update \
    && useradd -rm -d /home/dockuser -s /bin/bash -g root -G sudo -u ${USERID} dockuser -p "$(openssl passwd -1 dockuser)" \
    && apt-get install -y python3-pip \
    && apt-get install -y ros-${ROS_DISTRO}-cv-bridge \
    && apt-get clean

RUN pip install --no-cache-dir \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    torch 

RUN pip install --no-cache-dir torchvision==0.21.0

RUN pip install pandas numpy==1.23.*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/dockuser/.bashrc

ADD ./ros_ws /ros_ws

WORKDIR /ros_ws

CMD ["--tail-log"]
