FROM osrf/ros:foxy-desktop

ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y \
	python3-colcon-common-extensions \
	gazebo11 \
	libgazebo11 \
	libgazebo11-dev \
	ros-foxy-gazebo-ros-pkgs \
	ros-foxy-cv-bridge \
	ros-foxy-camera-calibration-parsers \
	libasio-dev \
	python3-pip \
	vim \
	&& rm -rf /var/lib/apt/lists/*

RUN pip install opencv-contrib-python numpy transformations

RUN mkdir -p /root/tello_ros_ws/src

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "source /root/tello_ros_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/tello_ros_ws

CMD ["/bin/bash"]
