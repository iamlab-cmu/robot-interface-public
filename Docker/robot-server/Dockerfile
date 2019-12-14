FROM ubuntu:16.04

RUN apt-get update

# Install ROS
COPY .docker_util_scripts/install_ros.sh /
RUN bash /install_ros.sh
RUN rm /install_ros.sh

# # Install packages
RUN apt-get install -y git iputils-ping

RUN apt-get install -y cmake build-essential
RUN apt-get install -y libeigen3-dev libboost-all-dev
RUN apt-get install -y libpoco-doc libpoco-dev

# # ======== Start: Block that requires SSH Key ==========
ARG SSH_KEY

RUN mkdir /root/.ssh/ && chmod 0700 /root/.ssh
RUN echo "${SSH_KEY}" > /root/.ssh/id_rsa && chmod 0600 /root/.ssh/id_rsa
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

RUN git clone --recurse-submodules git@github.com:iamlab-cmu/robot-interface.git

RUN rm -f /root/.ssh/id_rsa
# # ======== End: Block that requires SSH Key ==========

COPY .docker_util_scripts/build_robot_interface.sh /
RUN bash /build_robot_interface.sh
RUN rm /build_robot_interface.sh

COPY .docker_util_scripts/build_catkin.sh /
RUN bash /build_catkin.sh
RUN rm /build_catkin.sh

COPY run_robot_interface_and_ros.sh /
CMD bash /run_robot_interface_and_ros.sh