FROM eduartrobotik/eduart-robot:0.4.1

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# install mraa lib for UART communication in shield node
USER root
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
    && pip3 install flet

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_virtual_joy /home/$USER/ros/src/edu_virtual_joy
WORKDIR /home/$USER/ros

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
