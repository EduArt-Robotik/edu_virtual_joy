FROM eduartrobotik/eduart-robot:0.5.0

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install mraa lib for UART communication in shield node
USER root
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3.12-venv

USER $USER
RUN echo "Installing flet" \ 
    && mkdir /home/$USER/python_env \
    && cd /home/$USER/python_env \
    && python3 -m venv .flet \
    && source .flet/bin/activate \
    && pip3 install flet setuptools \
    && pip install 'flet[all]==0.25.1' --upgrade

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_virtual_joy /home/$USER/ros/src/edu_virtual_joy
WORKDIR /home/$USER/ros

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/python_env/.flet/bin/activate \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
