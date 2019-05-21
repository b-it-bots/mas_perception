FROM bitbots/bitbots-common:kinetic

LABEL maintainer="Argentina Ortega"

WORKDIR /kinetic
COPY mas-perception.rosinstall /kinetic

RUN wstool init --shallow src && \
    wstool merge -t src mas-perception.rosinstall && \
    cd src && \
    wstool remove mas_perception && \
    cd - && \
    wstool update -t src


COPY . /kinetic/src/mas_perception

RUN . /opt/ros/mas_stable/setup.sh && \
    apt-get update -qq && \
    rosdep update -q && \
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    rm -rf /kinetic/

WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
