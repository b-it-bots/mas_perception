FROM bitbots/bitbots-common:kinetic

LABEL maintainer="Argentina Ortega"

WORKDIR /travis
COPY mas-perception.rosinstall /travis
COPY ./ /travis/src/mas_perception

RUN wstool init --shallow src && \
    wstool merge -t src mas-perception.rosinstall && \
    cd src && \
    wstool remove mas_perception && \
    cd - && \
    wstool update -t src


RUN . /opt/ros/mas_stable/setup.sh && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    apt-get update -qq && \
    rosdep update -qq && \
    rosdep install -q --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build --no-status
