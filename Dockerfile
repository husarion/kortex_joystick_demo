FROM osrf/ros:noetic-desktop-full

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt install -y \
        git \
        python3-dev \
        python3-pip

RUN pip3 install \
        conan \
        rosdep \
        vcstool && \
    conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default 

WORKDIR /app

# Create and initialise ROS workspace
RUN mkdir -p ros_ws/src

COPY ./kortex_joystick_demo ros_ws/src/kortex_joystick_demo
COPY ./demo.repos /

RUN cd ros_ws && \
    vcs import src < /demo.repos && \
    mv src/ros_kortex/kortex_driver src/kortex_driver && \
    mv src/ros_kortex/kortex_description src/kortex_description && \
    mv src/ros_kortex/kortex_move_it_config src/kortex_move_it_config && \
    rm -rf src/ros_kortex

RUN cd ros_ws && \
    mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]