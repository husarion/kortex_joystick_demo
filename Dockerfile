FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt install --no-install-recommends -y \
        git \
        build-essential \
        python3-dev \
        python3-pip && \
    pip3 install \
        conan==1.59 \
        rosdep \
        vcstool && \
    conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

COPY ./demo.repos /

# import required packages
RUN mkdir src && vcs import src < /demo.repos &&\
    mv src/panther_ros/panther_description src/panther_description && \
    rm -rf src/panther_ros && \
    mv src/ros_kortex/kortex_driver src/kortex_driver && \
    mv src/ros_kortex/kortex_description src/kortex_description && \
    mv src/ros_kortex/kortex_move_it_config src/kortex_move_it_config && \
    rm -rf src/ros_kortex

# Create and initialise ROS workspace
COPY ./kortex_joystick_demo src/kortex_joystick_demo

# install and build
RUN apt-get update && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y -i --skip-keys gazebo --skip-keys rviz && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
