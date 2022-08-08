FROM osrf/ros:noetic-desktop-full

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update \
    && apt upgrade -y \
    && apt install -y git \
    python3-dev \
    python3-pip

RUN pip3 install conan rosdep\
    && conan config set general.revisions_enabled=1 \
    && conan profile new default --detect > /dev/null \
    && conan profile update settings.compiler.libcxx=libstdc++11 default 

WORKDIR /app

# Create and initialise ROS workspace
RUN mkdir -p ros_ws/src
COPY ./kortex_joystick_demo ros_ws/src/kortex_joystick_demo

RUN cd ros_ws \
    && mkdir build \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && git clone https://github.com/Kinovarobotics/ros_kortex.git src/ros_kortex \
    && git clone https://github.com/husarion/joy2twist.git src/joy2twist \
    && rosdep install --from-paths src --ignore-src -y \
    && catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# Clear 
RUN apt clean \
    && rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh / 
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]