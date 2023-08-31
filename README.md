# kortex_joystick_demo

This package contains a demo for using the Logitech f710 gamepad to control the Panther robot and Kinova Kortex gen3 manipulator.

---

## Setup Kinova manipulator with Panther

Connect the manipulator to your PC using an Ethernet cable.

1. Enable connection
   On Linux:

   - Go to: **settings -> network -> wired -> advanced -> IPv4**
   - Change IPV4 Method to Manual and set:
   - Address: **192.168.1.11**
   - Netmask: **255.255.255.0**
   - Restart network

2. Open Web browser and go to Kinova Kortex WebApp: [192.168.1.10](http://192.168.1.10)

3. Open a menu and go to **Network -> Ethernet**

4. Change IPv4 address to `10.15.20.4` and IPv4 gateway to: `10.15.20.1`. Save changes

   This will change the default Manipulator IP address from `192.168.1.11` to `10.15.20.4`. To access Kinova Kortex WebApp use a new IP address.

5. Connect manipulator to RUTX11 router in Panther robot with Ethernet cable.

For more info about Kinova Kortex Gen3 refer to [gen3-robots](https://www.kinovarobotics.com/product/gen3-robots)

---

## ROS node API

ROS node is translating the input from the gamepad topic `/joy` to kortex_driver topics responsible for managing the Kinova manipulator.

### Publish

- `/in/cartesian_velocity` _(kortex_driver/TwistCommand)_
- `/in/emergency_stop` _(std_msgs/Empty)_
- `/in/clear_faults` _(std_msgs/Empty)_

### Subscribe

- `/joy` _(sensor_msgs/Joy)_
- `/base_feedback` _(kortex_driver/BaseCyclic_Feedback)_

### Parameters

The following parameters change velocity limits for the Kinova manipulator and gripper closed position.

- `~max_linear_vel` _(float, default: 0.2)_
- `~min_linear_vel` _(float, default: 0.05)_
- `~max_angular_vel` _(float, default: 0.5)_
- `~min_angular_vel` _(float, default: 0.05)_
- `~gripper_closed_position` _(float, default: 0.8)_

---

## Controlling Kinova manipulator and mobile robot

By default, after a successful launch, the manipulator will be controlled by the gamepad. It is assumed that the joy interface has already been launched. In case of any issues, please follow the instructions under [joy2twist](https://github.com/husarion/joy2twist/tree/ros1) To start driving the robot and disable the manipulator press the **LB button** (disable movement). To switch back to using the manipulator press the **A button** (enable movement).

### Manipulator button mapping

|      Button      |           Function           |
| :--------------: | :--------------------------: |
|   `LEFT STICK`   | manipulator steering - `X Y` |
|  `RIGHT STICK`   |  manipulator steering - `Z`  |
|  `UP CROSS KEY`  |        increase speed        |
| `DOWN CROSS KEY` |        decrease speed        |
|       `A`        |       enable movement        |
|       `B`        |        emergency stop        |
|       `X`        |         linear mode          |
|       `Y`        |         angular mode         |
|       `LB`       |       disable movement       |
|       `LT`       |        close gripper         |
|       `RT`       |         open gripper         |

To move the manipulator use sticks. By default, cartesian _linear/angular_ movement in `X` and `Y` is held by the left stick and _linear/angular_ `Z` movement is controlled with the right stick.

![buttons](docs/gamepad_buttons.png)

### Robot button mapping

|    Button     |            Function             |
| :-----------: | :-----------------------------: |
| `LEFT STICK`  | velocity steering - angular `Z` |
| `RIGHT STICK` | velocity steering - linear `X`  |
|     `LB`      |         enable driving          |
|     `RB`      |        slow driving mode        |
|     `RT`      |        fast driving mode        |
|      `A`      |          Reset E-stop           |
|      `B`      |         Trigger E-stop          |
|     `LT`      |       Enable E-stop reset       |

If neither `RB` nor `RT` is pressed, the robot operates in _regular_ driving mode.

---

## Running on Panther robot

1. Access NUC via ssh

   ```bash
   ssh husarion@10.15.20.3
   ```

   The default password is `husarion`

2. Clone this repo

   ```bash
   cd ~/husarion_ws/src/
   git clone https://github.com/husarion/kortex_joystick_demo.git
   ```

3. Setup a gamepad according to [Setup joy](#setup-joy)

4. Setup virtual desktop

   ```bash
   cd kortex_joystick_demo/
   source ./kortex_joystick_demo/scripts/setup_virtual_desktop.sh
   ```

5. Launch demo

   ```bash
   roslaunch kortex_joystick_demo kortex_joystick_demo.launch
       robot_ip_address:=10.15.20.4
   ```

After a successful launch, open the Web browser and go to [10.15.20.3:8080](http://10.15.20.3:8080/vnc_auto.html). Enter a password (default `husarion`) and you should see RViz and Panther with the manipulator.

---

## Docker image

[![Build/Publish Docker Image](https://github.com/husarion/kortex_joystick_demo/actions/workflows/build-docker-image.yaml/badge.svg)](https://github.com/husarion/kortex_joystick_demo/actions/workflows/build-docker-image.yaml)

| ROS distro | Supported architectures      |
| ---------- | ---------------------------- |
| `noetic`   | `linux/amd64`, `linux/arm64` |

Available on [Docker Hub](https://hub.docker.com/r/husarion/kortex-joystick_demo/tags)

### Demo

#### Panther robot with Kinova manipulator installed on the top

1. Clone this repo on your Panther:

   ```bash
   git clone https://github.com/husarion/kortex_joystick_demo.git
   ```

2. Launch on Panther

   Run demo:

   ```bash
   cd kortex_joystick_demo/demo
   xhost local:docker
   docker compose up
   ```

3. Use Logitech gamepad to control either Panther or Kinova [Instruction](#controlling-kinova-manipulator-and-mobile-robot).
