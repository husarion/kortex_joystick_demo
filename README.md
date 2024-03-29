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
| `LEFT STICK`  | velocity steering - angular `X` |
| `RIGHT STICK` | velocity steering - linear `Z`  |
|     `LB`      |         enable driving          |
|     `RB`      |        slow driving mode        |
|     `RT`      |        fast driving mode        |
|      `A`      |          Reset E-stop           |
|      `B`      |         Trigger E-stop          |
|     `LT`      |       Enable E-stop reset       |

If neither `RB` nor `RT` is pressed, the robot operates in _regular_ driving mode.

---

## Docker image

[![Build/Publish Docker Image](https://github.com/husarion/kortex_joystick_demo/actions/workflows/build-docker-image.yaml/badge.svg)](https://github.com/husarion/kortex_joystick_demo/actions/workflows/build-docker-image.yaml)

| ROS distro | Supported architectures |
| :--------: | :---------------------: |
|  `noetic`  |      `linux/amd64`      |

Available on [Docker Hub](https://hub.docker.com/r/husarion/kortex-joystick/tags)

### Demo

#### Running on Panther's User Computer

1. Access User Computer via ssh

   ```bash
   ssh husarion@10.15.20.3
   ```

   The default password is `husarion`

2. Clone this repo on your Panther:

   ```bash
   git clone https://github.com/husarion/kortex_joystick_demo.git
   ```

3. If you are using Kinova Gen3 Lite, edit [.env](./demo/.env) file and change `KINOVA_ARM_MODEL` to `gen3_lite`.

4. Launch the demo

   Setup virtual desktop and run demo:

   ```bash
   cd kortex_joystick_demo/demo
   source ./setup_virtual_desktop.sh
   docker compose \
      -f compose.ouster.yaml \
      -f compose.kinova.yaml \
      -f compose.rviz.yaml \
      -f compose.vnc.yaml \
      up
   ```

5. Use Logitech gamepad to control either Panther or Kinova Manipulator ([Instruction](#controlling-kinova-manipulator-and-mobile-robot)).

#### Running on custom computer with gamepad

> **Note:**
> Gamepad docker is launched by default on the Panhter robot. To use it directly from your PC, you have to disable it on the robot first. See [Disable Gamepad on the Panther robot](#disable-gamepad-on-the-panther-robot) section for instructions.


1. Clone this repo

   ```bash
   git clone https://github.com/husarion/kortex_joystick_demo.git
   ```

2. Setup network (for this step you need to know your computer IP in Panthers network, you can check it using the `ifconfig` command). Then, edit [net.env](./demo/net.env) file and change `ROS_IP`. Example IP address can be: `10.15.20.147`.

3. If you are using Kinova Gen3 Lite, edit [.env](./demo/.env) file and change `KINOVA_ARM_MODEL` to `gen3_lite`.

4. Launch the demo

   ```bash
   cd kortex_joystick_demo/demo
   xhost local:docker
   docker compose \
      -f compose.ouster.yaml \
      -f compose.kinova.yaml \
      -f compose.rviz.yaml \
      -f compose.gamepad.yaml \
      up
   ```

5. Use Logitech gamepad to control either Panther or Kinova Manipulator ([Instruction](#controlling-kinova-manipulator-and-mobile-robot)).

#### Disable Gamepad on the Panther robot

1. Log into Built-in Computer:

```
ssh husarion@10.15.20.2
```

2. Edit `compose.yaml` file with command below and comment out or remove `gamepad_controller` service section:

```
nano compose.yaml
```

3. Restart Docker:

```
docker compose up -d --force-recreate
```
