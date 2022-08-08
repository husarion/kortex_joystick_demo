# kortex_joystick_demo

This package contains demo for using Logitech f710 gamepad to controll panther robot and kinova kortex gen3 manipulator.

## Setup joy

Connect joy via nano USB receiver and make sure it is in **DirectInput Mode** (switch in front of the pad with letters **D** and **X**, select **D**).

To test if joy works, use `jstest /dev/input/js0`.
If the output is:

```
jstest: No such file or directory
```

See `ls /dev/input | grep js` and find your joy number. If it differs, apply changes in *compose.yaml* and launch file.

## Running demo

1. Clone this repo

```bash
git clone https://github.com/husarion/kortex_joystick_demo.git
```

2. Launch demo

```bash
cd kortex_joystick_demo
docker-compose up --build
```

## Controlling kinova manipulator

By default, manipulator will be controlled by gamepad. To start driving and disable manipulator hold `LB` button.

### Manipulator button mapping

|  Button  |      Function      |
|:--------:|:------------------:|
|   `LB`   |   disable driving  |
|   `LT`   |    close gripper   |
|   `RT`   |    open gripper    |
|   `B`    |   emergency stop   |
|   `A`    |    clear faults    |
|   `X`    |    linear mode     |
|   `Y`    |    angular mode    |
|   `UP CROSS KEY`    |    increase speed    |
|   `DOWN CROSS KEY`  |    decrease speed    |


To move manipulator use sticks.
By default, cartesian linear/angular movement in `X` and `Y` are held by left stick and linear/angular `Z` movement is controlled with right stick.
To switch between *linear* and *angular* mode use *X* and *Y* buttons 

(see [Manipulator button mapping](#manipulator-button-mapping)


## Controlling robot

### Robot button mapping

|  Button  |      Function      |
|:--------:|:------------------:|
|   `LB`   |   enable driving   |
|   `RB`   | slow driving mode  |
|   `RT`   |  fast driving mode |

If neither `RB` nor `RT` is pressed, the robot operates in *regular* driving mode.

To drive robot use sticks.
By default, linear `X` and `Y` are held by the right stick. Angular `Z` is controlled with the left stick.