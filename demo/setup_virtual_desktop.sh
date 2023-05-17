#! /bin/bash

# Init virtual desktop
vncserver

# Make desktop accessible
export DISPLAY=:1
xhost local:root
