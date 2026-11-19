#!/bin/zsh -il
# Double-click this in Finder if Processing can't use Bluetooth.
# It runs the robot link in Terminal, which macOS lets use Bluetooth.
cd "$(dirname "$0")"
python3 -u robot_link.py
