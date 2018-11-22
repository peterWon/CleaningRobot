#!/bin/sh
cd $ROOT_WS
mkdir src && cd src
git clone https://github.com/pirobot/rbx1.git
git clone https://github.com/peterWon/CleaningRobot.git
cd $ROOT_WS/src/rbx1 && git checkout kinetic-devel-beta
cd $ROOT_WS/src/CleaningRobot
cd $ROOT_WS && catkin_make
