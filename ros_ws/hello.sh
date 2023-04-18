#!/bin/bash
home=`pwd`
dir=$(cd $(dirname $0); pwd)
cd $dir
source ./install/setup.sh
ros2 launch src/maze_solver/launch/hello.xml

cd $home