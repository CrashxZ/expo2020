# Expo-2020 (http://ai.tsu.ru/page14803029.html)


### Recommended Software (Links and instructions to the above mentioned software are given below):
1. Ubuntu 18.04
2. Robot Operating System(ROS) - Melodic
3. Ardupilot Repository
4. ardupilot_gazeebo repository
5. Git (sudo apt-get install git)
6. pymavlink (pip install pymavlink)
 
 

 #### Ubuntu 18.04
```
 https://releases.ubuntu.com/18.04/
```


 #### Robot Operating System(ROS) - Melodic
```
http://wiki.ros.org/melodic/Installation/Ubuntu
```
Install the ros melodic full desktop.


 #### Ardupilot Repository
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board CubeBlack
./waf copter
```

Reffer bellow for links, and troubleshooting guide :
1. https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
2. https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
3. https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html

 #### ardupilot_gazeebo repository
```
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
Set Path of Gazebo Models / Worlds... Open up .bashrc
```
sudo gedit ~/.bashrc
```
Copy & Paste following at the end of .bashrc file 
*Use the correct path, will varry on the installation location*
```
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```
Install is complete

Reffer bellow for links, and troubleshooting guide :
1. https://github.com/SwiftGust/ardupilot_gazebo
2. https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
3. https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html

#### Code for the session example can be found in the /mav_track folder

