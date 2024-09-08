# Low-level-control-of-PX4-using-ROS2-
# PX4 and ROS 2 setup
  This repo is derived from the following repo of AYHAM ALHARBAT of University of SAXION https://github.com/SaxionMechatronics/px4_offboard_lowlevel/tree/main
# Prerequisites 
- Ubuntu 22.04
- PX4
- ROS2 HUMBLE
- GAZEBO CLASSIC
- Micro XRCE-DDS Agent & Client



# Install PX4

1. [Clone PX4 Source code](https://github.com/PX4/PX4-Autopilot)

   - Clone into any directory 

```bash
    # for example into ~/llc
    cd ~/llc

    # clone
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

2. Run the ubuntu.sh with no arguments (in a bash shell) to install everything

```bash
    cd ~/llc
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
3. Restart the system

4. Verify the [NuttX](https://nuttx.apache.org/) installation
```bash
    arm-none-eabi-gcc --version

    # The following output will be shown 
    arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
    Copyright (C) 2019 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

# ROS2 With PX4
For more information on ROS 2 refer to the [ROS 2 Documentation](https://docs.ros.org/en/humble/)

1. [Install PX4](#install-px4) 
   - As explained above

2. [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Follow this [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2
   - Version : Humble
   - Ubuntu Version : [Jammy Jellyfish (22.04)](https://releases.ubuntu.com/jammy/) 

3. Install Python dependencies 
```bash
    pip install --user -U empy pyros-genmsg setuptools

    # if pip is not installed then:
    sudo apt update
    
    # Install pip for Python 3
    sudo apt install python3-pip
    
    # verify the install
    pip3 --version

    # upgrade pip3 to the latest version
    sudo pip3 install --upgrade pip
```

4. Setup Micro XRCE-DDS Agent & Client
```bash
    # change the diectory
    cd ~/llc

    # Fetch and build the agent
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
```
5. Start the agent
```bash
    # start the micro xrcedds agent in a new terminal window using this command line below
    MicroXRCEAgent udp4 -p 8888
```

6. Start the simulation
```bash
    # change directory to PX4
    cd ~/llc/PX4-Autopilot

    # Start a PX4 Gazebo simulation using:
    make px4_sitl gz_x500

    # The agent and client are now running they should connect.
```

# Troubleshooting

1. Unknown target 'gz_x500'
```bash
    # change directory 
    cd ~/llc/PX4-Autopilot

    # run this command
    make clean
    make distclean
```
# While installing PX4 Gazebo Garden will be Automatically Installed so now uninstall it and install Gazebo classic

```bash
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```
Note that aptitude is needed because it can resolve dependency conflicts (by removing certain packages) that apt is unable to handle.

# [QGroundControl](http://qgroundcontrol.com/)

1. Why Its Needed ?

QGroundControl is essential software for managing and controlling unmanned aerial vehicles (UAVs) and autonomous systems, offering mission planning, real-time monitoring, manual control, data visualization, firmware updates, logging, telemetry, and customization capabilities, facilitating safe and efficient operation of these vehicles.

2. [Installation](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

   - Before installing QGroundControl for the first time:

```bash
    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libqt5gui5 -y
    sudo apt install libfuse2 -y
```
   - Download QGroundControl image (from steps 2)
   - Probably downloaded image will be in /Download directory

```bash
    cd 
    cd Download

    chmod +x ./QGroundControl.AppImage
    # run the image
    ./QGroundControl.AppImage
```


# Package Setup
This setup guide assumes you have installed the Micro XRCE-DDS Agent and PX4 v1.14.0-rc1
as explained in [PX4 and ROS 2 setup](ros_px4_setup.md)
## 1. ROS 2 setup

### 1.1. Create a workspace
This guide assumes the workspace to be in ``~/llc/llc_ws/`` but any desired name/location can be used.
```bash
mkdir -p ~/llc/llc_ws/src
```

### 1.2. Clone the package
```bash
cd ~/llc/llc_ws/src
git clone https://github.com/SaxionMechatronics/px4_offboard_lowlevel.git
```

### 1.3. Clone px4_msgs
This package has been confirmed to work with a certain commit of px4_msgs, you can obtain this commit like so:
```bash
cd ~/llc/llc_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd px4_msgs
# git reset --hard e3d36168a97f0268ab97e626d14858ca644924ef
```

### 1.4. Build
```bash
cd ~/llc/llc_ws/
colcon build --symlink-install
```

## 2. PX4 Setup

### 2.1. Change set of ROS 2 PX4 topics
Before building PX4 you need to change the list of topics for the μXRCE Client, to do this replace the default topics configuration in ```PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml``` by ```px4-resources/dds_topics.yaml``` from this repository.

```bash
cp /PATH_TO/src/px4_offboard_lowlevel/px4-resources/dds_topics.yaml PATH_TO/PX4-Autopilot/src/modules/uxrce_dds_client/
```

### 2.2. Disable lockstep in PX4
Lockstep needs to be disabled in PX4 and Gazebo to make the simulation run correctly while using thrust/torque commands.

Enter the board config for px4 sitl
```bash
make px4_sitl boardconfig
```
Navigate to ``Toolchain`` and enable ``force disable lockstep``

Quit (Q) and save (Y)

Open the gazebo model you will be using for simulation, for example: ``PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf``.

Change enable_lockstep (line 466) from 1 to 0 like so:
```XML
<enable_lockstep>0</enable_lockstep>
```

### 2.4 (optional) Remove the asphalt plane from Gazebo-classic default world

I hate it, so here it is:
```bash
cp /PATH_TO/src/px4_offboard_lowlevel/px4-resources/empty.world PATH_TO/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```
