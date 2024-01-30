# rosardvarc

## Introduction
This is a repository for the `rosardvarc` ROS package.
## Installation
This package requires a valid installation of ROS. For ROS installation details, see [the ROS installation page](https://wiki.ros.org/ROS/Installation) or the [single-line ROS installation](https://wiki.ros.org/ROS/Installation/TwoLineInstall). Then follow the start of [this ROS environment setup tutorial](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to make a catkin workspace.

Once ROS is configured, this repo should be cloned into the `src` directory of a catkin workspace. This could be done with a command similar to:
```
cd ~/catkin_ws/src && git clone https://github.com/ARDVARC/rosardvarc.git
```

Once the repo is cloned, run the following command to build the package:
```
cd ~/catkin_ws && catkin_make
```

For more details on the ARDVARC project, see [the core project repo](https://github.com/ARDVARC/ARDVARC).