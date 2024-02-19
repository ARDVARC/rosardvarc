# rosardvarc

## Introduction
This is a repository for the `rosardvarc` ROS package.

## Installation
This package requires a valid installation of ROS. For ROS installation details, see [the ROS installation page](https://wiki.ros.org/ROS/Installation) or the [single-line ROS installation](https://wiki.ros.org/ROS/Installation/TwoLineInstall). 
```
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

Then follow the start of [this ROS environment setup tutorial](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to make a catkin workspace. I'd also recommend running the following to automatically setup your workspace every time you open a new terminal:
```
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

Once ROS is configured, this repo should be cloned into the `src` directory of a catkin workspace. This could be done with a command similar to:
```
cd ~/catkin_ws/src && git clone https://github.com/ARDVARC/rosardvarc.git
```

Once the repo is cloned, run the following command to build the package:
```
cd ~/catkin_ws && catkin_make
```

## Stay Up To Date
To automatically re-run `catkin_make` whenever you change branches, first ensure that your version of git supports git hooks by running:
```
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt-get update
sudo apt-get install git -y
```

Then, run the following command:
```
cd ~/catkin_ws/src/rosardvarc
git config --local core.hooksPath .githooks/
```

## Learn More
For more details on the ARDVARC project, see [the core project repo](https://github.com/ARDVARC/ARDVARC).
