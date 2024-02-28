# rosardvarc

## Introduction
This is a repository for the `rosardvarc` ROS package.

## Virtual Environment Setup
Update your Linux with: 
```
sudo apt update
```
```
sudo apt upgrade
```

Install Python
```
sudo apt install python3-pip
```

Update Numpy
```
pip install numpy -U
```
Install OpenCV
```
pip install opencv-python
```

Setup GitHub CLI on LINUX 
```
type -p curl >/dev/null || (sudo apt update && sudo apt install curl -y)
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
&& sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh -y
```
Upgrade
```
sudo apt update
sudo apt install gh
```

Authenticate
```
gh auth login
```

Make a directory for this project
```
mkdir <DIRECTORY>
cd <DIRECTORY>
```

Clone the ardvarc repo
```
gh repo clone ARDVARC/ARDVARC
```

For Using VSCode (Optional- Recommended for wsl)
```
code .
```


## Installation
This package requires a valid installation of ROS. For ROS installation details, see [the ROS installation page](https://wiki.ros.org/ROS/Installation) or the [single-line ROS installation](https://wiki.ros.org/ROS/Installation/TwoLineInstall). 
```
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
rosdep update
```

Then follow the start of [this ROS environment setup tutorial](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to make a catkin workspace:

If you just installed ROS from apt on Ubuntu then you will have setup.*sh files in '/opt/ros/<distro>/', and you could source them like so:
```
source /opt/ros/noetic/setup.bash
```
Let's create and build a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Before continuing source your new setup.*sh file:
```
source devel/setup.bash
```

To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
```
echo $ROS_PACKAGE_PATH
```
Your output should look like this
```
"/home/youruser/catkin_ws/src:/opt/ros/kinetic/share"
```


I'd also recommend running the following to automatically setup your workspace every time you open a new terminal:
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

## Running, Debugging, and rostools
If issues occur always run `catkin_make` from the catkin ws first as an initial debug

To display the rostopics, run `rostopic list` from the rosardvarc directory.

A visual representation of the ROS node can be displayed by running `rqt_graph`.

To listen to a topic manually, use `rostopic echo <TOPIC LOCATION>` to display all messages and `rostopic echo <TOPIC LOCATION> -n 1` to listen to the most recent.

**_Rogue Scripts:_**

When a python3 script does not automatically stop when using ctrl+c you can use `ps aux` to show all running scripts.

To kill a singular file use `kill -9 <SECOND COLUMN FROM LEFT>`. **Important Note: Be careful what you kill, do not kill important things.**

To kill all python-related files use `killall -9 python3`. Note that this will kill anything with the name start 'python3'.

**_Video Fake Generator Display_**

To see the video actually loop, first run the fake data shell script:
```
cd path/to/ARDVARC/FSW
./fake_generators_black_magic.sh
```

Then, in a different tab, run this ROS tool to see the published frames:
```
rosrun image_view image_view image:=/camera/frames
```


