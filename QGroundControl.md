## QgroundControl

Our current GUI is dependent upon QGroundControl and therefore needs to be installed for any interactive display

## Running QGroundControl from linux terminal
```
./QGroundControl.AppImage
```

## Installation

_**Ubuntu Linux**_

QGroundControl can be installed/run on Ubuntu LTS 20.04 (and later).

Ubuntu comes with a serial modem manager that interferes with any robotics related use of a serial port (or USB serial). Before installing QGroundControl you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install GStreamer in order to support video streaming.

**Before installing QGroundControl for the first time:**

On the command prompt enter:

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y
```

Logout and login again to enable the change to user permissions.

**To install QGroundControl:**

**Download** QGroundControl.AppImage here:
https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage 

Make sure to put the download into the ubuntu directory (Not catkin_ws)
You can go to the wsl location from windows using:

From Windows File manager:
```
\\wsl.localhost\Ubuntu-20.04\home\
```

From WSL Terminal:
```
explorer.exe .
```


**Install** using the terminal commands:
```
chmod +x ./QGroundControl.AppImage
```
