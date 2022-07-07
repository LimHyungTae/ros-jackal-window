# ROS Jackal Window Setting

Github page for running Clearpath Jackal on Window Using Robot Operating System (ROS).

### Characteristics

This repositoy contains 

* A roslaunch file to launch jackal
* Simplified keyop python script (Originally, jackal heavliy relies on the joystick-based operation)

## Contents
0. [Test Env.](#Test-Env.)
0. [What I Did](#What-I-Did)

### Test Env.

The code is tested successfully at
* Window10 Professional
* ROS Melodic
* Visual Studio 2019 Community

## What I Did

**The below commands are just for memo. This repository is already self-contained!**

### Install ros-melodic on Window10

- 1. Follow the [official instruction in ROS wiki](http://wiki.ros.org/Installation/Windows)
- 2. [choco](https://docs.chocolatey.org/en-us/choco/setup) should be installed successfully. 
	- `choco` is a package manager in Window, which is quite similar to `apt` in Linux or `brew` in Mac OS.
- 3. On the step 5.1, replace `noetic` with `melodic` to install ros-melodic.
- 4. On the step 6, check your visual studio version and type (i.e. community, professional, or enterprise). Then, change the `noetic` in the last row into `melodic`. An example command for Visual Studio 2019 Community is as follows:

```bash
C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64&& set ChocolateyInstall=c:\opt\chocolatey&& c:\opt\ros\melodic\x64\setup.bat
```

### Set Jackal repository

- 1. Here's the [official guide in Clearpath](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/winsetup.html)
- 2. First, `wstool` was installed based on [this site](https://ms-iot.github.io/ROSOnWindows/Build/source.html).

```bash
pip install -U rosdep rosinstall_generator wstool rosinstall
curl --output requirements.txt -L https://raw.githubusercontent.com/ms-iot/rosdistro-db/init_windows/rosdistro_cache/catkin-requirements.txt
pip install -U --no-deps --force-reinstall -r requirements.txt
```

- 3. Then, git clone the two essential repositories, [jackal](https://github.com/jackal/jackal) and [jackal_robot](https://github.com/jackal/jackal_robot).
	- **NOTE** that their default branch is `noetic-devel`. So, their branches are set to be `melodic-devel`.

```bash
git clone https://github.com/jackal/jackal 
cd jackal
git checkout -t origin/melodic-devel

git clone https://github.com/jackal/jackal_robot 
cd jackal_robot
git checkout -t origin/melodic-devel
```

### Check the port number.

Unlike Linux whose serial port name is like `/dev/ttyUSB0`, the serial port name on Window can be found in device manager (in Korean, 장치관리자).

For example, my jackal was assinged by COM3, so I changed the `jackal port` in `jackal_robot/jackal_base/launch/base.launch`.  

---
### Contact

If you have any questions, please let me know:

- Hyungtae Lim {[shapelim@kaist.ac.kr]()}
