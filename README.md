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

### ROS Setting
- 1. Install [ROS](http://torch.ch/docs/getting-started.html) on a machine. 
- 2. Thereafter, [jsk-visualization](https://github.com/jsk-ros-pkg/jsk_visualization) is required to visualize Ground Likelihood Estimation status.

```bash
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

- 3. Compile compile this package. We use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/),
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LimHyungTae/patchwork.git
cd .. && catkin build patchwork 
```

---
### Contact

If you have any questions, please let me know:

- Hyungtae Lim {[shapelim@kaist.ac.kr]()}
