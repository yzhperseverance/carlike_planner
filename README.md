# carlike_planner

This project is modified from the uneven_planner for a 2D scenario. The known bug is that the optimizer may fail to converge when the target point is close to an obstacle. It might be fixed in the future.

## requirements

```
sudo apt install ros-noetic-robot-state-publisher*
sudo apt install ros-noetic-joint-state-controller*
sudo apt install ros-noetic-controller*
sudo apt install ros-noetic-velocity-controllers*
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-position-controllers
sudo apt install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-hector-gazebo
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-joint-state-controller
sudo apt install ros-noetic-position-controllers
sudo apt install ros-noetic-velocity-controllers
sudo apt install ros-noetic-ompl
sudo apt install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy 
```

**ompl soft link**

Due to the special file structure of OMPL, CMake may not be able to find OMPL, so a symbolic link needs to be created.

```
sudo ln -s /opt/ros/melodic/include/ompl-1.6/ompl /opt/ros/melodic/include/ompl
```



**osqp-0.6.2 and osqp-eigen v0.8.0 for mpc controller:**

Firstly, go to [website of OSPQ](https://github.com/osqp/osqp/releases) and download `complete_sources.zip` from the Assets of `0.6.2`. Then unzip the code,

```
cd osqp
mkdir build && cd build
cmake ..
make
sudo make install
```

Go to [website of osqp-eigen](https://github.com/robotology/osqp-eigen/releases) and download `Source code.zip` from the Assets of `osqp-eigen v0.8.0`. Then unzip the code,

```
cd osqp-eigen-0.8.0
mkdir build && cd build
cmake ..
make
sudo make install
```

## start

```
./run.sh
```

<p align="center">
  <img src="https://github.com/yzhperseverance/carlike_planner/blob/main/figure.png"
</p> -->
