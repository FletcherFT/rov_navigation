# ROV Navigation #
This is a ROS package for learning how to tune an EKF state estimator (in this case provided by the [robot_localization package](http://wiki.ros.org/robot_localization)).

## Guide for Students in AMR 2022 ##

### Setup ###

Clone this repository into a ROS workspace (called $ROS_WS) in this example.

```
mkdir -p $ROS_WS/src
cd $ROS_WS/src
git clone https://github.com/FletcherFT/rov_navigation.git
```

Then install dependencies:

```
cd $ROS_WS
rosdep install --from-paths src -i -y
```

Finally, build and source the workspace.

```
catkin_make
echo $ROS_WS/devel/setup.bash >> $HOME/.bashrc
source $HOME/.bashrc
```

### Introduction ###

[robot\_localization](http://wiki.ros.org/robot_localization) is a package that provides two types of state estimator built interfacing within ROS: an Extended Kalman Filter (EKF) and an Unscented Kalman Filter (UKF).

This package provides a launch file, called [ekf.launch](launch/ekf.launch), that plays back a .bag file and launches an EKF node to perform state estimation on sensor measurements stored in the bag. The EKF node uses [ekf\_params.yaml](params/ekf_params.yaml) to configure the EKF node. The EKF node is very flexible in its configuration, you can add as many Odometry, Imu, TwistWithCovarianceStamped and PoseWithCovarianceStamped as you like. The description of how to do this is provided in the [documentation for robot\_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html), but the basic configuration has been setup in the ekf\_params.yaml file.

The goal for this lesson is to try to fuse the sensor measurements provided by the ROV to obtain a state estimate that is comparable to the estimate made by the EKF onboard the Flight Controller Unit, exposed as an Odometry topic through MAVROS, called `/bluerov2/mavros/global\_position/local`.

### Sensor Feedback ###

Depending on what .bag dataset you are using, you may have access to different topics. These are summarised here:

**/bluerov2/mavros/imu/data**: The filtered IMU data that fuses accelerometer, gyroscope and magnetometer data to produce an orientation estimate (in Quaternion), as well as accleration (no gravity compensation), and angular velocity.

**/bluerov2/mavros/imu/data\_raw**: The raw sensor measurements for accelerometer and gyroscopes.

**/bluerov2/mavros/imu/mag**: The raw magnetometer measurements.

NOTE: The IMU reports a covariance, but this is not provided by the FCU, it is specified within the MAVROS parameters by the user, so it might not be correct (in fact, the reported magnetometer covariance is an identity matrix, which is very unlikely to be correct. If you want to specify your own covariance, then you will need to make a node that subscribes to the topic in question and publish a new topic with the covariance overwritten. This topic should then be specified within the ekf\_params.yaml file. See [this node](nodes/republish_depth_as_pose) for an example of how to republish a topic as a **Pose/Twist**WithCovarianceStamped topic with a specified covariance.

**/bluerov2/mavros/scaled\_pressure2/depth**: The depth estimate calculated from the pressure sensor absolute pressure reading, assumed with a seawater density of 1024 kg/m³ and gravitational acceleration constant of 9.81 m/s².

**/bluerov2/mavros/global\_position/rel\_alt**: The calculated altitude (relative to a user specified reference frame) produced by the EKF.

**/bluerov2/mavros/fake\_gps/mocap/pose**: The estimated position of the ROV from the optitrack system.

**/bluerov2/waterlinked/pose\_with\_with\_cov\_stamped**: The estimated position of the ROV provided by the acoustic system.

### Calculating your own sensor covariance ###

As noted above, the reported covariance may not be correct. This is important because the EKF and UKF rely on covariance specifications for sensors to appropriately weight each measurement for fusion.

You can calculate your own measurement covariance, you just need a dataset containing sensor measurements which are taken while the vehicle is stationary and undisturbed. One such file has been included [here](data/BLUEROV2HEAVY_STATIONARY_SENSOR_NOISE_BAG.bag).

If, for example, you want to calculate the IMU orientation covariance matrix (a 3x3 matrix), first convert the IMU topic data into CSV form (you will a roscore running for this to work):

`rostopic echo -b data/BLUEROV2HEAVY_STATIONARY_SENSOR_NOISE_BAG.bag -p /bluerov2/mavros/imu/data > imu_data.csv`


Here is an example Python3 script to calculate covariance, you will need to install the `pandas` and `scipy` python packages for this to work (`python3 -m pip install pandas scipy`)


```
import pandas as pd
from scipy.spatial.transform import Rotation
import numpy as np


df = pd.read_csv("imu_data.csv")
euler = Rotation.from_quat(df[['field.orientation.x', 'field.orientation.y',     'field.orientation.z', 'field.orientation.w']]).as_euler("xyz")
covariance = np.cov(euler, rowvar=False)
print("Covariance Matrix: Roll,Pitch,Yaw")
print(repr(covariance))
print("Row Major Order:")
print(repr(covariance.flatten()))
```

Use the Row Major Form vector to replace the covariance vector for the Imu.orientation\_covariance message field.

### Running the state estimator ###

Modify the ekf.launch file and ekf\_params.yaml files to fit your own configuration. Make sure to include nodes that republish with your own covariance calculations.

As an example, I have calculate a pressure sensor depth reading variance of about `1.92263918e-05 m²`. I have exposed the override covariance within the republish\_depth\_as\_pose as a parameter which I specify within ekf.launch as an argument:

`roslaunch rov_navigation ekf.launch covariance:=1.92263918e-05 bag_file:=data/BLUEROV2HEAVY_STATIONARY_SENSOR_NOISE_BAG.bag`

The EKF produces a filtered Odometry topic on `/odometry/filtered`, I can compare this with `/bluerov2/mavros/global_position/local`.

You can compare them on `rviz` or with `rqt_plot`

In the STATIONARY case things are fairly boring, but one can see that the velocity estimate provided by the robot\_localization EKF begins to run away into ridiculous speeds. Clearly the filter is trusting the accelrometer measurements too much (or perhaps too little!).
