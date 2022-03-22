# rov_navigation
This is a ROS package for learning how to tune an EKF state estimator (in this case provided by the [robot_localization package](http://wiki.ros.org/robot_localization)).

## Guide for Students in AMR 2022 ##

1. Clone this repository into a workspace `git clone https://github.com/FletcherFT/rov_navigation.git`
2. Build and source the workspace.
3. Download one of the recorded bagfiles from [DTU learn](learn.inside.dtu.dk).
4. Startup a local ROS master (`roscore`)
5. Run the state estimator: `roslaunch rov_navigation ekf.launch covariance:=SOME_NUMBER bag_file:=/path/to/your/downloaded/bagfile.bag`
6. Compare `/bluerov2/mavros/global_position/local` with `/odometry/filtered` (Hint: Use `rviz`)
7. Adjust variables in `rov_navigation/params/ekf_params.yaml` to improve filter, calculate the variance of the depth sensor from a stationary bag file and specify it as the `covariance` argument to improve the depth estimation.
