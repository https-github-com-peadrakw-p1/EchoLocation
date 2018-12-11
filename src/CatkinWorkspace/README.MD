# ROS Code

This code contains the planner, planner, obstacle avoidance, image processing and Kalman filter. Each of the nodes can be run individually or as a whole. All the details as to how to run this project are detailed below.

## Running the code

The first thing you will need to do to run this code is compile it. You can compile it using catkin. To do that run the following:

```
$ catkin_make
```

Once the code has been compiled you should notice a build and devel folder. If you are using bash run:

```
$ source devel/setup.bash
```

If you use ZSH use:

```
$ source devel/setup.zsh
```

Running the code together can be done using

```
$ roslaunch voice_follower follow.launch
```

If you want to run any of the nodes in isolation you can use:

```
rosrun voice_follower {node}.py
```

## Prequrequistis

This code requires you have the following packages installed:

* ROS
* OpenCV
* Numpy

Numpy can be installed using pip. OpenCV is slightly more difficuly however there is a document in the docs folder describing the whole process. ROS has very detailed documentation on how to install it.