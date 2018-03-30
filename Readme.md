# RRT On a simple car model
This is an implementation of vanilla rrt in a simple kinematic car model.

## Dependencies
1. Ros (http://wiki.ros.org/ROS/Installation)
2. OpenCV3 (if you havent installed ros-desktop-full)

## How to build

Make a catkin_workspace
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/abhilashbalachandran/vanilla_rrt_car.git
$ cd ..
$ catkin_make
```

## Running the code

```
roslaunch rrt_car test.launch
```

You should see an opencv display window which shows the tree in red and the path in black. The delay is due to a waitkey() in the backtrack function in the main.cpp file. You can reduce the duration of waitKey to speed up the display. After that, you need to press a key while in the opencv display window (due to waitkey(0)). You will see the animation in the rviz window. Right now, the collision function in the main file has to be written (which is fairly straight forward, simply include the bounds for collision). Start and end points are specified in the main function of the main.cpp file.


## Future work
- [ ] Interactive way of defining collisions (image file or yaml file) and start and end points (ros parameters)
- [ ] Dynamic car model
- [ ] Trajectory tracking using iLQR
- [ ] Gazebo simulation

