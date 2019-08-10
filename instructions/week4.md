# Week 4
Welcome to Week 4 of ROS training exercises! We'll be learning about the **coordinate frames**, the **IMU**, and
**localization with dead reckoning** by integrating IMU data.

## IMU
IMU stands for Inertial Measurement Unit. A 9 DOF (degree of freedom) IMU measures accelerations, angular velocities,
and magnetic field in all 3 axis by using an accelerometer, gyroscope, and magnetometer. This means that by using an
IMU you will be able to tell your acceleration, angular velocity and heading.

## Coordinate frames and the IMU
One thing important when using the IMU is which **coordinate frame** the data is in. A **coordinate frame** is a
refers to the coordinate system that is used for the data. For example, the IMU can measure accelerations in the
x, y and z axes in the _robot's coordinate frame_, which is different from the _world's coordinate frame_.

A quick note: ROS has a [REP 103](https://www.ros.org/reps/rep-0103.html) which defines conventions for coordinate
frames. In particular, for bodies, X points forward, Y points left, and Z is pointing up, while for geographic
locations X points east, Y points north, and Z points up.

An easy way to understand this is through an example:
Let's say my IMU is mounted in the same direction as my robot. This means that when my IMU records an acceleration in
the X direction, it means that the robot has accelerated forwards in the +X direction.
However, let's say that my robot is located at the point (1,1) in the world and is pointed north. In the
_robot's coordinate frame_, the acceleration is in the +X direction. However, in the _world's coordinate frame_, because
the robot is pointed north, which corresponds to the +Y direction, the acceleration is in the +Y direction.
