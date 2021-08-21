# Geometry
### An Arduino library for representing where things are in relation to other things in 3D space.
#### Specifically this library has implementations support for:
* All your favourite 3D rotation formats, (as well as functions to convert them into rotation matrices and back again):
    * Rotation matrices
    * Quaternions
    * Euler angles (all 24 flavors of them)
    * Axis Angles (kind of)
* 3D poses (a rotation and a translation) for representing the position and orientation of a rigid body in space.
* Spatial velocities along with skew, logarithm and expontial functions to help with transforming them between coordinate frames

The aim of all this is to provide everything we need to represent the 3D pose of our robot arm end effectors, our drone centers of mass or any other 3D pose we're likely to encounter in robotics; and with a footprint small enough to fit on an 8-bit microcontroller.

![Transformation](https://user-images.githubusercontent.com/2457362/130311793-bbc8cf45-c66f-40ca-b84a-d33004398c5e.png)

## Getting Started
To get started, have a look at the [HowToUse example](https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino) which will get you up to speed with how 3D poses / coordinate transformations work. From there take a look at the [PoseGraph example](https://github.com/tomstewart89/Geometry/blob/master/examples/PoseGraph/PoseGraph.ino) for a nice self-contained implementation of something that resembles ROS's [TF package](http://wiki.ros.org/tf). Finally, if you want to get into some more fancy kinematics, take a look at the [InverseDynamics example](https://github.com/tomstewart89/Geometry/blob/master/examples/InverseDynamics/InverseDynamics.ino) (currently WIP)

