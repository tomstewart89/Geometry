# Geometry
An Arduino library for representing where things are in relation to other things in 3D space.

## What Does That Mean?
In lots of problems in robotics and computer vision we need to be able to describe the position and orientation (aka its pose) of something relative to some coordinate frame. With robot arms we may want to represent the pose of the end effector relative to the robot's base, in mobile robotics we may want to track the pose of the rover relative to its initial position. Whatever the case, we're often confronted with this kind of situation:

![Transformation](https://user-images.githubusercontent.com/2457362/130321399-372a2a47-45b4-4e0b-9bf4-92292f1828b2.png)

The aim of this library is to allow us to represent the pose (and spatial velocity) of `End Effector` in relative `Base` even if we don't have a direct measurement between the two.

### Representing rotation
Geometry supports all your favourite 3D rotation formats, as well as functions to convert them into rotation matrices and back again:

* Rotation matrices
* Quaternions
* Euler angles (all 24 flavors of them)
* Axis Angles (kind of)

The default representation is a rotation matrix, a 3x3 orthogonal matrix:

![Rotation](https://user-images.githubusercontent.com/2457362/130322394-87942364-6c78-4779-8bf1-f3654d8686d5.png)

Which can be converted to a quaternion like so:
```cpp
Rotation R;
Quaternion q(R); // create a quaternion from a rotation matrix
q.to_rotation_matrix(); // and convert back again
```
Or euler angles like so:
```cpp
EulerAngles euler(R); // create extrinsically rotated, XYZ rotation order euler angles from a rotation matrix
euler.to_rotation_matrix(); // and convert back again
```
If this note on extrinsic XYZ rotation order doesn't ring any bells, take a look at the [EulerAngles example](https://github.com/tomstewart89/Geometry/blob/master/examples/EulerAngles/EulerAngles.ino) for a nice rundown of how tricky that format can be.

Geometry also implements a class for angular velocity which can also be thought of as the axis-angle representation for rotation, where the axis of the rotation is scaled by its magnitude. An angular velocity can be converted into a rotation matrix using the exponential function and back again using the logarithm:

```cpp
AngularVelocity w(0.1, 0.2, 0.3);
Rotation R = exp(w);
AngularVelocity also_w = log(R);
```

### Pose and Spatial Velocity
A Pose combines a rotation with a translation to fully describe the location of a body in space. You can think of a Pose as a cartesian position concatenated onto a rotation matrix like so:

![Pose](https://user-images.githubusercontent.com/2457362/130322395-a98e373b-a4ae-413c-91d3-3f0ea118f53f.png)

Poses can represent the location of a body in space or a transformation from one coordinate frame to another. For more information on that take a look at the [HowToUse example](https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino). If you want to dive deeper, take a look at the [PoseGraph example](https://github.com/tomstewart89/Geometry/blob/master/examples/PoseGraph/PoseGraph.ino) for a nice self-contained implementation of something that resembles ROS's [TF package](http://wiki.ros.org/tf).

Analogous to rotation and angular velocity, the velocity counterpart of a pose is a `SpatialVelocity`. Which combines an angular velocity llinear velocity to fully represent the velocity of a body in space. We can convert a spatial velocity to a pose and back again using the exponential and logarithm functions:

```cpp
SpatialVelocity V(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
Pose T = exp(V);
SpatialVelocity also_V = log(T);
```

These conversions allow us to transform spatial velocities between different coordinate frames which allows for some pretty fancy kinematics algorithms. If you're interested in that, have a look at the [InverseDynamics example](https://github.com/tomstewart89/Geometry/blob/master/examples/InverseDynamics/InverseDynamics.ino).
