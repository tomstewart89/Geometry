#include <Geometry.h>

/*
 * This example sketch should show you everything you need to know to get up and running with the Geometry library. This
 * library is all about representing where things are in 3D space; be it your robot's end effector, the center
 * of mass of your drone or whatever else.
 *
 * This is actually a surprisingly tricky thing to wrap one's head around so I'll try to introduce things gradually
 * here. If once you're finished this still makes no sense then I'd recommend taking a look at chapter 3 of this book:
 * http://hades.mech.northwestern.edu/index.php/Modern_Robotics, it's a great read and it definitely helped me
 * understand rigid body motion a lot more clearly.
 */

using namespace Geometry;
using namespace BLA;

void setup()
{
    Serial.begin(115200);

    // First off, when we say 'where' something is, specifically what we want to describe is the translation and
    // rotation of that something relative to some coordinate frame. A coordinate frame looks like the following; it's
    // just a set of X, Y and Z directions attached to an origin. To make things a little less abstract, let's pretend
    // that we're trying to describe the position of a drone relative to it's ground station:

    //      Z
    //      |   Y
    //      |  /           o drone
    //      | /
    //      |/
    //   GS o-------- X

    // To do that is pretty straight forward, we just measure the drone's distance along the x, y and z axes of ground
    // station coordinate frame. We can then represent that in code using a Translation:
    Translation p_drone = {1.5, 0, 0.5};

    // A Translation is actually just an alias for a BLA::Matrix<3,1> so you can do all the usual linear algebra stuff
    // to it as you would any other vector, for example let's say you want to know the distance of the drone from the
    // ground station you can find that like so:

    Serial << "distance from drone to ground station: " << Norm(p_drone) << "\n";

    // If, rather than just its position, we wanted to keep track of the orientation of the drone too, we can use a
    // Pose object. A pose consists of a translation (the same as the one used to represent the drone position just now)
    // as well as a rotation matrix to encode, well, rotation.

    // Rotation matrices might seem complicated, but once you understand their anatomy they're actually really simple.
    // They're basically 3x3 matrices wherein each column describes the direction in which each of the axes of the
    // drone point as recorded from the perspective of the ground station.

    // Let's assume our drone has just taken off and it is flying upright, rotated 90 degrees about its z-axis relative
    // to the ground station:

    //      Z                 Z
    //      |   Y             |
    //      |  /        drone o--- Y
    //      | /              /
    //      |/              X
    //   GS o-------- X

    // We can make our pose object like so and since we already figured our the drone's position we can go ahead and
    // fill that in:
    Pose drone_pose;
    drone_pose.p = p_drone;

    // Now to fill in the rotation matrix, we'll go column-wise and enter in unit vectors (vectors of length = 1) for
    // the three axes of the drone:
    drone_pose.R.Column(0) = Matrix<3>(0.0, -1.0, 0.0);  // drone's X-axis points in the -ve Y-direction of the GS frame
    drone_pose.R.Column(1) = Matrix<3>(1.0, 0.0, 0.0);   // drone's Y-axis points in the +ve X-direction of the GS frame
    drone_pose.R.Column(2) = Matrix<3>(0.0, 0.0, 1.0);   // drone's Z-axis points in the +ve Z-direction of the GS frame

    // And that's it! We've now fully described the pose of our drone:
    Serial << "drone pose: " << drone_pose << "\n";

    // The neat thing about describing poses in this way is that we can chain them together really easily. To explain
    // what I mean by this let's pretend that our drone has just spotted another drone accelerating towards it:

    //      Z                 Z                 Z      Y
    //      |   Y             |                  \  _-
    //      |  /       drone  o--- Y              o-
    //      | /              /                   /   other drone
    //      |/              X                   X
    //   GS o-------- X

    // Our drone has some fancy perception system onboard and it can measure that the other drone is at the same
    // altitude as itself and is 2 units away along its Y-axis. It can also tell that the other drone is banking at 15
    // degrees (around 0.26 radians) about its X-axis.
    const float bank_angle = 0.26;

    // Using that information we can come up with a pose of the other drone relative to our drone:
    Pose other_drone;

    other_drone.p = {0, 2, 0};

    other_drone.R.Column(0) = Matrix<3>(1, 0, 0);
    other_drone.R.Column(1) = Matrix<3>(0, cos(bank_angle), sin(bank_angle));
    other_drone.R.Column(2) = Matrix<3>(0, -sin(bank_angle), cos(bank_angle));

    // Let's say that other drone wants to come in to land at our ground station. For that to happen we need to know the
    // pose of the other drone relative to the ground station. Turns out we can calculate that using the information we
    // already have very easily, like so:
    Pose other_drone_relative_to_GS = drone_pose * other_drone;

    Serial << "other_drone_relative_to_GS: " << other_drone_relative_to_GS << "\n";

    // And actually the other drone probably needs the pose of the ground station relative to it, which we can also
    // calculate quite easily:
    Pose GS_relative_to_other_drone = other_drone_relative_to_GS.inverse();

    // So what's the intutition behind these last couple of lines? Basically while poses are handy for representing the
    // 3D position and orientation of a thing, they can also be interpreted as a desription of how to change the
    // coordinate frame that a pose is being expressed in from one frame to another.

    // More concretely, we saw that the pose `drone_pose` describes the pose of our drone relative to the ground station
    // coordinate frame. However, this can also be thought of as a transformation from drone coordinates to ground
    // station coordinates. So in order to transform the pose of the other drone from our drone's coordinates to those
    // of the ground station, we could simply directly apply this transform.

    // As a sidenote, I often like to name these variables T_target_source where `T` refers to
    // transformation source is the frame that points / poses are currently expressed in and target is the frame that
    // we'd like them in. Rewriting that epression above using this notation we have:

    Pose& T_GS_drone = drone_pose;
    Pose& T_drone_otherdrone = other_drone;

    Serial << "T_GS_otherdrone: " << T_GS_drone * T_drone_otherdrone << "\n";

    // If we take the inverse of a transformation then we essentially flip the target and source frames, so:
    Serial << "T_otherdrone_GS: " << (T_GS_drone * T_drone_otherdrone).inverse();

    // If you'd like to see this chained transformation thing taken to the extreme, have a look at the PoseGraph
    // example!
}

void loop() {}
