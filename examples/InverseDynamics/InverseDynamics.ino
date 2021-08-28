#include "InverseDynamics.h"

/*
 * In the other examples we went through how to express the poses of things relative to eachother, this time we'll get a
 * little more fancy and extend that to not just their pose but also their velocity and acceleration too. Specifically,
 * we'll define a simple kinematic chain based on the following [URDF](http://wiki.ros.org/urdf/XML/model) and then
 * using that model we'll implement an algorithm for calculating the inverse dynamics of a kinematic chain.
 *
 * Inverse dynamics is the problem of finding what joint torques are required to produce a desired set of joint
 * accelerations given the current set of joint positions and velocities. There isn't really space to fully describe how
 * this all works here but if you want to read into it some more take a look at section 8.3 of this book:
 * http://hades.mech.northwestern.edu/index.php/Modern_Robotics (if you're working through all these examples then you
 * might have seen me mention that book before, I really like it!). Anyway, the implementation I went with here is the
 * Recursive Newton Euler Algorithm for inverse dynamics, it takes a bit of time to figure out the twist, acceleration
 * and force calculations but other than that it's not too hard to follow so by all means take a look at the
 * implementation in InverseDynamics.cpp

    <robot name="robot_arm">
        <link name="world"/>
        <link name="upper_arm">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01335" iyy="0.01335" izz="0.0008" ixy="0" ixz="0" iyz="0"/>
                <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
            </inertial>
        </link>
        <link name="forearm">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01335" iyy="0.01335" izz="0.0008" ixy="0" ixz="0" iyz="0"/>
                <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
            </inertial>
        </link>
        <link name="hand">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01335" iyy="0.01335" izz="0.0008" ixy="0" ixz="0" iyz="0"/>
                <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
            </inertial>
        </link>

        <joint type="revolute" name="shoulder">
            <parent link="world"/>
            <child link="upper_arm"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0.0 0.0 0.4" rpy="0.0 0.0 0.0"/>
        </joint>
        <joint type="revolute" name="elbow">
            <parent link="upper_arm"/>
            <child link="forearm"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0.0 0.0 0.4" rpy="0.0 0.0 0.0"/>
        </joint>
        <joint type="revolute" name="wrist">
            <parent link="forearm"/>
            <child link="hand"/>
            <axis xyz="0 1 0"/>
            <origin xyz="0.0 0.0 0.4" rpy="0.0 0.0 0.0"/>
        </joint>
    </robot>
*/

using namespace Geometry;
using namespace BLA;

void setup()
{
    Serial.begin(115200);

    // Let's start by declaring the kinematic chain described in the URDF above
    Link world;
    Link upper_arm(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});
    Link forearm(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});
    Link hand(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});

    // All these joints are revolute so their joint axes point in the positive y direction
    Joint shoulder(world, upper_arm, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});
    Joint elbow(upper_arm, forearm, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});
    Joint wrist(forearm, hand, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});

    // First off, let's set the arm so it's dangling straight down and stationary. Under these conditions we can
    // expect the joint torques to be zero
    shoulder.position = M_PI;
    elbow.position = wrist.position = 0;
    shoulder.velocity = elbow.velocity = wrist.velocity = 0.0;
    shoulder.acceleration = elbow.acceleration = wrist.acceleration = 0.0;

    // Now call the inverse dynamics algorithm passing in the root of our chain along with gravity
    inverse_dynamics(world, Twist(Matrix<6>(0, 0, 0, 0, 0, -9.81)));
    Serial << "Dangling: " << shoulder.torque << ", " << elbow.torque << ", " << wrist.torque << "\n";

    // Conversely if the arm is outstretched then we'll need the joints to generate quite a bit of torque in order to
    // support their child links:
    shoulder.position = M_PI_2;
    elbow.position = wrist.position = 0;
    shoulder.velocity = elbow.velocity = wrist.velocity = 0.0;
    shoulder.acceleration = elbow.acceleration = wrist.acceleration = 0.0;

    inverse_dynamics(world, Twist(Matrix<6>(0, 0, 0, 0, 0, -9.81)));
    Serial << "Outstreched: " << shoulder.torque << ", " << elbow.torque << ", " << wrist.torque << "\n";

    // If we apply an external force to the hand then we can remove most of this torque
    hand.external_force(3) = -2 * 9.81;  // Apply a force equal to the weight of the upper_arm and the forearm

    inverse_dynamics(world, Twist(Matrix<6>(0, 0, 0, 0, 0, -9.81)));
    Serial << "Supported: " << shoulder.torque << ", " << elbow.torque << ", " << wrist.torque << "\n";

    // And finally, we can analyse the arm when it's in motion or accelerating too
    shoulder.position = elbow.position = wrist.position = 0.1;
    shoulder.velocity = elbow.velocity = wrist.velocity = 0.2;
    shoulder.acceleration = elbow.acceleration = wrist.acceleration = 0.3;

    inverse_dynamics(world, Twist(Matrix<6>(0, 0, 0, 0, 0, -9.81)));
    Serial << "Waving around: " << shoulder.torque << ", " << elbow.torque << ", " << wrist.torque << "\n";
}
