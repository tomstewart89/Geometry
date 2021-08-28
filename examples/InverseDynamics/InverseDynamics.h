#pragma once

#include <Geometry.h>

class Link;

class Joint
{
   public:
    Geometry::Pose offset;
    Geometry::Twist axis;

    float position, velocity, acceleration;
    float torque;

    Joint(Link& parent, Link& child, const Geometry::Twist& axis_, const Geometry::Translation& xyz,
          const Geometry::EulerAngles& rpy);
};

class Link
{
   public:
    BLA::Matrix<6, 6> spatial_inertia;

    Joint* joint = NULL;  // the joint connecting this link to its parent
    Link *child = NULL, *parent = NULL;

    Geometry::Pose pose;
    Geometry::Twist twist, acceleration;
    Geometry::Wrench force, external_force;

    Link(float mass = 0, const BLA::Matrix<3, 3>& inertia = BLA::Zeros<3, 3>(),
         const Geometry::Translation& xyz = {0, 0, 0}, const Geometry::EulerAngles& rpy = {0, 0, 0});
};

void inverse_dynamics(Link& root, const Geometry::Twist& gravity);
