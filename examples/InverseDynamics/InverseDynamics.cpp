#include "InverseDynamics.h"

using namespace Geometry;
using namespace BLA;

Link::Link(float mass, const BLA::Matrix<3, 3> &inertia, const Geometry::Translation &xyz,
           const Geometry::EulerAngles &rpy)
    : external_force(Zeros<6>())
{
    Pose inertia_pose(rpy.to_rotation_matrix(), xyz);
    spatial_inertia = Zeros<6, 6>();
    spatial_inertia.Submatrix<3, 3>(0, 0) = inertia;
    spatial_inertia.Submatrix<3, 3>(3, 3) = Identity<3, 3>() * mass;
    spatial_inertia = ~adjoint(inertia_pose.inverse()) * spatial_inertia * adjoint(inertia_pose.inverse());
}

Joint::Joint(Link &parent, Link &child, const Geometry::Twist &axis_, const Geometry::Translation &xyz,
             const Geometry::EulerAngles &rpy)
    : axis(axis_), offset(Pose(rpy.to_rotation_matrix(), xyz).inverse())
{
    child.joint = this;
    parent.child = &child;
    child.parent = &parent;
}

void inverse_dynamics(Link &root, const Twist &gravity)
{
    root.pose = Identity<4, 4>();
    root.twist = Zeros<6>();
    root.acceleration = -gravity;

    Link *link = &root;

    // Starting from the root link, calculate the pose, twist and acceleration of each link relative to its parent
    while (link->child)
    {
        link = link->child;

        link->pose = exp(link->joint->axis * -link->joint->position) * link->joint->offset;

        link->twist = link->pose * link->parent->twist + link->joint->axis * link->joint->velocity;

        link->acceleration = link->pose * link->parent->acceleration +
                             link->twist * link->joint->axis * link->joint->velocity +
                             link->joint->axis * link->joint->acceleration;
    }

    //  Starting from each terminal link, calculate the joint forces that are required to bring about the observed
    //  accelerations
    while (link)
    {
        link->force = link->spatial_inertia * link->acceleration -
                      ~adjoint(link->twist) * link->spatial_inertia * link->twist - link->external_force;

        if (link->child)
        {
            link->force += link->child->pose * link->child->force;
        }

        if (link->joint)
        {
            link->joint->torque = (~link->force * link->joint->axis)(0);
        }

        link = link->parent;
    }
}
