#include <Geometry.h>

/*
 * TODO
 */

// struct Link
// {
//     Matrix<6, 6> spatial_inertia;
// };

// struct Joint
// {
//     const Link& parent;
//     const Link& child;

//     Joint(const Link& parent_, const Link& child) : parent(parent_), child(child_) {}

//     Transformation pose;  // The pose of the joint when theta = 0
//     SpatialVelocity axis;
//     float theta = 0;
// };

// x-------x

// T_link_joint * T_joint_child

void setup()
{
    // Link the_world;
    // Link arm;
    // Link forearm;
    // Link hand;

    // Joint shoulder(the_world, arm);
    // Joint elbow(arm, forearm);
    // Joint wrist(forearm, hand);

    // auto& joint = shoulder;
    // Transformation pose = BLA::Identity();

    // shoulder.pose * exp(joint.axis * -joint.theta)

    // while (true)
    // {
    //     if (link == &hand)
    //     {
    //         link.pose = exp(-link.joint.A * theta[link.joint.idx]) @link.joint.M;

    //         break;
    //     }
    // }
}

void loop() {}

// import numpy as np
// import copy
// import lxml.etree as etree
// import sys
// from functools import lru_cache
// from utils.smpl import parse_transform, parse_spatial_inertia
// from utils.geometry import exp, adj, skew

// sys.path.append(".")

// class Joint:
//     def __init__(self, idx, xml):
//         self.idx = idx
//         self.name = xml.attrib["name"]
//         self.parent = xml.find("parent").attrib["link"]
//         self.child = xml.find("child").attrib["link"]

//         # M is the pose of the parent link expressed in the current link frame when theta = 0.
//         self.M = np.linalg.inv(parse_transform(xml.find("origin"))) if xml.find("origin") is not None else np.eye(4)

//         # A is the screw axis of the joint connecting this link to its parent, expressed in the this link's frame
//         axis_xml = xml.find("axis")
//         self.A = np.array(
//             ([float(e) for e in axis_xml.attrib["xyz"].split(" ")] if axis_xml is not None else [1.0, 0.0, 0.0])
//             + [0.0, 0.0, 0.0]
//         )

// class Link:
//     def __init__(self, chain, link_xml):
//         self.name = link_xml.attrib["name"]
//         self.chain = chain

//         self.joint = None  # This is the joint connecting this link to its parent
//         self.children_joints = []

//         self.spatial_inertia = parse_spatial_inertia(link_xml.find("inertial"))
//         self.external_force = np.zeros(6)

//     def parent(self):
//         return self.chain.links[self.joint.parent]

//     def children(self):
//         return [self.chain.links[joint.child] for joint in self.children_joints]

// class KinematicChain:
//     def __init__(self, filename):

//         with open(filename, "r") as f:
//             urdf = etree.fromstring(f.read())

//         self.links = {l.attrib["name"]: Link(self, l) for l in urdf.findall("link")}
//         self.joints = {j.attrib["name"]: Joint(idx, j) for idx, j in enumerate(urdf.findall("joint"))}

//         # Connect up the links via the joints
//         for joint in self.joints.values():
//             self.links[joint.parent].children_joints.append(joint)
//             self.links[joint.child].joint = joint

//         self.get_root().twist = np.zeros(6)
//         self.get_root().acceleration = np.zeros(6)

//     @lru_cache(1)
//     def get_root(self):
//         link = next(iter(self.links.values()))

//         while link.joint:
//             link = link.parent()

//         return link

//     def lookup_transform_to_root(self, target_link):
//         link = self.links[target_link]
//         T_root_target = np.eye(4)

//         while link is not self.get_root():
//             T_root_target = np.linalg.inv(link.pose) @ T_root_target
//             link = link.parent()

//         return np.linalg.inv(T_root_target)

//     @lru_cache(1)
//     def forward_pass(self):
//         queue = self.get_root().children()
//         out = copy.copy(queue)

//         while len(queue):
//             link = queue.pop()
//             queue += link.children()
//             out += link.children()

//         return out

//     @lru_cache(1)
//     def backward_pass(self):
//         queue = [link for link in self.links.values() if len(link.children_joints) == 0]
//         out = copy.copy(queue)

//         for link in self.links.values():
//             link.visited = False

//         while len(queue):
//             link = queue.pop()
//             link.visited = True

//             if link is not self.get_root():
//                 if all(sibling.visited for sibling in link.parent().children()):
//                     queue.append(link.parent())
//                     out.append(link.parent())

//         return out

//     def inverse_dynamics(self, theta, theta_dot, theta_ddot):
//         # Starting from the root link, calculate the pose, twist and acceleration of each link relative to its parent
//         for link in self.forward_pass():
//             link.pose = exp(-link.joint.A * theta[link.joint.idx]) @ link.joint.M

//             link.twist = adj(link.pose) @ link.parent().twist + link.joint.A * theta_dot[link.joint.idx]

//             link.acceleration = (
//                 adj(link.pose) @ link.parent().acceleration
//                 + adj(link.twist) @ link.joint.A * theta_dot[link.joint.idx]
//                 + link.joint.A * theta_ddot[link.joint.idx]
//             )

//         # Starting from each terminal link, calculate the joint forces that are required to bring about the observed
//         accelerations for link in self.backward_pass():
//             link.force = (
//                 np.sum([adj(child.pose).T @ child.force for child in link.children()], axis=0)
//                 + link.spatial_inertia @ link.acceleration
//                 - adj(link.twist).T @ link.spatial_inertia @ link.twist
//                 - link.external_force
//             )

//             if link.joint:
//                 link.joint.tau = link.force.T @ link.joint.A

//         return np.array([joint.tau for joint in self.joints.values()])
