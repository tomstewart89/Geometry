#include <Geometry.h>

/*
 * This example sketch should show you everything you need to know in order to work with the Geometry library. This
 * library is all about representing where things are in 3D space; be it your robot's end effector, the center
 * of mass of your drone or whatever else.
 *
 * This is actually a surprisingly tricky thing to wrap one's head around so I'll try to introduce things gradually
 * here. If once you're finished this still makes no sense then I'd recommend taking a look at chapter 3 of this book:
 * http://hades.mech.northwestern.edu/index.php/Modern_Robotics, it's a great read and it definitely helped me
 * understand rigid body motion a lot more clearly.
 */

using namespace Geometry;

void setup()
{
    Serial.begin(115200);

    // First off, when we say 'where' something is, specifically what we want to describe is the translation and
    // rotation of that something relative to some coordinate frame. A coordinate frame looks like the following; it's
    // just a set of X, Y and Z directions attached to an origin. It's also handy to give names to coordinate frames so
    // we can refer to them more easily. We'll call this one 'A'

    //      Z
    //      |   Y
    //      |  /
    //      | /           o p1
    //      |/  o p2
    //  (A) O-------- X

    // Now if we want to describe where the point p1 is relative to the frame A, we just measure its distance along the
    // X, Y and Z axes of A. We can then represent that using a Translation:
    Translation p1_A = {1.5, 0, 0.5};

    // And we can describe point p2 in the same way:
    Translation p2_A;
    p2_A(0) = 0.25;
    p2_A(1) = 0.25;
    p2_A(2) = 0.0;  // hard to see in ASCII but p2 lies in the XY plane of A

    // We can find the offset between p1 to p2 like so:
    Serial << "how to get to p2 from p1: " << p2_A - p1_A;

    // Or the straight line distance between them like so:
    Serial << "distance from p1 to p2: " << Norm(p2_A - p1_A);

    // It's not uncommon to have multiple coordinate frames so let's define another called B:

    //          (B) O-------- X
    //             /|
    //            / |
    //           /  |     o p3
    //          /   |
    //         Y    Z

    // B looks a little different to A, but we can still use it to describe the position of the point p3:
    Translation p3_B = {1.0, 0.0, 1.0};

    // Let's say we happen to know that frames A and B are actually right next to each other, the origin of B is
    // positive 1 unit in A's Z-axis and positive 3 units in its X axis. Taken together the whole scene actually looks
    // like this:

    //      Z                       (B) O-------- X
    //      |   Y                      /|
    //      |  /                      / |
    //      | /           o p1       /  |     o p3
    //      |/  o p2                /   |
    //  (A) O-------- X            Y    Z

    // Now just as we did for p1 and p2, let's say we'd like to find the straight line distance (this is also called the
    // euclidean distance by the way) between points p1 and p3. To do that we'll need a representation of p3 in the A
    // frame which we can calulate using a Transformation matrix.

    Transformation T_BA;

    // To do that we'll need to  transform p3_B (point p3 expressed in the B coordinate frame) into the A frame.
}

void loop() {}