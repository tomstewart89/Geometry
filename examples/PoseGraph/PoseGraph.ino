#include "PoseGraph.h"

/*
 * In the HowToUse example we saw that multiple poses can be chained together to calculate a net transformation from one
 * frame to another. In this example we'll take that a little further and arrange these poses into a graph where the
 * nodes of that graph are coordinate frames and the edges are transforms that map one frame to the other. From there we
 * can traverse that graph and find the net transformation from any frame to any other frame so long as there exists a
 * sequence of transformations between them!
 *
 * This might seem a little abstract and pointless but actually if you've ever used ROS you may have come into contact
 * with a very handy package called (TF)[http://wiki.ros.org/tf] which provides similar functionality. Unlike TF, we
 * won't be bothering with timestamps here and we're using a graph rather than a tree so frames can have multiple
 * parents if they like.
 */

using namespace Geometry;
using namespace BLA;

void setup()
{
    Serial.begin(115200);

    // First off recall that if we have a bunch of poses all describing offsets from each other like so:
    Pose T_from_A_to_B, T_from_B_to_C, T_from_C_to_D;

    // Then we can find the net offset from A to D like so:
    Pose T_from_A_to_D = T_from_A_to_B * T_from_B_to_C * T_from_C_to_D;

    // Also for brevity, we generally drop the from__to_ and just use names like T_AD

    // If we want to manage all the offsets that we often encounter on a robot (especially a robot arm) then we can use
    // a graph! Here's one I prepared earlier:
    PoseGraph graph;

    // Let's add a few transforms to that graph to replicate that net transform from above
    graph.add_transform("A", "B", {Identity<3>(), {1, 0, 0}});
    graph.add_transform("B", "C", {Identity<3>(), {0, 1, 0}});
    graph.add_transform("C", "D", {Identity<3>(), {0, 0, 1}});

    auto lookup = graph.get_transform("A", "D");
    Serial << "D frame measured from A:\n" << lookup.transform << "\n";

    // We can throw in a bunch of others too without messing up the result:
    graph.add_transform("E", "D", {Identity<3>(), {1, 2, 3}});
    graph.add_transform("Z", "A", {Identity<3>(), {50, 0, 0}});

    Serial << "Still the same offset from A to D:\n" << graph.get_transform("A", "D").transform << "\n";

    // We can't however, add transforms that would introduce a loop into the graph, since that could mean that a frame
    // could be in two places at once depending on which transforms we used to find it; which doesn't make sense.
    bool could_add_transform = graph.add_transform("B", "E", {Identity<3>(), {1, 2, 3}});
    Serial << (could_add_transform ? "Could" : "Couldn't") << " add transform from B to E\n";

    // You might also be wondering, did I handle the edge case of looking up the pose of a frame relative to itself?
    // Well, yes I did! This should print as the identity matrix for rotation and zero translation:
    Serial << "The pose of a frame relative to itself:\n" << graph.get_transform("A", "A").transform << "\n";

    // As a final example, let's walk around in a circle and see that where we wind up is the same pose as where we
    // started:
    Pose translate_then_turn_left({0, -1, 0, 1, 0, 0, 0, 0, 1}, {0, 1, 0});

    graph.add_transform("bottom_left", "bottom_right", translate_then_turn_left);
    graph.add_transform("bottom_right", "top_right", translate_then_turn_left);
    graph.add_transform("top_right", "top_left", translate_then_turn_left);
    graph.add_transform("top_left", "back_where_we_started", translate_then_turn_left);

    // This too should print as the identity matrix for rotation and zero translation:
    Serial << "Back to where we started:\n"
           << graph.get_transform("bottom_left", "back_where_we_started").transform << "\n";
}

void loop() {}
