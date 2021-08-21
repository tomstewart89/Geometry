#include <BasicLinearAlgebra.h>
#include <Geometry.h>
#include <gtest/gtest.h>

using namespace BLA;

namespace HowToUse
{
#include "../examples/HowToUse/HowToUse.ino"
}

TEST(Examples, HowToUse)
{
    HowToUse::setup();

    EXPECT_STREQ(
        Serial.buf.str().c_str(),
        "distance from drone to ground station: 1.58\n"
        "drone pose: R: [[0.00,1.00,0.00],[-1.00,0.00,0.00],[0.00,0.00,1.00]] p: [[1.50],[0.00],[0.50]]\n"
        "other_drone_relative_to_GS: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: "
        "[[3.50],[0.00],[0.50]]\n"
        "T_GS_otherdrone: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: [[3.50],[0.00],[0.50]]\n"
        "T_otherdrone_GS: R: [[0.00,-1.00,0.00],[0.97,0.00,0.26],[-0.26,0.00,0.97]] p: [[-0.00],[-3.51],[0.42]]");
}

namespace EulerAnglesExample
{
#include "../examples/EulerAngles/EulerAngles.ino"
}

TEST(Examples, EulerAngles)
{
    EulerAnglesExample::setup();

    EXPECT_STREQ(Serial.buf.str().c_str(),
                 "[[-1.00,0.00,-0.00],[0.00,1.00,-0.00],[-0.00,-0.00,-1.00]]\n"
                 "[[1.00,0.00,-0.00],[0.00,1.00,-0.00],[0.00,0.00,1.00]]\n"
                 "Gimbal locked: [[0.00,1.00,-0.00],[-0.00,0.00,1.00],[1.00,0.00,0.00]]\n"
                 "Still gimbal locked: [[-0.00,0.00,-1.00],[-0.00,1.00,0.00],[1.00,0.00,-0.00]]\n"
                 "Tricky rotation in euler angles[[1.57],[-0.79],[3.14]]");
}

#include "../examples/PoseGraph/PoseGraph.h"

namespace PoseGraphExample
{
#include "../examples/PoseGraph/PoseGraph.ino"
}

TEST(Examples, PoseGraph)
{
    PoseGraphExample::setup();

    EXPECT_STREQ(Serial.buf.str().c_str(),
                 "D frame measured from A:\n"
                 "R: [[1.00,0.00,0.00],[0.00,1.00,0.00],[0.00,0.00,1.00]] p: [[1.00],[1.00],[1.00]]\n"
                 "Still the same offset from A to D:\n"
                 "R: [[1.00,0.00,0.00],[0.00,1.00,0.00],[0.00,0.00,1.00]] p: [[1.00],[1.00],[1.00]]\n"
                 "Couldn't add transform from B to E\n"
                 "The pose of a frame relative to itself:\n"
                 "R: [[1.00,0.00,0.00],[0.00,1.00,0.00],[0.00,0.00,1.00]] p: [[0.00],[0.00],[0.00]]\n"
                 "Back to where we started:\n"
                 "R: [[1.00,0.00,0.00],[0.00,1.00,0.00],[0.00,0.00,1.00]] p: [[-0.00],[-0.00],[-0.00]]\n");
}

namespace InverseDynamics
{
#include "../examples/InverseDynamics/InverseDynamics.ino"
}

TEST(Examples, InverseDynamics)
{
    InverseDynamics::setup();

    EXPECT_STREQ(Serial.buf.str().c_str(), "");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
