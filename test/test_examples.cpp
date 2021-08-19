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
        "drone pose: R: [[0.00,1.00,0.00],[-1.00,0.00,0.00],[0.00,0.00,1.00]] p: [[1.50],[0.00],[0.50]n"
        "other_drone_relative_to_GS: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: "
        "[[3.50],[0.00],[0.50]\n"
        "T_GS_otherdrone: R: [[0.00,0.97,-0.26],[-1.00,0.00,0.00],[0.00,0.26,0.97]] p: [[3.50],[0.00],[0.50]]\n"
        "T_otherdrone_GS: R: [[0.00,-1.00,0.00],[0.97,0.00,0.26],[-0.26,0.00,0.97]] p: [[-0.00],[-3.51],[0.42]]");
}

namespace ForwardKinematics
{
#include "../examples/ForwardKinematics/ForwardKinematics.ino"
}

TEST(Examples, ForwardKinematics)
{
    ForwardKinematics::setup();

    EXPECT_STREQ(Serial.buf.str().c_str(), "");
}

namespace TFGraph
{
#include "../examples/TFGraph/TFGraph.ino"
}

TEST(Examples, TFGraph)
{
    TFGraph::setup();

    std::cout << Serial.buf.str();

    // EXPECT_STREQ(Serial.buf.str().c_str(), "");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
