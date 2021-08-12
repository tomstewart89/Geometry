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

    EXPECT_STREQ(Serial.buf.str().c_str(), "");
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
