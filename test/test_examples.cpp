#include <BasicLinearAlgebra.h>
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
