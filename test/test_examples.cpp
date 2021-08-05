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

    EXPECT_STREQ(Serial.buf.str().c_str(),
                 "v(1): 43.67\n"
                 "B: [[9.79,9.33,11.62],[7.77,14.77,14.12],[11.33,15.72,12.12]]\n"
                 "identity matrix: [[1.00,-0.00,-0.00],[0.00,1.00,-0.00],[0.00,0.00,1.00]]");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
