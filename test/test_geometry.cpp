#include <Geometry.h>
#include <gtest/gtest.h>

using namespace Geometry;

TEST(Geometry, so3Exp)
{
    AngularVelocity so3 = {0, 0.454, 0.262};

    Rotation SO3 = exp(so3);
    Rotation SO3_expected = {0.866, -0.25, 0.433, 0.25, 0.967, 0.058, -0.433, 0.058, 0.899};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(SO3(i, j), SO3_expected(i, j), 1e-3);
        }
    }
}

TEST(Geometry, so3LogExp)
{
    AngularVelocity so3 = {0.53453, 0.214, 2.5554};
    AngularVelocity so3_log_exp = log(exp(so3));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(so3(i), so3_log_exp(i), 1e-3);
    }
}

TEST(Geometry, se3LogExp)
{
    Twist se3({0.1, 0.2, 0.3}, {0.5773, 0.5773, 0.5773});
    Twist se3_log_exp = log(exp(se3));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(se3.w(i), se3_log_exp.w(i), 1e-3);
    }

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(se3.v(i), se3_log_exp.v(i), 1e-3);
    }
}

TEST(Geometry, AdjointConversions)
{
    Pose Tsb({-1, 0, 0, 0, 1, 0, 0, 0, -1}, {4, 0.4, 0});

    Twist Vb({0, 0, -2}, {2.8, 4, 0.0});

    Twist Vs = adjoint(Tsb) * Vb;
    Twist Vs_expected({0, 0, 2}, {-2, -4, 0.0});

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(Vs.w(i), Vs_expected.w(i), 1e-3);
    }

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(Vs.v(i), Vs_expected.v(i), 1e-3);
    }
}

TEST(Geometry, SE3Inverse)
{
    Pose Tsb({-1, 0, 0, 0, 1, 0, 0, 0, -1}, {4, 0.4, 0});

    auto identity = Tsb.inverse() * Tsb;

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(identity.p(i), 0.0, 1e-5);

        for (int j = 0; j < 3; ++j)
        {
            if (i == j)
            {
                EXPECT_NEAR(identity.R(i, j), 1.0, 1e-5);
            }
            else
            {
                EXPECT_NEAR(identity.R(i, j), 0.0, 1e-5);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
