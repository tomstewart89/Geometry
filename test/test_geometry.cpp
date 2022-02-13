#include <Geometry.h>
#include <gtest/gtest.h>

using namespace Geometry;
using namespace BLA;

TEST(Geometry, so3Exp)
{
    AngularVelocity so3 = Matrix<3>(0, 0.454, 0.262);

    Rotation SO3 = exp(so3);
    Rotation SO3_expected = Matrix<3, 3>(0.866, -0.25, 0.433, 0.25, 0.967, 0.058, -0.433, 0.058, 0.899);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(SO3(i, j), SO3_expected(i, j), 1e-3);
        }
    }

    // Check the edge case when angular velocity is zero
    so3 = Matrix<3>(0.0, 0.0, 0.0);
    SO3 = exp(so3);
    SO3_expected = Identity<3>();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(SO3(i, j), SO3_expected(i, j), 1e-3);
        }
    }
}

TEST(Geometry, so3Log)
{
    Rotation SO3 = Identity<3>();

    AngularVelocity so3 = log(SO3);
    AngularVelocity so3_expected = Zeros<3>();

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(so3(i), so3_expected(i), 1e-3);
    }
}

TEST(Geometry, so3LogExp)
{
    AngularVelocity so3 = Matrix<3>(0.53453, 0.214, 2.5554);
    AngularVelocity so3_log_exp = log(exp(so3));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(so3(i), so3_log_exp(i), 1e-3);
    }
}

TEST(Geometry, se3LogExp)
{
    Twist se3 = Matrix<6>(0.1, 0.2, 0.3, 0.5773, 0.5773, 0.5773);
    Twist se3_log_exp = log(exp(se3));

    for (int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(se3(i), se3_log_exp(i), 1e-3);
    }
}

TEST(Geometry, AdjointConversions)
{
    Pose Tsb(Matrix<3, 3>(-1, 0, 0, 0, 1, 0, 0, 0, -1), {4, 0.4, 0});

    Twist Vb = Matrix<6>(0, 0, -2, 2.8, 4, 0.0);

    Twist Vs = adjoint(Tsb) * Vb;
    Twist Vs_expected = Matrix<6>(0, 0, 2, -2, -4, 0.0);

    for (int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(Vs(i), Vs_expected(i), 1e-3);
    }
}

TEST(Geometry, SE3Inverse)
{
    Pose Tsb(Matrix<3, 3>(-1, 0, 0, 0, 1, 0, 0, 0, -1), {4, 0.4, 0});

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
