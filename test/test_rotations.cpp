#include <OtherRotations.h>
#include <gtest/gtest.h>

using namespace Geometry;

TEST(MoreRotationFormats, QuaternionConversions)
{
    Quaternion q_expected = {0.707, 0, 0.707, 0};
    Rotation R_expected = {0, 0, 1, 0, -1, 0, 1, 0, 0};

    auto R = q_expected.to_rotation_matrix();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(R(i, j), R_expected(i, j), 1e-3);
        }
    }

    Quaternion q(R);

    EXPECT_NEAR(q.x(), q_expected.x(), 1e-3);
    EXPECT_NEAR(q.y(), q_expected.y(), 1e-3);
    EXPECT_NEAR(q.z(), q_expected.z(), 1e-3);
    EXPECT_NEAR(q.w(), q_expected.w(), 1e-3);
}

TEST(MoreRotationFormats, QuaternionMultiply)
{
    // Quaternions don't have to be unitary
    Quaternion q1(1, -2, 3, 4);
    Quaternion q2(-5, 6, 7, 8);
    Quaternion q = q1 * q2;

    EXPECT_FLOAT_EQ(q.x(), -44);
    EXPECT_FLOAT_EQ(q.y(), -14);
    EXPECT_FLOAT_EQ(q.z(), 48);
    EXPECT_FLOAT_EQ(q.w(), 28);

    // But they do if we plan to use them for representing rotations
    Quaternion q_AB = {0.707, 0, 0.707, 0};
    Quaternion q_BC = {1, 0, 0, 0};

    Rotation R_AC = q_AB.to_rotation_matrix() * q_BC.to_rotation_matrix();
    Rotation R_AC_expected = (q_AB * q_BC).to_rotation_matrix();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(R_AC(i, j), R_AC_expected(i, j), 1e-3);
        }
    }
}

TEST(MoreRotationFormats, EulerAngleConversions)
{
    EulerAngles euler_expected(0.1, 0.2, 0.3);

    Rotation R_expected = {0.936, -0.275, 0.218, 0.290, 0.956, -0.0370, -0.199, 0.098, 0.975};
    Rotation R = euler_expected.to_rotation_matrix();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(R(i, j), R_expected(i, j), 1e-3);
        }
    }

    EulerAngles euler(R);

    EXPECT_NEAR(euler.phi(), euler_expected.phi(), 1e-3);
    EXPECT_NEAR(euler.theta(), euler_expected.theta(), 1e-3);
    EXPECT_NEAR(euler.psi(), euler_expected.psi(), 1e-3);
}

TEST(MoreRotationFormats, OtherEulerAngleFramesAndOrders)
{
    EulerAngles euler_expected(0.1, 0.2, 0.3);

    for (int frame = 0; frame < 12; ++frame)
    {
        for (int order = 0; order < 12; ++order)
        {
            Rotation R_expected = euler_expected.to_rotation_matrix();

            EulerAngles euler(R_expected, EulerAngles::RotationFrame(frame), (EulerAngles::RotationOrder)order);

            // Several sets of euler angles can map to the same rotation matrix, so we'll convert back to a matrix to
            // remove that ambiguity
            Rotation R = euler.to_rotation_matrix();

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    EXPECT_NEAR(R(i, j), R_expected(i, j), 1e-3);
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
