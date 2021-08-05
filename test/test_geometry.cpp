#include <Geometry.h>
#include <gtest/gtest.h>

using namespace Geometry;

TEST(Geometry, so3_exp)
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

TEST(Geometry, so3_log_exp)
{
    AngularVelocity so3 = {0.53453, 0.214, 2.5554};
    AngularVelocity so3_log_exp = log(exp(so3));

    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(so3(i), so3_log_exp(i), 1e-3);
    }
}

TEST(Geometry, se3_log_exp)
{
    SpatialVelocity se3 = {0.1, 0.2, 0.3, 0.5773, 0.5773, 0.5773};
    SpatialVelocity se3_log_exp = log(exp(se3));

    for (int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(se3(i), se3_log_exp(i), 1e-3);
    }
}

TEST(Geometry, SE3_adj)
{
    Transformation Tsb = {-1, 0, 0, 4, 0, 1, 0, 0.4, 0, 0, -1, 0, 0, 0, 0, 1};

    SpatialVelocity Vb = {0, 0, -2, 2.8, 4, 0.0};

    auto Vs = adj(Tsb) * Vb;
    SpatialVelocity Vs_expected = {0, 0, 2, -2, -4, 0.0};

    for (int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(Vs(i), Vs_expected(i), 1e-3);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// import unittest
// import numpy as np
// from utils.geometry import exp, log, adj

// class TestGeometry(unittest.TestCase):
//     def test_so3_exp(self):
//         so3 = np.array([[0, 0.866, 0.5], [0, 0.866, 0.5]]) * 0.524

//         SO3 = exp(so3)
//         SO3_from_the_textbook = np.array(
//             [
//                 [0.866, -0.25, 0.433],
//                 [0.25, 0.967, 0.058],
//                 [-0.433, 0.058, 0.899],
//             ]
//         )

//         self.assertTrue(np.allclose(SO3[0], SO3_from_the_textbook, atol=1e-3))
//         self.assertTrue(np.allclose(SO3[1], SO3_from_the_textbook, atol=1e-3))

//     def test_log_exp_so3(self):

//         so3 = np.array([[0.53453, 0.214, 2.5554], [0.6, 0.8, 1.0]])
//         self.assertTrue(np.allclose(so3, log(exp(so3)), atol=1e-5))

//     def test_log_exp_se3(self):

//         se3 = np.array([[0.1, 0.2, 0.3, 0.5773, 0.5773, 0.5773], [0.2, 0.4, 0.6, 0.2, 0.2, 0.2]])
//         self.assertTrue(np.allclose(se3, log(exp(se3)), atol=1e-5))

//     def test_SE3_adj(self):
//         Tsb = np.array([[-1, 0, 0, 4], [0, 1, 0, 0.4], [0, 0, -1, 0], [0, 0, 0, 1]])

//         Vs = np.array([0, 0, 2, -2, -4, 0.0])
//         Vb = np.array([0, 0, -2, 2.8, 4, 0.0])

//         self.assertTrue(np.allclose(Vs, adj(Tsb) @ Vb, atol=1e-5))

// if __name__ == "__main__":
//     unittest.main()
