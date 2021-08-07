#include "Geometry.h"

namespace Geometry
{
Matrix<4, 4> skew(const Matrix<6>& se3)
{
    Matrix<4, 4> skew_m = Zeros<4, 4>();

    skew_m.Submatrix<3, 3>(0, 0) = skew(se3.Submatrix<3, 1>(0, 0));
    skew_m.Submatrix<3, 1>(0, 3) = se3.Submatrix<3, 1>(3, 0);
    return skew_m;
}

Rotation exp(const AngularVelocity& so3)
{
    auto theta = Norm((Matrix<3>)so3);

    if (fabs(theta) < 1e-5)
    {
        theta = 1.0;
    }

    auto so3_skew = skew(so3 / theta);
    return Identity<3>() + so3_skew * sin(theta) + so3_skew * so3_skew * (1 - cos(theta));
}

Transformation exp(const SpatialVelocity& se3)
{
    auto theta = Norm(se3.Submatrix<3, 1>(0, 0));

    if (fabs(theta) < 1e-5)
    {
        theta = 1.0;
    }

    auto so3_skew = skew(se3.Submatrix<3, 1>(0, 0) / theta);
    auto so3_skew_sq = so3_skew * so3_skew;

    Transformation SE3 = Identity<4>();
    SE3.Submatrix<3, 3>(0, 0) = exp(se3.Submatrix<3, 1>(0, 0));
    SE3.Submatrix<3, 1>(0, 3) =
        (Identity<3>() * theta + so3_skew * (1.0 - cos(theta)) + so3_skew_sq * (theta - sin(theta))) *
        se3.Submatrix<3, 1>(0, 3);

    return SE3;
}

AngularVelocity log(const Rotation& SO3)
{
    AngularVelocity so3;

    if (Norm(SO3 - Identity<3>()) < 1e-5)
    {
        so3 = Zeros<3>();
    }
    else if (Trace(SO3) == -1.0)
    {
        if (SO3(0, 0) != -1.0)
        {
            so3 = SO3.Column(0) * 1.0 / sqrt(2.0 * (1.0 + SO3(0, 0)));
            so3(0) += 1.0;
        }
        else
        {
            so3 = SO3.Column(1) * 1.0 / sqrt(2.0 * (1.0 + SO3(1, 1)));
            so3(1) += 1.0;
        }
    }
    else
    {
        auto theta = acos((Trace(SO3) - 1.0) / 2.0);
        auto omega_skew = (SO3 - ~SO3) / (2.0 * sin(theta));
        so3(0) = omega_skew(2, 1);
        so3(1) = omega_skew(0, 2);
        so3(2) = omega_skew(1, 0);
        so3 *= theta;
    }

    return so3;
}

SpatialVelocity log(const Transformation& SE3)
{
    SpatialVelocity se3;

    if (Norm(SE3.Submatrix<3, 3>(0, 0) - Identity<3>()) < 1e-5)
    {
        se3.Submatrix<3, 1>(0, 0).Fill(0);
        se3.Submatrix<3, 1>(0, 3) = SE3.Submatrix<3, 1>(0, 3);
    }
    else
    {
        auto so3 = log(SE3.Submatrix<3, 3>(0, 0));
        auto theta = Norm(so3);
        auto so3_skew = skew(so3 / theta);
        auto so3_skew_sq = so3_skew * so3_skew;

        auto G_inv = Identity<3>() / theta - so3_skew / 2.0 +
                     so3_skew_sq * (1.0 / theta - cos(theta / 2.0) / sin(theta / 2.0) / 2.0);

        se3.Submatrix<3, 1>(0, 0) = so3;
        se3.Submatrix<3, 1>(0, 3) = G_inv * SE3.Submatrix<3, 1>(0, 3);
    }

    return se3;
}

Matrix<6, 6> adjoint(const Transformation& SE3)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = SE3.Submatrix<3, 3>(0, 0);
    adj_m.Submatrix<3, 3>(3, 3) = SE3.Submatrix<3, 3>(0, 0);
    adj_m.Submatrix<3, 3>(3, 0) = skew(SE3.Submatrix<3, 1>(0, 3)) * SE3.Submatrix<3, 3>(0, 0);

    return adj_m;
}

Matrix<6, 6> adjoint(const SpatialVelocity& se3)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = skew(se3.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 3) = skew(se3.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 0) = skew(se3.Submatrix<3, 1>(0, 3));

    return adj_m;
}

Quaternion::Quaternion(float x, float y, float z, float w) : elems(x, y, z, w) {}

Quaternion::Quaternion(const Rotation& R)
{
    auto t = Trace(R) + 1.0;

    if (t > 1.0)
    {
        elems = {R(2, 1) - R(1, 2), R(1, 0) - R(0, 1), R(0, 2) - R(2, 0), t};
    }
    else
    {
        int i = 0, j = 1, k = 2;

        if (R(1, 1) > R(0, 0))
        {
            i = 1;
            j = 2;
            k = 0;
        }

        if (R(2, 2) > R(i, i))
        {
            i = 2;
            j = 0;
            k = 1;
        }

        elems(i) = R(i, i) - R(j, j) - R(k, k) + 1.0;
        elems(j) = R(i, j) + R(j, i);
        elems(k) = R(k, i) + R(i, k);
        elems(3) = R(k, j) - R(j, k);

        t = elems(i);
    }

    elems *= 1.0 / (2.0 * sqrt(t));
}

Rotation Quaternion::to_rotation_matrix() const
{
    auto nq = Norm(elems);

    if (nq < 1e-5)
    {
        return Identity<3>();
    }

    auto M = (elems * ~elems) * (2.0 / nq);

    return {1.0f - M(1, 1) - M(2, 2), M(0, 1) - M(2, 3),       M(0, 2) + M(1, 3),
            M(0, 1) + M(2, 3),        1.0 - M(0, 0) - M(2, 2), M(1, 2) - M(0, 3),
            M(0, 2) - M(1, 3),        M(1, 2) + M(0, 3),       1.0 - M(0, 0) - M(1, 1)};
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    return {
        elems(0) * other.elems(3) + elems(1) * other.elems(2) - elems(2) * other.elems(1) + elems(3) * other.elems(0),
        -elems(0) * other.elems(2) + elems(1) * other.elems(3) + elems(2) * other.elems(0) + elems(3) * other.elems(1),
        elems(0) * other.elems(1) - elems(1) * other.elems(0) + elems(2) * other.elems(3) + elems(3) * other.elems(2),
        -elems(0) * other.elems(0) - elems(1) * other.elems(1) - elems(2) * other.elems(2) + elems(3) * other.elems(3)};
}

EulerAngles::EulerAngles(float ai, float aj, float ak, RotationFrame frame_, RotationOrder order_)
    : angles(ai, aj, ak), frame(frame_), order(order_)
{
}

EulerAngles::EulerAngles(const Rotation& R, RotationFrame frame_, RotationOrder order_) : frame(frame_), order(order_)
{
    const bool parity = order % 4 / 2;
    const bool repetition = order % 2;

    int i = order / 4;
    int j = (i + parity + 1) % 3;
    int k = (i + 2 - parity) % 3;

    if (repetition)
    {
        auto sy = sqrt(R(i, j) * R(i, j) + R(i, k) * R(i, k));
        if (sy > 1e-5)
        {
            angles(0) = atan2(R(i, j), R(i, k));
            angles(1) = atan2(sy, R(i, i));
            angles(2) = atan2(R(j, i), -R(k, i));
        }
        else
        {
            angles(0) = atan2(-R(j, k), R(j, j));
            angles(1) = atan2(sy, R(i, i));
            angles(2) = 0.0;
        }
    }
    else
    {
        auto cy = sqrt(R(i, i) * R(i, i) + R(j, i) * R(j, i));
        if (cy > 1e-5)
        {
            angles(0) = atan2(R(k, j), R(k, k));
            angles(1) = atan2(-R(k, i), cy);
            angles(2) = atan2(R(j, i), R(i, i));
        }
        else
        {
            angles(0) = atan2(-R(j, k), R(j, j));
            angles(1) = atan2(-R(k, i), cy);
            angles(2) = 0.0;
        }
    }

    if (parity)
    {
        angles = -angles;
    }

    if (frame == RotationFrame::Rotating)
    {
        auto tmp = angles(0);
        angles(0) = angles(2);
        angles(2) = tmp;
    }
}

Rotation EulerAngles::to_rotation_matrix() const
{
    const bool parity = order % 4 / 2;
    const bool repetition = order % 2;

    auto angles_copy = angles;

    Rotation R;

    const int i = order / 4;
    const int j = (i + parity + 1) % 3;
    const int k = (i + 2 - parity) % 3;

    if (frame == RotationFrame::Rotating)
    {
        auto tmp = angles_copy(0);
        angles_copy(0) = angles_copy(2);
        angles_copy(2) = tmp;
    }

    if (parity)
    {
        angles_copy = -angles_copy;
    }

    auto si = sin(angles_copy(0));
    auto sj = sin(angles_copy(1));
    auto sk = sin(angles_copy(2));

    auto ci = cos(angles_copy(0));
    auto cj = cos(angles_copy(1));
    auto ck = cos(angles_copy(2));

    auto cc = ci * ck;
    auto cs = ci * sk;
    auto sc = si * ck;
    auto ss = si * sk;

    if (repetition)
    {
        R(i, i) = cj;
        R(i, j) = sj * si;
        R(i, k) = sj * ci;
        R(j, i) = sj * sk;
        R(j, j) = -cj * ss + cc;
        R(j, k) = -cj * cs - sc;
        R(k, i) = -sj * ck;
        R(k, j) = cj * sc + cs;
        R(k, k) = cj * cc - ss;
    }
    else
    {
        R(i, i) = cj * ck;
        R(i, j) = sj * sc - cs;
        R(i, k) = sj * cc + ss;
        R(j, i) = cj * sk;
        R(j, j) = sj * ss + cc;
        R(j, k) = sj * cs - sc;
        R(k, i) = -sj;
        R(k, j) = cj * si;
        R(k, k) = cj * ci;
    }

    return R;
}

}  // namespace Geometry
