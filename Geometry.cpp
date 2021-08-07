#include "Geometry.h"

using namespace BLA;

namespace Geometry
{
SpatialVelocity operator*(const BLA::Matrix<6, 6>& A, const SpatialVelocity& V)
{
    SpatialVelocity ret;

    ret.w = A.Submatrix<3, 3>(0, 0) * V.w + A.Submatrix<3, 3>(0, 3) * V.v;
    ret.v = A.Submatrix<3, 3>(3, 0) * V.w + A.Submatrix<3, 3>(3, 3) * V.v;

    return ret;
}

Matrix<3, 3> skew(const Matrix<3>& w)
{
    Matrix<3, 3> skew_m;

    skew_m(0, 0) = 0.0;
    skew_m(1, 1) = 0.0;
    skew_m(2, 2) = 0.0;

    skew_m(0, 1) = -w(2);
    skew_m(0, 2) = w(1);
    skew_m(1, 2) = -w(0);

    skew_m(1, 0) = -skew_m(0, 1);
    skew_m(2, 0) = -skew_m(0, 2);
    skew_m(2, 1) = -skew_m(1, 2);

    return skew_m;
}

Matrix<4, 4> skew(const Matrix<6>& V)
{
    Matrix<4, 4> skew_m = Zeros<4, 4>();

    skew_m.Submatrix<3, 3>(0, 0) = skew(V.Submatrix<3, 1>(0, 0));
    skew_m.Submatrix<3, 1>(0, 3) = V.Submatrix<3, 1>(3, 0);
    return skew_m;
}

Rotation exp(const AngularVelocity& w)
{
    auto theta = Norm((Matrix<3>)w);

    if (fabs(theta) < 1e-5)
    {
        theta = 1.0;
    }

    auto so3_skew = skew(w / theta);
    return Identity<3>() + so3_skew * sin(theta) + so3_skew * so3_skew * (1 - cos(theta));
}

Transformation exp(const SpatialVelocity& V)
{
    auto theta = Norm(V.w);

    if (fabs(theta) < 1e-5)
    {
        theta = 1.0;
    }

    auto so3_skew = skew(V.w / theta);
    auto so3_skew_sq = so3_skew * so3_skew;

    Transformation T;
    T.R = exp(V.w);
    T.p = (Identity<3>() * theta + so3_skew * (1.0 - cos(theta)) + so3_skew_sq * (theta - sin(theta))) * V.v;

    return T;
}

AngularVelocity log(const Rotation& R)
{
    AngularVelocity w;

    if (Norm(R - Identity<3>()) < 1e-5)
    {
        w = Zeros<3>();
    }
    else if (Trace(R) == -1.0)
    {
        if (R(0, 0) != -1.0)
        {
            w = R.Column(0) * 1.0 / sqrt(2.0 * (1.0 + R(0, 0)));
            w(0) += 1.0;
        }
        else
        {
            w = R.Column(1) * 1.0 / sqrt(2.0 * (1.0 + R(1, 1)));
            w(1) += 1.0;
        }
    }
    else
    {
        auto theta = acos((Trace(R) - 1.0) / 2.0);
        auto omega_skew = (R - ~R) / (2.0 * sin(theta));
        w(0) = omega_skew(2, 1);
        w(1) = omega_skew(0, 2);
        w(2) = omega_skew(1, 0);
        w *= theta;
    }

    return w;
}

SpatialVelocity log(const Transformation& T)
{
    SpatialVelocity V;

    if (Norm(T.R - Identity<3>()) < 1e-5)
    {
        V.w.Fill(0);
        V.v = T.p;
    }
    else
    {
        auto w = log(T.R);
        auto theta = Norm(w);
        auto so3_skew = skew(w / theta);
        auto so3_skew_sq = so3_skew * so3_skew;

        auto G_inv = Identity<3>() / theta - so3_skew / 2.0 +
                     so3_skew_sq * (1.0 / theta - cos(theta / 2.0) / sin(theta / 2.0) / 2.0);

        V.w = w;
        V.v = G_inv * T.p;
    }

    return V;
}

Matrix<6, 6> adjoint(const Transformation& T)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = T.R;
    adj_m.Submatrix<3, 3>(3, 3) = T.R;
    adj_m.Submatrix<3, 3>(3, 0) = skew(T.p) * T.R;

    return adj_m;
}

Matrix<6, 6> adjoint(const SpatialVelocity& V)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = skew(V.w);
    adj_m.Submatrix<3, 3>(3, 3) = skew(V.w);
    adj_m.Submatrix<3, 3>(3, 0) = skew(V.v);

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
