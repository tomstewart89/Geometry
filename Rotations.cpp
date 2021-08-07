#include "Rotations.h"

using namespace BLA;

namespace Geometry
{
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
