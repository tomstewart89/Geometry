#include "Geometry.h"

namespace Geometry
{
Pose::Pose(const Rotation& R_, const Translation& p_) : R(R_), p(p_) {}

Pose operator*(const Pose& pose, const Pose& other) { return Pose(pose.R * other.R, pose.R * other.p + pose.p); }

Translation operator*(const Pose& pose, const Translation& other) { return pose.R * other + pose.p; }

Twist operator*(const Pose& pose, const Twist& twist) { return adjoint(pose) * twist; }

Twist operator*(const Twist& twist, const Twist& other) { return adjoint(twist) * other; }

Wrench operator*(const Pose& pose, const Wrench& wrench) { return ~adjoint(pose) * wrench; }

Pose Pose::inverse() const { return Pose(~R, -(~R * p)); }

BLA::Matrix<3, 3> skew(const BLA::Matrix<3>& w)
{
    BLA::Matrix<3, 3> skew_m;

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

BLA::Matrix<4, 4> skew(const BLA::Matrix<6>& V)
{
    BLA::Matrix<4, 4> skew_m = BLA::Zeros<4, 4>();

    skew_m.Submatrix<3, 3>(0, 0) = skew(V.Submatrix<3, 1>(0, 0));
    skew_m.Submatrix<3, 1>(0, 3) = V.Submatrix<3, 1>(3, 0);
    return skew_m;
}

Rotation exp(const AngularVelocity& w)
{
    auto theta = Norm((BLA::Matrix<3>)w);

    if (theta == 0)
    {
        return BLA::Identity<3>();
    }
    else
    {
        auto so3_skew = skew(w / theta);
        return BLA::Identity<3>() + so3_skew * sin(theta) + so3_skew * so3_skew * (1 - cos(theta));
    }
}

Pose exp(const Twist& V)
{
    auto theta = Norm(V.Submatrix<3, 1>(0, 0));
    Pose T;

    if (theta == 0)
    {
        T.R = BLA::Identity<3>();
        T.p = V.Submatrix<3, 1>(3, 0);
    }
    else
    {
        auto so3_skew = skew(V.Submatrix<3, 1>(0, 0) / theta);
        auto so3_skew_sq = so3_skew * so3_skew;

        T.R = exp(V.Submatrix<3, 1>(0, 0));
        T.p = (BLA::Identity<3>() * theta + so3_skew * (1.0 - cos(theta)) + so3_skew_sq * (theta - sin(theta))) *
              V.Submatrix<3, 1>(3, 0);
    }

    return T;
}

AngularVelocity log(const Rotation& R)
{
    AngularVelocity w;

    if (Norm(R - BLA::Identity<3>()) < 1e-5)
    {
        w = BLA::Zeros<3>();
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

Twist log(const Pose& T)
{
    Twist V;

    if (Norm(T.R - BLA::Identity<3>()) < 1e-5)
    {
        V.Submatrix<3, 1>(0, 0).Fill(0);
        V.Submatrix<3, 1>(3, 0) = T.p;
    }
    else
    {
        auto w = log(T.R);
        auto theta = Norm(w);
        auto so3_skew = skew(w / theta);
        auto so3_skew_sq = so3_skew * so3_skew;

        auto G_inv = BLA::Identity<3>() / theta - so3_skew / 2.0 +
                     so3_skew_sq * (1.0 / theta - cos(theta / 2.0) / sin(theta / 2.0) / 2.0);

        V.Submatrix<3, 1>(0, 0) = w;
        V.Submatrix<3, 1>(3, 0) = G_inv * T.p;
    }

    return V;
}

BLA::Matrix<6, 6> adjoint(const Pose& T)
{
    BLA::Matrix<6, 6> adj_m = BLA::Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = T.R;
    adj_m.Submatrix<3, 3>(3, 3) = T.R;
    adj_m.Submatrix<3, 3>(3, 0) = skew(T.p) * T.R;

    return adj_m;
}

BLA::Matrix<6, 6> adjoint(const Twist& V)
{
    BLA::Matrix<6, 6> adj_m = BLA::Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = skew(V.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 3) = skew(V.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 0) = skew(V.Submatrix<3, 1>(3, 0));

    return adj_m;
}

Print& operator<<(Print& strm, const Pose& T)
{
    strm.print("R: ");
    strm << T.R;
    strm.print(" p: ");
    strm << T.p;

    return strm;
}

Print& operator<<(Print& strm, const Twist& V)
{
    strm.print("w: ");
    strm << V.Submatrix<3, 1>(0, 0);
    strm.print(" v: ");
    strm << V.Submatrix<3, 1>(3, 0);

    return strm;
}

}  // namespace Geometry
