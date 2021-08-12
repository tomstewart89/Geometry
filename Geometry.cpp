#include "Geometry.h"

using namespace BLA;

namespace Geometry
{
Transformation::Transformation(const Rotation& R_, const Translation& p_) : R(R_), p(p_) {}

Transformation::Transformation(const Matrix<4, 4>& mat) : R(mat.Submatrix<3, 3>(0, 0)), p(mat.Submatrix<3, 1>(0, 3)) {}

Transformation& Transformation::operator=(const BLA::Matrix<4, 4>& mat)
{
    R = mat.Submatrix<3, 3>(0, 0);
    p = mat.Submatrix<3, 1>(0, 3);

    return *this;
}

Transformation Transformation::operator*(const Transformation& other)
{
    return Transformation(R * other.R, R * other.p + p);
}

Translation Transformation::operator*(const Translation& other) { return R * other + p; }

Transformation Transformation::inv() const { return Transformation(~R, -(~R * p)); }

SpatialVelocity::SpatialVelocity(const AngularVelocity& w_, const LinearVelocity& v_) : w(w_), v(v_) {}

SpatialVelocity::SpatialVelocity(const Matrix<6>& mat) : w(mat.Submatrix<3, 1>(0, 0)), v(mat.Submatrix<3, 1>(3, 0)) {}

SpatialVelocity& SpatialVelocity::operator=(const Matrix<6>& mat)
{
    w = mat.Submatrix<3, 1>(0, 0);
    v = mat.Submatrix<3, 1>(3, 0);

    return *this;
}

SpatialVelocity SpatialVelocity::operator*(float theta) { return SpatialVelocity(w * theta, v * theta); }

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

}  // namespace Geometry
