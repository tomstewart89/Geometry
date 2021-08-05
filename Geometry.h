#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>

#include <iostream>

#include "/home/tom/snap/arduino/61/Arduino/libraries/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "Arduino.h"

using namespace BLA;

namespace Geometry
{
using AngularVelocity = Matrix<3, 1>;
using Rotation = Matrix<3, 3>;
using SpatialVelocity = Matrix<6, 1>;
using Transformation = Matrix<4, 4>;

template <typename MemT>
Matrix<3, 3> skew(const Matrix<3, 1, MemT>& so3)
{
    Matrix<3, 3> skew_m;

    skew_m(0, 0) = 0.0;
    skew_m(0, 1) = -so3(2);
    skew_m(0, 2) = so3(1);
    skew_m(1, 0) = so3(2);
    skew_m(1, 1) = 0.0;
    skew_m(1, 2) = -so3(0);
    skew_m(2, 0) = -so3(1);
    skew_m(2, 1) = so3(0);
    skew_m(2, 2) = 0.0;

    return skew_m;
}

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

Matrix<6, 6> adj(const Transformation& SE3)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = SE3.Submatrix<3, 3>(0, 0);
    adj_m.Submatrix<3, 3>(3, 3) = SE3.Submatrix<3, 3>(0, 0);
    adj_m.Submatrix<3, 3>(3, 0) = skew(SE3.Submatrix<3, 1>(0, 3)) * SE3.Submatrix<3, 3>(0, 0);

    return adj_m;
}

Matrix<6, 6> adj(const SpatialVelocity& se3)
{
    Matrix<6, 6> adj_m = Zeros<6, 6>();

    adj_m.Submatrix<3, 3>(0, 0) = skew(se3.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 3) = skew(se3.Submatrix<3, 1>(0, 0));
    adj_m.Submatrix<3, 3>(3, 0) = skew(se3.Submatrix<3, 1>(0, 3));

    return adj_m;
}
}  // namespace Geometry

// // A point class for representing coordinates in a 3 dimensional space
// class Point : public Matrix<3, 1>
// {
//    public:
//     Point() { Fill(0); }
//     Point(const Point &obj) : Matrix<3, 1>() { (*this) = obj; }
//     Point(const Matrix<3, 1> &obj) { (*this) = obj; }

//     float Magnitude();
//     float DotProduct(Point &obj);
//     Point CrossProduct(Point &p);

//     float &X() { return (*this)(0); }
//     float &Y() { return (*this)(1); }
//     float &Z() { return (*this)(2); }

//     template <class opMemT>
//     Point &operator=(const Matrix<3, 1, opMemT> &obj)
//     {
//         for (int i = 0; i < 3; i++) (*this)(i, 0) = obj(i, 0);

//         return *this;
//     }
// };

// // A rotation matrix class in a 3 dimensional space
// class Rotation : public Matrix<3, 3>
// {
//    public:
//     Rotation() { *this = Identity<3, 3>(); }
//     Rotation(const Rotation &obj) : Matrix<3, 3>() { (*this) = obj; }
//     Rotation(const Matrix<3, 3> &obj) { (*this) = obj; }

//     Rotation &FromEulerAngles(float phi, float theta, float psi);
//     Matrix<3, 2> ToEulerAngles();

//     Rotation &RotateX(float phi);
//     Rotation &RotateY(float theta);
//     Rotation &RotateZ(float psi);

//     template <class opMemT>
//     Rotation &operator=(const Matrix<3, 3, opMemT> &obj)
//     {
//         for (int i = 0; i < Rows; i++)
//             for (int j = 0; j < Cols; j++) (*this)(i, j) = obj(i, j);

//         return *this;
//     }
// };

// // A transformation matrix class (rotation plus a coordinate) in a 3 dimensional space
// class Transformation
// {
//    public:
//     Rotation R;
//     Point p;

//     Transformation()
//     {
//         R = Identity<3, 3>();
//         p.Fill(0);
//     }
//     Transformation(const Transformation &obj) { (*this) = obj; }

//     Transformation &operator*=(Transformation &obj);
//     Transformation operator*(Transformation &obj);

//     float &operator()(int row, int col);

//     float &X() { return p(0); }
//     float &Y() { return p(1); }
//     float &Z() { return p(2); }

//     Transformation &RotateX(float phi);
//     Transformation &RotateY(float theta);
//     Transformation &RotateZ(float psi);

//     Transformation &Translate(float x, float y, float z);

//     template <class opMemT>
//     Transformation &operator=(const Matrix<4, 4, opMemT> &obj)
//     {
//         R = obj.Submatrix(Slice<0, 3>(), Slice<0, 3>());
//         p = obj.Submatrix(Slice<0, 3>(), Slice<3, 4>());

//         return *this;
//     }
// };

// // Stream inserters operator for printing to strings or the serial port
// Print &operator<<(Print &strm, const Point &obj);
// Print &operator<<(Print &strm, const Rotation &obj);
// Print &operator<<(Print &strm, const Transformation &obj);

#endif  // GEOMETRY_H
