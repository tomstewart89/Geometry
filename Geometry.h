#pragma once

#include <math.h>

#include <iostream>

#include "/home/tom/snap/arduino/61/Arduino/libraries/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "Arduino.h"

using namespace BLA;

namespace Geometry
{
using AngularVelocity = Matrix<3>;
using Rotation = Matrix<3, 3>;
using SpatialVelocity = Matrix<6>;
using Transformation = Matrix<4, 4>;

class Quaternion
{
    Matrix<4> elems;

   public:
    Quaternion(float x, float y, float z, float w);

    Quaternion(const Rotation& R);

    float& x() { return elems(0); }
    float& y() { return elems(1); }
    float& z() { return elems(2); }
    float& w() { return elems(3); }

    Rotation to_rotation_matrix() const;

    Quaternion operator*(const Quaternion& other) const;
};

class EulerAngles
{
   public:
    enum RotationFrame
    {
        Static = 0,
        Rotating
    };

    enum RotationOrder
    {
        XYZ = 0,
        XYX,
        XZY,
        XZX,
        YZX,
        YZY,
        YXZ,
        YXY,
        ZXY,
        ZXZ,
        ZYX,
        ZYZ
    };

    Matrix<3> angles;
    const RotationFrame frame;
    const RotationOrder order;

    EulerAngles(float ai, float aj, float ak, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    EulerAngles(const Rotation& R, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    Rotation to_rotation_matrix() const;
};

template <typename MemT>
Matrix<3, 3> skew(const Matrix<3, 1, MemT>& w)
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

Matrix<4, 4> skew(const Matrix<6>& v);

Rotation exp(const AngularVelocity& w);
Transformation exp(const SpatialVelocity& v);

AngularVelocity log(const Rotation& R);
SpatialVelocity log(const Transformation& T);

Matrix<6, 6> adjoint(const Transformation& T);
Matrix<6, 6> adjoint(const SpatialVelocity& v);

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
