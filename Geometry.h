#pragma once

#include <math.h>

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

namespace Geometry
{
using Translation = BLA::Matrix<3>;
using Rotation = BLA::Matrix<3, 3>;

class Transformation
{
   public:
    Rotation R;
    Translation p;

    Transformation() = default;
    Transformation(const Rotation& R_, const Translation& p_) : R(R_), p(p_) {}
    Transformation(const BLA::Matrix<4, 4>& mat) : R(mat.Submatrix<3, 3>(0, 0)), p(mat.Submatrix<3, 1>(0, 3)) {}

    Transformation& operator=(const BLA::Matrix<4, 4>& mat)
    {
        R = mat.Submatrix<3, 3>(0, 0);
        p = mat.Submatrix<3, 1>(0, 3);

        return *this;
    }

    Transformation operator*(const Transformation& other) { return Transformation(R * other.R, R * other.p + p); }

    Translation operator*(const Translation& other) { return R * other + p; }

    Transformation inv();
};

using LinearVelocity = BLA::Matrix<3>;
using AngularVelocity = BLA::Matrix<3>;

class SpatialVelocity
{
   public:
    AngularVelocity w;
    LinearVelocity v;

    SpatialVelocity() = default;
    SpatialVelocity(const AngularVelocity& w_, const LinearVelocity& v_) : w(w_), v(v_) {}
    SpatialVelocity(const BLA::Matrix<6>& mat) : w(mat.Submatrix<3, 1>(0, 0)), v(mat.Submatrix<3, 1>(3, 0)) {}

    SpatialVelocity& operator=(const BLA::Matrix<6>& mat)
    {
        w = mat.Submatrix<3, 1>(0, 0);
        v = mat.Submatrix<3, 1>(3, 0);

        return *this;
    }
};

SpatialVelocity operator*(const BLA::Matrix<6, 6>& A, const SpatialVelocity& V);

BLA::Matrix<3, 3> skew(const BLA::Matrix<3>& w);
BLA::Matrix<4, 4> skew(const BLA::Matrix<6>& v);

Rotation exp(const AngularVelocity& w);
Transformation exp(const SpatialVelocity& v);

AngularVelocity log(const Rotation& R);
SpatialVelocity log(const Transformation& T);

BLA::Matrix<6, 6> adjoint(const Transformation& T);
BLA::Matrix<6, 6> adjoint(const SpatialVelocity& v);

class Quaternion
{
    BLA::Matrix<4> elems;

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
    BLA::Matrix<3> angles;

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

    float& phi() { return angles(0); }
    float& theta() { return angles(1); }
    float& psi() { return angles(2); }

    const RotationFrame frame;
    const RotationOrder order;

    EulerAngles(float phi, float theta, float psi, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    EulerAngles(const Rotation& R, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    Rotation to_rotation_matrix() const;
};

}  // namespace Geometry
