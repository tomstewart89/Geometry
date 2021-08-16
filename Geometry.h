#pragma once

#include <math.h>

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Rotations.h"

namespace Geometry
{
using Translation = BLA::Matrix<3>;
using Rotation = BLA::Matrix<3, 3>;
using LinearVelocity = BLA::Matrix<3>;
using AngularVelocity = BLA::Matrix<3>;

class Transformation
{
   public:
    Rotation R;
    Translation p;

    Transformation() = default;
    Transformation(const Rotation& R_, const Translation& p_);

    template <typename MemT>
    Transformation(const BLA::Matrix<4, 4, MemT>& mat)
        : R(mat.template Submatrix<3, 3>(0, 0)), p(mat.template Submatrix<3, 1>(0, 3))
    {
    }

    template <class MemT>
    Transformation& operator=(const BLA::Matrix<4, 4, MemT>& mat)
    {
        R = mat.template Submatrix<3, 3>(0, 0);
        p = mat.template Submatrix<3, 1>(0, 3);

        return *this;
    }

    Transformation operator*(const Transformation& other);
    Translation operator*(const Translation& other);

    Transformation inv() const;
};

class SpatialVelocity
{
   public:
    AngularVelocity w;
    LinearVelocity v;

    SpatialVelocity() = default;
    SpatialVelocity(const AngularVelocity& w_, const LinearVelocity& v_);
    SpatialVelocity(const BLA::Matrix<6>& mat);

    SpatialVelocity& operator=(const BLA::Matrix<6>& mat);

    SpatialVelocity operator*(float theta);
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

Print& operator<<(Print& strm, const Transformation& T);
Print& operator<<(Print& strm, const SpatialVelocity& T);

}  // namespace Geometry
