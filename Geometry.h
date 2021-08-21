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

class Pose
{
   public:
    Rotation R;
    Translation p;

    Pose() = default;
    Pose(const Rotation& R_, const Translation& p_);

    template <typename MemT>
    Pose(const BLA::Matrix<4, 4, MemT>& mat)
        : R(mat.template Submatrix<3, 3>(0, 0)), p(mat.template Submatrix<3, 1>(0, 3))
    {
    }

    template <class MemT>
    Pose& operator=(const BLA::Matrix<4, 4, MemT>& mat)
    {
        R = mat.template Submatrix<3, 3>(0, 0);
        p = mat.template Submatrix<3, 1>(0, 3);

        return *this;
    }

    Pose operator*(const Pose& other);
    Translation operator*(const Translation& other);

    Pose inv() const;
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
Pose exp(const SpatialVelocity& v);

AngularVelocity log(const Rotation& R);
SpatialVelocity log(const Pose& T);

BLA::Matrix<6, 6> adjoint(const Pose& T);
BLA::Matrix<6, 6> adjoint(const SpatialVelocity& v);

Print& operator<<(Print& strm, const Pose& T);
Print& operator<<(Print& strm, const SpatialVelocity& T);

}  // namespace Geometry
