#pragma once

#include <math.h>

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "OtherRotations.h"

namespace Geometry
{
using Translation = BLA::Matrix<3>;
using LinearVelocity = BLA::Matrix<3>;

class AngularVelocity : public BLA::Matrix<3>
{
   public:
    AngularVelocity() = default;

    template <typename T>
    AngularVelocity(const BLA::Matrix<3, 1, T>& obj) : BLA::Matrix<3>(obj)
    {
    }

    template <typename T>
    AngularVelocity& operator=(const BLA::Matrix<3, 1, T>& obj)
    {
        BLA::Matrix<3>::operator=(obj);
        return *this;
    }
};

class Rotation : public BLA::Matrix<3, 3>
{
   public:
    Rotation() = default;

    template <typename T>
    Rotation(const BLA::Matrix<3, 3, T>& obj) : BLA::Matrix<3, 3>(obj)
    {
    }

    template <typename T>
    Rotation& operator=(const BLA::Matrix<3, 3, T>& obj)
    {
        BLA::Matrix<3, 3>::operator=(obj);
        return *this;
    }
};

class Twist : public BLA::Matrix<6>
{
   public:
    Twist() = default;

    template <typename T>
    Twist(const BLA::Matrix<6, 1, T>& obj) : BLA::Matrix<6>(obj)
    {
    }

    template <typename T>
    Twist& operator=(const BLA::Matrix<6, 1, T>& obj)
    {
        BLA::Matrix<6>::operator=(obj);
        return *this;
    }
};

class Wrench : public BLA::Matrix<6>
{
   public:
    Wrench() = default;

    template <typename T>
    Wrench(const BLA::Matrix<6, 1, T>& obj) : BLA::Matrix<6>(obj)
    {
    }

    template <typename T>
    Wrench& operator=(const BLA::Matrix<6, 1, T>& obj)
    {
        BLA::Matrix<6>::operator=(obj);
        return *this;
    }
};

class Pose
{
   public:
    Rotation R;
    Translation p;

    Pose() = default;
    Pose(const Rotation& R_, const Translation& p_);

    template <typename T>
    Pose(const BLA::Matrix<4, 4, T>& mat) : R(mat.template Submatrix<3, 3>(0, 0)), p(mat.template Submatrix<3, 1>(0, 3))
    {
    }

    template <class T>
    Pose& operator=(const BLA::Matrix<4, 4, T>& mat)
    {
        R = mat.template Submatrix<3, 3>(0, 0);
        p = mat.template Submatrix<3, 1>(0, 3);

        return *this;
    }

    Pose inverse() const;
};

Translation operator*(const Pose& pose, const Translation& other);
Pose operator*(const Pose& pose, const Pose& other);
Twist operator*(const Pose& pose, const Twist& other);
Wrench operator*(const Pose& pose, const Wrench& other);
Twist operator*(const Twist& twist, const Twist& other);

BLA::Matrix<3, 3> skew(const BLA::Matrix<3>& w);
BLA::Matrix<4, 4> skew(const BLA::Matrix<6>& v);

BLA::Matrix<6, 6> adjoint(const Pose& T);
BLA::Matrix<6, 6> adjoint(const Twist& v);

Rotation exp(const AngularVelocity& w);
Pose exp(const Twist& v);

AngularVelocity log(const Rotation& R);
Twist log(const Pose& T);

Print& operator<<(Print& strm, const Pose& T);
Print& operator<<(Print& strm, const Twist& T);

}  // namespace Geometry
