#pragma once

#include <math.h>

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Rotations.h"

namespace Geometry
{
using Rotation = BLA::Matrix<3, 3>;

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