#pragma once

#include <math.h>

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Geometry.h"

namespace Geometry
{
class Rotation;

class Quaternion
{
    BLA::Matrix<4> elems;

   public:
    Quaternion() = default;

    Quaternion(float x, float y, float z, float w);

    Quaternion(const Rotation& R);

    float& x() { return elems(0); }
    float& y() { return elems(1); }
    float& z() { return elems(2); }
    float& w() { return elems(3); }

    Rotation to_rotation_matrix() const;

    Quaternion operator*(const Quaternion& other) const;

    friend Print& operator<<(Print& strm, const Quaternion& quat);
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

    float& first() { return angles(0); }
    float& second() { return angles(1); }
    float& third() { return angles(2); }

    const RotationFrame frame;
    const RotationOrder order;

    EulerAngles() = default;

    EulerAngles(float ai, float aj, float ak, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    EulerAngles(const Rotation& R, RotationFrame frame = RotationFrame::Static,
                RotationOrder order = RotationOrder::XYZ);

    Rotation to_rotation_matrix() const;

    friend Print& operator<<(Print& strm, const EulerAngles& euler);
};

}  // namespace Geometry
