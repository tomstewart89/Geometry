#include <Geometry.h>

/*
 * When trying to describe a 3D rotation, we can use our trusty rotation matrices that we went through in the HowToUse
 * example, but as it turns out there's a whole slew of different rotation representations. This example will go through
 * euler angles which are petty common and are thought to be a nice simple way to represent rotation. As it turns out
 * though, euler angles are actually much more complicated than they seem and have a few nasty caveats.
 */

using namespace Geometry;
using namespace BLA;

void setup()
{
    Serial.begin(115200);

    // Euler angles decompose a 3D rotation into a sequence of rotations about the X, Y & Z axes (not necessarily in
    // that order, but we'll get to that). To create one we can use the EulerAngles class like so:
    EulerAngles euler(M_PI, 0, -M_PI);

    // This is the rotation we arrive at if we do (1) a +90 degree rotation about the x-axis followed by (2) a -90
    // degree rotation about the z-axis

    //   Z
    //   |  Y                                             Y
    //   | /                                             /
    //   |/          (1)                 (2)            /
    //   o----- X    -->     o----- X    -->    X -----o
    //                      /|                         |
    //                     / |                         |
    //                    Y  |                         |
    //                       Z                         Z

    // And if we convert these euler angles into a rotation matrix we can look at its column vectors and confirm that
    // this is the case.
    Serial << euler.to_rotation_matrix() << "\n";

    // The thing is, there's no rule that says that we have to apply these rotations in XYZ order, in fact "proper euler
    // angles" (according to wikipedia anyway) use ZXZ, XYX or similar while "Tait-Bryan angles" (which are also called
    // euler angles) use XYZ, YZX etc.

    // There's also another consideration of whether each successive rotation is applied relative to the original X/Y/Z
    // axes or wherever those axes are currently, having been rotated either once or twice already.

    // So when working with euler angles, it's not enough to just specify the three angles it's also necessary to
    // specify the rotation order and whether the axes we're rotating about are static or rotating. The defaults are XYZ
    // and static respectively, but you can override those like so:
    EulerAngles also_euler(M_PI, 0, -M_PI, EulerAngles::RotationFrame::Rotating, EulerAngles::RotationOrder::ZYZ);

    // Which gives a different rotation entirely:
    Serial << also_euler.to_rotation_matrix() << "\n";

    // And then there's gimbal lock. Gimbal lock occurs when in the course of our three successive rotations, two of the
    // axes of those rotations line up. For example consider:
    EulerAngles gimbal_locked(M_PI, -M_PI_2, M_PI_2);

    //   Z                                      X                 X
    //   |  Y                                   |                 |  Z
    //   | /                                    |                 | /
    //   |/          (1)                 (2)    |           (3)   |/
    //   o----- X    -->     o----- X    -->    o----- Z    -->   o----- Y
    //                      /|                 /
    //                     / |                /
    //                    Y  |               Y
    //                       Z

    Serial << "Gimbal locked: " << gimbal_locked.to_rotation_matrix() << "\n";

    // If you look closely you'll notice that both the first and the third rotations both occur about the X-axis. In
    // fact if we pick any arbitrary alpha here, we'll end up with the same rotation:
    float alpha = 0.324;
    EulerAngles still_gimbal_locked(-alpha, -M_PI_2, alpha);
    Serial << "Still gimbal locked: " << still_gimbal_locked.to_rotation_matrix() << "\n";

    // Also note that regardless what numbers we plug in for the first and third rotations, as long as the second is -pi
    // / 2, we'll never be able to describe a rotation whose X-axis doesn't point in the Z-direction of the original
    // frame (i.e we won't be able to rotate about the X-axis)

    // It's still possible to represent whatever 3D rotation we like using euler angles, but in the case above we'll
    // have to choose a different value for the second rotation.
    Rotation tricky_rotation = {-0.707, 0.0, -1.0, 0.0, 0.707, 0.0, 0.707, 0.707, 0.0};
    EulerAngles tricky_rotation_in_euler_angles(tricky_rotation);
    Serial << "Tricky rotation in euler angles" << tricky_rotation_in_euler_angles;

    // All this is to say, the despite their apparent simplicity, euler angles are actually pretty complicated. For most
    // things we're better off sticking with rotation matrices or quaternions.
}

void loop() {}
