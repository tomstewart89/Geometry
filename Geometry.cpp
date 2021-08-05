// #include "Geometry.h"

// Point Point::CrossProduct(Point &p)
// {
//     Point ret;

//     ret.X() = (*this).Y() * p.Z() - (*this).Z() * p.Y();
//     ret.Y() = (*this).Z() * p.X() - (*this).X() * p.Z();
//     ret.Z() = (*this).X() * p.Y() - (*this).Y() * p.X();

//     return ret;
// }

// float Point::Magnitude()
// {
//     float ret = 0;

//     for (int i = 0; i < 3; i++)
//         ret += pow((*this)(i), 2);

//     return sqrt(ret);
// }

// float Point::DotProduct(Point &obj)
// {
//     float sum = 0;

//     for (int i = 0; i < 3; i++)
//         sum += (*this)(i)*obj(i);

//     return sum;
// }

// Rotation &Rotation::FromEulerAngles(float psi, float theta, float phi)
// {
//     (*this)(0, 0) = cos(phi) * cos(theta);
//     (*this)(1, 0) = cos(theta) * sin(phi);
//     (*this)(2, 0) = -sin(theta);

//     (*this)(0, 1) = cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi);
//     (*this)(1, 1) = cos(psi) * cos(phi) + sin(psi) * sin(phi) * sin(theta);
//     (*this)(2, 1) = cos(theta) * sin(psi);

//     (*this)(0, 2) = sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta);
//     (*this)(1, 2) = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
//     (*this)(2, 2) = cos(psi) * cos(theta);

//     return (*this);
// }

// Matrix<3, 2> Rotation::ToEulerAngles()
// {
//     Matrix<3, 2> ret;

//     if ((*this)(2, 0) != 0)
//     {
//         ret(1, 0) = -asin((*this)(2, 0));
//         ret(1, 1) = M_PI - ret(1, 0);

//         ret(0, 0) = atan2((*this)(2, 1) / cos(ret(1, 0)), (*this)(2, 2) / cos(ret(1, 0)));
//         ret(0, 1) = atan2((*this)(2, 1) / cos(ret(1, 1)), (*this)(2, 2) / cos(ret(1, 1)));

//         ret(2, 0) = atan2((*this)(1, 0) / cos(ret(1, 0)), (*this)(0, 0) / cos(ret(1, 0)));
//         ret(2, 1) = atan2((*this)(1, 0) / cos(ret(1, 1)), (*this)(0, 0) / cos(ret(1, 1)));
//     }
//     else
//     {
//         ret(2, 0) = ret(2, 1) = 0;

//         if ((*this)(2, 0) == -1)
//         {
//             ret(1, 0) = ret(1, 1) = M_PI_2;
//             ret(0, 0) = ret(0, 1) = atan2((*this)(0, 1), (*this)(0, 2));
//         }
//         else
//         {
//             ret(1, 0) = ret(1, 1) = -M_PI_2;
//             ret(0, 0) = ret(0, 1) = atan2(-(*this)(0, 1), -(*this)(0, 2));
//         }
//     }

//     return ret;
// }

// Rotation &Rotation::RotateX(float phi)
// {
//     float tmp1, tmp2;

//     tmp1 = (*this)(1, 0) * cos(phi) - (*this)(2, 0) * sin(phi);
//     tmp2 = (*this)(2, 0) * cos(phi) + (*this)(1, 0) * sin(phi);
//     (*this)(1, 0) = tmp1;
//     (*this)(2, 0) = tmp2;

//     tmp1 = (*this)(1, 1) * cos(phi) - (*this)(2, 1) * sin(phi);
//     tmp2 = (*this)(2, 1) * cos(phi) + (*this)(1, 1) * sin(phi);
//     (*this)(1, 1) = tmp1;
//     (*this)(2, 1) = tmp2;

//     tmp1 = (*this)(1, 2) * cos(phi) - (*this)(2, 2) * sin(phi);
//     tmp2 = (*this)(2, 2) * cos(phi) + (*this)(1, 2) * sin(phi);
//     (*this)(1, 2) = tmp1;
//     (*this)(2, 2) = tmp2;

//     return (*this);
// }

// Rotation &Rotation::RotateY(float theta)
// {
//     float tmp1, tmp2;

//     tmp1 = (*this)(0, 0) * cos(theta) + (*this)(2, 0) * sin(theta);
//     tmp2 = (*this)(2, 0) * cos(theta) - (*this)(0, 0) * sin(theta);
//     (*this)(0, 0) = tmp1;
//     (*this)(2, 0) = tmp2;

//     tmp1 = (*this)(0, 1) * cos(theta) + (*this)(2, 1) * sin(theta);
//     tmp2 = (*this)(2, 1) * cos(theta) - (*this)(0, 1) * sin(theta);
//     (*this)(0, 1) = tmp1;
//     (*this)(2, 1) = tmp2;

//     tmp1 = (*this)(0, 2) * cos(theta) + (*this)(2, 2) * sin(theta);
//     tmp2 = (*this)(2, 2) * cos(theta) - (*this)(0, 2) * sin(theta);
//     (*this)(0, 2) = tmp1;
//     (*this)(2, 2) = tmp2;

//     return (*this);
// }

// Rotation &Rotation::RotateZ(float psi)
// {
//     float tmp1, tmp2;

//     tmp1 = (*this)(0, 0) * cos(psi) - (*this)(1, 0) * sin(psi);
//     tmp2 = (*this)(1, 0) * cos(psi) + (*this)(0, 0) * sin(psi);
//     (*this)(0, 0) = tmp1;
//     (*this)(1, 0) = tmp2;

//     tmp1 = (*this)(0, 1) * cos(psi) - (*this)(1, 1) * sin(psi);
//     tmp2 = (*this)(1, 1) * cos(psi) + (*this)(0, 1) * sin(psi);
//     (*this)(0, 1) = tmp1;
//     (*this)(1, 1) = tmp2;

//     tmp1 = (*this)(0, 2) * cos(psi) - (*this)(1, 2) * sin(psi);
//     tmp2 = (*this)(1, 2) * cos(psi) + (*this)(0, 2) * sin(psi);
//     (*this)(0, 2) = tmp1;
//     (*this)(1, 2) = tmp2;

//     return (*this);
// }

// Transformation &Transformation::operator*=(Transformation &obj)
// {
//     p.Matrix<3>::operator=(R *obj.p + p);
//     R.Matrix<3, 3>::operator=(R *obj.R);

//     return *this;
// }

// Transformation Transformation::operator*(Transformation &obj)
// {
//     Transformation ret;

//     ret.p.Matrix<3>::operator=(R *obj.p + p);
//     ret.R.Matrix<3, 3>::operator=(R *obj.R);

//     return ret;
// }

// float &Transformation::operator()(int row, int col)
// {
//     static float dummy;

//     if (col == 3)
//         return (row == 3) ? (dummy = 1) : p(row);
//     else
//         return (row == 3) ? (dummy = 0) : R(row, col);
// }

// Transformation &Transformation::RotateX(float phi)
// {
//     Point tmp;
//     R.RotateX(phi);

//     tmp.X() = p.X();
//     tmp.Y() = cos(phi) * p.Y() - sin(phi) * p.Z();
//     tmp.Z() = sin(phi) * p.Y() + cos(phi) * p.Z();

//     p = tmp;

//     return *this;
// }

// Transformation &Transformation::RotateY(float theta)
// {
//     Point tmp;
//     R.RotateY(theta);

//     tmp.X() = cos(theta) * p.X() - sin(theta) * p.Z();
//     tmp.Y() = p.Y();
//     tmp.Z() = sin(theta) * p.X() + cos(theta) * p.Z();

//     p = tmp;

//     return *this;
// }

// Transformation &Transformation::RotateZ(float psi)
// {
//     Point tmp;
//     R.RotateZ(psi);

//     tmp.X() = cos(psi) * p.X() - sin(psi) * p.Y();
//     tmp.Y() = sin(psi) * p.X() + cos(psi) * p.Y();
//     tmp.Z() = p.Z();

//     p = tmp;

//     return *this;
// }

// Transformation &Transformation::Translate(float x, float y, float z)
// {
//     (*this).p(0) += x;
//     (*this).p(1) += y;
//     (*this).p(2) += z;

//     return (*this);
// }

// Print &operator<<(Print &strm, const Point &obj)
// {
//     strm << (const Matrix<3, 1> &)obj;
//     return strm;
// }

// Print &operator<<(Print &strm, const Rotation &obj)
// {
//     strm << (const Matrix<3, 3> &)obj;
//     return strm;
// }

// // Stream inserter operator for printing to strings or the serial port
// Print &operator<<(Print &strm, const Transformation &obj)
// {
//     strm << '{';

//     for (int i = 0; i < 4; i++)
//     {
//         strm << '{';

//         for (int j = 0; j < 4; j++)
//         {
//             if (j == 3)
//                 strm << ((i == 3) ? 1 : obj.p(i, j));
//             else
//                 strm << ((i == 3) ? 0 : obj.R(i, j));

//             strm << ((j == 4 - 1) ? '}' : ',');
//         }

//         strm << (i == 4 - 1 ? '}' : ',');
//     }
//     return strm;
// }

// class SE3
// {
//     float elems[6];
// };

// class SO3
// {
// };

// class so3
// {
// };

// class se3
// {
//     Matrix<3> linear;
//     Matrix<3> angular;
// };

// class Twist
// {
//     Matrix<3> linear;
//     Matrix<3> angular;
// };

// @flatten_expand((3,))
// def exp_so3(so3):
//     theta = np.linalg.norm(so3, axis=1)[:, None]
//     theta[np.abs(theta) < np.finfo(float).eps] = 1.0

//     so3_skew = skew(so3 / theta)
//     return (
//         np.eye(3)
//         + np.sin(theta[:, :, None]) * so3_skew
//         + (1 - np.cos(theta[:, :, None])) * np.matmul(so3_skew, so3_skew)
//     )

// Matrix<4, 4> exp(const so3 &s)
// {
//     auto theta = Norm(so3);

// }

// se3 so3 SO3 SE3

// translation, angular velocity, rotation twist, transformation

// Let's go with twist, rotation, screw and transformation

// import numpy as np
// import math

// def flatten_expand(shape):
//     def decorator(function):
//         def wrapper(mat):
//             mat = np.array(mat, copy=False)  # in case it isn't one already
//             flattened_dims = mat.shape[: -len(shape)]
//             out = function(mat.reshape((-1,) + shape))
//             return out.reshape(flattened_dims + out.shape[1:])

//         return wrapper

//     return decorator

// @flatten_expand((3,))
// def skew_so3(so3):
//     skew_m = np.zeros(so3.shape[:-1] + (3, 3))
//     skew_m[:, 0, 1] = -so3[:, 2]
//     skew_m[:, 0, 2] = so3[:, 1]
//     skew_m[:, 1, 0] = so3[:, 2]
//     skew_m[:, 1, 2] = -so3[:, 0]
//     skew_m[:, 2, 0] = -so3[:, 1]
//     skew_m[:, 2, 1] = so3[:, 0]
//     return skew_m

// @flatten_expand((6,))
// def skew_se3(se3):
//     skew_m = np.zeros(se3.shape[:-1] + (4, 4))
//     skew_m[:, :3, :3] = skew(se3[:, :3])
//     skew_m[:, :3, 3] = se3[:, 3:]
//     return skew_m

// def skew(v):
//     if v.shape[-1] == 3:
//         return skew_so3(v)
//     elif v.shape[-1] == 6:
//         return skew_se3(v)
//     else:
//         raise ValueError()

// @flatten_expand((3, 3))
// def log_SO3(SO3):

//     so3 = np.zeros([SO3.shape[0], 3])

//     for i in range(SO3.shape[0]):
//         if np.all(SO3[i] == np.eye(3)):
//             so3[i] = 0
//         elif np.trace(SO3[i]) == -1:
//             if SO3[i, 0, 0] != -1:
//                 so3[i] = 1 / np.sqrt(2 * (1 + SO3[i, 0, 0])) * SO3[i][[0, 1, 2], [0, 0, 0]] + [1, 0, 0]
//             else:
//                 so3[i] = 1 / np.sqrt(2 * (1 + SO3[i, 1, 1])) * SO3[i][[0, 1, 2], [1, 1, 1]] + [0, 1, 0]
//         else:
//             theta = np.arccos((np.trace(SO3[i]) - 1) / 2)
//             omega_skew = (SO3[i] - SO3[i].T) / (2 * np.sin(theta))
//             so3[i] = omega_skew[[2, 0, 1], [1, 2, 0]] * theta

//     return so3

// @flatten_expand((4, 4))
// def log_SE3(SE3):
//     se3 = np.zeros([SE3.shape[0], 6])

//     is_identity = np.trace(SE3[:, :3, :3], axis1=1, axis2=2) == 3
//     se3[is_identity, 3:] = SE3[is_identity, :3, 3]

//     so3 = log_SO3(SE3[~is_identity, :3, :3])
//     theta = np.linalg.norm(so3, axis=1).reshape(-1, 1, 1)
//     so3_skew = skew(so3) / theta
//     so3_skew_sq = so3_skew @ so3_skew

//     G_inv = np.eye(3) / theta - so3_skew / 2 + (1 / theta - np.cos(theta / 2) / np.sin(theta / 2) / 2) * so3_skew_sq

//     se3[~is_identity, :3] = so3
//     se3[~is_identity, 3:] = (G_inv @ SE3[~is_identity, :3, 3, None])[:, :, 0]

//     return se3

// def log(m):
//     if m.shape[-2:] == (3, 3):
//         return log_SO3(m)
//     elif m.shape[-2:] == (4, 4):
//         return log_SE3(m)
//     else:
//         raise ValueError()

// @flatten_expand((3,))
// def exp_so3(so3):
//     theta = np.linalg.norm(so3, axis=1)[:, None]
//     theta[np.abs(theta) < np.finfo(float).eps] = 1.0

//     so3_skew = skew(so3 / theta)
//     return (
//         np.eye(3)
//         + np.sin(theta[:, :, None]) * so3_skew
//         + (1 - np.cos(theta[:, :, None])) * np.matmul(so3_skew, so3_skew)
//     )

// @flatten_expand((6,))
// def exp_se3(se3):
//     theta = np.linalg.norm(se3[:, :3], axis=1).reshape(-1, 1, 1)  # this lets us broadcast theta along dim 3 below
//     theta[np.abs(theta) < np.finfo(float).eps] = 1.0

//     so3_skew = skew(se3[:, :3] / theta[:, :, 0])
//     so3_skew_sq = so3_skew @ so3_skew
//     p = (np.eye(3) * theta + (1 - np.cos(theta)) * so3_skew + (theta - np.sin(theta)) * so3_skew_sq) @ se3[:, 3:,
//     None]

//     SE3 = np.zeros([se3.shape[0], 4, 4]) + np.eye(4)
//     SE3[:, :3, :3] = exp(se3[:, :3])
//     SE3[:, :3, 3] = p[:, :, 0]

//     return SE3

// def exp(v):
//     if v.shape[-1] == 3:
//         return exp_so3(v)
//     elif v.shape[-1] == 6:
//         return exp_se3(v)
//     else:
//         raise ValueError()

// @flatten_expand((4, 4))
// def adj_SE3(SE3):
//     adj_m = np.zeros((SE3.shape[0], 6, 6))
//     adj_m[:, :3, :3] = SE3[:, :3, :3]
//     adj_m[:, -3:, -3:] = SE3[:, :3, :3]
//     adj_m[:, -3:, :3] = np.matmul(skew(SE3[:, :3, 3]), SE3[:, :3, :3])
//     return adj_m

// @flatten_expand((6,))
// def adj_se3(se3):
//     adj_m = np.zeros((se3.shape[0], 6, 6))
//     adj_m[:, :3, :3] = skew(se3[:, :3])
//     adj_m[:, -3:, -3:] = adj_m[:, :3, :3]
//     adj_m[:, -3:, :3] = skew(se3[:, 3:])
//     return adj_m

// def adj(v):
//     if v.shape[-1] == 6:
//         return adj_se3(v)
//     elif v.shape[-2:] == (4, 4):
//         return adj_SE3(v)
//     else:
//         raise ValueError()

// @flatten_expand((3,))
// def euler_from_exp(so3):  # assumes rzyx
//     M = exp(so3)

//     cy = np.sqrt(M[:, 0, 0] * M[:, 0, 0] + M[:, 1, 0] * M[:, 1, 0])
//     sing_idx = cy < np.finfo(float).eps

//     rpy = np.zeros((so3.shape[0], 3))

//     rpy[~sing_idx, 0] = np.arctan2(M[~sing_idx, 1, 0], M[~sing_idx, 0, 0])  # ax
//     rpy[:, 1] = np.arctan2(-M[:, 2, 0], cy)  # ay
//     rpy[~sing_idx, 2] = np.arctan2(M[~sing_idx, 2, 1], M[~sing_idx, 2, 2])  # az
//     rpy[sing_idx, 2] = np.arctan2(-M[sing_idx, 1, 2], M[sing_idx, 1, 1])  # az

//     return rpy

// def align_vectors(v1, v2):
//     c = np.dot(v1, v2)
//     v = np.cross(v1, v2)
//     skew_v = skew(v)
//     return np.eye(3) + skew_v + np.matmul(skew_v, skew_v) * 1 / (1 + c)

// def quaternion_from_matrix(matrix):
//     q = np.empty((4,), dtype=np.float64)
//     M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
//     t = np.trace(M)
//     if t > M[3, 3]:
//         q[3] = t
//         q[2] = M[1, 0] - M[0, 1]
//         q[1] = M[0, 2] - M[2, 0]
//         q[0] = M[2, 1] - M[1, 2]
//     else:
//         i, j, k = 0, 1, 2
//         if M[1, 1] > M[0, 0]:
//             i, j, k = 1, 2, 0
//         if M[2, 2] > M[i, i]:
//             i, j, k = 2, 0, 1
//         t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
//         q[i] = t
//         q[j] = M[i, j] + M[j, i]
//         q[k] = M[k, i] + M[i, k]
//         q[3] = M[k, j] - M[j, k]
//     q *= 0.5 / math.sqrt(t * M[3, 3])
//     return q

// def quaternion_matrix(quaternion):
//     q = np.array(quaternion[:4], dtype=np.float64, copy=True)
//     nq = np.dot(q, q)

//     if nq < np.finfo(float).eps * 4.0:
//         return np.identity(4)

//     q *= math.sqrt(2.0 / nq)
//     q = np.outer(q, q)
//     return np.array(
//         (
//             (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0),
//             (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0),
//             (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
//             (0.0, 0.0, 0.0, 1.0),
//         ),
//         dtype=np.float64,
//     )

// def euler_from_matrix(M):  # assumes "sxyz"

//     cy = math.sqrt(M[0, 0] * M[0, 0] + M[1, 0] * M[1, 0])

//     if cy > np.finfo(float).eps * 4.0:
//         ax = math.atan2(M[2, 1], M[2, 2])
//         ay = math.atan2(-M[2, 0], cy)
//         az = math.atan2(M[1, 0], M[0, 0])
//     else:
//         ax = math.atan2(-M[1, 2], M[1, 1])
//         ay = math.atan2(-M[2, 0], cy)
//         az = 0.0

//     return ax, ay, az

// def euler_matrix(ai, aj, ak):  # assumes "sxyz"

//     si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
//     ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
//     cc, cs = ci * ck, ci * sk
//     sc, ss = si * ck, si * sk

//     return np.array(
//         [
//             [cj * ck, sj * sc - cs, sj * cc + ss, 0],
//             [cj * sk, sj * ss + cc, sj * cs - sc, 0],
//             [-sj, cj * si, cj * ci, 0],
//             [0, 0, 0, 1],
//         ]
//     )

// @flatten_expand((3,))
// def translation_matrix(direction):
//     M = np.eye(4)[None].repeat(direction.shape[0], axis=0)
//     M[:, :3, 3] = direction[:, :3]
//     return M

// @flatten_expand((3, 3))
// def rotation_matrix(rotation):
//     M = np.eye(4)[None].repeat(rotation.shape[0], axis=0)
//     M[:, :3, :3] = rotation[:, :3, :3]
//     return M

// if __name__ == "__main__":

//     se3 = np.array([[0.1, 0.2, 0.3, 0.5773, 0.5773, 0.5773], [0.2, 0.4, 0.6, 0.2, 0.2, 0.2]])

//     print(log(exp(se3)))