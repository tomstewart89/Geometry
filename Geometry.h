#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include <math.h>

// A point class for representing coordinates in a 3 dimensional space
class Point : public Matrix<3,1,float>
{
public:
    Point() { Reset(); }
    Point(const Point &obj) : Matrix<3,1,float>() { (*this) = obj; }
    Point(const Matrix<3,1,float> &obj) { (*this) = obj; }

    void Reset();

    float Magnitude();
    float DotProduct(Point &obj);
    Point CrossProduct(Point &p);

    float &X() { return (*this)(0); }
    float &Y() { return (*this)(1); }
    float &Z() { return (*this)(2); }

    Point &operator=(const Matrix<3,1> &obj);
};

// A rotation matrix class in a 3 dimensional space
class Rotation : public Matrix<3,3,float>
{
public:
    Rotation() { Reset(); }
    Rotation(const Rotation &obj) : Matrix<3,3,float>() { (*this) = obj; }
    Rotation(const Matrix<3,3,float> &obj) { (*this) = obj; }

    void Reset();

    Rotation &FromEulerAngles(float phi, float theta, float psi);
    Matrix<3,2> ToEulerAngles();

    Rotation &RotateX(float phi);
    Rotation &RotateY(float theta);
    Rotation &RotateZ(float psi);

    Rotation &operator=(const Matrix<3,3> &obj);
};

// A transformation matrix class (rotation plus a coordinate) in a 3 dimensional space
class Transformation
{
    static float dummyElement;
public:
    Rotation R;
    Point p;

    Transformation() { Reset(); }
    Transformation(const Transformation &obj) { (*this) = obj; }

    void Reset();

    Transformation &operator*=(Transformation &obj);
    Transformation operator*(Transformation &obj);

    Transformation &operator=(const Matrix<4,4> &obj);

    float &operator()(int row, int col);

    float &X() { return p(0); }
    float &Y() { return p(1); }
    float &Z() { return p(2); }

    Transformation &RotateX(float phi);
    Transformation &RotateY(float theta);
    Transformation &RotateZ(float psi);

    Transformation &Translate(float x, float y, float z);
};

// Stream inserters operator for printing to strings or the serial port
Print &operator<<(Print &strm, const Point &obj);
Print &operator<<(Print &strm, const Rotation &obj);
Print &operator<<(Print &strm, const Transformation &obj);

#endif // GEOMETRY_H
