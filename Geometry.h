#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

#include <math.h>

// A point class for representing coordinates in a 3 dimensional space
class Point : public Matrix<3,1,float>
{
public:
    Point() { Fill(0); }
    Point(const Point &obj) : Matrix<3,1,float>() { (*this) = obj; }
    Point(const Matrix<3,1,float> &obj) { (*this) = obj; }

    float Magnitude();
    float DotProduct(Point &obj);
    Point CrossProduct(Point &p);

    float &X() { return (*this)(0); }
    float &Y() { return (*this)(1); }
    float &Z() { return (*this)(2); }

    template<class opMemT> Point &operator=(const Matrix<3,1,float,opMemT> &obj)
    {
        for(int i = 0; i < 3; i++)
            (*this)(i,0) = obj(i,0);

        return *this;
    }
};

// A rotation matrix class in a 3 dimensional space
class Rotation : public Matrix<3,3,float>
{
public:
    Rotation() { *this = Identity<3,3>(); }
    Rotation(const Rotation &obj) : Matrix<3,3,float>() { (*this) = obj; }
    Rotation(const Matrix<3,3,float> &obj) { (*this) = obj; }

    Rotation &FromEulerAngles(float phi, float theta, float psi);
    Matrix<3,2> ToEulerAngles();

    Rotation &RotateX(float phi);
    Rotation &RotateY(float theta);
    Rotation &RotateZ(float psi);

    template<class opMemT> Rotation &operator=(const Matrix<3,3,float,opMemT> &obj)
    {
        for(int i = 0; i < Rows(); i++)
            for(int j = 0; j < Cols(); j++)
                (*this)(i,j)  = obj(i,j);

        return *this;
    }
};

// A transformation matrix class (rotation plus a coordinate) in a 3 dimensional space
class Transformation
{
public:
    Rotation R;
    Point p;

    Transformation() { R = Identity<3,3>(); p.Fill(0); }
    Transformation(const Transformation &obj) { (*this) = obj; }

    Transformation &operator*=(Transformation &obj);
    Transformation operator*(Transformation &obj);

    float &operator()(int row, int col);

    float &X() { return p(0); }
    float &Y() { return p(1); }
    float &Z() { return p(2); }

    Transformation &RotateX(float phi);
    Transformation &RotateY(float theta);
    Transformation &RotateZ(float psi);

    Transformation &Translate(float x, float y, float z);

    template<class opMemT> Transformation &operator=(const Matrix<4,4,float,opMemT> &obj)
    {
        R = obj.Submatrix(Range<3>(0),Range<3>(0));
        p = obj.Submatrix(Range<3>(3),Range<1>(0));

        return *this;
    }
};

// Stream inserters operator for printing to strings or the serial port
Print &operator<<(Print &strm, const Point &obj);
Print &operator<<(Print &strm, const Rotation &obj);
Print &operator<<(Print &strm, const Transformation &obj);

#endif // GEOMETRY_H

