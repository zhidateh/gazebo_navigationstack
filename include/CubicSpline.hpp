#define ARMA_USE_LAPACK
#include <iostream>
#include <armadillo>
#include <math.h>
#include <stdio.h>



namespace CubicSpline 
{
    class Spline;
    class Spline2D;
    struct SplinePath;
    SplinePath calcSplinePath(std::vector<double> _x, std::vector<double>_y, double _ds);
}

struct CubicSpline::SplinePath
{
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> ryaw;
    std::vector<double> rk;
    std::vector<double> s;
};


class CubicSpline::Spline
{
    //Spline(std::vector<double> _x, std::vector<double> _y);

public:
    std::vector<double> x,y,a,b,d;
    int nx;

    arma::mat c;
    arma::mat calcA(std::vector<double> vec);
    arma::mat calcB(std::vector<double> vec);

    void __init__(std::vector<double> _x, std::vector<double> _y);
    double calc(double t);
    double calcd(double t);
    double calcdd(double t);
    int searchIndex(double _x);

};

class CubicSpline::Spline2D
{
    //Spline2D(std::vector<double> _x, std::vector<double> _y);


public:

    void __init__(std::vector<double> _x, std::vector<double> _y);
    std::vector<double> s,ds;

    CubicSpline::Spline sx;
    CubicSpline::Spline sy;

    int nx;

    arma::mat c;
    arma::mat calcA(std::vector<double> vec);
    arma::mat calcB(std::vector<double> vec);

    std::vector<double> calcS(std::vector<double> _x, std::vector<double> _y);
    std::pair<double,double> calcPosition(double _s);
    double calcCurvature(double _s);
    double calcYaw(double _s);

};

