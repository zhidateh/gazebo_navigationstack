#include "CubicSpline.hpp"

void CubicSpline::Spline::__init__(std::vector<double> _x, std::vector<double> _y)
{
    x = _x;
    y = _y;

    nx = _x.size();

    std::vector<double> h;
    for(int i= 1;i < nx;i++){
        h.push_back( _x.at(i) - _x.at(i-1));
    }

    a = _y;

    arma::mat A = calcA(h);
    arma::mat B = calcB(h);
    
    c = arma::solve(A,B,arma::solve_opts::fast);

    for(int i=0;i<nx -1; ++i){
        d.push_back( (c(i+1) - c(i))/(3*h.at(i)));      
        double tb = (a.at(i+1) - a.at(i))/h.at(i) - h.at(i) * (c(i+1) + 2*c(i))/3;
        b.push_back(tb);
    }

}

arma::mat CubicSpline::Spline::calcA(std::vector<double> vec)
{

    arma::mat A = arma::zeros(nx,nx);
    A(0,0) = 1.0;

    for(int i =0; i<nx-1;++i){
        if (i != nx-2) A(i+1,i+1) = 20 * (vec.at(i) + vec.at(i+1));
        A(i+1,i) = vec.at(i);
        A(i,i+1) = vec.at(i);
    }

    A(0,1) = 0.0;
    A(nx -1, nx-2) = 0.0;
    A(nx -1, nx-1) = 1.0;

    return A;
}

arma::mat CubicSpline::Spline::calcB(std::vector<double> vec)
{
    arma::mat B = arma::zeros(nx);

    for(int i =0; i<nx-2;++i){
        B(i+1) = 3 * ( a.at(i+2) - a.at(i+1)) / vec.at(i+1) - 3*(a.at(i+1) - a.at(i))/vec.at(i);
    }

    return B;
}

double CubicSpline::Spline::calc(double t)
{
    if (t < x.front()) return -1;
    else if (t > x.back()) return -1;

    int i = searchIndex(t);
    double dx = t - x.at(i);
    double result = a.at(i) + b.at(i) * dx + c(i) * std::pow(dx,2) + d.at(i) * std::pow(dx,3);

    return result;
}

double CubicSpline::Spline::calcd(double t)
{
    if (t < x.front()) return -1;
    else if (t > x.back()) return -1;

    int i = searchIndex(t);
    double dx = t - x.at(i);
    double result = b.at(i) + 2*c.at(i) *dx + 3*d.at(i) * std::pow(dx,2);

    return result;
}

double CubicSpline::Spline::calcdd(double t)
{
    if (t < x.front()) return -1;
    else if (t > x.back()) return -1;

    int i = searchIndex(t);
    double dx = t - x.at(i);
    double result = 2*c.at(i)  + 6*d.at(i) *dx;

    return result;
}

int CubicSpline::Spline::searchIndex(double _x)
{
    std::vector<double>::iterator z;
    z = std::upper_bound(x.begin(),x.end(),_x);
    return z - x.begin() -1;
}


/////////////////////////////////////////////////////////////////////////////////////////////



void CubicSpline::Spline2D::__init__(std::vector<double> _x, std::vector<double> _y)
{
    s = calcS(_x,_y);

    sx.__init__(s,_x);
    sy.__init__(s,_y);

}

std::vector<double> CubicSpline::Spline2D::calcS(std::vector<double> _x, std::vector<double> _y)
{
    for(int i= 1;i < nx;i++){
        double dx = _x.at(i) - _x.at(i-1);
        double dy =  _y.at(i) - _y.at(i-1);

        ds.push_back( std::sqrt(std::pow(dx,2) + std::pow(dy,2)));
    }

    double _s = ds.front();
    std::vector<double> cumsum ;
    //std::partial_sum(ds.begin(), ds.end(), cumsum.begin(), std::plus<double>()); 
    cumsum.push_back(0);
    double _sum =0;
    for(int i=0;i< ds.size();++i){
        cumsum.push_back( ds.at(i) + _sum );
        _sum += ds.at(i);
    }

    return cumsum;
}

std::pair<double,double> CubicSpline::Spline2D::calcPosition(double _s)
{
    double x = sx.calc(_s);
    double y = sy.calc(_s);
    return std::make_pair(x,y);
}

double CubicSpline::Spline2D::calcCurvature(double _s)
{
    double dx   = sx.calcd(_s);
    double ddx  = sx.calcdd(_s);
    double dy   = sy.calcd(_s);
    double ddy  = sy.calcdd(_s);
    double k = (ddy * dx - ddx * dy )/(std::pow((std::pow(dx,2) + std::pow(dy,2)),1.5));
    
    return k;
}


double CubicSpline::Spline2D::calcYaw(double _s)
{
    double dx    = sx.calcd(_s);
    double dy    = sy.calcd(_s);
    double yaw   = std::atan2(dy,dx);
    return yaw;
}

///////////////////////////////////////////////////////////////////////////

CubicSpline::SplinePath CubicSpline::calcSplinePath(std::vector<double> _x, std::vector<double> _y, double _ds )
{
    CubicSpline::SplinePath sp;
    
    CubicSpline::Spline2D sp2d;
    sp2d.__init__(_x,_y);
    std::vector<double> _s;
    for(double i=0; i <sp2d.s.back(); i += _ds ){
        _s.push_back(i);
    }

    for (double is: _s){
        std::pair<double,double> xy = sp2d.calcPosition(is);
        double ix = xy.first;
        double iy = xy.second;

        sp.rx.push_back(ix);
        sp.ry.push_back(iy);
        sp.ryaw.push_back(sp2d.calcYaw(is));
        sp.rk.push_back(sp2d.calcCurvature(is));

    }

    sp.s = _s;

    return sp;
}