#include "FastMath.h"
#include "LUT.h"

const double Multiplier = 81.4873308631f;

double FastMath::FastSin(double theta){
    if (theta < 0.0f) theta += 6.28318530718f;
    if (theta >= 6.28318530718f) theta -= 6.28318530718f;    
    return SinTable[(int) (Multiplier*theta)] ;
    }
    
double FastMath::FastCos(double theta){
    return FastSin(1.57079632679f - theta);
    }

double FastMath::map(double x, double x1, double x2, double y1, double y2)
{
 //LIMITING
 if(x<x1) x = x1;
 if(x>x2) x = x2;
 
 return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}