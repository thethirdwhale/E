#include "main.h"
#include "variables.hpp"

double boundRad (double Radians){
    Radians = fmod(Radians, 6.2831);
    if (Radians < 0) Radians += 6.2831;
    return Radians;
}

double boundDeg (double Degrees){
    Degrees = fmod(Degrees, 360);
    if (Degrees < 0) Degrees += 360;
    return Degrees;
}
double cap(double value, double cap){
    if (value > cap) return cap;
    else if (value < -cap) return -cap;
    else return value;
}

double absadd(double value, double add){
    if (value > 0) value += add;
    else if (value < 0) value -= add;
    return value;
}