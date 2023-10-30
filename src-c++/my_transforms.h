#ifndef MY_TRANSFORMS_H
#define MY_TRANSFORMS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

class MyTransforms {
    
public:
    static double rssiToDistance(double rssi);
    static std::vector<std::vector<double>> roll(double phi);
    static std::vector<std::vector<double>> pitch(double theta);
    static std::vector<std::vector<double>> yaw(double psi);
};

#endif