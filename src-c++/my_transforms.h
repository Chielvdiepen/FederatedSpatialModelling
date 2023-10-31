#ifndef MY_TRANSFORMS_H
#define MY_TRANSFORMS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

class MyTransforms {
    
public:
    double distanceToRssi(const std::vector<double> &nodeCoord1, const std::vector<double> &nodeCoord2);
    static double rssiToDistance(double rssi);
    static std::vector<std::vector<double>> roll(double phi);
    static std::vector<std::vector<double>> pitch(double theta);
    static std::vector<std::vector<double>> yaw(double psi);
};

#endif