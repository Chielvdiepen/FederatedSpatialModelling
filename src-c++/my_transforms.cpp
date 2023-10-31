#include "my_transforms.h"

double MyTransforms::distanceToRssi(const std::vector<double> &nodeCoord1, const std::vector<double> &nodeCoord2) {
    double c1x, c1y, c1z, c2x, c2y, c2z;
    std::tie(c1x, c1y, c1z) = std::make_tuple(nodeCoord1[0], nodeCoord1[1], nodeCoord1[2]);
    std::tie(c2x, c2y, c2z) = std::make_tuple(nodeCoord2[0], nodeCoord2[1], nodeCoord2[2]);

    double distance = std::sqrt(std::pow(c1x - c2x, 2) + std::pow(c1y - c2y, 2) + std::pow(c1z - c2z, 2)); 
    double rssi = -70.0 - 10.0 * 2.0 * log10(distance);

    return rssi;
}

double MyTransforms::rssiToDistance(double rssi) {
    return std::pow(10, (-rssi - 70) / 20);
}

std::vector<std::vector<double>> MyTransforms::roll(double phi) {
    phi = M_PI * phi / 180.0;
    return {{1, 0, 0},
            {0, std::cos(phi), -std::sin(phi)},
            {0, std::sin(phi), std::cos(phi)}};
}

std::vector<std::vector<double>> MyTransforms::pitch(double theta) {
    theta = M_PI * theta / 180.0;
    return {{std::cos(theta), 0, std::sin(theta)},
            {0, 1, 0},
            {-std::sin(theta), 0, std::cos(theta)}};
}

std::vector<std::vector<double>> MyTransforms::yaw(double psi) {
    psi = M_PI * psi / 180.0;
    return {{std::cos(psi), -std::sin(psi), 0},
            {std::sin(psi), std::cos(psi), 0},
            {0, 0, 1}};
}