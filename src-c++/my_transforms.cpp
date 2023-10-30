#include "my_transforms.h"

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