#include "adjacent.h"

Adjacent::Adjacent(const Triangle& triangle) : triangle(triangle) {
    mapAngle = 0.0;
    otherCoord = std::make_pair(0.0, 0.0);
    basealtiX = std::make_pair(0, 0.0);
    basealtiH = std::make_pair(0, 0.0);
    mappedHeight = 0.0;
}

std::string Adjacent::toString() const {
    return "Triangle(" + (triangle.ID) +
           "): base-coord=(" + std::to_string(otherCoord.first) + "," + std::to_string(otherCoord.second) +
           "), map angle= " + std::to_string(mapAngle) +
           " | xpos altitude=(" + std::to_string(basealtiX.first) + "," + std::to_string(basealtiX.second) +
           ") | altitude height=(" + std::to_string(basealtiH.first) + "," + std::to_string(basealtiH.second) +
           ")-> Mapped Height = " + std::to_string(mappedHeight);
}

void Adjacent::setCoord(std::pair<double, double> coord) {
    otherCoord = coord;
}

void Adjacent::setAngle(double angle) {
    mapAngle = angle;
}

void Adjacent::setAltiX(std::pair<int, double> xpos) {
    basealtiX = xpos;
}

void Adjacent::setAltiH(std::pair<int, double> height) {
    basealtiH = height;
}

void Adjacent::setMapHeight(double height) {
    mappedHeight = height;
}
