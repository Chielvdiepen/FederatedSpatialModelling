#ifndef ADJACENT_H
#define ADJACENT_H

#include "triangle.h"
#include <string>

class Adjacent {
    
public:
    Triangle triangle;
    double mapAngle;
    std::pair<char, double> otherCoord;
    std::pair<char, double> basealtiX;
    std::pair<char, double> basealtiH;
    double mappedHeight;

    Adjacent(const Triangle& triangle);
    std::string toString() const;
    void setCoord(std::pair<double, double> coord);
    void setAngle(double angle);
    void setAltiX(std::pair<int, double> xpos);
    void setAltiH(std::pair<int, double> height);
    void setMapHeight(double height);

private:
    // No private members
};

#endif

