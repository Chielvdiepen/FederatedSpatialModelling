#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "edge.h"
#include "node.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>

class Triangle {

public:
    Triangle(Edge edge1, Edge edge2, Edge edge3);
    std::vector<Edge> edges;
    double area;
    std::vector<Node> nodes;
    std::vector<int> unique;
    std::string ID;
    double contribution;
    std::pair<int, double> angle;
    std::pair<int, double> altitudeH;
    std::vector<std::pair<int, double>> altitudeX;
    
    std::string toString() const;
    
    bool operator==(const Triangle &other) const;
    bool hasEdge(Edge edgeToCheck) const;
    Node getLastNode(Edge baseEdge) const;
    Edge getOtherBaseEdge(Edge baseEdge) const;

private:
    std::pair<int, double> getAngle() const;
    double getArea() const;
    std::pair<int, double> getAltitudeH() const;
    std::vector<std::pair<int, double>> getAltitudeX() const;
};

#endif
