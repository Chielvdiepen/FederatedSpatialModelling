#ifndef EDGE_H
#define EDGE_H

#include "node.h"
#include "my_transforms.h"
#include <string>
#include "node.h"

class Edge {

public:
    Edge(Node src, Node dst, double rssi);
    std::pair<int, int> id;
    Node src;
    Node dst;
    double rssi;
    double dist;
    
    std::string toString() const;
    friend std::ostream &operator << (std::ostream &os, const Edge &edge);
    bool operator==(const Edge &other) const;
    size_t hash() const;
    bool compare(const Edge &other) const;

private:
    // No private members    
};

#endif // EDGE_H
