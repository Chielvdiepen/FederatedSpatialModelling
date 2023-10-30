#include "triangle.h"

Triangle::Triangle(Edge edge1, Edge edge2, Edge edge3): edges({edge1, edge2, edge3}) {
    area = getArea();
    nodes = {edge1.src, edge3.src, edge3.dst};
    
    for (const Node &node : nodes) {
        unique.push_back(node.uuid);
    }
    std::sort(unique.begin(), unique.end());
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());

    std::stringstream ss;
    for (int i : unique) {
        ss << i << "|";
    }
    ID = ss.str();

    contribution = area / std::max({edge1.dist, edge2.dist, edge3.dist});

    angle = getAngle();
    altitudeH = getAltitudeH();
    altitudeX = getAltitudeX();
}



std::string Triangle::toString() const {
    std::string edgesStr;
    for (const Edge &edge: edges) {
        edgesStr += edge.toString() + ", ";
    }
    
    return "Triangle " + ID + ": Area= " + std::to_string(area) +
           ", Contribution= " + std::to_string(contribution) +
           ", Edges= " + edgesStr +
           ", Angles=(" + std::to_string(angle.first) + "," +
           std::to_string(angle.second) + "), Altitude=(x=[(To " +
           std::to_string(altitudeX[0].first) + "," +
           std::to_string(altitudeX[0].second) + "),(To " +
           std::to_string(altitudeX[1].first) + "," +
           std::to_string(altitudeX[1].second) + ")], H=(" +
           std::to_string(altitudeH.first) + "," +
           std::to_string(altitudeH.second) + "))";
}

bool Triangle::operator==(const Triangle &other) const {
    return unique == other.unique;
}

bool Triangle::hasEdge(Edge edgeToCheck) const {
    for (const Edge& edge : edges) {
        if (edge.compare(edgeToCheck)) {
            return true;
        }
    }
    return false;
}

Node Triangle::getLastNode(Edge baseEdge) const {
    for (const Node &node: nodes) {
        if (node != baseEdge.src && node != baseEdge.dst) {
            return node;
        }
    }
}

Edge Triangle::getOtherBaseEdge(Edge baseEdge) const {
    if (edges[0].compare(baseEdge)) {
        return edges[1];
    } else {
        return edges[0];
    }
}

std::pair<int, double> Triangle::getAngle() const {
    double a = edges[2].dist;
    double b = edges[1].dist;
    double c = edges[0].dist;
    double p = (std::pow(b,2) + std::pow(c,2) - std::pow(a,2)) / (2 * b * c);
    double q = std::max(-1.0, std::min(1.0, p));
    double angle = std::acos(q) * 180 / M_PI;
    return std::make_pair(edges[0].src.uuid, angle);
}

double Triangle::getArea() const {
    double a = edges[2].dist;
    double b = edges[1].dist;
    double c = edges[0].dist;
    double s = (a + b + c) / 2;
    double div = s * (s - a) * (s - b) * (s - c);
    if (div < 0) {
        div = 0;
    }
    return std::sqrt(div);
}

std::pair<int, double> Triangle::getAltitudeH() const {
    double a = edges[2].dist;
    double height = (2 * area) / a;
    return std::make_pair(edges[0].src.uuid, height);
}

std::vector<std::pair<int, double>> Triangle::getAltitudeX() const {
    double abSquared = std::pow(edges[0].dist,2);
    double acSquared = std::pow(edges[1].dist,2);
    double hASquared = std::pow(altitudeH.second,2);

    double divB = abSquared - hASquared;
    double divC = acSquared - hASquared;

    divB = std::max(divB, 0.0);
    divC = std::max(divC, 0.0);

    return std::vector<std::pair<int, double>>{
        std::make_pair(edges[0].dst.uuid, std::sqrt(divB)),
        std::make_pair(edges[1].dst.uuid, std::sqrt(divC))
    };
}


// from edge import Edge
// import math
// import numpy as np

// class Triangle:

//     def __init__(self, edge1: Edge, edge2: Edge, edge3: Edge):
//         self.edges = [edge1,edge2,edge3]
//         self.area = self.getArea()
//         self.nodes = [edge1.src, edge3.src, edge3.dst] # startnode|opposite edge src node|opposite edge dst node
//         self.unique = sorted(set(node.uuid for node in self.nodes))
//         self.ID = "|".join(map(str, self.unique))
//         self.contribution = self.area / max(edge1.dist, edge2.dist, edge3.dist) # higher is better

//         self.angle = self.getAngle()
//         self.altiH = self.getAltitudeH()
//         self.altiX = self.getAltitudeX() # from both adjacent edge perspectives, list of two

//     def __str__(self) -> str:
//         edges_str = ', '.join(map(str, self.edges))
//         return f"Triangle {self.ID}: Area= {self.area:.3f}, Contribution= {self.contribution:.3f}, Edges= {edges_str}, Angles=({self.angle[0]},{self.angle[1]:.3f}), Altitude=(x=[(To {self.altiX[0][0]},{self.altiX[0][1]:.3f}),(To {self.altiX[1][0]},{self.altiX[1][1]:.3f})], H=({self.altiH[0]},{self.altiH[1]:.3f}))"
    
//     def __repr__(self):
//         return str(self)
    
//     def __hash__(self) -> int:
//         return hash(f'{self.unique}')
      
//     def __eq__(self, other: object) -> bool:
//         return self.__hash__() == other.__hash__()
    
//     # Checks if triangle has the edge_to_check in its edges
//     def hasEdge(self, edge_to_check: Edge):
//         for i, edge in enumerate(self.edges):
//             if edge.compare(edge_to_check):
//                 return True
//         return False

//     # Returns the last node spanning the triangle, based on the base_edge src and dst node
//     def getLastNode(self, base_edge: Edge):
//         for node in self.nodes:
//             if node != base_edge.src and node != base_edge.dst:
//                 return node 

//     def getOtherBaseEdge(self, base_edge: Edge):
//         if self.edges[0].compare(base_edge):
//             return self.edges[1]
//         else:
//             return self.edges[0]      

//     # Calculates the angle of the triangle from the node perspective corner     
//     def getAngle(self): # ab0 ac1 bc2              
//         a = self.edges[2].dist # edge bc  
//         b = self.edges[1].dist # edge ac  
//         c = self.edges[0].dist # edge ab
//         p = (b**2+c**2-a**2)/(2*b*c)
//         q = max(-1, min(1, p))
//         angle = np.rad2deg(np.arccos(q))
//         return (self.edges[0].src.uuid, angle)
    
//     # calculate the area of the triangle with Heron's formula
//     def getArea(self):
//         a = self.edges[2].dist # edge bc  
//         b = self.edges[1].dist # edge ac  
//         c = self.edges[0].dist # edge ab
//         # Heron's formula 
//         s = (a + b + c) / 2
//         div = s * (s - a) * (s - b) * (s - c)
//         # check if nodes are inline -> angle 0 or 180
//         if div < 0: 
//             div = 0
//         return math.sqrt(div) 

//     # Calculates the altitude height of the triangle from node perspective with Heron's formula, see getArea() 
//     # for example: self = A, ABC got edge ab,ac,bc -> altitude height is from corner A to side bc       
//     def getAltitudeH(self):
//         a = self.edges[2].dist # opposite edge bc         
//         height = (2 * self.area) / a
//         return (self.edges[0].src.uuid, height)
    
//     # Calculates the altitude X position with pythogoras from the node perspective corner onto the opposite edge. 
//     # for example: self = A, ABC got edge ab,ac,bc -> altitude x position is from perspective of A, and lays on the opposite edge (0,0,0)->(x,0,0)   
//     def getAltitudeX(self):  # ab0 ac1 bc2
//         ab_squared = self.edges[0].dist ** 2
//         ac_squared = self.edges[1].dist ** 2
//         hA_squared = self.altiH[1] ** 2

//         # Pythagoras formula
//         divB = ab_squared - hA_squared
//         divC = ac_squared - hA_squared

//         divB = math.sqrt(max(divB, 0))
//         divC = math.sqrt(max(divC, 0))

//         return [
//             (self.edges[0].dst.uuid, divB),
//             (self.edges[1].dst.uuid, divC)
//         ]