#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <string>
#include <tuple>
#include <utility>
#include <set>
#include "edge.h"
#include "triangle.h"
#include "adjacent.h"
#include "config.h"

class Node {

public:
    Node(const std::string& HWuuid);
    std::string GetUUID() const;
    void PrintNodeInfo() const;
    void ScanSurroundings(const std::map<std::string, std::vector<Edge>>& networkEdges);
    bool RequestNodeSearch(Node& targetNode, Node& searchedNode);
    std::pair<Node*, int> NodeSearch(const Node& checkFor);
    Edge* CheckEdge(const Node& checkEdgeDst);
    bool AddEdge(Node& node1, Node& node2, double rssi);
    bool CreateEdgeWith(const std::pair<Node*, double>& otherSur);
    bool CheckTriangle(const Triangle& checkTriangle);
    Triangle* TriangleSearch(const Edge& oppositeEdge);
    std::pair<Adjacent*, int> AdjDictSearch(const Edge& baseEdge, const std::string& thirdNodeUUID);
    Triangle* RequestTriangleSearch(Node& targetNode, const Edge& searchedEdge);
    Adjacent* RequestAdjTriangleSearch(Node& targetNode, const Edge& searchedEdge);
    bool CreateTriangleWith(Edge& edge1, Edge& edge2);
    void TriangleProcedure();
    void CreateAdjTriangles();
    double GetMapAngle(Node& defaultOtherNode, Node& otherNode, double baseAltiX, double altiX, double baseAltiH, double altiH);
    std::pair<std::pair<std::string, double>, double> MapAltiRequest(Node& askNode, const Edge& baseEdge, double selfAngle);
    void MapAdjacents();
    void MakeMap();
    void TranslateCoord();
    void Init(const std::map<std::string, std::vector<Edge>>& networkEdges);
    void Main(const std::map<std::string, std::vector<Edge>>& networkEdges);

private:
    std::string uuid;
    std::tuple<double, double, double> coord;
    std::vector<std::pair<Node*, double>> SurNodes;
    std::vector<Edge> adjEdgeList;
    std::vector<Edge> oppEdgeList;
    std::vector<Triangle> triangleList;
    std::map<Edge, std::vector<Adjacent>> adjTrianglesDict;
    std::map<Edge, std::map<std::string, std::tuple<double, double, double>>> mapCoordsDict;
    std::map<Edge, std::vector<Edge>> mapEdgeSetDict;
    std::map<Edge, std::map<std::string, std::tuple<double, double, double>>> mappingDict;
    std::map<Edge, std::map<std::string, std::tuple<double, double, double>>> transCoordsDict;
};

#endif  // NODE_H
