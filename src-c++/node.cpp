#include "node.h"

    Node(std::string HWuuid) : uuid(HWuuid) {
        coord = std::make_tuple(0, 0, 0);
    }

    void scanSurroundings(std::map<std::string, std::vector<Edge>> network_edges) {
        std::set<std::pair<Node*, double>> surNodeSet;
        for (Edge edge : network_edges[uuid]) {
            surNodeSet.insert(std::make_pair(edge.dst, edge.rssi));
        }
        SurNodes.assign(surNodeSet.begin(), surNodeSet.end());
        if (RSSI) {
            std::sort(SurNodes.begin(), SurNodes.end(),
                [](const std::pair<Node*, double>& a, const std::pair<Node*, double>& b) {
                    return a.second > b.second;
                });
        }
    }

    bool compare(Node* other) {
        return uuid == other->uuid;
    }

    bool requestNodeSearch(Node* target_node, Node* searched_Node) {
        // send BLE message to target and wait for response
        bool found = target_node->nodeSearch(searched_Node);
        if (!found) {
            std::cout << uuid << ": Node NOT visible" << std::endl;
        }
        return found;
    }

    bool nodeSearch(Node* checkFor) {
        for (const auto& surNode : SurNodes) {
            if (surNode.first == checkFor) {
                return true;
            }
        }
        return false;
    }

    Edge* checkEdge(Node* check_edgeDst) {
        for (Edge& edge : adjEdgeList) {
            if (edge.dst == check_edgeDst) {
                return &edge;
            }
        }
        return nullptr;
    }

    bool addEdge(Node* node1, Node* node2, double rssi) {
        if (!checkEdge(node2)) {
            Edge newEdge(node1, node2, rssi);
            adjEdgeList.push_back(newEdge);
            return true;
        }
        return false;
    }

    bool createEdgeWith(const std::pair<Node*, double>& other_Sur) {
        bool check = requestNodeSearch(other_Sur.first, this);
        if (!check) {
            std::cout << uuid << ": Can not form edge, Node not visible!" << std::endl;
            return false;
        }
        return addEdge(this, other_Sur.first, other_Sur.second);
    }

    bool checkTriangle(const Triangle& checkTriangle) {
        for (const Triangle& triangle : triangleList) {
            if (triangle == checkTriangle) {
                return true;
            }
        }
        return false;
    }

    Triangle* triangleSearch(const Edge& opposite_edge) {
        for (Triangle& triangle : triangleList) {
            if (triangle.hasEdge(opposite_edge)) {
                return &triangle;
            }
        }
        return nullptr;
    }

    std::pair<Adjacent, int> adjDictSearch(const Edge& baseEdge, const std::string& thirdNodeUUID) {
        for (int i = 0; i < adjTrianglesDict[baseEdge].size(); i++) {
            if (adjTrianglesDict[baseEdge][i].triangle.hasNode(thirdNodeUUID)) {
                return std::make_pair(adjTrianglesDict[baseEdge][i], i);
            }
        }
        return std::make_pair(Adjacent(), -1);
    }

    std::pair<bool, double> requestTriangleSearch(Node* target_Node, const Edge& searched_Edge) {
        return target_Node->triangleSearch(searched_Edge);
    }

    std::pair<bool, double> requestAdjTriangleSearch(Node* target_Node, const Edge& searched_Edge) {
        return target_Node->adjMapSearch(searched_Edge);
    }

    bool addTriangle(const Edge& edge1, const Edge& edge2, const Edge& common_edge) {
        Triangle temp(edge1, edge2, common_edge);
        if (!checkTriangle(temp) && temp.area() > 0.5) {
            oppEdgeList.push_back(common_edge);
            std::sort(oppEdgeList.begin(), oppEdgeList.end(),
                [](const Edge& a, const Edge& b) {
                    return a.rssi > b.rssi;
                });
            triangleList.push_back(temp);
            std::sort(triangleList.begin(), triangleList.end(),
                [](const Triangle& a, const Triangle& b) {
                    return a.area() < b.area();
                });
            return true;
        }
        return false;
    }

    Triangle* createTempTriangle(Node* defaultOtherNode, Node* otherNode) {
        auto checkForOther = requestNodeSearch(defaultOtherNode, otherNode);
        auto checkForDef = requestNodeSearch(otherNode, defaultOtherNode);
        if (!checkForOther.first || !checkForDef.first) {
            std::cout << uuid << ": Check Error " << defaultOtherNode->uuid << " " << otherNode->uuid << std::endl;
            return nullptr;
        }
        if (checkForOther.first == otherNode && checkForDef.first == defaultOtherNode) {
            Edge common_edge(defaultOtherNode, otherNode, checkForDef.second);
            Triangle temp(Edge(this, defaultOtherNode, nodeSearch(defaultOtherNode).second),
                         Edge(this, otherNode, nodeSearch(otherNode).second), common_edge);
            return &temp;
        }
        else {
            std::cout << uuid << ": Compare Error " << defaultOtherNode->uuid << " " << otherNode->uuid << std::endl;
            return nullptr;
        }
    }

    Triangle* requestTempTriangle(Node* target_Node, Node* defaultNode, Node* otherNode) {
        return target_Node->createTempTriangle(defaultNode, otherNode);
    }

    bool createTriangleWith(const Edge& edge1, const Edge& edge2) {
        Node* dst1 = edge1.dst;
        Node* dst2 = edge2.dst;
        if (compare(dst1) || compare(dst2)) {
            std::cout << uuid << ": Can not make Triangle with itself" << std::endl;
            return false;
        }

        auto check
