#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <thread>
#include "node.h"
#include "edge.h"
#include "my_transforms.h"
#include "config.h"

using Coord3D = std::tuple<double, double, double>;

const std::vector<Coord3D> RefNodeCoords = config::DoublePyramid; // Assuming RefNodeCoords is a global variable

std::vector<Node> initialize_nodes(std::vector<Node> &nodeList, int num_nodes, double office_width, double office_length, double office_height) {
    std::vector<std::string> names;
    for (char c = 'A'; c <= 'Z'; ++c) {
        names.push_back(std::string(1, c));
    }

    if (num_nodes == 0) {
        for (int i = 0; i < names.size(); ++i) {
            nodeList.push_back(Node(names[i]));
        }
    } else {
        std::vector<std::vector<Coord3D>> coordlist;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0, office_width);

        for (int i = 0; i < num_nodes; ++i) {
            nodeList.push_back(Node(names[i]));
            double x = std::round(dist(gen) * 2) / 2;
            double y = std::round(dist(gen) * 2) / 2;
            double z = std::round(dist(gen) * 2) / 2;
            coordlist.push_back({std::make_tuple(x, y, z)});
        }

        RefNodeCoords = coordlist; // Assuming RefNodeCoords is a global variable
    }

    std::cout << "List of nodes:" << std::endl;
    for (const Node &node : nodeList) {
        std::cout << node.uuid << std::endl;
    }
    std::cout << std::endl;

    return nodeList;
}

std::vector<Edge> create_edges(const std::vector<Node>& nodeList, const std::vector<std::vector<Coord3D>> &RefNodeCoords, bool RSSI, bool PLOT) {
    std::vector<Edge> NetworkEdges;

    for (int i = 0; i < nodeList.size(); ++i) {
        std::vector<Edge> table;
        for (int j = 0; j < nodeList.size(); ++j) {
            if (i != j) {
                table.push_back(Edge(&nodeList[i], &nodeList[j], MyTransforms::distanceToRssi(RefNodeCoords[i], RefNodeCoords[j])));
            }
        }

        if (RSSI) {
            std::sort(table.begin(), table.end(), [](const Edge& a, const Edge& b) {
                return a.rssi > b.rssi;
            });
        }

        NetworkEdges.insert(NetworkEdges.end(), table.begin(), table.end());
    }

    std::cout << "Reference Network Edges:" << std::endl;
    for (int i = 0; i < NetworkEdges.size(); ++i) {
        std::cout << "Node " << NetworkEdges[i].node1->uuid << ":= Edges" << std::endl;
    }

    if (PLOT) {
        // Call your plotting function here
    }

    return NetworkEdges;
}

void run_simulation(const std::vector<Node>& nodeList, const std::vector<Edge>& NetworkEdges) {
    std::vector<std::thread> thread_list;
    for (int i = 0; i < nodeList.size(); ++i) {
        thread_list.push_back(std::thread(&Node::main, &nodeList[i], std::ref(NetworkEdges)));
    }

    for (std::thread& thread : thread_list) {
        thread.join();
    }
}

int main() {
    std::vector<Node> nodeList;
    std::vector<std::vector<double>> RefNodeCoords;
    double office_width = 10.0; // Replace with your values
    double office_length = 10.0;
    double office_height = 3.0;
    int num_nodes = 5; // Replace with your desired number of nodes
    bool RSSI = true; // Set to true if RSSI should be used
    bool PLOT = true; // Set to true if plotting should be done

    initialize_nodes(nodeList, num_nodes, office_width, office_length, office_height);
    create_edges(nodeList, RefNodeCoords, RSSI, PLOT);
    run_simulation(nodeList, NetworkEdges);

    return 0;
}
