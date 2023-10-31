#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <tuple>

namespace config
{
    using Coord3D = std::tuple<double, double, double>;

    // Simulation parameters
    extern double office_width;
    extern double office_length;
    extern double office_height;
    
    extern bool DIM2D;
    extern int MAXTRIANGLES;
    extern bool RSSI;
    extern bool PLOT;

    extern int num_nodes;

    extern const std::vector<Coord3D> DoublePyramid;
    extern const std::vector<Coord3D> TetraederPlus;
    extern const std::vector<Coord3D> Tetraeder;
    extern const std::vector<Coord3D> FlatUmbrella;
    extern const std::vector<Coord3D> SimpleTriangle;
}

#endif