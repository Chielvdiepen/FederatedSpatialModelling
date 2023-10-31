#include "config.h"

namespace config
{
    // Simulation parameters

    double office_width = 5.0;
    double office_length = 5.0;
    double office_height = 3.0;

    int num_nodes = 0;

    bool DIM2D = false;
    int MAXTRIANGLES = 25;
    bool RSSI = false;
    bool PLOT = false;

    // Global constant arrays of 3D coordinates
    const std::vector<Coord3D> DoublePyramid = {
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(3.0, 0.0, 0.0),
        std::make_tuple(0.0, 3.0, 0.0),
        std::make_tuple(3.0, 3.0, 0.0),
        std::make_tuple(6.0, 0.0, 0.0),
        std::make_tuple(6.0, 3.0, 0.0),
        std::make_tuple(1.5, 1.5, 2.0),
        std::make_tuple(4.5, 1.5, 2.0)
    };

    const std::vector<Coord3D> TetraederPlus = {
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(4.0, 0.0, 0.0),
        std::make_tuple(3.5, 2.0, 0.0),
        std::make_tuple(1.5, 3.0, 2.0),
        std::make_tuple(3.0, 3.0, 3.0)
    };

    const std::vector<Coord3D> Tetraeder = {
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(4.0, 0.0, 0.0),
        std::make_tuple(3.5, 2.0, 0.0),
        std::make_tuple(1.5, 3.0, 2.0)
    };

    const std::vector<Coord3D> FlatUmbrella = {
        std::make_tuple(5.0, 5.0, 0.0),
        std::make_tuple(1.0, 2.0, 0.0),
        std::make_tuple(1.0, 9.5, 0.0),
        std::make_tuple(7.0, 9.0, 0.0),
        std::make_tuple(9.5, 1.5, 0.0),
        std::make_tuple(15.0, 15.0, 0.0)
    };

    const std::vector<Coord3D> SimpleTriangle = {
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(4.0, 0.0, 0.0),
        std::make_tuple(3.5, 2.0, 0.0)
    };

}