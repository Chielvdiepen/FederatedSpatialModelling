from federated_simulator import initialize_nodes, create_edges, run_simulation, mapPlot
from config import office_width, office_length, office_height, num_nodes, RefNodeCoords, RSSI, PLOT

if __name__ == "__main__":

    nodeList = initialize_nodes(RefNodeCoords, num_nodes, office_width, office_length, office_height)
    NetworkEdges, TagDistances = create_edges(nodeList, RefNodeCoords, RSSI, PLOT)
    run_simulation(nodeList, NetworkEdges, TagDistances)

    nodeA = nodeList[0]

    nodeA.printNodeInfo()

    for (key, mapEdgeSet, mapCoords) in zip(nodeA.mapEdgeSetDict.keys(), nodeA.mapEdgeSetDict.values(), nodeA.transCoordsDict.values()):
        print(f'Base_edge {key}:')
        for coord in sorted(mapCoords.items()):
            print(f"Node {coord[0]}: ({coord[1][0]:.3f},{coord[1][1]:.3f},{coord[1][2]:.3f})")
        print()

        if PLOT:
            mapPlot(list(mapEdgeSet), mapCoords)