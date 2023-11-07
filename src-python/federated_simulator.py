import random
from node import Node
from edge import Edge
import threading
import matplotlib.pyplot as plt
import networkx as nx
from mpl_toolkits.mplot3d import Axes3D
from mymath import distance_to_rssi

# Plot the reference node network
def compareGraph3D(Network_Dict, RefNode_List):
    G = nx.Graph()

    i = 0
    for nodeID, edges in Network_Dict.items():
        G.add_node(nodeID, pos=RefNode_List[i])
        for edge in edges:
            G.add_edge(edge.src.uuid, edge.dst.uuid, Rssi=edge.rssi)
        i += 1

    pos = nx.get_node_attributes(G, 'pos')
    rssi = nx.get_edge_attributes(G, "Rssi")

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')  # Create a 3D axis

    # Draw nodes as scatter plot and text labels
    for nodeID, coords in pos.items():
        ax.scatter(*coords, c="blue", s=100, marker='o')
        ax.text(*coords, f'Node {nodeID}\n({coords[0]}, {coords[1]}, {coords[2]})', ha='center', va='bottom')

    # Draw edges in 3D
    for edge in G.edges():
        xs, ys, zs = zip(*[pos[edge[i]] for i in range(2)])
        ax.plot(xs, ys, zs, color='black')
        rssi_value = rssi.get(edge, "")
        ax.text(sum(xs)/2,sum(ys)/2,sum(zs)/2, f"{rssi_value:.3f}", color='red')

    ax.set_title("Node plot with signal strength (RSSI) (3D)")
    ax.set_ylabel("Y")
    ax.set_xlabel("X")
    ax.set_zlabel("Z")
    plt.show()

# Plot the mapping of the node network
def mapPlot(Edge_list, node_Dict):
    G = nx.Graph()

    for node in node_Dict.items():
        G.add_node(node[0], pos=(round(node[1][0],3),round(node[1][1],3),round(node[1][2],3)))

    for edge in Edge_list:
        G.add_edge(edge.src.uuid, edge.dst.uuid, Rssi=edge.rssi)

    pos = nx.get_node_attributes(G, 'pos')
    rssi = nx.get_edge_attributes(G, "Rssi")

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')  # Create a 3D axis

    # Draw nodes as scatter plot and text labels
    for nodeID, coords in pos.items():
        ax.scatter(*coords, c="blue", s=100, marker='o')
        ax.text(*coords, f'Node {nodeID}\n({coords[0]}, {coords[1]}, {coords[2]})', ha='center', va='bottom')

    # Draw edges in 3D
    for edge in G.edges():
        xs, ys, zs = zip(*[pos[edge[i]] for i in range(2)])
        ax.plot(xs, ys, zs, color='black')
        rssi_value = rssi.get(edge, "")
        ax.text(sum(xs)/2,sum(ys)/2,sum(zs)/2, f"{rssi_value:.3f}", color='red')

    ax.set_title("Adjacent map of visible nodes (3D)")
    ax.set_ylabel("Y")
    ax.set_xlabel("X")
    ax.set_zlabel("Z")
    plt.show()

def initialize_nodes(RefNodeCoords, num_nodes, office_width, office_length, office_height):

    nodeList = []

    names = [chr(x) for x in range(ord('A'), ord('Z') + 1)]

    if num_nodes == 0:
        for i in range(len(RefNodeCoords)): 
            nodeList.append(Node(names[i]))
    else:
        coordlist = []
        for i in range(num_nodes):
            nodeList.append(Node(names[i]))
            x = round(random.uniform(0, office_width) * 2) / 2
            y = round(random.uniform(0, office_length) * 2) / 2
            z = round(random.uniform(0, office_height) * 2) / 2
            coordlist.append((x,y,z))
        
        RefNodeCoords = coordlist

    print("List of nodes:")
    edge_str = ', '.join(str(node.uuid) for node in nodeList)
    print(edge_str+'\n')

    return nodeList

def create_edges(nodeList, RefNodeCoords, RSSI, PLOT):
    NetworkEdges = {}
    for i in range(len(nodeList)):
        table = []
        for j in range(len(nodeList)):
            if nodeList[i] != nodeList[j]:
                table.append(Edge(nodeList[i], nodeList[j], distance_to_rssi(RefNodeCoords[i],RefNodeCoords[j])))
        table = sorted(table, key=lambda x: x.rssi, reverse=RSSI) # if rssi set True
        NetworkEdges[nodeList[i].uuid] = table

    print('Reference Network Edges:')
    for key, edges in NetworkEdges.items():
        print(f'Node {key}:= Edges{edges}')

    if PLOT:
        compareGraph3D(NetworkEdges,RefNodeCoords)
    
    return NetworkEdges

def run_simulation(nodeList, NetworkEdges):
    thread_list = []
    for i in range(len(nodeList)):
        thread = threading.Thread(target=nodeList[i].main, args=(NetworkEdges, ))
        thread_list.append(thread)
        thread.start()

    for thread in thread_list:
        thread.join()