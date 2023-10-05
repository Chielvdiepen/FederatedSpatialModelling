# Federated Spatial Modelling
Federated spatial modelling algorithm for BLE nodes in a mesh network

![Federated spatial model](https://github.com/Chielvdiepen/FederatedSpatialModelling/assets/32455651/8da12d3a-49e6-48de-8a5d-f88b33b72870)

The spatial algorithm is based on triangles that are being formed with other surrounding nodes. Their mutual connections are explored from every node's perspective. Each node forms its own spatial model, with proportionate spatial information.
![triangulation](https://github.com/Chielvdiepen/FederatedSpatialModelling/assets/32455651/00c80564-ce80-4b7b-99f0-25a3d498c05c)

The angle between two adjacent triangles with the same base edge is calculated with the translation of the distance between the two 'third' nodes
![Dihedral](https://github.com/Chielvdiepen/FederatedSpatialModelling/assets/32455651/68e22ed1-f9bc-4eed-836f-bfc3379c53ea)
