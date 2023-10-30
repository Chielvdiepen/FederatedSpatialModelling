#include "edge.h"

Edge::Edge(Node src, Node dst, double rssi) : src(src), dst(dst), rssi(rssi) {
    id = std::make_pair(src.uuid, dst.uuid);
    dist = MyTransforms::rssiToDistance(rssi);
}

std::string Edge::toString() const {
    return "(" + std::to_string(src.uuid) + "," + std::to_string(dst.uuid) + "," + std::to_string(dist) + ")";
}

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    os << edge.toString();
    return os;
}

bool Edge::operator==(const Edge &other) const {
    return id == other.id;
}

size_t Edge::hash() const {
    return std::hash<std::string>{}(id);
}

bool Edge::compare(const Edge& other) const {
    return (std::make_pair(src.uuid, dst.uuid) == other.id) || (std::make_pair(src.uuid, dst.uuid) == std::make_pair(other.dst.uuid, other.src.uuid));
}