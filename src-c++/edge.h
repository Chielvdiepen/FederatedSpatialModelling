#ifndef EDGE_H
#define EDGE_H

#include "node.h"
#include "my_transforms.h"
#include <string>
#include "node.h"

typedef uint8_t stone_id_t;

class Edge {
public:
	Edge(stone_id_t source, stone_id_t target, int8_t rssi) : source(source), target(target), rssi(rssi) {}

	stone_id_t source;
	stone_id_t target;
	int8_t rssi;
	float distance; // TODO: add conversion distance from rssi

	/**
	 * @brief Compare two edges.
	 * 
	 * @param other 
	 * @return true 
	 * @return false 
	 */
	bool compare(const Edge &other) const;
};

#endif // EDGE_H
