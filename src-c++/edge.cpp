#include "edge.h"

/******************************************   Edge   *********************************************/

/**
 * @brief Construct a new Edge:: Edge object
 * 
 * @param source 
 * @param target 
 * @param rssi 
 */
Edge::Edge(stone_id_t source, stone_id_t target, int8_t rssi) : source(source), target(target), rssi(rssi) {}

/**
 * @brief Compare two edges.
 * 
 * @param other 
 * @return true 
 * @return false 
 */
bool Edge::compare(const Edge &other) const {
	return source == other.source && target == other.target;
}