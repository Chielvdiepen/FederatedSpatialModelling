#include "triangle.h"

/*****************************************  Triangle  ********************************************/

/**
 * @brief Construct a new Triangle:: Triangle object
 * 
 * @param base_edge 
 * @param adj_edge 
 * @param opposite_edge 
 */
Triangle::Triangle(Edge *base_edge, Edge *adj_edge, Edge *opposite_edge) : base_edge(base_edge), adj_edge(adj_edge), opposite_edge(opposite_edge) {}

/**
 * @brief Calculates the area of the triangle and returns it.
 * @return float 
 */
float Triangle::getArea() {
	float a = opposite_edge->distance;
    float b = adj_edge->distance;
    float c = base_edge->distance;

    float s = (a + b + c) / 2;
    float div = s * (s - a) * (s - b) * (s - c);

    return std::sqrt(max(div,0));
}

/**
 * @brief Calculates the angle of the triangle and returns it.
 * 
 * @return float 
 */
float Triangle::getAngle() {
	float a = opposite_edge->distance;
    float b = adj_edge->distance;
    float c = base_edge->distance;

	float p = (std::pow(b,2) + std::pow(c,2) - std::pow(a,2)) / (2 * b * c);

	return std::acos(clamp(p,-1,1)) * (180 / M_PI);
}

/**
 * @brief Calculates the altitude of the triangle from the current perspective of the node and returns it.
 * @return float 
 */
float Triangle::getAltitude() {
	float area = getArea();
    return (2.0 * area) / opposite_edge->distance;	 
}

/**
 * @brief Calculates the position on the base edge this triangle from the current perspective of the node with pythagoras. 
 * It returns this length from this position to the target node.
 * 
 * @param targetID 
 * @return float 
 */
float Triangle::getAlitudeBasePositionTo(stone_id_t targetID) {
 	float altitude = getAltitude();
    float hASquared = altitude*altitude;
	float basePositionX = 0.0;

	if(base_edge->target == targetID){
		basePositionX = std::pow(base_edge->distance,2) - hASquared;
	}
	if(adj_edge->target == targetID){
		basePositionX = std::pow(adj_edge->distance,2) - hASquared;
	}

    return max(basePositionX,0);
}

/**
 * @brief Returns the third node of the triangle based on the base edge.
 * Base edge is alway self->otherNode
 * 
 * @param baseEdge 
 * @return stone_id_t 
 */
stone_id_t Triangle::getThirdNode(Edge& baseEdge) {
	return getOtherBase(baseEdge)->target;
}

/**
 * @brief Get the Other outgoing base edge
 * 
 * @param baseEdge 
 * @return Edge pointer 
 */
Edge* Triangle::getOtherBase(Edge& baseEdge) {
	if(baseEdge.compare(*base_edge)){
		return adj_edge;
	}
	return base_edge;
}

/**
 * @brief Get a specific edge from the triangle.
 * 
 * @param source 
 * @param target 
 * @return Edge pointer 
 */
Edge* Triangle::getEdge(stone_id_t source, stone_id_t target) {
	if(base_edge->source == source && base_edge->target == target){
		return base_edge;
	}
	if(adj_edge->source == source && adj_edge->target == target){
		return adj_edge;
	}
	if(opposite_edge->source == source && opposite_edge->target == target){
		return opposite_edge;
	}
	return nullptr;
}

template <class T, class S>
auto min(T l, S r) {
	return l < r ? l : r;
}

template <class T, class S>
auto max(T l, S r) {
	return l > r ? l : r;
}

template <class V, class L, class U>
auto clamp(V value, L lower, U upper) {
	return min(max(value, lower), upper);
}