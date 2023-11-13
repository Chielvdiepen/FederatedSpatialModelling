#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "edge.h"
#include "node.h"

#define _USE_MATH_DEFINES
#include <cmath>

/**
 * @brief Triangle class
 * 
 */
class Triangle {
public:
	static constexpr uint8_t NUM_EDGES = 3;

	/**
	 * @brief Construct a new Triangle object of three edge pointers
	 * 
	 * @param adj_edge1 
	 * @param adj_edge2 
	 * @param opposite_edge 
	 */
	Triangle(Edge *adj_edge1, Edge *adj_edge2, Edge *opposite_edge) : edges{*adj_edge1, *adj_edge2, *opposite_edge} {}
    // OR
	Triangle(Edge *base_edge, Edge *adj_edge, Edge *opposite_edge) : base_edge(base_edge), adj_edge(adj_edge), opposite_edge(opposite_edge) {}
	
	/**
	 * Edges that make up the triangle.
	 */
	Edge edges[NUM_EDGES];
	// OR
	Edge *base_edge;
	Edge *adj_edge;
	Edge *opposite_edge;

	// TODO: add triangle id, needed?

	/**
	 * Gets the area of the triangle.
	 */
	float getArea();

	/**
	 * Get the angle to the triangle w.r.t. the current crownstone.
	 */
	float getAngle();

	/**
	 * Get the altitude of the triangle.
	 */
	float getAltitude();

	/**
	 * @brief Get the Alitude Base Position of the Altitude's x position to the target crownstone.
	 * 
	 * @param target 
	 * @return float 
	 */
	float getAlitudeBasePositionTo(stone_id_t targetID);

	/**
	 * @brief Returns the third node of the triangle based on the base edge.
	 * Base edge is alway self->otherNode
	 * 
	 * @param baseEdge 
	 * @return stone_id_t 
	 */
	stone_id_t getThirdNode(Edge& baseEdge); 

	/**
	 * @brief Get the Other outgoing base edge
	 * 
	 * @param baseEdge 
	 * @return Edge pointer 
	 */
	Edge* getOtherBase(Edge& baseEdge);

	/**
	 * @brief Get a specific edge from the triangle.
	 * 
	 * @param source 
	 * @param target 
	 * @return Edge pointer
	 */
	Edge* getEdge(stone_id_t source, stone_id_t target);
	
};

#endif // TRIANGLE_H