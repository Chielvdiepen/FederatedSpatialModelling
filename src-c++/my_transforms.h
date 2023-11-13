#ifndef MY_TRANSFORMS_H
#define MY_TRANSFORMS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

class MyTransforms {
    
public:

    /**
	 * @brief Convert RSSI to distance.
	 * 
	 * @param rssi 
	 * @return float 
	 */
	static float rssToDistance(int8_t rssi);

	/**
	 * @brief Roll transformation.
	 * 
	 * @param phi 
	 * @return std::vector<float> 
	 */
	static std::vector<float> roll(float phi);

	/**
	 * @brief Pitch transformation.
	 * 
	 * @param theta 
	 * @return std::vector<float> 
	 */
	static std::vector<float> pitch(float theta); 

	/**
	 * @brief Yaw transformation.
	 * 
	 * @param psi 
	 * @return std::vector<float> 
	 */
    static std::vector<float> yaw(float psi);

	/**
	 * @brief Multiply two matrices or matrix with vector
	 * 
	 * @param matrix 
	 * @param matvec 
	 * @return std::vector<float> 
	 */
	std::vector<float> matrix3_multiply(const std::vector<float>& matrix, const std::vector<float>& vectMatrix);

};

#endif