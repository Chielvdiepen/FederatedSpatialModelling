#include "my_transforms.h"

	float rssToDistance(int8_t rssi) {
    	return std::pow(10, (-69.1-rssi) / 10*2.05);
	}

	std::vector<float> roll(float phi) {
        phi = M_PI * phi / 180.0;
        return {1, 0, 0,
                0, std::cos(phi), -std::sin(phi),
                0, std::sin(phi), std::cos(phi)};
    }

    std::vector<float> pitch(float theta) {
        theta = M_PI * theta / 180.0;
        return {std::cos(theta), 0, std::sin(theta),
                0, 1, 0,
                -std::sin(theta), 0, std::cos(theta)};
    }

    std::vector<float> yaw(float psi) {
        psi = M_PI * psi / 180.0;
        return {std::cos(psi), -std::sin(psi), 0,
                std::sin(psi), std::cos(psi), 0,
                0, 0, 1};
    }

	std::vector<float> matrix3_multiply(const std::vector<float>& matrix, const std::vector<float>& vectMatrix) {
		size_t size = vectMatrix.size();
		std::vector<float> result(size, 0.0);

		if(size == 3){
			for (size_t i = 0; i < 3; ++i) {
				for (size_t j = 0; j < 3; ++j) {
					result[i] += matrix[i * 3 + j] * vectMatrix[j];
				}
			}
		}
		if(size == 9){
			for (size_t i = 0; i < 3; ++i) {
				for (size_t j = 0; j < 3; ++j) {
					for (size_t k = 0; k < 3; ++k) {
						result[i * 3 + j] += matrix[i * 3 + k] * vectMatrix[k * 3 + j];
					}
				}
			}
		}
		return result;
	}