#pragma once

#include <glm/glm.hpp> // GLM Mathematics
#include <glm/vec3.hpp>
#include "Particle.h"

/*
* Defines a particle spring
*/


class ParticleSpring
{
public:

	struct Spring
	{
		Particle *p1, *p2;

		float restLength;

		float Ks, Kd;

		int springType;
	};


	ParticleSpring(Particle *part1, Particle *part2, float _Ks, float _Kd, int _springType) 
	{
		Spring spring;

		spring.p1 = part1;
		spring.p2 = part2;

		spring.Ks = _Ks;
		spring.Kd = _Kd;

		spring.springType = _springType;

		glm::vec3 p2Pos = (spring.p2->position);
		glm::vec3 p1Pos = (spring.p1->position);

		glm::vec3 differenceVector = (p1Pos - p2Pos);
		spring.restLength = sqrt(glm::dot(differenceVector, differenceVector));

	}
};