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

		Particle *p1, *p2;

		float restLength;

		float Ks, Kd;

		int springType;


	ParticleSpring(Particle *part1, Particle *part2, float _Ks, float _Kd, int _springType) 
	{

		p1 = part1;
		p2 = part2;

		Ks = _Ks;
		Kd = _Kd;

		springType = _springType;

		glm::vec3 p2Pos = (p2->position);
		glm::vec3 p1Pos = (p1->position);

		glm::vec3 differenceVector = (p1Pos - p2Pos);
		restLength = sqrt(glm::dot(differenceVector, differenceVector));
	}


	void satisfySpring(float timestep)
	{
		

		glm::vec3 p2Pos = (p2->position);
		glm::vec3 p2PosOld = (p2->oldPosition);

		glm::vec3 p1Pos = (p1->position);
		glm::vec3 p1PosOld = (p1->oldPosition);


		glm::vec3 v1 = p1->getVerletVelocity(timestep);
		glm::vec3 v2 = p2->getVerletVelocity(timestep);

		glm::vec3 deltaPos = p1Pos - p2Pos;
		glm::vec3 deltaVel = v1 - v2;

		float distance = glm::length(deltaPos);

		float leftTerm = -Ks * (distance - restLength);
		float rightTerm = Kd * (glm::dot(deltaVel, deltaPos) / distance);

		glm::vec3 SpringForce = (leftTerm + rightTerm) * glm::normalize(deltaPos);

		if (springType == 2)
		{
		}

		p1->applySpringForce(SpringForce, timestep);
		p2->applySpringForce(-1.0f * SpringForce, timestep);
		

	}

	
};