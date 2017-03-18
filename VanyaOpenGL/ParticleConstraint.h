#pragma once

#include <glm/glm.hpp> // GLM Mathematics
#include <glm/vec3.hpp>
#include "Particle.h"

// A particle definition for use within cloth simulation
// Online resources used: http://cg.alexandra.dk/?p=147, https://graphics.stanford.edu/~mdfisher/cloth.html, http://www.darwin3d.com/gamedev/articles/col0599.pdf




class ParticleConstraint
{
public:

	float restDistance;

	Particle *p1, *p2;

	// @TODO Maybe this is silly
	float strength;



	ParticleConstraint(Particle *part1, Particle *part2, float _strength) : p1(part1), p2(part2), strength(_strength)
	{
		glm::vec3 p2Pos = (p2->position);
		glm::vec3 p1Pos = (p1->position);

		glm::vec3 differenceVector = (p1Pos - p2Pos);
		restDistance = glm::length(differenceVector);
	}

	//This basically ensures a particle is within a rest distance of a connected particle
	void satisfyConstraint()
	{
		glm::vec3 p2Pos = (p2->position);
		glm::vec3 p1Pos = (p1->position);

		glm::vec3 differenceVector(p2Pos - p1Pos);
		float diffLength = glm::length(differenceVector);
		glm::vec3 correction = differenceVector * /*glm::abs*/(strength - restDistance / diffLength);
		glm::vec3 halfcorrection = correction * 0.5f;

		p1->offsetPosition(halfcorrection);
		p2->offsetPosition(-halfcorrection);

		//p1->isPinned = true;
		//p2->isPinned = true;
	}

};