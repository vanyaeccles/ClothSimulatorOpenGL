#pragma once

#include <glm/glm.hpp> // GLM Mathematics
#include <glm/vec3.hpp>
#include "Particle.h"

class Triangle
{
public:

	Particle *p1, *p2, *p3;

	glm::vec3 normal;

	float mass;

	Triangle(Particle *_p1, Particle *_p2, Particle *_p3) : p1(_p1), p2(_p2), p3(_p3)
	{
		normal = getTriangleNormal();

		mass = p1->mass + p2->mass + p3->mass;
	}


	void addWindForcesForClothTriangle(const glm::vec3 direction, float timestep)
	{
		normal = getTriangleNormal();
		glm::vec3 d = normalize(normal);
		glm::vec3 force = normal*(dot(d, direction));
		p1->applyForce(force, timestep);
		p2->applyForce(force, timestep);
		p3->applyForce(force, timestep);
	}

	glm::vec3 getTriangleNormal()
	{
		glm::vec3 pos1 = p1->position;
		glm::vec3 pos2 = p2->position;
		glm::vec3 pos3 = p3->position;

		glm::vec3 vector1 = pos2 - pos1;
		glm::vec3 vector2 = pos3 - pos1;

		return /*glm::normalize*/(glm::cross(vector1, vector2));
	}

	//Adapted from Christer Ericson's Real-Time Collision Detection via http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
	glm::vec3 getBaryCentricCoordinates(glm::vec3 point)
	{
		glm::vec3 bary;

		glm::vec3 v0 = p2->position - p1->position;
		glm::vec3 v1 = p3->position - p1->position;
		glm::vec3 v2 = point - p1->position;

		float d00 = glm::dot(v0, v0);
		float d01 = glm::dot(v0, v1);
		float d11 = glm::dot(v1, v1);
		float d20 = glm::dot(v2, v0);
		float d21 = glm::dot(v2, v1);
		float denom = d00 * d11 - d01 * d01;

		
		float v = (d11 * d20 - d01 * d21) / denom;
		float w = (d00 * d21 - d01 * d20) / denom;
		float u = 1.0f - v - w;


		return bary;
	}


};