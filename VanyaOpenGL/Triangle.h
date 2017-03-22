#pragma once

#include <glm/glm.hpp> // GLM Mathematics
#include <glm/vec3.hpp>
#include "Particle.h"

class Triangle
{
public:

	Particle *p1, *p2, *p3;

	glm::vec3 centerPos;
	glm::vec3 normal;

	float mass;


	ClothTriangleAABoundingBox bBox;
	GLfloat minX, minY, minZ, maxX, maxY, maxZ, xDist, yDist, zDist = 0.0f;


	bool isBroadPhaseColliding;

	bool isRepelling;

	bool isColliding;


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


	glm::vec3 getTriangleCenterPos()
	{

		glm::vec3 pos1 = p1->position;
		glm::vec3 pos2 = p2->position;
		glm::vec3 pos3 = p3->position;

		glm::vec3 midPoint(0, 0, 0);
		midPoint = pos1 + pos2 + pos3;
		
		//Get the mean of the points
		midPoint /= 3.0f;

		centerPos = midPoint;

		return midPoint;
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


	void FlagCollisionColour()
	{
		glm::vec3 red(1.0f, 0.0f, 0.0f);
		p1->setColour(red);
		p2->setColour(red);
		p3->setColour(red);
	}


	void GetBoundingBox()
	{
		getTriangleCenterPos();

		glm::vec3 pos1 = p1->getPosition();
		glm::vec3 pos2 = p2->getPosition();
		glm::vec3 pos3 = p3->getPosition();

		minX = centerPos.x - 0.005f, maxX = centerPos.x + 0.005f, minY = centerPos.y - 0.005f, maxY = centerPos.y + 0.005f, minZ = centerPos.z - 0.005f, maxZ = centerPos.z + 0.005f;

		//std::cout << "pos1 " << glm::to_string(pos1) << std::endl;
		//std::cout << "pos2 " << glm::to_string(pos2) << std::endl;
		//std::cout << "pos3 " << glm::to_string(pos3) << std::endl;

		getTriangleMaxMin(pos1);
		getTriangleMaxMin(pos2);
		getTriangleMaxMin(pos3);


		//std::cout << "xMax " << maxX << std::endl;
		//std::cout << "xMin " << minX << std::endl;

		/*bBox.posX = centerPos.x + maxX + 0.1f;
		bBox.negX = centerPos.x + minX + 0.1f;
		bBox.posY = centerPos.y + maxY + 0.1f;
		bBox.negY = centerPos.y + minY + 0.1f;
		bBox.posZ = centerPos.z + maxZ + 0.1f;
		bBox.negZ = centerPos.z + minZ + 0.1f;*/

		xDist = glm::abs(minX - maxX);
		yDist = glm::abs(minY - maxY);
		zDist = glm::abs(minZ - maxZ);
	}

	void getTriangleMaxMin(glm::vec3 vertexPos)
	{
		//Sets the max and min for xyz everytime the triangle is drawn	
		/*GLfloat vPx = glm::normalize(vertexPos.x);
		GLfloat vPy = glm::normalize(vertexPos.y);
		GLfloat vPz = glm::normalize(vertexPos.z);*/

		if (vertexPos.x < minX) minX = vertexPos.x;// -0.1f;
		if (vertexPos.y < minY) minY = vertexPos.y;// -0.1f;
		if (vertexPos.z < minZ) minZ = vertexPos.z;// -0.1f;
		if (vertexPos.x > maxX) maxX = vertexPos.x;// +0.1f;
		if (vertexPos.y > maxY) maxY = vertexPos.y;// +0.1f;
		if (vertexPos.z > maxZ) maxZ = vertexPos.z;// +0.1f;

		//minX = 0;
		//minY = 0;
		//maxZ = 0.1f;
		//minZ = 0.1f;
	}

};