#pragma once
#include <glm/vec3.hpp>
#include <vector>


// A particle definition for use within cloth simulation
// Online resources used: http://cg.alexandra.dk/?p=147, https://graphics.stanford.edu/~mdfisher/cloth.html, http://www.darwin3d.com/gamedev/articles/col0599.pdf


class Particle
{
public:

	bool isPinned;
	float mass;
	float massinv;
	
	glm::vec3 position;
	glm::vec3 oldPosition;

	glm::vec3 velocity;
	glm::vec3 acceleration;

	glm::vec3 force;

	glm::vec3 normal;

	glm::vec3 colour;

//public:
	//default constructor
	Particle()
	{
	}


	Particle(glm::vec3 _position, float _mass) : position(_position), oldPosition(_position), acceleration(glm::vec3(0, 0, 0)), mass(_mass), isPinned(false), normal(glm::vec3(0, 0, 0))
	{
		position = _position;
		oldPosition = _position;
		acceleration = glm::vec3(0.0f);
		mass = 1.0f;
		massinv = 1.0f / mass;
		isPinned = false;
		normal = glm::vec3(0.0f);

		colour[0] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));
		colour[1] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));
		colour[2] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));
	}


	glm::vec3& getPosition()
	{ 
		return position; 
	}



	void applyGravityForce(glm::vec3 _force, float timestep)
	{
		acceleration += _force * massinv * timestep;
	}


	void applySpringForce(glm::vec3 _force, float timestep) 
	{
		acceleration += _force * massinv * timestep;
	}

	void applyForce(glm::vec3 _force, float timestep)
	{
		acceleration += _force * massinv * timestep;
	}


	void verletIntegration(float dampingConstant, float timestep)
	{
		//Unless they are pinned, they can move
		if (!isPinned)
		{
			glm::vec3 currentPosition = position;
			position = position + (position - oldPosition) * (1.0f - dampingConstant) + acceleration * timestep;
			oldPosition = currentPosition;
			zeroAcceleration();
		}
	}

	glm::vec3 getVerletVelocity(float timestep)
	{
		return (position - oldPosition) / timestep;
	}


	void zeroAcceleration()
	{
		acceleration = glm::vec3(0.0f);
	}

	void pinParticle()
	{
		isPinned = true;
	}

	void offsetPosition(const glm::vec3 offset)
	{
		if (!isPinned)
		{
			position += offset;
		}
	}

	void setMass(const float _mass)
	{
		mass = _mass;
		massinv = 1.0f / mass;
	}
	
};