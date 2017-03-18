#pragma once
#include <glm/vec3.hpp>
#include <vector>

//Older implementation

#define NUM_PARTICLES 144

struct Particle1
{
	bool movable;
	float mass;

	glm::vec3 position;
	glm::vec3 oldPosition;

	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 force;

	glm::vec3 normal;

	glm::vec3 colour;
	
};


class ParticleSystem
{

public:


	Particle1 parti[NUM_PARTICLES];


	float upperRandBound = 10.0f;

	float k_R = 0.8f;
	GLfloat coResFactor = 1.0f + k_R;


	void updateNoForce(float timestep)
	{
		//Per particle, update phase based on forces
		for (int i = 0; i < NUM_PARTICLES; i++)
		{


			clearForces(parti[i]); // clears accumulated forces

			//parti[i].force = computeForces(parti[i], reverseGrav, rightFan, leftFan); // Applies updated force


			//Euler's method
			//Set Updated phase for particle
			parti[i].position += parti[i].velocity * timestep;
			parti[i].velocity += parti[i].force / parti[i].mass * timestep;


			//Colour change with velocity
			parti[i].colour[0] -= (parti[i].velocity[1] * 0.001);
			parti[i].colour[1] -= (parti[i].velocity[1] * 0.001);
			parti[i].colour[2] -= (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / upperRandBound));
		}
	}


	void update(float timestep, bool grav)
	{


		//Per particle, update phase based on forces
		for (int i = 0; i < NUM_PARTICLES; i++) 
		{


			clearForces(parti[i]); // clears accumulated forces

			parti[i].force = computeForces(parti[i], grav); // Applies updated force


			//Euler's method
			//Set Updated phase for particle
			parti[i].position += parti[i].velocity * timestep;
			parti[i].velocity += parti[i].force / parti[i].mass * timestep;
			

			//Colour change with velocity
			parti[i].colour[0] -= (parti[i].velocity[1] * 0.001);
			parti[i].colour[1] -= (parti[i].velocity[1] * 0.001);
			parti[i].colour[2] -= (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / upperRandBound));
		}
	}
	


	
	glm::vec3 computeForces(Particle1 p, bool grav)
	{
		//Get updated forces

		glm::vec3 compForce(0.0f);

		glm::vec3 gravity;

		if (!grav)
		{
			gravity = glm::vec3(0.0f, 9.81f, 0.0f);
		}
		if (grav)
		{
			gravity = glm::vec3(0.0f, -0.981f, 0.0f);
		}


		compForce += gravity * p.mass;

		// Drag coefficient for sphere in fluid
		float dragCoefficient = 0.0047f;

		compForce -= dragCoefficient * p.velocity;



		return compForce;
	}


	void clearForces(Particle1 p)
	{
		p.force = glm::vec3(0.0f);
	}



	void particlePlaneCollisionCheck(glm::vec3 planeNormal, glm::vec3 planePoint)
	{

		// Collision Handling - Post Processing Method
		for (int i = 0; i < NUM_PARTICLES; i++)
		{

			//glm::vec3 normal = glm::normalize(glm::cross(planePoint, planePoint2));

			float dotPartiPlane = glm::dot((parti[i].position - planePoint), planeNormal);

			if (dotPartiPlane < 0)
			{
				glm::vec3 newDeltaPos = -(dotPartiPlane * planeNormal);

				//Update Position
				parti[i].position += newDeltaPos;


				glm::vec3 velociPos(parti[i].velocity[0], parti[i].velocity[1], parti[i].velocity[2]);
				float dotPartiPlaneVelocity = glm::dot(velociPos, planeNormal);
				glm::vec3 newDeltaVeloci = -(dotPartiPlaneVelocity * planeNormal);

				//Update Velocity
				parti[i].velocity += coResFactor * newDeltaVeloci;
			}
		}
	}





	void initializeParticles()
	{
		parti[0].position = glm::vec3(0.0f);
		
		for (int i = 0; i < NUM_PARTICLES; i++) 
		{
			parti[i].velocity[0] = 0.0f;
			parti[i].velocity[1] = 0.0f;
			parti[i].velocity[2] = 0.0f;

			parti[i].force[0] = 0.0f;
			parti[i].force[1] = 0.0f;
			parti[i].force[2] = 0.0f;

			/*parti[i].colour[0] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));
			parti[i].colour[1] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));
			parti[i].colour[2] = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 1));*/

			parti[i].mass = 1.0f;


			if (i % 12 != 0)
			{
				parti[i].position[0] = parti[i - 1].position[0] + 0.5f;
				parti[i].position[1] = parti[i - 1].position[1];
				parti[i].position[2] = 0.0f;
			}

			else if (i != 0 && i % 12 == 0)
			{
				std::cout << i << std::endl;

				parti[i].position[0] = parti[i - 12].position[0];
				parti[i].position[1] = parti[i - 12].position[1] - 0.5f;
				parti[i].position[2] = 0.0f;

				/*parti[i].position[0] =  0.5f;
				parti[i].position[1] = parti[i - 1].position[1];
				parti[i].position[2] = 0.0f;*/
			}

			
		}
	}

};

