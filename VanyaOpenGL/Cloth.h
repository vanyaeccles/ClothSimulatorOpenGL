#pragma once
#include <vector>

#include "Particle.h"
#include "ParticleConstraint.h"

class Cloth
{
public:

	int particleGridWidth;
	int particleGridHeight;

	float width;
	float height;

	// 'Spring' Constraints
	bool structural, shear, flexion;
	float structuralS = 1.0f;
	float shearS = 1.0f;
	float flexionS = 1.0f;

	std::vector<Particle> particles;
	std::vector<ParticleConstraint> particleConstraints;


	Particle* getParticle(int x, int y)
	{
		return &particles[y*particleGridWidth + x];
	}

	void constrainParticles(Particle* p1, Particle* p2, float strength)
	{
		particleConstraints.push_back(ParticleConstraint(p1, p2, strength));
	}

	


//public:

	Cloth(float width, float height, int particleWidthNumber, int particleHeightNumber, bool _structural, bool _shear, bool _flexion) 
	: particleGridWidth(particleWidthNumber), particleGridHeight(particleHeightNumber), width(width), height(height), structural(_structural), shear(_shear), flexion(_flexion)
	{
		//initialise the size of the cloth grid
		particleGridWidth = particleWidthNumber;
		particleGridHeight = particleHeightNumber;

		
		initialiseParticleGrid();

		addConstraints(structural, shear, flexion);

		//initialiseParticleSecondaryConstraints();

		pinCloth();

	}





	void initialiseParticleGrid()
	{
		//initialise the vector of particles
		particles.resize(particleGridWidth * particleGridHeight);

		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{
				glm::vec3 particlePos(width * (i / (float)particleGridWidth), -height * (j / (float)particleGridHeight), 0.0f);

				// insert particle in column i at j'th row
				particles[j * particleGridWidth + i] = Particle(particlePos, 1.0f);

				//Make the hems a little heavier as in Bridson et al
				if (i == 0 || i == particleGridWidth || j == 0 || j == particleGridHeight)
					particles[j * particleGridWidth + i] = Particle(particlePos, 2.0f);
			}
		}
	}



	void addConstraints(bool structuralConstraints, bool shearConstraints, bool flexionConstraints)
	{
		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{
				if (structuralConstraints)
				{
					// masses [i, j]--[i+1, j], [i, j]--[i, j+1]
					if (i < particleGridWidth - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j), structuralS);

					if (j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i, j + 1), structuralS);

					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1), structuralS);

					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						constrainParticles(getParticle(i + 1, j), getParticle(i, j + 1), structuralS);
				}

				if (shearConstraints)
				{ 
					// masses [i,j]--[i+1, j+1], [i+1, j]--[i, j+1]
					/*if (i < particleGridWidth - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1));

					if (j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1));*/

					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
					{
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1), shearS);
						constrainParticles(getParticle(i + 1, j), getParticle(i, j + 1), shearS);

						
					}
						

					//if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
					//	constrainParticles(getParticle(i + 1, j), getParticle(i, j + 1));
				}

				if (flexionConstraints)
				{
					// masses [i,j]--[i+2, j], [i, j]--[i, j+2]

					if (i < particleGridWidth - 2 && j < particleGridHeight - 2)
					{
						constrainParticles(getParticle(i, j), getParticle(i + 2, j), flexionS);
						constrainParticles(getParticle(i, j), getParticle(i, j + 2), flexionS);
					}

					/*if (i < particleGridWidth - 2)
						constrainParticles(getParticle(i, j), getParticle(i + 2, j));

					if (j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 2, j));

					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i, j + 2));

					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i, j + 2));*/
				}
			}
		}
	}

	// masses [i,j]--[i+1, j+1], [i+1, j]--[i, j+1]
	void shearConstraints()
	{
		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{

			}
		}
	}

	// masses [i,j]--[i+2, j], [i, j]--[i, j+2]
	void flexionConstraints()
	{
		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{

			}
		}
	}


	void initialiseParticleSecondaryConstraints()
	{	
		// Connect secondary NESW neighbours with a distance constraint (distance 2 and sqrt(4) in the grid)
		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{
				if (i < particleGridWidth - 2)
					constrainParticles(getParticle(i, j), getParticle(i + 2, j), structuralS);

				if (j < particleGridHeight - 2)
					constrainParticles(getParticle(i, j), getParticle(i, j + 2), structuralS);

				if (i < particleGridWidth - 2 && j < particleGridHeight - 2)
					constrainParticles(getParticle(i, j), getParticle(i + 2, j + 2), structuralS);

				if (i < particleGridWidth - 2 && j < particleGridHeight - 2)						
					constrainParticles(getParticle(i + 2, j), getParticle(i, j + 2), structuralS);
			}
		}
	}


	void pinCloth()
	{

		// making the upper left most three and right most three particles unmovable, for nice hanging effect

		for (int i = 0; i < 2; i++)
		{
			//getParticle(0 + i, 0)->offsetPosition(glm::vec3(0.5, 0.0, 0.0));
			//getParticle(0 + i, 0)->pinParticle();

			//getParticle(0 + i, 0)->offsetPosition(glm::vec3(-0.5, 0.0, 0.0)); 
			getParticle(particleGridWidth - 1 - i, 0)->pinParticle();
		}

		//getParticle(10, 0)->offsetPosition(glm::vec3(0.0, 0.0, 1.0)); 
		//getParticle(10, 0)->pinParticle();

		//getParticle(0, 0)->offsetPosition(glm::vec3(0.0, 0.0, 0.0));
		//getParticle(0, 0)->pinParticle();

		//getParticle(0, 0)->pinParticle();
		//getParticle(particleGridWidth-1, particleGridHeight-1)->pinParticle();
		//getParticle(particleGridWidth - 11, particleGridHeight-1)->pinParticle();
	}


	void Update(float dampingConstant, int constraintIterations, float timestep)
	{
		std::vector<ParticleConstraint>::iterator constraint;
		for (int i = 0; i < constraintIterations; i++) // iterate over all constraints several times
		{
			for (constraint = particleConstraints.begin(); constraint != particleConstraints.end(); constraint++)
			{
				(*constraint).satisfyConstraint(); // satisfy constraint.
			}
		}

		std::vector<Particle>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).verletIntegration(dampingConstant, timestep); // calculate the position of each particle at the next time step.
		}
	}



	/* used to add gravity (or any other arbitrary vector) to all particles*/
	void addForce(const glm::vec3 direction, float timestep)
	{
		std::vector<Particle>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).applyGravityForce(direction, timestep); // add the forces to each particle
		}
	}

};