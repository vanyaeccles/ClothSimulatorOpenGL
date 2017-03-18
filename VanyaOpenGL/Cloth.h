#pragma once
#include <vector>

#include "Particle.h"
#include "ParticleConstraint.h"
#include "ParticleSpring.h"

class Cloth
{
public:

	int particleGridWidth;
	int particleGridHeight;

	float width;
	float height;

	//Constraints
	bool structural, shear, flexion;
	float structuralS = 1.0f;
	float shearS = 1.0f;
	float flexionS = 1.0f;

	//Springs
	const int strucSpring = 0;
	const int shearSpring = 1;
	const int flexionSpring = 2;

	float	structuralKs = 50.75f, structuralKd = -0.25f;
	float	shearKs = 50.75f, shearKd = -0.25f;
	float	flexionKs = 50.95f, flexionKd = -0.25f;


	// Collision variables
	float k_R = 0.0f;
	GLfloat coResFactor = 1.0f + k_R;



	std::vector<Particle> particles;
	std::vector<ParticleConstraint> particleConstraints;

	std::vector<ParticleSpring> particleSprings;




	Particle* getParticle(int x, int y)
	{
		return &particles[y*particleGridWidth + x];
	}

	void constrainParticles(Particle* p1, Particle* p2, float strength)
	{
		particleConstraints.push_back(ParticleConstraint(p1, p2, strength));
	}

	void addSpring2Particles(Particle* p1, Particle* p2, float _Ks, float _Kd, int _springType)
	{
		particleSprings.push_back(ParticleSpring(p1, p2, _Ks, _Kd, _springType));
	}
	



	Cloth(float width, float height, int particleWidthNumber, int particleHeightNumber, bool _structural, bool _shear, bool _flexion) 
	: particleGridWidth(particleWidthNumber), particleGridHeight(particleHeightNumber), width(width), height(height), structural(_structural), shear(_shear), flexion(_flexion)
	{
		//initialise the size of the cloth grid
		particleGridWidth = particleWidthNumber;
		particleGridHeight = particleHeightNumber;

		
		initialiseParticleGrid();

		//addConstraints(structural, shear, flexion);

		addSprings(structural, shear, flexion);

		//initialiseParticleSecondaryConstraints();

		pinCloth();

	}

	// This creates and fills the grid with particles
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


#pragma region CONSTRAINTS

	void satisfyConstraints(int constraintIterations)
	{
		std::vector<ParticleConstraint>::iterator constraint;
		for (int i = 0; i < constraintIterations; i++) // iterate over all constraints several times
		{
			for (constraint = particleConstraints.begin(); constraint != particleConstraints.end(); constraint++)
			{
				(*constraint).satisfyConstraint(); // satisfy constraint.
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

#pragma endregion



#pragma region SPRINGS

	void addSprings(bool structuralConstraints, bool shearConstraints, bool flexionConstraints)
	{
		for (int i = 0; i < particleGridWidth; i++)
		{
			for (int j = 0; j < particleGridHeight; j++)
			{
				if (structuralConstraints)
				{
					// binds masses [i, j]--[i+1, j], [i, j]--[i, j+1]
					if (i < particleGridWidth - 1)
						addSpring2Particles(getParticle(i, j), getParticle(i + 1, j), structuralKs, structuralKd, structuralS);

					if (j < particleGridHeight - 1)
						addSpring2Particles(getParticle(i, j), getParticle(i, j + 1), structuralKs, structuralKd, structuralS);

					//if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
					//	addSpring2Particles(getParticle(i, j), getParticle(i + 1, j + 1), structuralKs, structuralKd, structuralS);

					//if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						//addSpring2Particles(getParticle(i + 1, j), getParticle(i, j + 1), structuralKs, structuralKd, structuralS);
				}

				if (shearConstraints)
				{
					// binds masses [i,j]--[i+1, j+1], [i+1, j]--[i, j+1]
					if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
					{
						addSpring2Particles(getParticle(i, j), getParticle(i + 1, j + 1), shearKs, shearKd, shearSpring);
						addSpring2Particles(getParticle(i + 1, j), getParticle(i, j + 1), shearKs, shearKd, shearSpring);
					}
				}

				if (flexionConstraints)
				{
					// binds masses [i,j]--[i+2, j], [i, j]--[i, j+2]

					if (i < particleGridWidth - 2 && j < particleGridHeight - 2)
					{
						addSpring2Particles(getParticle(i, j), getParticle(i + 2, j), flexionKs, flexionKd, flexionSpring);
						addSpring2Particles(getParticle(i, j), getParticle(i, j + 2), flexionKs, flexionKd, flexionSpring);
					}
				}
			}
		}
	}


	void satisfySprings(int springIterations, float timestep)
	{
		std::vector<ParticleSpring>::iterator spring;
		for (int i = 0; i < springIterations; i++) // iterate over all springs several times
		{
			for (spring = particleSprings.begin(); spring != particleSprings.end(); spring++)
			{
				(*spring).satisfySpring(timestep); // satisfy spring
			}
		}

	}

#pragma endregion




	void pinCloth()
	{

		// making the upper left most three and right most three particles unmovable, for nice hanging effect

		for (int i = 0; i < 2; i++)
		{
			//getParticle(0 + i, 0)->offsetPosition(glm::vec3(0.5, 0.0, 0.0));
			getParticle(0 + i, 0)->pinParticle();

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


	void Update(float dampingConstant, int constraintIterations, int springIterations, float timestep)
	{

		//satisfyConstraints(constraintIterations);

		satisfySprings(springIterations, timestep);


		verletIntegrations(dampingConstant, timestep);
		
	}


#pragma region FORCES

	/* used to add gravity (or any other arbitrary vector) to all particles*/
	void addForce(const glm::vec3 direction, float timestep)
	{
		std::vector<Particle>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).applyGravityForce(direction, timestep); // add the forces to each particle
		}
	}

	void addWindForcesForClothTriangle(Particle *p1, Particle *p2, Particle *p3, const glm::vec3 direction, float timestep)
	{
		glm::vec3 normal = getTriangleNormal(p1, p2, p3);
		glm::vec3 d = normalize(normal);
		glm::vec3 force = normal*(dot(d, direction));
		p1->applyForce(force, timestep);
		p2->applyForce(force, timestep);
		p3->applyForce(force, timestep);
	}

	void applyWindForce(const glm::vec3 direction, float timestep)
	{
		for (int x = 0; x < particleGridWidth - 1; x++)
		{
			for (int y = 0; y < particleGridHeight - 1; y++)
			{
				addWindForcesForClothTriangle(getParticle(x + 1, y), getParticle(x, y), getParticle(x, y + 1), direction, timestep);
				addWindForcesForClothTriangle(getParticle(x + 1, y + 1), getParticle(x + 1, y), getParticle(x, y + 1), direction, timestep);
			}
		}
	}


	glm::vec3 getTriangleNormal(Particle *p1, Particle *p2, Particle *p3)
	{
		glm::vec3 pos1 = p1->position;
		glm::vec3 pos2 = p2->position;
		glm::vec3 pos3 = p3->position;

		glm::vec3 vector1 = pos2 - pos1;
		glm::vec3 vector2 = pos3 - pos1;

		return glm::cross(vector1, vector2);
	}

#pragma endregion



#pragma region COLLISION

	void bruteForceParticlePlaneCollisionCheck(glm::vec3 planeNormal, glm::vec3 planePoint)
	{
		std::vector<Particle>::iterator parti;
		for (parti = particles.begin(); parti != particles.end(); parti++)
		{
			glm::vec3 partiPosition = (parti->position);

			float dotPartiPlane = glm::dot((partiPosition - planePoint), planeNormal);


			if (dotPartiPlane < 0)
			{
				glm::vec3 newDeltaPos = -(dotPartiPlane * planeNormal);

				parti->offsetPosition(newDeltaPos);


				/*glm::vec3 partiVelocity = (parti->velocity);

				float dotPartiPlaneVelocity = glm::dot(partiVelocity, planeNormal);
				glm::vec3 newDeltaVeloci = -(dotPartiPlaneVelocity * planeNormal);



				parti->offsetVelocity(coResFactor * newDeltaVeloci);*/
			}

		}


	}


#pragma endregion







#pragma region INTEGRATION

	void verletIntegrations(float dampingConstant, float timestep)
	{
		std::vector<Particle>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).verletIntegration(dampingConstant, timestep); // calculate the position of each particle at the next time step.
		}
	}

#pragma endregion



};