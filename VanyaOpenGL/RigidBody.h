#pragma once
#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_


#include <vector>
#include <glm/glm.hpp> // GLM Mathematics
#include <glm/gtc/matrix_access.hpp> 
#include "Mesh.h"
#include "Model.h"
#include "BoundingObject.h"
#include "DistanceChecks.h"

#define NUM_RIGIDBODIES 1







class RigidBody
{
    private:
		float IbodyVal;


	public:

		int bodyID;

		/* Constant quantities */

		//Mass
		float mass; //doesn't change
		//Inverse mass
		float massInv;
		//Ibody
		glm::mat3 Ibody;
		//Inverse Ibody
		glm::mat3 IbodyInv;


		/* State variables */

		//position  x(t)
		glm::vec3 position;
		//Orientation Matrix R(t)
		glm::mat3 orientationMatrix;
		//P(t)
		glm::vec3 linearMomentum;
		//L(t)
		glm::vec3 angularMomentum;


		/* Derived quantities (auxiliary variables) */

		//Inertial tensor matrix Iinv
		glm::mat3 tensorMatrix;
		

		//velocity v(t)
		glm::vec3 velocity;
		//angular velocity w(t)
		glm::vec3 angVelocity;



		/* Computed quantities */

		std::vector <glm::vec3> forces;

		glm::vec3 netforce;
		glm::vec3 nettorque;

		
		
		//Model
		Model model;
		Model modelCopy;

		// 12 cube vertices vector (removes repitition of vertices)
		vector <glm::vec3> modelVertices;

		// For setting the bounding box
		GLfloat minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f, minZ = 0.0f, maxZ = 0.0f;
		GLfloat xDist, yDist, zDist;

		//Collision
		BoundingSphere boundSphere;
		AABoundingBox boundBox;

		DistanceChecker dChecker;


		bool gravityF = true;
		bool dragF = true;
		bool springF = false;
		

		bool broadPhaseCollision = false;
		bool narrowPhaseCollision = false;
		glm::vec4  boundBoxColour;



	//default constructor
	RigidBody()
	{
		mass = 2.0f;
		massInv = 1 / mass;
		position = glm::vec3(0.0f, 0.0f, 0.0f);
		velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		angVelocity = glm::vec3(0.0f, 0.0f, 0.0f);

		Ibody = calcIbody();
		IbodyInv = glm::inverse(Ibody);


		boundSphere.center = position;
		
	}
	
	


	// Update position + orientation matrix
	void Update(float timestep)
	{
		//Update the vertices
		vertexUpdate();


		//Update 

		position += velocity * timestep;

		glm::mat3 AngVMatrix(0.0f);
		AngVMatrix[0].x = 0.0f;
		AngVMatrix[0].y = angVelocity.z;
		AngVMatrix[0].z = -angVelocity.y;
		AngVMatrix[1].x = -angVelocity.z;
		AngVMatrix[1].y = 0.0f;
		AngVMatrix[1].z = angVelocity.x;
		AngVMatrix[2].x = angVelocity.y;
		AngVMatrix[2].y = -angVelocity.x;
		AngVMatrix[2].z = 0.0f;

		orientationMatrix = orientationMatrix + (orientationMatrix * AngVMatrix * timestep);


		//reorthonormalize the R(t)
		orientationMatrix = glm::orthonormalize(orientationMatrix);


		ApplyForceAndTorque(timestep);


		//collision update
		BroadPhase();

	}



	
#pragma region "Collision Handling"
	void BroadPhase()
	{
		getBoundingSphere();

		getBoundingBox();

		flagCollision();
	}


	void flagCollision()
	{
		if (broadPhaseCollision)
			this->boundBoxColour = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
		else
			this->boundBoxColour = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
	}

	void getBoundingSphere()
	{
		boundSphere.center = position;

		//boundSphere.radius = 0.45f;
		boundSphere.radius = 0.5f;
	}


	void getBoundingBox()
	{
		xDist = glm::abs(minX - maxX) + 0.01f;
		yDist = glm::abs(minY - maxY) + 0.01f;
		zDist = glm::abs(minZ - maxZ) + 0.01f; 

		/*boundBox.posX = position.x + xDist / 2; 
		boundBox.negX = position.x - xDist / 2; 
		boundBox.posY = position.y + yDist / 2;
		boundBox.negY = position.y - yDist / 2;
		boundBox.posZ = position.z + zDist / 2;
		boundBox.negZ = position.z - zDist / 2;*/

		boundBox.posX = position.x + maxX + 0.1f;
		boundBox.negX = position.x + minX + 0.1f;
		boundBox.posY = position.y + maxY + 0.1f;
		boundBox.negY = position.y + minY + 0.1f;
		boundBox.posZ = position.z + maxZ + 0.1f;
		boundBox.negZ = position.z + minZ + 0.1f;
	}


	 
	void getModelMaxMin(glm::vec3 vertexPos)
	{
		//Sets the max and min for xyz everytime the model is drawn	
		GLfloat vPx = glm::normalize(vertexPos.x);
		GLfloat vPy = glm::normalize(vertexPos.y);
		GLfloat vPz = glm::normalize(vertexPos.z);

		if (vertexPos.x < minX) minX = vertexPos.x;
		if (vertexPos.y < minY) minY = vertexPos.y;
		if (vertexPos.z < minZ) minZ = vertexPos.z;
		if (vertexPos.x > maxX) maxX = vertexPos.x;
		if (vertexPos.y > maxY) maxY = vertexPos.y;
		if (vertexPos.z > maxZ) maxZ = vertexPos.z;
	}

#pragma endregion



#pragma region "Forces/Torques"
	void ApplyForceAndTorque(float timestep)
	{
		

		calcNetForce(gravityF, dragF, springF);

		calcNetTorque();


		//apply force(s)
		for (int i = 0; i < forces.size(); i++)
		{
			//glm::vec3 force1 = forces[i];
			applyForce(forces[i], timestep);

		}

		//      applyTorque(timestep);
	}



	void applyImpulseForce(glm::vec3 Jmpulse)
	{
		velocity += Jmpulse * massInv;
	}

	void applyImpulseTorque(glm::vec3 Jmpulse, glm::vec3 ra)
	{
		angVelocity += tensorMatrix * glm::cross(ra, Jmpulse);
	}


	void applyForce(glm::vec3 force, float timestep)
	{
		velocity += (force * timestep) / mass;
	}



	void applyTorque(float timestep)
	{
		calcIinv();
		angVelocity += tensorMatrix * nettorque * timestep;
	}


	void calcNetForce(bool grav, bool drag, bool spring)
	{
		//recalculate force
		forces.clear();

		

		if (grav)
		{
			glm::vec3 gravity(0.0f, -0.0981, 0.0f);
			gravity = gravity * mass;
			forces.push_back(gravity);
		}
		

		if (drag)
		{
			float dragCoefficient = 0.08f;
			glm::vec3 dragForce = -velocity * dragCoefficient;
			forces.push_back(dragForce);
		}

		if (spring)
		{
			// NB not working currently
			float springK = 0.52f;
			float kDrag = 0.5f;

			glm::vec3 springCenter;
			springCenter.x = -springK * (glm::length(position.x) - 2.0f); // +kDrag *(glm::dot(velocity.x, position.x) / glm::length(position.x)) * (position.x / glm::length(position.x));
			springCenter.y = -springK * (glm::length(position.y) - 2.0f); // +kDrag * (glm::dot(velocity.y, position.y) / glm::length(position.y)) * (position.y / glm::length(position.y));
			springCenter.z = -springK * (glm::length(position.z) - 2.0f); // +kDrag * (glm::dot(velocity.z, position.z) / glm::length(position.z)) * (position.z / glm::length(position.z));

			springCenter = -springCenter;

			glm::vec3 nSpringCenter = -springCenter;
			//forces.push_back(springCenter);
		}

		//force = gravity + dragForce + springCenter;
	}





	void calcNetTorque()
	{
		glm::vec3 torqueSum; 

		//for (int i = 0; i < model.meshes.size(); i++)
		//{
		//	for (int j = 0; j < model.meshes[i].vertices.size(); j++)
		//	{
		//		Vertex &vertex = model.meshes[i].vertices[j];
		//		//Calc the torque at that vertex
		//		for (int i = 0; i < forces.size(); i++)
		//		{
		//			torqueSum += glm::cross((vertex.Position - position), forces[i]);
		//		}
		//	}
		//}


		for (int i = 0; i < modelVertices.size(); i++)
		{
			//if(narrowPhaseCollision)
			
			//Calc the torque at that vertex
			for (int j = 0; j < forces.size(); j++)
			{
				torqueSum += glm::cross((modelVertices[1] - position), forces[j]); // apply force to one position?
			}


		}

		//Calc net torque
		this->nettorque = torqueSum;
	}

#pragma endregion



#pragma region "Updating/Correcting"

	

	void vertexUpdate()
	{
		//Reset the vertices vector
		modelVertices.clear();

		//Reset the bounding box
		minX = position.x - 0.5f, maxX = position.x + 0.5f, minY = position.y - 0.5f, maxY = position.y + 0.5f, minZ = position.z - 0.5f, maxZ = position.z + 0.5f;
		//Update the model vertex positions
		for (int i = 0; i < model.meshes.size(); i++)
		{
			for (int j = 0; j < model.meshes[i].vertices.size(); j++)
			{
				Vertex &vertex = model.meshes[i].vertices[j];
				Vertex &unchanged = modelCopy.meshes[i].vertices[j];
				vertex.Position = geometryDrawUpdatePerVertex(unchanged.Position);
				vertex.Normal = orientationMatrix * vertex.Normal;

				getModelMaxMin(vertex.Position);

				if (j < 8)
				{
					//add to the vertices
					modelVertices.push_back(vertex.Position);

					//std::cout << j << " : " << modelVertices[j].x << " " << modelVertices[j].y << " " << modelVertices[j].z << std::endl;
				}
				
				
				
			}

			model.meshes[i].setupMesh();
		}
	}


	

	glm::vec3 geometryDrawUpdatePerVertex(glm::vec3 vertex)
	{
		glm::vec3 newvertex = (orientationMatrix * vertex) + position;
		return newvertex;
	}


	glm::mat3 reorthonormOrientationMatrix(glm::mat3 orientationMatrix)
	{
		glm::vec3 Cx = orientationMatrix[0];
		glm::vec3 Cy = orientationMatrix[1];
		glm::vec3 Cz = orientationMatrix[2];

		Cx = Cx / (glm::length(Cx));
		Cy = glm::cross(Cz, Cx);
		Cy = Cy / (glm::length(Cy));
		Cz = glm::cross(Cz, Cx);
		Cz = Cz / (glm::length(Cz));

		orientationMatrix[0] = Cx;
		orientationMatrix[1] = Cy;
		orientationMatrix[2] = Cz;


	}

	glm::mat3 gramSchmidt(glm::mat3 orientationMatrix)
	{
		glm::vec3 r1 = glm::row(orientationMatrix, 0);
		glm::vec3 r2 = glm::row(orientationMatrix, 1);
		glm::vec3 r3 = glm::row(orientationMatrix, 2);

		float k = 0.25f;

		

		for (int i = 0; i < 10; i++)
		{

			r1 = r1 - k*((glm::dot(r1, r2))/glm::dot(r2,r2))*r2 - k*((glm::dot(r1, r3)) / glm::dot(r3, r3))*r3;
			r2 = r2 - k*((glm::dot(r2, r1)) / glm::dot(r1, r1))*r1 - k*((glm::dot(r2, r3)) / glm::dot(r3, r3))*r3;
			r3 = r3 - k*((glm::dot(r3, r2)) / glm::dot(r1, r1))*r1 - k*((glm::dot(r3, r2)) / glm::dot(r2, r2))*r2;

		}

		glm::mat3 orthonormalisedOM;

		//reset the matrix
		glm::row(orthonormalisedOM, 0, r1);
		glm::row(orthonormalisedOM, 1, r2);
		glm::row(orthonormalisedOM, 2, r3);

		return orthonormalisedOM;

	}

#pragma endregion




#pragma region "Inertial Tensor"

	void calcIinv() //double check inverse!
	{
		//glm::mat3 InertM = orientationMatrix * Ibody * glm::transpose(orientationMatrix);

		glm::mat3 tensorMatrix0 = orientationMatrix * IbodyInv * glm::transpose(orientationMatrix);

		this->tensorMatrix = tensorMatrix0;
		//return glm::inverse(tensorMatrix0);
	}


	//Calculates inertial tensor cube approximation
	glm::mat3 calcIbody()
	{
		//Assumes cube of dimension 1.0
		IbodyVal = ((1.0 / 12.0) * mass * (1.0f + 1.0f));

		Ibody[0].x = IbodyVal;
		Ibody[1].y = IbodyVal;
		Ibody[2].z = IbodyVal;


		IbodyInv = glm::inverse(Ibody);

		return IbodyInv;
	}

#pragma endregion




	//Creating
	void setModel(GLchar* path)
	{
		this->model = Model(path);
		this->modelCopy = Model(path);
	}

	//Body ID
	int getID()
	{
		return bodyID;
	}



};

#endif