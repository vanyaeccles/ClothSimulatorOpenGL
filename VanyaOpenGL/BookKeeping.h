#pragma once
#ifndef _BOOKKEEPING_H_
#define _BOOKKEEPING_H_


#include <iostream> // Std. Includes
#include <string>
#include <algorithm>
#include <vector>
#include <GL/glew.h> 
#include <GLFW/glfw3.h> // GLFW
#include <glm/glm.hpp> // GLM Mathematics

#include "SweepAndPrune.h"
#include "CollisionResponse.h"

float lowerRandBoundPosition = -5.0f;
float upperRandBoundPosition = 5.0f;



// Typically a list (or table) of potentially colliding pairs is maintained. The
// broadphase culls trivially non-colliding cases from the potential colliders list before
// passing on to the narrow phase

class BroadPhaseCollisionDetection
{

public:

	vector<RigidBody> bodies;

	SweepAndPruneData sap;

	CollisionResponse ColRes;

	bool potentiallyColliding[NUM_RIGIDBODIES][NUM_RIGIDBODIES];
	
	/*vector<GLfloat> xList;
	vector<GLfloat> yList;
	vector<GLfloat> zList;*/

	vector<RigidBody> xList;
	vector<RigidBody> yList;
	vector<RigidBody> zList;


	BroadPhaseCollisionDetection()
	{ 
	}



	//setup the vector of rigid bodies
	void initialise()
	{
		// resize the rigid body vector
		getRBvector();

		//Give the rigid bodies a model and starting position
		setBodies();

		// Initialise to false
		setPotCol();

		//Initialise the lists
		//initializeLists();
	}


	void update()
	{
		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			//bodies[i].broadPhaseCollision = false;
		}

		sap.setPotColDim();

		setandsortLists();

		traverseList();

		//checkBoundBoxCollision();
	}





	// Sweep and Prune


	//Traverse lists and flag
	void traverseList()
	{
		for (int i = 0; i < NUM_RIGIDBODIES * 2; i++)
		{
			for (int j = i + 1; j < NUM_RIGIDBODIES * 2; j++)
			{

				//Check X

				//check its not the same object
				if (xList[i].bodyID != xList[j].bodyID)
				{
					if (xList[i].boundBox.posX < xList[j].boundBox.negX || xList[i].boundBox.negX > xList[j].boundBox.posX)
					{
						//non-overlapping
						sap.overlapStatusX[xList[i].bodyID][xList[j].bodyID] = false;
						sap.overlapStatusX[xList[j].bodyID][xList[i].bodyID] = false;
						//std::cout << " non overlapping " << std::endl;
						//break;
					}

					if (xList[i].boundBox.posX >= xList[j].boundBox.negX && xList[i].boundBox.negX <= xList[j].boundBox.posX)
					{
						//overlapping
						sap.overlapStatusX[xList[i].bodyID][xList[j].bodyID] = true;
						sap.overlapStatusX[xList[j].bodyID][xList[i].bodyID] = true;
						//std::cout << i << " " << j << " true" << std::endl;
						//std::cout << xList[i].maxX << " " << xList[j].maxX << " " << std::endl;
						//std::cout << " overlapping " << std::endl;
					}
				}


				
				

				//Check Y

				//check if its the same object
				if (yList[i].bodyID != yList[j].bodyID)
				{
					if (yList[i].boundBox.posY < yList[j].boundBox.negY || yList[i].boundBox.negY > yList[j].boundBox.posY)
					{
						//non-overlapping
						sap.overlapStatusY[yList[i].bodyID][yList[j].bodyID] = false;
						sap.overlapStatusX[yList[j].bodyID][yList[i].bodyID] = false;
						//break;
					}

					if (yList[i].boundBox.posY >= yList[j].boundBox.negY && yList[i].boundBox.negY <= yList[j].boundBox.posY)
					{
						//overlapping
						sap.overlapStatusY[yList[i].bodyID][yList[j].bodyID] = true;
						sap.overlapStatusX[yList[j].bodyID][yList[i].bodyID] = true;
					}
				}

				




				//Check Z

				////check if its the same object
				if (zList[i].bodyID != zList[j].bodyID)
				{
					if (zList[i].boundBox.posZ < zList[j].boundBox.negZ || zList[i].boundBox.negZ > zList[j].boundBox.posZ)
					{
					//non-overlapping
					sap.overlapStatusZ[zList[i].bodyID][zList[j].bodyID] = false;
					sap.overlapStatusZ[zList[j].bodyID][zList[i].bodyID] = false;
					//break;
					}

					if (zList[i].boundBox.posZ >= zList[j].boundBox.negZ && zList[i].boundBox.negZ <= zList[j].boundBox.posZ)
					{
					//overlapping
					sap.overlapStatusZ[zList[i].bodyID][zList[j].bodyID] = true;
					sap.overlapStatusZ[zList[j].bodyID][zList[i].bodyID] = true;
					}
				}
				
				

			}
		}
	}

	


	//fill the lists with rigidbodies, sorts by mins and maxes 
	void TsetandsortLists()
	{
		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			if (listPlaceChanged(xList, bodies[i]))
			{

			}


		}
	}

	bool listPlaceChanged(vector<RigidBody> list, RigidBody body)
	{

	}


	//fill the lists with rigidbodies, sorts by mins and maxes 
	void setandsortLists()
	{
		//Empty the lists
		xList.clear();
		yList.clear();
		zList.clear();

		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			/*xList.push_back(bodies[i].minX);
			xList.push_back(bodies[i].maxX);
			
			yList.push_back(bodies[i].minY);
			yList.push_back(bodies[i].maxY);
			
			zList.push_back(bodies[i].minZ);
			zList.push_back(bodies[i].maxZ);*/

			xList.push_back(bodies[i]);
			xList.push_back(bodies[i]);

			yList.push_back(bodies[i]);
			yList.push_back(bodies[i]);

			zList.push_back(bodies[i]);
			zList.push_back(bodies[i]);
		}


		//sorts them into ordered list
		insertion_sortX(xList);
		insertion_sortY(yList);
		insertion_sortZ(zList);

		
	}

	////insertion sort algorithm
	//void insertion_sort(std::vector<GLfloat>& array) 
	//{
	//	for (auto it = array.begin(), end = array.end(); it != end; ++it) 
	//	{
	//		// 1. Search
	//		auto const insertion_point = std::upper_bound(array.begin(), it, *it);
	//		// 2. Insert
	//		std::rotate(insertion_point, it, it + 1);
	//	}
	//}

	//insertion sort algorithm (sorts rigidbodies based on object maxX)
	void insertion_sortX(vector<RigidBody>& array) 
	{
		for (int i = 1; i < array.size(); ++i) {
			for (int j = i; j > 0 && array[j - 1].maxX > array[j].maxX; --j) {
				std::swap(array[j - 1], array[j]);
			}
		}
		//std::cout << " 0: " << array[0].bodyID << " 0: " << array[1].bodyID << " 1: " << array[2].bodyID << " 1: " << array[3].bodyID << std::endl;
	}

	//insertion sort algorithm (sorts rigidbodies based on object maxY)
	void insertion_sortY(vector<RigidBody>& array)
	{
		for (int i = 1; i < array.size(); ++i) {
			for (int j = i; j > 0 && array[j - 1].maxY > array[j].maxY; --j) {
				std::swap(array[j - 1], array[j]);
			}
		}
		//std::cout << " 0: " << array[0].bodyID << " 0: " << array[1].bodyID << " 1: " << array[2].bodyID << " 1: " << array[3].bodyID << std::endl;
	}

	//insertion sort algorithm (sorts rigidbodies based on object maxZ)
	void insertion_sortZ(vector<RigidBody>& array)
	{
		for (int i = 1; i < array.size(); ++i) {
			for (int j = i; j > 0 && array[j - 1].maxZ > array[j].maxZ; --j) {
				std::swap(array[j - 1], array[j]);
			}
		}
		//std::cout << " 0: " << array[0].bodyID << " 0: " << array[1].bodyID << " 1: " << array[2].bodyID << " 1: " << array[3].bodyID << std::endl;
	}






	////goes through vector of bodies and checks for bounding sphere collision
	void checkBoundBoxBruteForceCollision()
	{

		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			for (int j = 0; j < NUM_RIGIDBODIES; j++)
			{

				if (i != j)
				{
					//std::cout << boundSphereCollision(bodies[i], bodies[j]) << std::endl;

					if (bruteForceBoxCollision(bodies[i], bodies[j]))
					{
						//Bounding Spheres are overlapping
						bodies[i].broadPhaseCollision = true;
						bodies[j].broadPhaseCollision = true;
						potentiallyColliding[i][j] = true;
					}

					if (!bruteForceBoxCollision(bodies[i], bodies[j]))
					{
						//bodies[i].broadPhaseCollision = false;
						//bodies[j].broadPhaseCollision = false;
						potentiallyColliding[i][j] = false;
					}

					//std::cout << potentiallyColliding[i][j] << std::endl;
				}
			}
		}
	}

	bool bruteForceBoxCollision(RigidBody& rb1, RigidBody& rb2)
	{
		bool xOverlap = rb1.boundBox.posX >= rb2.boundBox.negX && rb1.boundBox.negX <= rb2.boundBox.posX;
		bool yOverlap = rb1.boundBox.posY >= rb2.boundBox.negY && rb1.boundBox.negY <= rb2.boundBox.posY;
		bool zOverlap = rb1.boundBox.posZ >= rb2.boundBox.negZ && rb1.boundBox.negZ <= rb2.boundBox.posZ;

		if (xOverlap && yOverlap && zOverlap)
		{
			return true;
		}

		else
		{
			return false;
		}
	}



	//goes through vector of bodies and checks for bounding sphere collision
	void checkBoundSphereCollision()
	{

		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			for (int j = 0; j < NUM_RIGIDBODIES; j++)
			{
				
				if (i != j)
				{
					//std::cout << boundSphereCollision(bodies[i], bodies[j]) << std::endl;

					if (boundSphereCollision(bodies[i], bodies[j]))
					{
						//Bounding Spheres are overlapping
						bodies[i].broadPhaseCollision = true;
						bodies[j].broadPhaseCollision = true;
						potentiallyColliding[i][j] = true;
					}

					if (!boundSphereCollision(bodies[i], bodies[j]))
					{
						//bodies[i].broadPhaseCollision = false;
						//bodies[j].broadPhaseCollision = false;
						potentiallyColliding[i][j] = false;
					}

					//std::cout << potentiallyColliding[i][j] << std::endl;
				}
				
			}
		}

	}

	bool boundSphereCollision(RigidBody& rb1, RigidBody& rb2)
	{

		//distance between their centres
		GLfloat d = rb1.dChecker.point2point((rb1.boundSphere.center), (rb2.boundSphere.center));
		//distance of both of their radii
		GLfloat rarb = rb1.boundSphere.radius + rb2.boundSphere.radius;

		if (d < rarb)
		{
			return true;
		}

		else
		{
			return false;
		}

	}


	//goes through vector of bodies xyz overlaps and checks for bounding box collision
	void checkBoundBoxCollision()
	{
		for (int i = 0; i < NUM_RIGIDBODIES; i++)
		{
			for (int j = 0; j < NUM_RIGIDBODIES; j++)
			{
					
				if (i != j)
				{
					bool xCheck = sap.checkOverlapX(i,j);
					bool yCheck = sap.checkOverlapY(i,j);
					bool zCheck = sap.checkOverlapZ(i,j);


					

					if (xCheck && yCheck && zCheck)
					{
						//Bounding Boxes are overlapping!
						bodies[i].broadPhaseCollision = true;
						bodies[j].broadPhaseCollision = true;
						potentiallyColliding[i][j] = true;

					}

					else if (!(xCheck && yCheck && zCheck))
					{
						//bodies[i].broadPhaseCollision = false;
						//bodies[j].broadPhaseCollision = false;
						potentiallyColliding[i][j] = false;
					}
					//std::cout << potentiallyColliding[i][j] << std::endl;
				}
			}
		}
	}






	void getRBvector()
	{
		//vector<RigidBody> bodies;
		this->bodies.resize(NUM_RIGIDBODIES);

	}

	void setPotCol()
	{
		for (int i = 0; i < NUM_RIGIDBODIES; i++)
			for (int j = 0; j < NUM_RIGIDBODIES; j++)
				potentiallyColliding[i][j] = false;
		        
	}


	void setBodies()
	{
		bodies[0].setModel("U:/Physics/FinalProject/Stuff/Models/cube.obj");

		bodies[1].setModel("U:/Physics/FinalProject/Stuff/Models/largercube.obj");

		bodies[2].setModel("U:/Physics/FinalProject/Stuff/Models/largestcube.obj");

		for (int i = 0; i < bodies.size(); i++)
		{

			//bodies[i].setModel("U:/Physics/Phys4/Stuff/Models/cube.obj");

			
			/*
			bodies[i].position[0] = lowerRandBoundPosition + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / upperRandBoundPosition - lowerRandBoundPosition));
			bodies[i].position[1] = lowerRandBoundPosition + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / upperRandBoundPosition - lowerRandBoundPosition));
			bodies[i].position[2] = lowerRandBoundPosition + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX / upperRandBoundPosition - lowerRandBoundPosition));
			*/
			
			//bodies[i].position[0] = 0.0f;
			//bodies[i].position[1] = 3.0f;
			//bodies[i].position[2] = 0.0f;
			
			//give them an integer ID
			bodies[i].bodyID = i;


		}

		
	}
	



};





#endif