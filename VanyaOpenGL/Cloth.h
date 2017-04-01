#pragma once
#include <vector>

#include "Particle.h"
#include "ParticleConstraint.h"
#include "ParticleSpring.h"
#include "Triangle.h"
#include "BoundingVolumeHierarchy.h"

#include "DistanceChecks.h"

class Cloth
{
public:
	//Numbers of particles in the x and y directions
	int gridWidth;
	int gridHeight;

	// Spacing along the x and y directions
	float width;
	float height;

	float clothThickness;

	//Constraints
	bool structural, shear, flexion;
	float structuralS = 1.0f;
	float shearS = 1.0f;
	float flexionS = 1.0f;

	//Springs
	const int strucSpring = 0;
	const int shearSpring = 1;
	const int flexionSpring = 2;
	//Spring constants
	float	structuralKs = 50.75f, structuralKd = -0.25f;
	float	shearKs = 50.75f, shearKd = -0.25f;
	float	flexionKs = 50.95f, flexionKd = -0.25f;


	// Collision variables
	float k_R = 0.0f;
	GLfloat coResFactor = 1.0f + k_R;

	// Bounding Volume Hierarchy
	Node rootNode;
	std::vector<Node*> ClothNodes;

	// Cloth contents
	std::vector<Particle> particles;
	std::vector<ParticleConstraint> particleConstraints;
	std::vector<ParticleSpring> particleSprings;
	std::vector<Triangle> clothTriangles;


	//Distance checker
	DistanceChecker dChecker;



	Particle* getParticle(int x, int y)
	{
		return &particles[y*gridWidth + x];
	}

	void constrainParticles(Particle* p1, Particle* p2, float strength)
	{
		particleConstraints.push_back(ParticleConstraint(p1, p2, strength));
	}

	void addSpring2Particles(Particle* p1, Particle* p2, float _Ks, float _Kd, int _springType)
	{
		particleSprings.push_back(ParticleSpring(p1, p2, _Ks, _Kd, _springType));
	}

	void makeTriangle(Particle* p1, Particle* p2, Particle* p3, Particle* p4, int id)
	{
		clothTriangles.push_back(Triangle(p1, p2, p3, p4, id));
	}



	Cloth(float width, float height, float _clothThickness, int particleWidthNumber, int particleHeightNumber, bool _structural, bool _shear, bool _flexion, int threshBVH)
		: clothThickness(_clothThickness), gridWidth(particleWidthNumber), gridHeight(particleHeightNumber), width(width), height(height), structural(_structural), shear(_shear), flexion(_flexion)
	{
		//initialise the size of the cloth grid
		gridWidth = particleWidthNumber;
		gridHeight = particleHeightNumber;


		initialiseParticleGrid();

		addConstraints(structural, shear, flexion);

		//addSprings(structural, shear, flexion);

		//initialiseParticleSecondaryConstraints();


		addTriangles();

		//pinCloth();

		// Builds a BVH of AABBs
		BuildAABBVH(&rootNode, this->clothTriangles, threshBVH);
	}




	void Update(float dampingConstant, int constraintIterations, int springIterations, float timestep)
	{

		satisfyConstraints(constraintIterations);

		//satisfySprings(springIterations, timestep);


		verletIntegrations(dampingConstant, timestep);


		for (int i = 0; i < clothTriangles.size(); i++)
			clothTriangles[i].GetBoundingBox();
	}




#pragma region INIT_PARTICLES

	// This creates and fills the grid with particles
	void initialiseParticleGrid()
	{
		//initialise the vector of particles
		particles.resize(gridWidth * gridHeight);

		for (int i = 0; i < gridWidth; i++) // @vertical
		{
			for (int j = 0; j < gridHeight; j++)
			{
				//glm::vec3 particlePos(width * (i / (float)gridWidth), -height * (j / (float)gridHeight), 0.0f); // For @vertical hanging cloth
				glm::vec3 particlePos(width * (i / (float)gridWidth), 0.0f, height * (j / (float)gridHeight)); // For @horizontal cloth

				// insert particle in column i at j'th row
				particles[j * gridWidth + i] = Particle(particlePos, 1.0f);


				//Make the hems a little heavier as in Bridson et al
				/*
				'The heavier edges and corners give the cloth an attractive flare similar to that of real cloth where tailors often make hems a little heavier'
				*/
				if (i == 0 || i == gridWidth || j == 0 || j == gridHeight)
					particles[j * gridWidth + i] = Particle(particlePos, 1.5f);
			}
		}
	}


	// Fixes the particle grid at specified particle positions
	void pinCloth()
	{
		for (int i = 0; i < 1; i++) //@vertical
		{
			//getParticle(0 + i, 0)->offsetPosition(glm::vec3(0.5, 0.0, 0.0));
			getParticle(0 + i, 0)->pinParticle();

			//getParticle(0 + i, 0)->offsetPosition(glm::vec3(-0.5, 0.0, 0.0)); 
			getParticle(gridWidth - 1 - i, 0)->pinParticle();
		}

		//getParticle(10, 0)->offsetPosition(glm::vec3(0.0, 0.0, 1.0)); 
		//getParticle(10, 0)->pinParticle();

		//getParticle(0, 0)->offsetPosition(glm::vec3(0.0, 0.0, 0.0));
		//getParticle(0, 0)->pinParticle();

		//getParticle(0, 0)->pinParticle();
		//getParticle(particleGridWidth-1, particleGridHeight-1)->pinParticle();
		//getParticle(particleGridWidth - 11, particleGridHeight-1)->pinParticle();
	}

#pragma endregion




#pragma region "BOUNDING_VOL_HIERARCHY"

	void ClearNodes()
	{
		ClothNodes.clear();
	}

	Node* GetClothRootNode()
	{
		return &(this->rootNode);
	}

	void CheckBVH(Node *node, Particle *p, float timestep)
	{
		if (!node)
		{
			//std::cout << "null pointer";
			return;
		}

		// check for intersection
		if (!Point2AABBIntersectionTest(p->position, node->boundingVolume))
		{
			//std::cout << "Not intersecting" << std::endl;
			return;
		}
			

		//std::cout << "BB colision" << std::endl;


		// if the node is a leaf, check all the triangles for collision
		if (node->leaf)
		{
			for (int i = 0; i < node->nodeTriangles.size(); i++)
				onBoundingBoxCollisionPoint2Tri(&node->nodeTriangles[i], p, timestep);
			return;
		}

		if (node->leaf == false)
		{
			CheckBVH(node->leftChild, p, timestep);
			CheckBVH(node->rightChild, p, timestep);
		}
	}


	// For testing a point intersecting with an AABB
	bool Point2AABBIntersectionTest(glm::vec3 point, AABB box)
	{
		bool xIntersect = (point.x >= box.negX && point.x <= box.posX);
		bool yIntersect = (point.y >= box.negY && point.y <= box.posY);
		bool zIntersect = (point.z >= box.negZ && point.z <= box.posZ);


		if (xIntersect && yIntersect && zIntersect)
			return true;
		else
			return false;
	}



	// builds the bounding box hierarchy
	void BuildAABBVH(Node *node, std::vector<Triangle> triangles, int threshold)
	{
		

		// Sets the mins and maxs with a default value from the triangle vector
		float MinX = triangles[0].minX, MaxX = triangles[0].maxX, MinY = triangles[0].minY, MaxY = triangles[0].maxY, MinZ = triangles[0].minZ, MaxZ = triangles[0].maxZ;

		

		//Get the min and maxs of the bounding boxes
		for (int i = 1; i < triangles.size(); i++)
		{
			if (triangles[i].minX < MinX) MinX = triangles[i].minX;
			if (triangles[i].minY < MinY) MinY = triangles[i].minY;
			if (triangles[i].minZ < MinZ) MinZ = triangles[i].minZ;
			if (triangles[i].maxX > MaxX) MaxX = triangles[i].maxX;
			if (triangles[i].maxY > MaxY) MaxY = triangles[i].maxY;
			if (triangles[i].maxZ > MaxZ) MaxZ = triangles[i].maxZ;
		}
		//std::cout << triangles.size() << std::endl;


		//this nodes AABB is calculated and stored
		node->setNodeBoundingVolume(MinX, MaxX, MinY, MaxY, MinZ, MaxZ);



		//Check if its small enough to be leaf, and then assign the remaining triangles to its object list
		if (triangles.size() <= threshold)
		{
			//std::cout << "Set as leaf" << std::endl;
			node->setLeaf(triangles);
			ClothNodes.push_back(node);
			return;
		}



		// sorts the objects 
		triangles = sortObjectsAlongLongestAxis(triangles);
		//split the sorted list into two lists
		std::vector<Triangle> rightObjects = splitRightObjects(triangles);
		std::vector<Triangle> leftObjects = splitLeftObjects(triangles);

		//std::cout << "left is " << leftObjects.size() << std::endl;
		//std::cout << "right is " << rightObjects.size() << std::endl;



		//node->leftChild = new Node();
		node->setLeftChild(new Node());
		//node->rightChild = new Node();
		node->setRightChild(new Node());


		ClothNodes.push_back(node);
		

		if (leftObjects.size() >= threshold)
		{
			BuildAABBVH(node->leftChild, leftObjects, threshold);
			//std::cout << "Leftie" << std::endl;
		}

		if (rightObjects.size() >= threshold)
		{
			BuildAABBVH(node->rightChild, rightObjects, threshold);
			//std::cout << "Rightie" << std::endl;
		}
	}




	std::vector<Triangle> sortObjectsAlongLongestAxis(std::vector<Triangle> triangles) //@TODO double check
	{
		GLfloat minX = triangles[0].minX, maxX = triangles[0].maxX, minY = triangles[0].minY, maxY = triangles[0].maxY, minZ = triangles[0].minZ, maxZ = triangles[0].maxZ;



		//gets the mins/maxs out of all the triangles (based on their upper center) in the vector
		for (int i = 1; i < triangles.size(); i++)
		{
			glm::vec3 cenTri = triangles[i].getTriangleUpperCenter();

			if (cenTri.x < minX) minX = cenTri.x;
			if (cenTri.y < minY) minY = cenTri.y;
			if (cenTri.z < minZ) minZ = cenTri.z;
			if (cenTri.x > maxX) maxX = cenTri.x;
			if (cenTri.y > maxY) maxY = cenTri.y;
			if (cenTri.z > maxZ) maxZ = cenTri.z;
		}

		GLfloat xDis = glm::length(maxX - minX);
		GLfloat yDis = glm::length(maxY - minY);
		GLfloat zDis = glm::length(maxZ - minZ);

		if (xDis >= yDis && xDis >= zDis)
		{
			//sort on x
			for (int i = 1; i < triangles.size(); ++i) {
				for (int j = i; j > 0 && triangles[j - 1].upperCenter.x > triangles[j].upperCenter.x; --j) {
					std::swap(triangles[j - 1], triangles[j]);
				}
			}
			return triangles;
		}

		if (yDis >= xDis && yDis >= zDis)
		{
			//sort on y
			for (int i = 1; i < triangles.size(); ++i) {
				for (int j = i; j > 0 && triangles[j - 1].upperCenter.y > triangles[j].upperCenter.y; --j) {
					std::swap(triangles[j - 1], triangles[j]);
				}
			}
			return triangles;
		}

		if (zDis >= xDis && zDis >= yDis)
		{
			//sort on z
			for (int i = 1; i < triangles.size(); ++i) {
				for (int j = i; j > 0 && triangles[j - 1].upperCenter.z > triangles[j].upperCenter.z; --j) {
					std::swap(triangles[j - 1], triangles[j]);
				}
			}
			return triangles;
		}
	}



	std::vector<Triangle> splitLeftObjects(std::vector<Triangle> triangles)
	{
		int midPoint = triangles.size() * 0.5f;
		std::vector<Triangle> leftTriangles;
		leftTriangles.clear();

		for (int i = 0; i < midPoint; i++)
		{
			leftTriangles.push_back(triangles[i]);
		}

		return leftTriangles;
	}

	std::vector<Triangle> splitRightObjects(std::vector<Triangle> triangles)
	{
		int midPoint = triangles.size() * 0.5f;
		std::vector<Triangle> rightTriangles;
		rightTriangles.clear();

		for (int i = triangles.size() - 1; i > midPoint; i--)
		{
			rightTriangles.push_back(triangles[i]);
		}

		return rightTriangles;
	}

#pragma endregion



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
		for (int i = 0; i < gridWidth; i++)
		{
			for (int j = 0; j < gridHeight; j++)
			{
				if (structuralConstraints)
				{
					// masses [i, j]--[i+1, j], [i, j]--[i, j+1]
					if (i < gridWidth - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j), structuralS);

					if (j < gridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i, j + 1), structuralS);

					if (i < gridWidth - 1 && j < gridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1), structuralS);

					if (i < gridWidth - 1 && j < gridHeight - 1)
						constrainParticles(getParticle(i + 1, j), getParticle(i, j + 1), structuralS);
				}

				if (shearConstraints)
				{
					// masses [i,j]--[i+1, j+1], [i+1, j]--[i, j+1]
					/*if (i < particleGridWidth - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1));

					if (j < particleGridHeight - 1)
						constrainParticles(getParticle(i, j), getParticle(i + 1, j + 1));*/

					if (i < gridWidth - 1 && j < gridHeight - 1)
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

					if (i < gridWidth - 2 && j < gridHeight - 2)
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
		for (int i = 0; i < gridWidth; i++)
		{
			for (int j = 0; j < gridHeight; j++)
			{

			}
		}
	}

	// masses [i,j]--[i+2, j], [i, j]--[i, j+2]
	void flexionConstraints()
	{
		for (int i = 0; i < gridWidth; i++)
		{
			for (int j = 0; j < gridHeight; j++)
			{

			}
		}
	}


	void initialiseParticleSecondaryConstraints()
	{
		// Connect secondary NESW neighbours with a distance constraint (distance 2 and sqrt(4) in the grid)
		for (int i = 0; i < gridWidth; i++)
		{
			for (int j = 0; j < gridHeight; j++)
			{
				if (i < gridWidth - 2)
					constrainParticles(getParticle(i, j), getParticle(i + 2, j), structuralS);

				if (j < gridHeight - 2)
					constrainParticles(getParticle(i, j), getParticle(i, j + 2), structuralS);

				if (i < gridWidth - 2 && j < gridHeight - 2)
					constrainParticles(getParticle(i, j), getParticle(i + 2, j + 2), structuralS);

				if (i < gridWidth - 2 && j < gridHeight - 2)
					constrainParticles(getParticle(i + 2, j), getParticle(i, j + 2), structuralS);
			}
		}
	}

#pragma endregion



#pragma region SPRINGS

	/*
	'In our basic model, particles are arranged in a rectangular array with structural springs connecting immediate neighbors.
	Diagonal springs provide shear support,
	and springs connected to every other node (with a stabilization spring attached to the center node in between) resist bending's
	*/

	void addSprings(bool structuralConstraints, bool shearConstraints, bool flexionConstraints)
	{
		for (int i = 0; i < gridWidth; i++)
		{
			for (int j = 0; j < gridHeight; j++)
			{
				if (structuralConstraints)
				{
					// binds masses [i, j]--[i+1, j], [i, j]--[i, j+1]
					if (i < gridWidth - 1)
						addSpring2Particles(getParticle(i, j), getParticle(i + 1, j), structuralKs, structuralKd, structuralS);

					if (j < gridHeight - 1)
						addSpring2Particles(getParticle(i, j), getParticle(i, j + 1), structuralKs, structuralKd, structuralS);

					//if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
					//	addSpring2Particles(getParticle(i, j), getParticle(i + 1, j + 1), structuralKs, structuralKd, structuralS);

					//if (i < particleGridWidth - 1 && j < particleGridHeight - 1)
						//addSpring2Particles(getParticle(i + 1, j), getParticle(i, j + 1), structuralKs, structuralKd, structuralS);
				}

				if (shearConstraints)
				{
					// binds masses [i,j]--[i+1, j+1], [i+1, j]--[i, j+1]
					if (i < gridWidth - 1 && j < gridHeight - 1)
					{
						addSpring2Particles(getParticle(i, j), getParticle(i + 1, j + 1), shearKs, shearKd, shearSpring);
						addSpring2Particles(getParticle(i + 1, j), getParticle(i, j + 1), shearKs, shearKd, shearSpring);
					}
				}

				if (flexionConstraints)
				{
					// binds masses [i,j]--[i+2, j], [i, j]--[i, j+2]

					if (i < gridWidth - 2 && j < gridHeight - 2)
					{
						addSpring2Particles(getParticle(i, j), getParticle(i + 2, j), flexionKs, flexionKd, flexionSpring);
						addSpring2Particles(getParticle(i, j), getParticle(i, j + 2), flexionKs, flexionKd, flexionSpring);

						addSpring2Particles(getParticle(i, j), getParticle(i + 2, j + 2), flexionKs, flexionKd, flexionSpring);
						addSpring2Particles(getParticle(i, j), getParticle(i + 2, j + 2), flexionKs, flexionKd, flexionSpring);

					}

					if (i < gridWidth - 4 && j < gridHeight - 4)
					{
						addSpring2Particles(getParticle(i, j), getParticle(i + 4, j), flexionKs, flexionKd, flexionSpring);
						addSpring2Particles(getParticle(i, j), getParticle(i, j + 4), flexionKs, flexionKd, flexionSpring);

						addSpring2Particles(getParticle(i, j), getParticle(i + 4, j + 4), flexionKs, flexionKd, flexionSpring);
						addSpring2Particles(getParticle(i, j), getParticle(i + 4, j + 4), flexionKs, flexionKd, flexionSpring);

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


#pragma region TRIANGLES

	void addTriangles()
	{
		for (int x = 0; x < gridWidth - 1; x++)
		{
			for (int y = 0; y < gridHeight - 1; y++)
			{
				makeTriangle(getParticle(x + 1, y), getParticle(x, y), getParticle(x, y + 1), getParticle(x + 1, y + 1), x + y);
				makeTriangle(getParticle(x + 1, y + 1), getParticle(x + 1, y), getParticle(x, y + 1), getParticle(x, y), x + y);
			}
		}
	}

#pragma endregion



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


	void applyWindForce(const glm::vec3 direction, float timestep)
	{
		std::vector<Triangle>::iterator triangle;
		for (triangle = clothTriangles.begin(); triangle != clothTriangles.end(); triangle++)
		{
			(*triangle).addWindForcesForClothTriangle(direction, timestep); // add the forces to each particle in the triangle
		}
	}

#pragma endregion



#pragma region COLLISION

	void onBoundingBoxCollisionPoint2Tri(Triangle *clothTri, Particle *cpoint, float timestep)
	{
		dChecker.ResetChecker();

		/*
		'Proximity is determined for both point-triangle pairs and edge-edge pairs.
		If a pair is close enough, then two kinds of repul-sion forces are applied.
		The first is based on an inelastic collision, and the second is a spring based force'
		*/

		// I inelastic repulsion

		//Compute the closest point with voronoi
		dChecker.voronoiSingleTriangle(cpoint->position, clothTri->p1->position, clothTri->p2->position, clothTri->p3->position);
		glm::vec3 contactPoint = dChecker.closestPoint;


		if (glm::abs(dChecker.distance) > 0.2f || glm::abs(dChecker.distance) == 0.0000f) //for some reason the returned distance is 0 on the first few checks
		{
			//std::cout << "No collision" << std::endl;
			return;
		}

		else
		{
			/*std::cout << "Distance: " << dChecker.distance << std::endl;
			std::cout << "Closest point: " << glm::to_string(contactPoint) << std::endl;
			std::cout << "Particle Pos: " << glm::to_string(cpoint.getPosition()) << std::endl;

			std::cout << "T1: " << glm::to_string(clothTri.p1->getPosition()) << std::endl;
			std::cout << "T2: " << glm::to_string(clothTri.p2->getPosition()) << std::endl;
			std::cout << "T3: " << glm::to_string(clothTri.p3->getPosition()) << std::endl;

			std::cout << "Triangle: " << clothTri.id << std::endl;*/
			//std::cout << "collision!" << std::endl;
			//return;
		}

		glm::vec3 contactNormal = glm::normalize(clothTri->getTriangleNormal()); //contact normal approximated as the triangle normal

		//Gets contact point in barycentric coordinates
		glm::vec3 baryPoint = clothTri->getBaryCentricCoordinates(contactPoint);

		GLfloat vrel; //@TODO Double-Check
		glm::vec3 interpolatedTriangleVelocity = (baryPoint.x * clothTri->p1->getVerletVelocity(timestep)) + (baryPoint.y * clothTri->p2->getVerletVelocity(timestep)) + (baryPoint.z * clothTri->p3->getVerletVelocity(timestep));
		glm::vec3 pointVelocity = cpoint->getVerletVelocity(timestep);

		vrel = glm::dot(contactNormal, (pointVelocity - interpolatedTriangleVelocity));

		//'To stop the imminent collision we apply an inelastic impulse of magnitude Ic=mvN/ 2 in the normal direction'
		glm::vec3 inelasticImpulse = clothTri->mass * vrel * contactNormal / 2.0f;

		//std::cout << glm::to_string(inelasticImpulse) << std::endl;

		applyImpulse2Triangle(inelasticImpulse, clothTri, cpoint, baryPoint, contactPoint, contactNormal, timestep);



		//Change the colour of the triangle hit
		clothTri->FlagCollisionColour();
		cpoint->setColour(glm::vec3(1.0f, 0.0f, 0.0f));


		//return;



		// II 'The  spring  based  repulsion  force' - @TODO
		// 'spring repulsion force is limited to a maximum when the objects touch, thus avoiding problems with stiffness'
		float overlap = clothThickness - (glm::dot(cpoint->getPosition() - (baryPoint.x * clothTri->p1->position) - (baryPoint.y * clothTri->p2->position) - (baryPoint.z * clothTri->p3->position), contactNormal));

		//std::cout << "over: " << glm::dot(cpoint->getPosition() - (baryPoint.x * clothTri->p1->position) - (baryPoint.y * clothTri->p2->position) - (baryPoint.z * clothTri->p3->position), contactNormal) << std::endl;


		// 'we found that matching the stiffness of the stretch springs in the cloth gave good results'
		glm::vec3 springRepulsionForce = shearKs * overlap * contactNormal;

		float overlapThreshold = (0.1f * overlap) / timestep;

		// ' If the normal component of relative velocity vN >= 0.1d/dt already we apply no repulsion'
		if (vrel >= overlapThreshold)
		{
			return;
		}

		else
		{
			//std::cout << "Spring based repulsion" << std::endl;


			//std::cout << "1: " << (timestep * shearKd * overlap) << std::endl;
			//std::cout << "2: " << clothTri->mass * (overlapThreshold - vrel) << std::endl;

			//This doesn't seem to work well? @TODO
			glm::vec3 springImpulse = contactNormal * -std::min((/*timestep * */shearKs * overlap), clothTri->mass * (overlapThreshold - vrel));

			springImpulse = contactNormal * -clothTri->mass * (overlapThreshold - vrel);
			//std::cout << "SpringImpulse Force: " << glm::to_string(springImpulse) << std::endl;

			applyImpulse2Triangle(springImpulse, clothTri, cpoint, baryPoint, contactPoint, contactNormal, timestep);


			//
			// III calculate and apply friction impulse
			//
			//get the precollision relative tangential velocity, projection of the relative velocity onto the triangle @TODO double check
			glm::vec3 vrelT;
			vrelT = (pointVelocity - interpolatedTriangleVelocity) - vrel / (pow(glm::length(contactNormal), 2)) * contactNormal;

			//std::cout << "VrelT: " << glm::to_string(vrelT) << std::endl;

			glm::vec3 interpolatedTriangleVelocityNEW = (baryPoint.x * clothTri->p1->getVerletVelocity(timestep)) + (baryPoint.y * clothTri->p2->getVerletVelocity(timestep)) + (baryPoint.z * clothTri->p3->getVerletVelocity(timestep));
			glm::vec3 pointVelocityNEW = cpoint->getVerletVelocity(timestep);
			GLfloat vrelNEW = glm::dot(contactNormal, (pointVelocityNEW - interpolatedTriangleVelocityNEW));

			GLfloat deltaVrel = (vrelNEW - vrel)/*/timestep*/;

			float fricCoeff = 0.45f;
			GLfloat velTerm = deltaVrel / glm::length(vrelT); //@TODO double check

			glm::vec3 fricVel = std::max((1.0f - fricCoeff * velTerm), 0.0f) * vrelT;

			

			if (glm::length(fricVel) > 0)
			{
				//std::cout << "fricVel: " << glm::to_string(fricVel) << std::endl;
				//applyImpulse2Triangle(fricVel, clothTri, cpoint, baryPoint, contactPoint, contactNormal, timestep);
			}

		}
	}





	void applyImpulse2Triangle(glm::vec3 impulse, Triangle *clothTri, Particle *parti, glm::vec3 baryPoint, glm::vec3 contactPoint, glm::vec3 contactNormal, float timestep)
	{
		// 'For the point-triangle case,  where an interior point of triangle~x1~x2~x3 with barycentric coordinates w1,w2,w3 
		// is interacting with point~x4 , we compute adjusted impulses'


		glm::vec3 adjustedImpulse = (2.0f * impulse) / (1.0f + (pow(baryPoint[0], 2.0f)) + (pow(baryPoint[1], 2.0f)) + (pow(baryPoint[2], 2.0f)));

		// 'Weighting the impulses in this way introduces appropriate torques for off-center interactions as well as giving continuity across triangle boundaries'

		float b1 = baryPoint[0], b2 = baryPoint[1], b3 = baryPoint[2];

		glm::vec3 vp1 = (b1 * (adjustedImpulse * clothTri->massinv) * contactNormal) * 100.0f;
		glm::vec3 vp2 = (b2 * (adjustedImpulse * clothTri->massinv) * contactNormal) * 100.0f;
		glm::vec3 vp3 = (b3 * (adjustedImpulse * clothTri->massinv) * contactNormal) * 100.0f;


		clothTri->p1->velocity += vp1;
		clothTri->p2->velocity += vp2;
		clothTri->p3->velocity += vp3;

		//Apply the velocity change to the triangle points
		clothTri->p1->postCollisionApplyVelocity(vp1, timestep);
		clothTri->p2->postCollisionApplyVelocity(vp2, timestep);
		clothTri->p3->postCollisionApplyVelocity(vp3, timestep);

		//Apply the velocity change to the particle
		glm::vec3 pointVelocity = parti->getVerletVelocity(timestep) - (adjustedImpulse * parti->massinv) * contactNormal;

		//parti->postCollisionApplyVelocity(pointVelocity, timestep);
	}








	//Simple check for bruteforce cloth-plane collision, inelastic - @TODO incorporate with BVH
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





	void CheckCollisionWithSphere(glm::vec3 sCenter, float sRadius) //@TODO doesn't work great with springs, slightly better with constraints
	{
		std::vector<Particle>::iterator particle;
		for (particle = particles.begin(); particle != particles.end(); particle++)
		{
			glm::vec3 distance = (*particle).getPosition() - sCenter;
			float disLength = glm::length(distance);

			if (disLength < sRadius)
			{
				//(*particle).oldPosition += (/*glm::normalize*/(distance)* sRadius - disLength); 
				(*particle).position += (/*glm::normalize*/(distance) * (sRadius - disLength));
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