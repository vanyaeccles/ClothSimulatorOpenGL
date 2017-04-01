#pragma once

#include <iostream> // Std. Includes
#include <list>
#include <glm/vec3.hpp>
#include <glm/matrix.hpp>
#include <vector>
#include <GL/glew.h> 

#include "BoundingObject.h"


class Node
{

public:

	bool leaf;

	//a node has a bounding volume that is large enough to enclose all of the objects that are inside this or the children of this node
	AABB boundingVolume;

	// references to it’s children
	Node *leftChild=NULL;
	Node *rightChild=NULL;

	//a vector of the objects it contains
	std::vector<Triangle> nodeTriangles;

	Node()
	{

	}

	/*Node(int i)
	{
		leftChild = new Node();
		rightChild = new Node();
	}*/

	void setLeftChild(Node *node)
	{
		leftChild = node;
	}

	void setRightChild(Node *node)
	{
		rightChild = node;
	}

	bool isLeafNode()
	{
		if (leaf) return true;
		else return false;
	}

	void setLeaf(std::vector<Triangle> triangles)
	{
		//std::cout << "Set as leaf" << std::endl;
		this->leaf = true;
		this->nodeTriangles = triangles;
	}


	void setNodeBoundingVolume(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ)
	{
		
		
		boundingVolume.xDist = glm::abs(MinX - MaxX);
		boundingVolume.yDist = glm::abs(MinY - MaxY);
		boundingVolume.zDist = glm::abs(MinZ - MaxZ);


		GLfloat xC = (MaxX + MinX) * 0.5, yC = (MaxY + MinY)  * 0.5, zC = (MaxZ + MinZ) * 0.5;

		boundingVolume.centerPos = glm::vec3(xC, yC, zC);


		boundingVolume.negX = MinX /*+ xC*/ - 0.1f;
		boundingVolume.negY = MinY /*+ yC*/ - 0.1f;
		boundingVolume.negZ = MinZ /*+ zC*/ - 0.1f;

		boundingVolume.posX = MaxX /*+ xC*/ + 0.1f;
		boundingVolume.posY = MaxY /*+ yC */+ 0.1f;
		boundingVolume.posZ = MaxZ /*+ zC */+ 0.1f;
	}


	void addChild(Node* child);

	void removeFromParent();

};