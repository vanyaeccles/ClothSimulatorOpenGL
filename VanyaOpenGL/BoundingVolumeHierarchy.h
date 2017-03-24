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

	bool leaf = false;

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

	Node(int i)
	{
		leftChild = new Node();
		rightChild = new Node();
	}


	void setLeaf(std::vector<Triangle> triangles)
	{
		leaf = true;
		nodeTriangles = triangles;
	}

	void setNodeBoundingVolume(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ)
	{
		boundingVolume.negX = MinX;
		boundingVolume.negY = MinY;
		boundingVolume.negZ = MinZ;

		boundingVolume.posX = MaxX;
		boundingVolume.posY = MaxY;
		boundingVolume.posZ = MaxZ;
	}


	void addChild(Node* child);

	void removeFromParent();

	
	AABB calculateBoundingBox(); // Axis-aligned bounding box

private:

	glm::vec3  m_positionRelativeToParentNode;

	glm::mat4 m_transformation;


};