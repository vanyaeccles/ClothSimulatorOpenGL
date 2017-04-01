#pragma once
#ifndef _BOUNDINGOBJECT_H_
#define _BOUNDINGOBJECT_H_

#include <glm/vec3.hpp>
#include <vector>
#include <GL/glew.h> 


struct BoundingSphere
{

	glm::vec3 center;

	GLfloat radius;

};

struct AABoundingBox
{

	GLfloat posX;
	GLfloat negX;
	
	GLfloat posY;
	GLfloat negY;

	GLfloat posZ;
	GLfloat negZ;
};


struct AABB
{
	GLfloat posX;
	GLfloat negX;

	GLfloat xDist;

	GLfloat posY;
	GLfloat negY;

	GLfloat yDist;

	GLfloat posZ;
	GLfloat negZ;

	GLfloat zDist;

	glm::vec3 centerPos;
};

#endif