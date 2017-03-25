#pragma once

#include <vector>
#include <glm/glm.hpp> // GLM Mathematics
//#include <glm/gtc/matrix_access.hpp> 
//#include "Mesh.h"
//#include "Model.h"


class DistanceChecker
{
	

	public:

	GLfloat distance;

	glm::vec3 closestPoint;

	glm::vec3 closestPolytopeVertex;
	int vertexID;


	DistanceChecker()
	{
		distance = 0.0f;

		closestPoint = glm::vec3 (0.0f, 0.0f, 0.0f);

	}


	//Point to point

	GLfloat point2point(glm::vec3 p0, glm::vec3 p1)
	{
		GLfloat d = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));


		//this->distance = d;
		return d;
	}


	//// Point to Line

	GLfloat point2lineDistance(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{
		
		glm::vec3 hTop = glm::cross(p2 - p1, p0 - p1);

		//GLfloat h = point2point(glm::vec3(0.0f, 0.0f, 0.0f), hTop) / point2point(p2, p1);
		GLfloat h = glm::length(hTop) / point2point(p2, p1);

		GLfloat d = glm::min(glm::min( h, point2point(p0, p1)), point2point(p0, p2));


		this->distance = d;

		return d;

	}

	void ResetChecker()
	{
		closestPoint = glm::vec3(100.0f);
		distance = 100.0f;
	}


	glm::vec3 point2LinePointDistance(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{
		glm::vec3 closestP;

		GLfloat d = point2lineDistance(p0, p1, p2);


		glm::vec3 uHat = glm::normalize( p2 - p1 );

		glm::vec3 pE = p1 + (glm::dot((p0 - p1),uHat)) * uHat;


		glm::vec3 hTop = glm::cross(p2 - p1, p0 - p1);
		GLfloat h = glm::length(hTop) / point2point(p2, p1);

		//perform check
		
		if (d == point2point(p0, p1))
			closestP = p1;
		else if (d == point2point(p0, p2))
			closestP = p2;
		else if (d == h)
		{
			closestP = pE;
			//std::cout << pE.x << pE.y << pE.z << endl;
		}
			
		//this->closestPoint = closestP;

		return closestP;
	}


	// Point to Plane

	GLfloat point2planeDistance(glm::vec3 p0, glm::vec3 p1, glm::vec3 planeNormal)
	{
		GLfloat d;

		d = glm::dot((p0 - p1), planeNormal);

		this->distance = d;

		return d;
	}


	glm::vec3 point2plane(glm::vec3 p0, glm::vec3 p1, glm::vec3 planeNormal)
	{
		glm::vec3 pf;

		pf = p0 - (glm::dot((p0 - p1), planeNormal)) * planeNormal;

		this->closestPoint = pf;

		return pf;
	}


	void convexPolytope2Plane(vector <glm::vec3> polytopeVertices, glm::vec3 planePoint, glm::vec3 planeNormal)
	{
		GLfloat d;
		glm::vec3 pf;
		int bodyID;


		//Get the distance from all the vertices to the plane
		for (int i; i < polytopeVertices.size(); i++)
		{
			if (i = 0) //get the first distance
			{
				d = glm::dot((polytopeVertices[i] - planePoint), planeNormal);
				bodyID = i;
			}

			GLfloat dNext = glm::dot((polytopeVertices[i] - planePoint), planeNormal);

			if (dNext < d) // get the shortest distance
			{
				d = dNext;
				bodyID = i;
			}
		}

		//Get the point on the plane that is closest to the closest vertex
		pf = point2plane(polytopeVertices[bodyID], planePoint, planeNormal);


		this->vertexID = bodyID;
		this->closestPolytopeVertex = polytopeVertices[bodyID];

		this->closestPoint = pf;
		this->distance = d;
	}


	// Voronoi region testing - Single Triangle

	int voronoiSingleTriangle(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3)
	{
		//check vertices
		bool v1, v2, v3;

		v1 = voronoiVertexChecker(p0, p1, p2, p3);
		v2 = voronoiVertexChecker(p0, p2, p1, p3);
		v3 = voronoiVertexChecker(p0, p3, p2, p1);

		
			 
		//check edges
		bool v12, v23, v31;

		v12 = voronoiEdgeChecker(p0, p1, p2, p3);
		v23 = voronoiEdgeChecker(p0, p2, p3, p1);
		v31 = voronoiEdgeChecker(p0, p3, p1, p2);
		

		


		//get distance, point
		GLfloat dist;
		glm::vec3 point;

		//Vertices first
		if (v1)
		{
			dist = point2point(p0, p1);
			this->closestPoint = p1;

			//std::cout << "v1" << std::endl;
			return 1;

		}
		else if (v2)
		{
			dist = point2point(p0, p2);
			this->closestPoint = p2;

			//std::cout << "v2" << std::endl;
			return 2;
		}
		else if (v3)
		{
			dist = point2point(p0, p3);
			this->closestPoint = p3;

			//std::cout << "v3" << std::endl;
			return 3;
		}
		// Then edges
		else if (v12)
		{
			dist = point2lineDistance(p0, p1, p2);
			this->closestPoint = point2LinePointDistance(p0, p1, p2);

			//std::cout << "v12" << std::endl;
			return 4;
		}
		else if (v23)
		{
			dist = point2lineDistance(p0, p2, p3);
			this->closestPoint = point2LinePointDistance(p0, p2, p3);

			//std::cout << "v23" << std::endl;
			return 5;
		}
		else if (v31)
		{
			dist = point2lineDistance(p0, p3, p1);
			this->closestPoint = point2LinePointDistance(p0, p3, p1);

			//std::cout << "v31" << std::endl;
			return 6;
		}
		// Its the face
		else
		{
			glm::vec3 nHat = glm::normalize(glm::cross((p2 - p1), (p3 - p1)));

			dist = glm::dot((p0 - p1), nHat);


			this->closestPoint = p0 - (glm::dot((p0 - p1), nHat) * nHat);

			this->distance = dist;


			//std::cout << "face!" << std::endl;
			return 7;
		}
			
		//Either way we've found the 
		this->closestPoint = point;
		this->distance = dist;
		//std::cout << distance << std::endl;


	}


	bool voronoiVertexChecker(glm::vec3 point, glm::vec3 vertexToCheck, glm::vec3 otherVertex, glm::vec3 otherVertex2)
	{
		GLfloat checker1, checker2;

		checker1 = glm::dot((point - vertexToCheck), (otherVertex - vertexToCheck));

		checker2 = glm::dot((point - vertexToCheck), (otherVertex2 - vertexToCheck));

		if (checker1 <= 0 && checker2 <= 0)
			return true;
		else
			return false;

	}


	bool voronoiEdgeChecker(glm::vec3 point, glm::vec3 vertexToCheck, glm::vec3 vertexToCheck2, glm::vec3 otherVertex)
	{
		GLfloat checker1, checker2, checker3;

		checker1 = glm::dot((point - vertexToCheck), (vertexToCheck2 - vertexToCheck));
		checker2 = glm::dot((point - vertexToCheck2), (vertexToCheck - vertexToCheck2));



		glm::vec3 n123 = glm::cross((vertexToCheck2 - vertexToCheck), (otherVertex - vertexToCheck));
		checker3 = glm::dot( (point - vertexToCheck) , (glm::cross((vertexToCheck2 - vertexToCheck), n123) ) ); //double check checker3


		

		



		if (checker1 >= 0 && checker2 >= 0 && checker3 >= 0)
			return true;
		else
			return false;
	}

};





