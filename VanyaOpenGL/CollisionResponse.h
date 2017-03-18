#pragma once
#ifndef _COLLISIONRESPONSE_H_
#define _COLLISIONRESPONSE_H_

//#include <GLFW/glfw3.h> // GLFW
//#include <glm/glm.hpp> // GLM Mathematics
#include <glm/gtx/string_cast.hpp>


//#include "RigidBody.h"



class CollisionResponse
{

	public:
	
	DistanceChecker dc;


	//Magnitude of impulse
	glm::vec3 jmpulse;


	glm::vec3 padot;
	glm::vec3 pbdot;
	

	GLfloat vrel;
	glm::vec3 ra, rb;


	vector<glm::vec3> contactPoints;

	glm::vec3 contactPointA;
	glm::vec3 contactPointB;
	glm::vec3 contactNormal;

	glm::vec3 planePos;
	glm::vec3 planeNormal;
	glm::vec3 planeVel;
	glm::vec3 planeAngVel;

	GLfloat term1, term2, term3, term4, numerator;


	RigidBody rb1;
	RigidBody rb2;

	bool applyImpulse;


	CollisionResponse()
	{
	}


	CollisionResponse(RigidBody rb1, glm::vec3 cPoint, glm::vec3 cNormal)
	{
		this->rb1 = rb1;
		this->contactPointA = cPoint;
		this->contactPointB = cPoint;
		this->contactNormal = cNormal;
	}


	// For rigid body to rigid body contact modelling
	void jRigidBodyRigidBody(glm::vec3 body1Pos, glm::vec3 body2Pos, glm::vec3 contactNormal, GLfloat coeffRest)
	{															

		padot = glm::cross((rb1.velocity + rb1.angVelocity), (contactPointA - rb1.position));
		pbdot = glm::cross((rb2.velocity + rb2.angVelocity), (contactPointB - rb2.position));
			
		ra = contactPointA - body1Pos;
		rb = contactPointB - body2Pos;

		vrel = glm::dot(contactNormal, (padot - pbdot));
		
		term1 = rb1.massInv;
		term2 = rb2.massInv;
		term3 = glm::dot(contactNormal, glm::cross(rb1.tensorMatrix * (glm::cross(ra, contactNormal)), ra));
		term4 = glm::dot(contactNormal, glm::cross(rb2.tensorMatrix * (glm::cross(rb, contactNormal)), rb));

		numerator = -(1 + coeffRest) * vrel;

		GLfloat jmpulse = numerator / (term1 + term2 + term3 + term4);

		glm::vec3 jforce = jmpulse * contactNormal;
	}



	glm::vec3 jRigidBodyPlane(glm::vec3 bodyPos, glm::vec3 planePos, glm::vec3 planeNormal, GLfloat coeffRest)
	{	
		//Static plane
		planeVel = glm::vec3(0.0f, 0.0f, 0.0f);
		planeAngVel = glm::vec3(0.0f, 0.0f, 0.0f);
		

		contactPointB = contactPointA;
		contactNormal = planeNormal;
		
		this->ra = (contactPointA - bodyPos);
		this->rb = (contactPointB - planePos);
		padot = rb1.velocity + glm::cross((rb1.angVelocity), (ra));
		pbdot = glm::vec3(0.0f); // planeVel + glm::cross(planeAngVel, (contactPointB - planePos));

		
		

		vrel = glm::dot(contactNormal, (padot - pbdot));


		rb1.calcIinv();

		term1 = rb1.massInv;
		term2 = 0.0f; //Plane has infinite mass
		term3 = glm::dot(contactNormal, glm::cross(rb1.tensorMatrix * (glm::cross(ra, contactNormal)), ra));
		term4 = 0.0f; //Plane has infinite moment of inertia

		


		
		numerator = -(1 + coeffRest) * vrel;

		GLfloat jmpulse = numerator / (term1 + term2 + term3 + term4);


		glm::vec3 jforce = jmpulse * contactNormal;

		if (vrel >= 0)
		{
			jforce = glm::vec3(0.0f);
		}
		else {
			//std::cout << "vrel:" << vrel << std::endl;
			//std::cout << "angular vel is " << glm::to_string(rb1.angVelocity) << std::endl;
		}

		return jforce;
	}






	void singleRigidBodyPlaneNarrowPhaseCheck(RigidBody rigbod, GLfloat narrowPhasePlaneThreshold)
	{

		this->rb1 = rigbod;

		contactPoints.clear();


		for (int i = 0; i < rb1.modelVertices.size(); i++)
		{

			glm::vec3 potentialContactP = rb1.modelVertices[i];

			GLfloat point2planeD = dc.point2planeDistance(potentialContactP, planePos, planeNormal);


			if (point2planeD <= narrowPhasePlaneThreshold) // 0.000000001
			{
				rb1.narrowPhaseCollision = true;

				//get all the contact points in a vector
				contactPoints.push_back(rb1.modelVertices[i]);
			}
		}
		//get the min of the contact points
		if (contactPoints.size()>0) {
 			std::sort(contactPoints.begin(), contactPoints.end(), [](auto i, auto j) {return i.y < j.y; });
			float minHeight = contactPoints[0].y;
			std::cout << "min is " << minHeight << std::endl;
			std::vector<glm::vec3> newContactPoints;
			for (int i = 0; i < contactPoints.size(); i++) {
				if (contactPoints[i].y==minHeight) {
					newContactPoints.push_back(contactPoints[i]);
					//std::cout << "accepted cp is " << glm::to_string(*(newContactPoints.end() - 1)) << std::endl;
				}
				else {
					//std::cout<<"rejected cp is "<<glm::to_string((contactPoints[i])) << std::endl;
				}
			}
			contactPoints.clear();
  			contactPoints.resize(newContactPoints.size());
			std::copy(newContactPoints.begin(), newContactPoints.end(), contactPoints.begin());
			//contactPoints = std::vector<glm:vec3>(newContactPoints);
			newContactPoints.clear();
		}
		
		if (contactPoints.size() == 3)
		{
			exit(-1);
		}

		if (contactPoints.size() == 0)
		{
			rb1.narrowPhaseCollision = false;
		}

		else {
			//rb1.narrowPhaseCollision = true;
			contactPointA = glm::vec3(0, 0, 0);
			for (int i = 0; i < contactPoints.size(); i++) {
				contactPointA += contactPoints[i];
			}
			//std::cout << "contacts size:" << contactPoints.size() << std::endl;

			//Get the mean of the contact points
			contactPointA /= (float)contactPoints.size();
		}

	}






	//void getImpulseContactPoint()
	//{

	//	std::cout << "Average " << std::endl;


	//	for (int i = 0; i < contactPoints.size(); i++)
	//	{
	//		if (contactPoints.size() == 1)
	//		{
	//			std::cout << "vertex" << std::endl;
	//			contactPointA = contactPoints[0];
	//			break;
	//		}
	//		else if (contactPoints.size() == 2 /*|| contactPoints.size() == 3*/)
	//		{
	//			std::cout << "edge" << std::endl;
	//			glm::vec3 p1 = contactPoints[0];
	//			glm::vec3 p2 = contactPoints[1];
	//			//midpoint
	//			contactPointA = glm::vec3((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0);

	//			std::cout << "Average " << glm::to_string(contactPointA) << std::endl;
	//			std::cout << "point1 " << glm::to_string(contactPoints[0]) << std::endl;
	//			std::cout << "point2 " << glm::to_string(contactPoints[1]) << std::endl;
	//			break;
	//		}

	//		else if (contactPoints.size() == 4)
	//		{
	//			std::cout << "face" << std::endl;
	//			glm::vec3 p1 = contactPoints[0];
	//			glm::vec3 p2 = contactPoints[1];
	//			glm::vec3 p3 = contactPoints[2];
	//			glm::vec3 p4 = contactPoints[3];

	//			glm::vec3 mp1((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0);
	//			glm::vec3 mp2((p3.x + p4.x) / 2.0, (p3.y + p4.y) / 2.0, (p3.z + p4.z) / 2.0);

	//			contactPointA = glm::vec3((mp1.x + mp2.x) / 2.0, (mp1.y + mp2.y) / 2.0, (mp1.z + mp2.z) / 2.0);

	//			break;
	//		}
	//	}
	//}


};

#endif