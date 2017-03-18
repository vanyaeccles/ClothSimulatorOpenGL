#pragma once

#include "ParticleSystem.h"
#include <glm/gtx/string_cast.hpp>

//Older implementation

class ClothSystem
{
public:

	ParticleSystem cloParts;

	Model model;
	Model modelCopy;

	vector <glm::vec3> modelVertices;


	void initialize()
	{
		setModel("U:/Physics/FinalProject/Stuff/Models/clothModel.obj");

		vertexInitialize();

		GetUniqueModelVertices(); //@TODO
	}


	void Update(float timestep)
	{
		//Update the vertices

		vertexUpdate(timestep);

	}
	



	


#pragma region "Forces"


	void computeForces(Vertex p)
	{
		glm::vec3 compForce;

		glm::vec3 gravity(0.0f, 0.981f, 0.0f);

		compForce += gravity * p.Mass;

		p.Force = compForce;

		//std::cout << "Debug" << std::endl;
	}


#pragma endregion;



	void vertexUpdate(float timestep)
	{
		modelVertices.clear();

		for (int i = 0; i < model.meshes.size(); i++)
		{
			for (int j = 0; j < model.meshes[i].vertices.size(); j++)
			{
				Vertex &vertex = model.meshes[i].vertices[j];
				Vertex &unchanged = modelCopy.meshes[i].vertices[j];
				vertex.Position = unchanged.Position;
				//vertex.Normal = orientationMatrix * vertex.Normal;

				
				computeForces(vertex);
				vertex.Position += vertex.Velocity * timestep;
				vertex.Velocity += vertex.Force / vertex.Mass * timestep;

				//if (j < 81)
				//{
				modelVertices.push_back(vertex.Position);
				//}

				if (j == 0)
				{
					//std::cout << "Position: " << glm::to_string(vertex.Position) << std::endl;
				}
			}
		}
	}




	void vertexInitialize()
	{
		for (int i = 0; i < model.meshes.size(); i++)
		{
			for (int j = 0; j < model.meshes[i].vertices.size(); j++)
			{
				Vertex &vertex = model.meshes[i].vertices[j];
				Vertex &unchanged = modelCopy.meshes[i].vertices[j];
				vertex.Position = unchanged.Position;
				//vertex.Normal = orientationMatrix * vertex.Normal;

				vertex.Mass = 1.0f;
				vertex.Velocity = glm::vec3(0.0f);
				vertex.Force = glm::vec3(0.0f);
			}
		}
	}



	void GetUniqueModelVertices()
	{
		std::vector<std::vector<int>> myVec;

		std::sort(myVec.begin(), myVec.end());

		//while (modelVertices.size() < 169)
		//{
		myVec.erase(std::unique(myVec.begin(), myVec.end()), myVec.end());
		//}
	}
	


	//Creating
	void setModel(GLchar* path)
	{
		this->model = Model(path);
		this->modelCopy = Model(path);
	}
};
