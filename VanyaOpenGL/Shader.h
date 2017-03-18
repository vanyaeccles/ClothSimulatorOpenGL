// Adapted from the online tuturials by Joey de Vries found at 'http://learnopengl.com/'



//Links multiple shaders in a shader program object  which is activated when rendering objects.
//IMPOTANT - Rendering calls will use the active shader program shaders

//Provides exceptions when there are shader errors


#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <GL/glew.h>

class Shader
{
public:
	GLuint Program;  // Program ID
					 // Constructor reads and generates the shader
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath)
	{
		// Uses C++ filestreams to read content from the files intro several string objects
		// 1. Retrieve the vertex/fragment source code from filePath
		std::string vertexCode;
		std::string fragmentCode;
		std::ifstream vShaderFile;
		std::ifstream fShaderFile;
		// ensures ifstream objects can throw exceptions:
		vShaderFile.exceptions(std::ifstream::badbit);
		fShaderFile.exceptions(std::ifstream::badbit);
		try
		{
			// Open files
			vShaderFile.open(vertexPath);
			fShaderFile.open(fragmentPath);
			std::stringstream vShaderStream, fShaderStream;
			// Read file's buffer contents into streams
			vShaderStream << vShaderFile.rdbuf();
			fShaderStream << fShaderFile.rdbuf();
			// close file handlers
			vShaderFile.close();
			fShaderFile.close();
			// Convert stream into string
			vertexCode = vShaderStream.str();
			fragmentCode = fShaderStream.str();
		}
		catch (std::ifstream::failure e)
		{
			std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}
		const GLchar* vShaderCode = vertexCode.c_str();
		const GLchar * fShaderCode = fragmentCode.c_str();
		// 2. Compile shaders
		GLuint vertex, fragment;
		GLint success;
		GLchar infoLog[512];
		// VERTEX SHADER
		//create shader object (referenced by ID)
		vertex = glCreateShader(GL_VERTEX_SHADER);
		//attach shader source code to shader object and compile the shader
		glShaderSource(vertex, 1, &vShaderCode, NULL);
		glCompileShader(vertex);
		// Check for compile time errors, print them
		glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(vertex, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
		}
		// FRAGMENT SHADER 
		//create shader object (referenced by ID)
		fragment = glCreateShader(GL_FRAGMENT_SHADER);
		//attach shader source code to shader object and compile the shader
		glShaderSource(fragment, 1, &fShaderCode, NULL);
		glCompileShader(fragment);
		// Check for compile time errors, print them
		glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(fragment, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
		}
		// Shader Program
		// Shader program object is final linked version of multiple shaders combined
		// Have to link shaders (link outputs of each shader to the inputs of the next one)
		//creates program and returns ID to program object
		this->Program = glCreateProgram();
		//Attaches shaders
		glAttachShader(this->Program, vertex);
		glAttachShader(this->Program, fragment);
		//Links shaders
		glLinkProgram(this->Program);
		// Check if linking the shader program failed, retrieve and print error log
		glGetProgramiv(this->Program, GL_LINK_STATUS, &success);
		if (!success)
		{
			glGetProgramInfoLog(this->Program, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
		}
		// Delete the shaders once we've linked them into the program object, don't need them anymore
		glDeleteShader(vertex);
		glDeleteShader(fragment);

	}
	// function that activates and uses the current shader program object
	void Use()
	{
		glUseProgram(this->Program);
	}
};

#endif

