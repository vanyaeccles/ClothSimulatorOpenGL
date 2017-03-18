/*

// This is just for showing how it works!


#include "scene_object.h"

scene_object::scene_object(){
}

scene_object::scene_object(char* mesh_name, char* tex_name){
	GLfloat* vp = NULL; // array of vertex points
	GLfloat* vn = NULL; // array of vertex normals
	GLfloat* vt = NULL; // array of texture coordinates

	assert (load_obj_file (mesh_name, vp, vt, vn,  point_count));

	fill_vao_vbo(point_count, vp, vn, vt, vao);

	assert (load_texture (tex_name, &tex_handle));
}


scene_object::scene_object(int in_point_count, GLfloat* points, GLfloat* normals, GLfloat* tex_coords, char* tex_name){
	point_count = in_point_count;

	fill_vao_vbo(point_count, points, normals, tex_coords, vao);

	assert (load_texture (tex_name, &tex_handle));
}


void scene_object::operator=(scene_object &source){
	
	point_count = source.point_count;
	vao = source.vao;
	model = source.model;
	proj = source.proj;
	view = source.view;
	M_loc = source.M_loc;
	P_loc = source.P_loc;
	V_loc = source.V_loc;
	tex_handle = source.tex_handle;	
}

void scene_object::set_mat_locs(int in_M_loc, int in_P_loc, int in_V_loc){
	M_loc = in_M_loc;
	P_loc = in_P_loc;
	V_loc = in_V_loc;
}


void scene_object::update_mats(bool M, bool P, bool V){
	if(M)
		glUniformMatrix4fv (M_loc, 1, GL_FALSE, model.m);
	if(P)
		glUniformMatrix4fv (P_loc, 1, GL_FALSE, proj.m);
	if(V)
		glUniformMatrix4fv (V_loc, 1, GL_FALSE, view.m);
}

void scene_object::draw(){
	glBindVertexArray (vao);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex_handle);

	glDrawArrays (GL_TRIANGLES, 0, point_count);
}

// Load a texture
bool scene_object::load_texture (const char* file_name, GLuint* tex) {
	int x, y, n;
	int force_channels = 4;
	unsigned char* image_data = stbi_load (file_name, &x, &y, &n, force_channels);

	if (!image_data) {
		fprintf (stderr, "ERROR: could not load %s\n", file_name);
		return false;
	}

	// NPOT check
	if ((x & (x - 1)) != 0 || (y & (y - 1)) != 0) {
		fprintf (
			stderr, "WARNING: texture %s is not power-of-2 dimensions\n", file_name
		);
	}

	int width_in_bytes = x * 4;
	unsigned char *top = NULL;
	unsigned char *bottom = NULL;
	unsigned char temp = 0;
	int half_height = y / 2;

	// Flips image to make it right way up
	for (int row = 0; row < half_height; row++) {
		top = image_data + row * width_in_bytes;
		bottom = image_data + (y - row - 1) * width_in_bytes;

		for (int col = 0; col < width_in_bytes; col++) {
			temp = *top;
			*top = *bottom;
			*bottom = temp;
			top++;
			bottom++;
		}
	}

	glGenTextures (1, tex);
	glBindTexture (GL_TEXTURE_2D, *tex);
	glTexImage2D (
		GL_TEXTURE_2D,
		0,
		GL_RGBA,
		x,
		y,
		0,
		GL_RGBA,
		GL_UNSIGNED_BYTE,
		image_data
	);

	glGenerateMipmap (GL_TEXTURE_2D);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	GLfloat max_aniso = 0.0f;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_aniso);
	// set the maximum!
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_aniso);
	return true;
}


//
// This shows how to use it!!
//
void scene_object::fill_vao_vbo(int point_count, GLfloat* vp, GLfloat* vn, GLfloat* vt, GLuint& vao){

	GLuint points_vbo;

	// ask GL to generate a VBO, and give us a handle to refer to it with
	glGenBuffers (1, &points_vbo);
	// "bind" the VBO. all future VBO operations affect this VBO until we bind a different one
	glBindBuffer (GL_ARRAY_BUFFER, points_vbo);
	// copy our points from RAM into our VBO on graphics hardware
	glBufferData (GL_ARRAY_BUFFER, 3*sizeof(float)*point_count, vp, GL_STATIC_DRAW);
 
	GLuint normals_vbo;
	glGenBuffers (1, &normals_vbo);
	glBindBuffer (GL_ARRAY_BUFFER, normals_vbo);
	glBufferData (GL_ARRAY_BUFFER, 3*sizeof(float)*point_count, vn, GL_STATIC_DRAW);

	GLuint texcoords_vbo;
	glGenBuffers (1, &texcoords_vbo);
	glBindBuffer (GL_ARRAY_BUFFER, texcoords_vbo);
	glBufferData (GL_ARRAY_BUFFER, 2*sizeof(float)*point_count, vt, GL_STATIC_DRAW);
     
	//ask GL to generate a VAO, and give us a handle to refer to it with
	glGenVertexArrays (1, &vao);
	// "bind" the VAO
	glBindVertexArray (vao);
	// tell the VAO that it should enable the first shader input variable (number 0)
	glEnableVertexAttribArray (0);
	// tell the VAO that it should enable the second shader input variable (number 1)
	glEnableVertexAttribArray (1);
	// tell the VAO that it should enable the third shader input variable (number 2)
	glEnableVertexAttribArray (2);
	// bind our VBO. it's already bound at the moment but this is good practice in case of code changes later
	glBindBuffer (GL_ARRAY_BUFFER, points_vbo);
	// tell the VAO that it should use our VBO to find its first input variable (number 0).
	//that input variable should use 3 consecutive floats as a variable (it will be a vec3)
	//GL_FALSE means we dont need to normalise it (useful for some clever data tricks in advanced stuff)
	//each variable comes one-after-the-other in the data, so there's no need for stride (0), and starts at
	//the beginning of the buffer, so no offset either (NULL) 
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
 
	glBindBuffer (GL_ARRAY_BUFFER, normals_vbo);
	glVertexAttribPointer (1, 3, GL_FLOAT, GL_TRUE, 0, NULL);

		
	glBindBuffer (GL_ARRAY_BUFFER, texcoords_vbo);
	glVertexAttribPointer (2, 2, GL_FLOAT, GL_FALSE, 0, NULL);
}

*/