#include <iostream>    // std::cout  
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <string>         // std::string
#include <cstddef>         // std::size_t
#include "objloader.h"


void OBJLoader:: computeNormals(std::vector<glm::vec3> const &vertices, std::vector<int> const &indices, std::vector<glm::vec3> &normals){
		
	    normals.resize(vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));
		
		// Compute per-vertex normals here!

		int i;
		for (i = 0; i < indices.size(); i += 3)
		{
			glm::vec3 p1 = vertices[indices[i]];
			glm::vec3 p2 = vertices[indices[i + 1]];
			glm::vec3 p3 = vertices[indices[i + 2]];

			glm::vec3 normal = glm::normalize(glm::cross((p2 - p1), (p3 - p1)));
			
			normals[indices[i]] += normal;
			normals[indices[i + 1]] += normal;
			normals[indices[i + 2]] += normal;
		
		}
		

		for (i = 0; i < normals.size(); i++)
			normals[i] = glm::normalize(normals[i]);
}


OBJLoader::OBJLoader() :
mVertices(0),
mNormals(0),
mColors(0),
vIndices(0)
{
	std::cout << "Called OBJFileReader constructor" << std::endl;
}

OBJLoader::~OBJLoader()
{
	std::cout << "Called OBJFileReader destructor" << std::endl;
}

bool OBJLoader::load(const char *filename)
{
	// Open OBJ file
	std::ifstream OBJFile(filename);
	if (!OBJFile.is_open()) {
		std::cerr << "Could not open " << filename << std::endl;
		return false;
	}
	
	// Extract vertices and indices
	std::string line;
	glm::vec3 vertex;
	glm::vec3 normal;
	glm::vec2 uv;
	double fric;
	while (!OBJFile.eof()) {
		std::getline(OBJFile, line);
		if ((line.find('#') == -1) && (line.find('m') == -1)){
			if (line.find('v') != -1) {

				if ((line.find('t') == -1) && (line.find('n') == -1)){
					std::istringstream vertexLine(line.substr(2));
					vertexLine >> vertex.x;
					vertexLine >> vertex.y;
					vertexLine >> vertex.z;
				    mVertices.push_back(vertex);
					
					//printf("Vertex is: %f %f %f\n", vertex.x, vertex.y, vertex.z);
					vertex = glm::normalize(vertex);//Normalizing the vertices to get the values between 0-1
					vertex.x = abs(vertex.x);
					vertex.y = abs(vertex.y);
					vertex.z = abs(vertex.z);
					//color is assigned here
					mColors.push_back(vertex);
					//assign friction based on the color of the object
					if(vertex.x > vertex.y && vertex.x > vertex.z) fric = 0.9;
					if(vertex.y > vertex.x && vertex.y > vertex.z) fric = 0.4;
					if(vertex.z > vertex.x && vertex.z > vertex.y) fric=0.1;
					mFriction.push_back(fric);

					
				}
				else if(line.find('n') != -1) {
					std::istringstream textureLine(line.substr(3));
					textureLine >> normal.x;
					textureLine >> normal.y;
					textureLine >> normal.z;
				    mNormals.push_back(normal);
				}
			}

			else if (line.find("f ") != -1) {
				std::istringstream faceLine(line);
				std::string val1;
				faceLine >> val1;
				int val, val2;
				int tIndices[3];
				char slash;
				for (int n = 0; n < 3; n++){
					 faceLine >> val;
					 vIndices.push_back(val- 1);
					 nIndices.push_back(val- 1);
					 tIndices[n] = (val- 1);
				}

				  tris.push_back(Triangle(tIndices[0], tIndices[1], tIndices[2]));  


			}
	    }
	}

	// Close OBJ file
	OBJFile.close();

	
	// Compute normals
	computeNormals(mVertices, vIndices, mNormals);

	unitize(mVertices);
	Generate(); //generate the map of vertices and connections.
	
	return true;
}

void OBJLoader:: unitize(std::vector<glm::vec3> &vertices) {
	float min_x = vertices[0].x, max_x = vertices[0].x,
	min_y = vertices[0].y, max_y = vertices[0].y,
	min_z = vertices[0].z, max_z = vertices[0].z;

	

	for(int i = 1; i < vertices.size(); i++)
	{
		if(vertices[i].x < min_x)
			min_x = vertices[i].x;
		if(vertices[i].x > max_x)
			max_x = vertices[i].x;
		if(vertices[i].y < min_y)
			min_y = vertices[i].y;
		if(vertices[i].y > max_y)
			max_y = vertices[i].y;
		if(vertices[i].z < min_z)
			min_z = vertices[i].z;
		if(vertices[i].z > max_z)
			max_z = vertices[i].z;
	}

	float center_x = (max_x + min_x) / 2,
	center_y = (max_y + min_y) / 2,
	center_z = (max_z + min_z) / 2;

	for(int i = 0; i < vertices.size(); i++)
	{
		vertices[i].x -= center_x;
		vertices[i].y -= center_y;
		vertices[i].z -= center_z;
	}

	float width = glm::abs(max_x - min_x),
	height = glm::abs(max_y - min_y),
	depth = glm::abs(max_z - min_z);

	//printf("%f, %f, %f\n", width, height, depth);

	float scale = 2 / glm::max(glm::max(width, height), depth);

	for(int i = 0; i < vertices.size(); i++)
	{
		vertices[i] *= scale;
		//printf("%f, %f, %f\n", vertices[i].x, vertices[i].y, vertices[i].z);
	}
}


std::vector<glm::vec3> const &OBJLoader::getVertices() const
{
	
	
	return mVertices;
}

std::vector<glm::vec3> const &OBJLoader::getNormals() const
{
	return mNormals;
}

std::vector<glm::vec3> const &OBJLoader::getColors() const
{
        return mColors;
}

std::vector<int> const &OBJLoader::getVertexIndices() const
{
	return vIndices;
}

std::vector<Triangle> const &OBJLoader::getTriangles() const
{
	return tris;
}

std::vector<double> const &OBJLoader::getFriction() const
{
	return mFriction;
}


/******************************************************************************************************************/
void OBJLoader::drawColorObj(){

	vec3 vertex_one, vertex_two, vertex_three;
	vec3 norm_one, norm_two, norm_three;
	vec3 color_one, color_two, color_three;
	computeNormals(mVertices, vIndices, mNormals);
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();
	//glEnable(GL_COLOR_MATERIAL);
	glBegin(GL_TRIANGLES);

	for (int i = 0; i < tris.size(); i++){
		
	     Triangle &tri = tris[i];
		 
		 vertex_one = mVertices[tri.vert[0]];
		 vertex_two = mVertices[tri.vert[1]];
		 vertex_three = mVertices[tri.vert[2]];

		 norm_one = mNormals[tri.vert[0]];
		 norm_two = mNormals[tri.vert[1]];
		 norm_three = mNormals[tri.vert[2]];

		 color_one = mColors[tri.vert[0]];
		 color_two = mColors[tri.vert[1]];
		 color_three = mColors[tri.vert[2]];


		 glNormal3f(norm_one.x, norm_one.y, norm_one.z);
		 glColor3f(color_one.x, color_one.y, color_one.z);
		 glVertex3f(vertex_one.x, vertex_one.y, vertex_one.z);

		// glEnable(GL_COLOR_MATERIAL);

		 glNormal3f(norm_two.x, norm_two.y,norm_two.z );
		 glColor3f(color_two.x, color_two.y, color_two.z);
		 glVertex3f(vertex_two.x, vertex_two.y, vertex_two.z );

		// glEnable(GL_COLOR_MATERIAL);

		 glNormal3f(norm_three.x, norm_three.y, norm_three.z);
		 glColor3f(color_three.x, color_three.y, color_three.z);
		 glVertex3f(vertex_three.x, vertex_three.y, vertex_three.z);
	}

	glEnd();

	glPopMatrix();
	glPopAttrib();

}

void OBJLoader::Generate(){
	for(int i = 0; i<tris.size(); i++){
		Triangle &tri = tris[i];
		Link(tri.vert[0], tri.vert[1]);
		Link(tri.vert[1], tri.vert[2]);
		Link(tri.vert[2], tri.vert[0]);
	}
}

void OBJLoader::Link(int a, int b){
	
	(net)[a].insert(b);
}

void OBJLoader::deformSurface(int nearestVertex, vec3 newProxyPosition, set<int>nearestNeighbour){
	vec3 myNormal; 

	myNormal.x=newProxyPosition.x-mVertices[nearestVertex].x;
	myNormal.y=newProxyPosition.y-mVertices[nearestVertex].y;
	myNormal.z=newProxyPosition.z-mVertices[nearestVertex].z;

	mVertices[nearestVertex].x+=myNormal.x;
	mVertices[nearestVertex].y+=myNormal.y;
	mVertices[nearestVertex].z+=myNormal.z;

	for(set<int>::iterator cur_b= nearestNeighbour.begin(); cur_b!= nearestNeighbour.end(); cur_b++){
		mVertices[*cur_b].x += myNormal.x/2;
		mVertices[*cur_b].y += myNormal.y/2;
		mVertices[*cur_b].z += myNormal.z/2;
	}

}
/******************************************************************************************************************/