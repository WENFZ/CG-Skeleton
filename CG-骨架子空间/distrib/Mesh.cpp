#include "Mesh.h"
#include <cassert>
using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.

	FILE* file = fopen(filename, "r");
	assert(file != NULL);
	while (true)
	{
		char type;
		if (-1 == fscanf(file, "%c", &type))
		{
			break;
		}
		if (type == 'v')
		{
			float f1, f2, f3;
			fscanf(file, "%f %f %f", &f1, &f2, &f3);
			bindVertices.push_back(Vector3f(f1, f2, f3));
		}
		if (type == 'f')
		{
			int i1, i2, i3;
			fscanf(file, "%d %d %d", &i1, &i2, &i3);
			faces.push_back(Tuple3u(i1, i2, i3));
		}
	}
	fclose(file);
	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

	// list of vertices from the OBJ file
	// in the "bind pose"
	//std::vector< Vector3f > bindVertices; // 源位置

	// each face has 3 indices
	// referencing 3 vertices
	//std::vector< Tuple3u > faces; // 小块皮肤

	// current vertex positions after animation
	//std::vector< Vector3f > currentVertices; // 变换后位置

	// list of vertex to joint attachments
	// each element of attachments is a vector< float > containing
	// one attachment weight per joint
	//std::vector< std::vector< float > > attachments; // 权重
	glBegin(GL_TRIANGLES);
	for (auto piece : faces)
	{
		float x1, y1, z1;
		float x2, y2, z2;
		float x3, y3, z3;

		x1 = currentVertices[piece[0] - 1][0];
		y1 = currentVertices[piece[0] - 1][1];
		z1 = currentVertices[piece[0] - 1][2];

		x2 = currentVertices[piece[1] - 1][0];
		y2 = currentVertices[piece[1] - 1][1];
		z2 = currentVertices[piece[1] - 1][2];

		x3 = currentVertices[piece[2] - 1][0];
		y3 = currentVertices[piece[2] - 1][1];
		z3 = currentVertices[piece[2] - 1][2];
		Vector3f n = Vector3f::cross(Vector3f(x2 - x1, y2 - y1, z2 - z1), Vector3f(x3 - x1, y3 - y1, z3 - z1)).normalized();
		
		// draw a piece of skin
		
		glNormal3f(n.x(), n.y(), n.z());
		glVertex3f(x1, y1, z1);
		glVertex3f(x2, y2, z2);
		glVertex3f(x3, y3, z3);
	}
	glEnd();


}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	FILE* file = fopen(filename, "r");
	assert(file != NULL);
	float att;
	while (true)
	{
		int read = -1;
		vector<float> attachment;
		attachment.push_back(0);
		for (int i = 1; i < numJoints; i++)
		{
			read = fscanf(file, "%f", &att);
			attachment.push_back(att);
		}
		if (read == -1)
		{
			break;
		}
		attachments.push_back(attachment);
	}
	fclose(file);
}
