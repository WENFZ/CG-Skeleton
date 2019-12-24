#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}
#include <cassert>
void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
	// create some joints here,and organize them into a dag
	// pointer to the root joint
	// Joint* m_rootJoint;
	// the list of joints.
	// std::vector< Joint* > m_joints;

	FILE* file = fopen(filename, "r");
	assert(file != NULL);
	while (true)
	{
		float dx, dy, dz;
		int p;
		int read= fscanf(file, "%f %f %f %d", &dx, &dy, &dz, &p);
		if (read == -1)
		{
			break;
		}
		Joint* joint = new Joint;
		joint->transform = Matrix4f::translation(dx, dy, dz);
		
		m_joints.push_back(joint);
		if (p == -1)
		{
			m_rootJoint = joint;
			joint->currentJointToWorldTransform = Matrix4f::identity();
		}
		else
		{
			m_joints[p]->children.push_back(joint);
			joint->currentJointToWorldTransform = m_joints[p]->currentJointToWorldTransform * Matrix4f::translation(dx, dy, dz);
		}
	}
	fclose(file);
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
	
	drawJointsHelper(m_rootJoint);
}

void SkeletalModel::drawJointsHelper(Joint* curJoint)
{
	// draw single joint
	m_matrixStack.push(curJoint->transform);
	glLoadMatrixf(m_matrixStack.top());
	glutSolidSphere(0.025f, 12, 12);
	// for each child
	for (auto childJoint : curJoint->children)
	{
		drawJointsHelper(childJoint);
	}
	m_matrixStack.pop();
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	drawSkeletonHelper(m_rootJoint,m_matrixStack.top());
}

void SkeletalModel::drawSkeletonHelper(Joint* curJoint,Matrix4f m)
{
	for (auto childJoint : curJoint->children)
	{
		// draw a box according to the pair curJoint,childJoint

		Vector3f zvec(0, 0, 1);
		Vector3f offset = childJoint->transform.getCol(3).xyz();
		auto len = offset.abs();
		float angle = acosf(Vector3f::dot(zvec, offset) / len);
		Vector3f axis = Vector3f::cross(zvec, offset).normalized();

		Matrix4f transform = Matrix4f::identity();
	
		transform = transform * Matrix4f::scaling(0.025f, 0.025f, len);	// 调整尺寸
		transform = transform * Matrix4f::translation(0, 0, 0.5f);		// 调整位置
		transform = Matrix4f::rotation(axis, angle) * transform;		// 旋转
		transform = m*curJoint->transform * transform;

		glLoadMatrixf(transform);
		glutSolidCube(1.0);
	}
	for (auto childJoint : curJoint->children)
	{
		drawSkeletonHelper(childJoint, m * curJoint->transform);
	}
}


void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	m_joints[jointIndex]->transform.setSubmatrix3x3(0, 0, (Matrix4f::rotateX(rX) * Matrix4f::rotateY(rY) * Matrix4f::rotateZ(rZ)).getSubmatrix3x3(0, 0));
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
	computeBindWorldToJointTransformsHelper(m_rootJoint,Matrix4f::identity());
}

void SkeletalModel::computeBindWorldToJointTransformsHelper(Joint* curJoint,Matrix4f parent)
{
	// update curJoint's bindWorldToJointTransform.
	auto joint2world = parent * curJoint->transform;
	curJoint->bindWorldToJointTransform = joint2world.inverse();
	for (auto child : curJoint->children)
	{
		// update its child's bindWorldToJointTransform
		computeBindWorldToJointTransformsHelper(child, joint2world);
	}
}





void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

	updateCurrentJointToWorldTransformsHelper(m_rootJoint, Matrix4f::identity());
}

void SkeletalModel::updateCurrentJointToWorldTransformsHelper(Joint* curJoint,Matrix4f& parent)
{
	curJoint->currentJointToWorldTransform = parent * curJoint->transform;
	for (auto child : curJoint->children)
	{
		updateCurrentJointToWorldTransformsHelper(child, curJoint->currentJointToWorldTransform);
	}
}


void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
	//return;
	int size = m_mesh.currentVertices.size();
	int jointNum = m_joints.size();
	for (int i = 0; i < size; i++)
	{

		Vector4f currentV;
		for (int j = 0; j < jointNum; j++)
		{
			// bindPoses[0] = bones[0].worldToLocalMatrix * transform.localToWorldMatrix;
			// https://forum.unity.com/threads/some-explanations-on-bindposes.86185/
			if (!(m_mesh.attachments[i][j] >-0.000001 && m_mesh.attachments[i][j] <0.000001))
			{
				//auto v = m_joints[j]->currentJointToWorldTransform * m_joints[j]->bindWorldToJointTransform * Vector4f(m_mesh.bindVertices[i], 1);
				currentV = currentV + m_mesh.attachments[i][j] * (m_joints[j]->currentJointToWorldTransform * m_joints[j]->bindWorldToJointTransform * Vector4f(m_mesh.bindVertices[i], 1));
			}
		}
		m_mesh.currentVertices[i] = currentV.xyz();
	}
}

