#include "MyRigidBody.h"
using namespace Simplex;
//Allocation
void MyRigidBody::Init(void)
{
	m_pMeshMngr = MeshManager::GetInstance();
	m_bVisibleBS = false;
	m_bVisibleOBB = true;
	m_bVisibleARBB = false;

	m_fRadius = 0.0f;

	m_v3ColorColliding = C_RED;
	m_v3ColorNotColliding = C_WHITE;

	m_v3Center = ZERO_V3;
	m_v3MinL = ZERO_V3;
	m_v3MaxL = ZERO_V3;

	m_v3MinG = ZERO_V3;
	m_v3MaxG = ZERO_V3;

	m_v3HalfWidth = ZERO_V3;
	m_v3ARBBSize = ZERO_V3;

	m_m4ToWorld = IDENTITY_M4;
}
void MyRigidBody::Swap(MyRigidBody& a_pOther)
{
	std::swap(m_pMeshMngr, a_pOther.m_pMeshMngr);
	std::swap(m_bVisibleBS, a_pOther.m_bVisibleBS);
	std::swap(m_bVisibleOBB, a_pOther.m_bVisibleOBB);
	std::swap(m_bVisibleARBB, a_pOther.m_bVisibleARBB);

	std::swap(m_fRadius, a_pOther.m_fRadius);

	std::swap(m_v3ColorColliding, a_pOther.m_v3ColorColliding);
	std::swap(m_v3ColorNotColliding, a_pOther.m_v3ColorNotColliding);

	std::swap(m_v3Center, a_pOther.m_v3Center);
	std::swap(m_v3MinL, a_pOther.m_v3MinL);
	std::swap(m_v3MaxL, a_pOther.m_v3MaxL);

	std::swap(m_v3MinG, a_pOther.m_v3MinG);
	std::swap(m_v3MaxG, a_pOther.m_v3MaxG);

	std::swap(m_v3HalfWidth, a_pOther.m_v3HalfWidth);
	std::swap(m_v3ARBBSize, a_pOther.m_v3ARBBSize);

	std::swap(m_m4ToWorld, a_pOther.m_m4ToWorld);

	std::swap(m_CollidingRBSet, a_pOther.m_CollidingRBSet);
}
void MyRigidBody::Release(void)
{
	m_pMeshMngr = nullptr;
	ClearCollidingList();
}
//Accessors
bool MyRigidBody::GetVisibleBS(void) { return m_bVisibleBS; }
void MyRigidBody::SetVisibleBS(bool a_bVisible) { m_bVisibleBS = a_bVisible; }
bool MyRigidBody::GetVisibleOBB(void) { return m_bVisibleOBB; }
void MyRigidBody::SetVisibleOBB(bool a_bVisible) { m_bVisibleOBB = a_bVisible; }
bool MyRigidBody::GetVisibleARBB(void) { return m_bVisibleARBB; }
void MyRigidBody::SetVisibleARBB(bool a_bVisible) { m_bVisibleARBB = a_bVisible; }
float MyRigidBody::GetRadius(void) { return m_fRadius; }
vector3 MyRigidBody::GetColorColliding(void) { return m_v3ColorColliding; }
vector3 MyRigidBody::GetColorNotColliding(void) { return m_v3ColorNotColliding; }
void MyRigidBody::SetColorColliding(vector3 a_v3Color) { m_v3ColorColliding = a_v3Color; }
void MyRigidBody::SetColorNotColliding(vector3 a_v3Color) { m_v3ColorNotColliding = a_v3Color; }
vector3 MyRigidBody::GetCenterLocal(void) { return m_v3Center; }
vector3 MyRigidBody::GetMinLocal(void) { return m_v3MinL; }
vector3 MyRigidBody::GetMaxLocal(void) { return m_v3MaxL; }
vector3 MyRigidBody::GetCenterGlobal(void){	return vector3(m_m4ToWorld * vector4(m_v3Center, 1.0f)); }
vector3 MyRigidBody::GetMinGlobal(void) { return m_v3MinG; }
vector3 MyRigidBody::GetMaxGlobal(void) { return m_v3MaxG; }
vector3 MyRigidBody::GetHalfWidth(void) { return m_v3HalfWidth; }
matrix4 MyRigidBody::GetModelMatrix(void) { return m_m4ToWorld; }
void MyRigidBody::SetModelMatrix(matrix4 a_m4ModelMatrix)
{
	//to save some calculations if the model matrix is the same there is nothing to do here
	if (a_m4ModelMatrix == m_m4ToWorld)
		return;

	//Assign the model matrix
	m_m4ToWorld = a_m4ModelMatrix;

	//Calculate the 8 corners of the cube
	vector3 v3Corner[8];
	//Back square
	v3Corner[0] = m_v3MinL;
	v3Corner[1] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z);
	v3Corner[2] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z);
	v3Corner[3] = vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z);

	//Front square
	v3Corner[4] = vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[5] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[6] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z);
	v3Corner[7] = m_v3MaxL;

	//Place them in world space
	for (uint uIndex = 0; uIndex < 8; ++uIndex)
	{
		v3Corner[uIndex] = vector3(m_m4ToWorld * vector4(v3Corner[uIndex], 1.0f));
	}

	//Identify the max and min as the first corner
	m_v3MaxG = m_v3MinG = v3Corner[0];

	//get the new max and min for the global box
	for (uint i = 1; i < 8; ++i)
	{
		if (m_v3MaxG.x < v3Corner[i].x) m_v3MaxG.x = v3Corner[i].x;
		else if (m_v3MinG.x > v3Corner[i].x) m_v3MinG.x = v3Corner[i].x;

		if (m_v3MaxG.y < v3Corner[i].y) m_v3MaxG.y = v3Corner[i].y;
		else if (m_v3MinG.y > v3Corner[i].y) m_v3MinG.y = v3Corner[i].y;

		if (m_v3MaxG.z < v3Corner[i].z) m_v3MaxG.z = v3Corner[i].z;
		else if (m_v3MinG.z > v3Corner[i].z) m_v3MinG.z = v3Corner[i].z;
	}

	//we calculate the distance between min and max vectors
	m_v3ARBBSize = m_v3MaxG - m_v3MinG;
}
//The big 3
MyRigidBody::MyRigidBody(std::vector<vector3> a_pointList)
{
	Init();
	//Count the points of the incoming list
	uint uVertexCount = a_pointList.size();

	//If there are none just return, we have no information to create the BS from
	if (uVertexCount == 0)
		return;

	//Max and min as the first vector of the list
	m_v3MaxL = m_v3MinL = a_pointList[0];

	//Get the max and min out of the list
	for (uint i = 1; i < uVertexCount; ++i)
	{
		if (m_v3MaxL.x < a_pointList[i].x) m_v3MaxL.x = a_pointList[i].x;
		else if (m_v3MinL.x > a_pointList[i].x) m_v3MinL.x = a_pointList[i].x;

		if (m_v3MaxL.y < a_pointList[i].y) m_v3MaxL.y = a_pointList[i].y;
		else if (m_v3MinL.y > a_pointList[i].y) m_v3MinL.y = a_pointList[i].y;

		if (m_v3MaxL.z < a_pointList[i].z) m_v3MaxL.z = a_pointList[i].z;
		else if (m_v3MinL.z > a_pointList[i].z) m_v3MinL.z = a_pointList[i].z;
	}

	//with model matrix being the identity, local and global are the same
	m_v3MinG = m_v3MinL;
	m_v3MaxG = m_v3MaxL;

	//with the max and the min we calculate the center
	m_v3Center = (m_v3MaxL + m_v3MinL) / 2.0f;

	//we calculate the distance between min and max vectors
	m_v3HalfWidth = (m_v3MaxL - m_v3MinL) / 2.0f;

	//Get the distance between the center and either the min or the max
	m_fRadius = glm::distance(m_v3Center, m_v3MinL);
}
MyRigidBody::MyRigidBody(MyRigidBody const& a_pOther)
{
	m_pMeshMngr = a_pOther.m_pMeshMngr;

	m_bVisibleBS = a_pOther.m_bVisibleBS;
	m_bVisibleOBB = a_pOther.m_bVisibleOBB;
	m_bVisibleARBB = a_pOther.m_bVisibleARBB;

	m_fRadius = a_pOther.m_fRadius;

	m_v3ColorColliding = a_pOther.m_v3ColorColliding;
	m_v3ColorNotColliding = a_pOther.m_v3ColorNotColliding;

	m_v3Center = a_pOther.m_v3Center;
	m_v3MinL = a_pOther.m_v3MinL;
	m_v3MaxL = a_pOther.m_v3MaxL;

	m_v3MinG = a_pOther.m_v3MinG;
	m_v3MaxG = a_pOther.m_v3MaxG;

	m_v3HalfWidth = a_pOther.m_v3HalfWidth;
	m_v3ARBBSize = a_pOther.m_v3ARBBSize;

	m_m4ToWorld = a_pOther.m_m4ToWorld;

	m_CollidingRBSet = a_pOther.m_CollidingRBSet;
}
MyRigidBody& MyRigidBody::operator=(MyRigidBody const& a_pOther)
{
	if (this != &a_pOther)
	{
		Release();
		Init();
		MyRigidBody temp(a_pOther);
		Swap(temp);
	}
	return *this;
}
MyRigidBody::~MyRigidBody() { Release(); };
//--- a_pOther Methods
void MyRigidBody::AddCollisionWith(MyRigidBody* a_pOther)
{
	/*
		check if the object is already in the colliding set, if
		the object is already there return with no changes
	*/
	auto element = m_CollidingRBSet.find(a_pOther);
	if (element != m_CollidingRBSet.end())
		return;
	// we couldn't find the object so add it
	m_CollidingRBSet.insert(a_pOther);
}
void MyRigidBody::RemoveCollisionWith(MyRigidBody* a_pOther)
{
	m_CollidingRBSet.erase(a_pOther);
}
void MyRigidBody::ClearCollidingList(void)
{
	m_CollidingRBSet.clear();
}
bool MyRigidBody::IsColliding(MyRigidBody* const a_pOther)
{
	//check if spheres are colliding as pre-test
	bool bColliding = (glm::distance(GetCenterGlobal(), a_pOther->GetCenterGlobal()) < m_fRadius + a_pOther->m_fRadius);
	
	//if they are colliding check the SAT
	if (bColliding)
	{
		if(SAT(a_pOther) != eSATResults::SAT_NONE)
			bColliding = false;// reset to false
	}

	if (bColliding) //they are colliding
	{
		this->AddCollisionWith(a_pOther);
		a_pOther->AddCollisionWith(this);
	}
	else //they are not colliding
	{
		this->RemoveCollisionWith(a_pOther);
		a_pOther->RemoveCollisionWith(this);
	}

	return bColliding;
}
void MyRigidBody::AddToRenderList(void)
{
	if (m_bVisibleBS)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
		else
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
	}
	if (m_bVisibleOBB)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorColliding);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorNotColliding);
	}
	if (m_bVisibleARBB)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
	}
}

uint MyRigidBody::SAT(MyRigidBody* const a_pOther)
{
	/*
	Your code goes here instead of this comment;

	For this method, if there is an axis that separates the two objects
	then the return will be different than 0; 1 for any separating axis
	is ok if you are not going for the extra credit, if you could not
	find a separating axis you need to return 0, there is an enum in
	Simplex that might help you [eSATResults] feel free to use it.
	(eSATResults::SAT_NONE has a value of 0)
	*/

	/*
	//Create the axis' in which you are testing for SAT
	vector4 xAxis(1, 0, 0, 0);
	vector4 yAxis(0, 1, 0, 0);
	vector4 zAxis(0, 0, 1, 0);
	vector3 axis[15];
	
	//populate first the basic directions using the points, use local coordinates first
	//Get X axis First square
	axis[0] = glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z));
	//Get Y axis First square
	axis[1] = glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z));
	//Get Z axis First square
	axis[2] = glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z));
	
	axis[0] = vector3(m_m4ToWorld * xAxis);
	axis[1] = vector3(m_m4ToWorld * yAxis);
	axis[2] = vector3(m_m4ToWorld * zAxis);

	axis[3] = vector3(a_pOther->GetModelMatrix() * xAxis);
	axis[4] = vector3(a_pOther->GetModelMatrix() * yAxis);
	axis[5] = vector3(a_pOther->GetModelMatrix() * zAxis);

	//Calculate same items for the Second Square in same order.
	vector3 otherMaxL = a_pOther->GetMaxLocal();
	vector3 otherMinL = a_pOther->GetMinLocal();
	//axis[3] = glm::cross(vector3(otherMaxL), vector3(otherMinL.x, otherMaxL.y, otherMaxL.z));
	//axis[4] = glm::cross(vector3(otherMaxL), vector3(otherMaxL.x, otherMinL.y, otherMaxL.z));
	//axis[5] = glm::cross(vector3(otherMaxL), vector3(otherMaxL.x, otherMaxL.y, otherMinL.z));

	//Counter to hold where to store the cross products
	int counter = 6;
	//For Loop calculates the cross products between each of the axis'
	for (int i = 0; i < 3; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			//axis[counter] = glm::cross(glm::abs(axis[i]), glm::abs(axis[j]));
			axis[counter] = glm::cross(axis[i], axis[j]);
			counter++;
		}
	}
	
	//Convert each axis from local to global
	for(int i = 0; i < 15; i++)
	{
		//turn into a unit vector
		axis[i] = glm::normalize(axis[i]);
		axis[i] = vector3(m_m4ToWorld * vector4(axis[i], 1.0f));
	} 

	//Get the corners for each shape
	vector3 shape1[4] = {
		vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z),
		vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z),
		vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z),
		vector3(m_v3MinL.x, m_v3MinL.y, m_v3MinL.z) };
	vector3 shape2[4] = {
		vector3(otherMaxL.x, otherMaxL.y, otherMaxL.z),
		vector3(otherMaxL.x, otherMaxL.y, otherMinL.z),
		vector3(otherMaxL.x, otherMinL.y, otherMinL.z),
		vector3(otherMinL.x, otherMinL.y, otherMinL.z) };

	//Convert each corner to global coordinates
	for(int i = 0; i < 4; i++)
	{
		shape1[i] = vector3(m_m4ToWorld * vector4(shape1[i], 0.0f));
	}
	for (int i = 0; i < 4; i++)
	{
		shape2[i] = vector3(m_m4ToWorld * vector4(shape2[i], 0.0f));
	}

	//Time to test each axis!!
	for(int i = 0; i < 15; i++)
	{
		int sh1MaxVal = glm::dot(axis[i], shape1[0]);
		int sh1MinVal = glm::dot(axis[i], shape1[0]);
		int sh2MaxVal = glm::dot(axis[i], shape2[0]);
		int sh2MinVal = glm::dot(axis[i], shape2[0]);
		for each(vector3 vec in shape1)
		{
			int test = glm::dot(axis[i], vec);
			if (test < sh1MinVal)
			{
				sh1MinVal = test;
			}
			if (test > sh1MaxVal)
			{
				sh1MaxVal = test;
			}
		}
		//Project Each point onto the axis and get min and max values
		for each(vector3 vec in shape2)
		{
			int test = glm::dot(axis[i], vec);
			if (test < sh2MinVal)
			{
				sh2MinVal = test;
			}
			if (test > sh2MaxVal)
			{
				sh2MaxVal = test;
			}
		}
		if (sh1MinVal > sh2MaxVal || sh2MinVal > sh1MaxVal)
			return eSATResults::SAT_NONE;
	} */

	
	vector4 xAxis(1, 0, 0, 0);
	vector4 yAxis(0, 1, 0, 0);
	vector4 zAxis(0, 0, 1, 0);

	vector3 sh1axis[3];
	sh1axis[0] = vector3(m_m4ToWorld * xAxis);
	sh1axis[1] = vector3(m_m4ToWorld * yAxis);
	sh1axis[2] = vector3(m_m4ToWorld * zAxis);

	vector3 sh2axis[3];
	sh2axis[0] = vector3(a_pOther->GetModelMatrix() * xAxis);
	sh2axis[1] = vector3(a_pOther->GetModelMatrix() * yAxis);
	sh2axis[2] = vector3(a_pOther->GetModelMatrix() * zAxis);
	

	////Counter to hold where to store the cross products
	//int counter = 6;
	////For Loop calculates the cross products between each of the axis'
	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 3; j < 6; j++)
	//	{
	//		//axis[counter] = glm::cross(glm::abs(axis[i]), glm::abs(axis[j]));
	//		axis[counter] = glm::cross(axis[i], axis[j]);
	//		counter++;
	//	}
	//}
	////Convert each axis from local to global
	//for(int i = 0; i < 15; i++)
	//{
	//	//turn into a unit vector
	//	axis[i] = glm::normalize(axis[i]);
	//	axis[i] = vector3(m_m4ToWorld * vector4(axis[i], 1.0f));
	//}

	//Get the centerpoints of each BB
	vector3 sh1Center = GetCenterGlobal();
	vector3 sh2Center = a_pOther->GetCenterGlobal();

	vector3 sh1Half = GetHalfWidth();
	vector3 sh2Half = a_pOther->GetHalfWidth();

	//vector3 sh1axis[3] = {
	//	glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z)),
	//	glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z)),
	//	glm::cross(vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MaxL.z), vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z)) };
	//vector3 sh2axis[3] = {
	//	glm::cross(vector3(otherMaxL), vector3(otherMinL.x, otherMaxL.y, otherMaxL.z)),
	//	glm::cross(vector3(otherMaxL), vector3(otherMaxL.x, otherMinL.y, otherMaxL.z)),
	//	glm::cross(vector3(otherMaxL), vector3(otherMaxL.x, otherMaxL.y, otherMinL.z)) };

	float ra, rb;
	//Creating rotation matricies
	glm::mat3x3 R, AbsR;

	//Computing rotation matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i][j] = glm::dot(sh1axis[i], sh2axis[j]);
		}
	}

	// Compute translation vector t
	vector3 t = sh2Center - sh1Center;

	// Bring translation into a’s coordinate frame
	t = vector3(glm::dot(t, sh1axis[0]), glm::dot(t, sh1axis[1]), glm::dot(t, sh1axis[2])); //vector3(m_m4ToWorld * vector4(t, 1.0f)); 

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			AbsR[i][j] = glm::abs(R[i][j]) + FLT_EPSILON;
		}
	}

	for (int i = 0; i < 3; i++) {
		ra = sh1Half[i];
		rb = sh2Half[0] * AbsR[i][0] + sh2Half[1] * AbsR[i][1] + sh2Half[2] * AbsR[i][2];
		if (glm::abs(t[i]) > ra + rb) 
			return 0;
	}

	for (int i = 0; i<3; i++) {
		ra = sh1Half[0] * AbsR[0][i] + sh1Half[1] * AbsR[1][i] + sh1Half[2] * AbsR[2][i];
		rb = sh2Half[i];
		if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb) 
			return 0;
	}

	ra = sh1Half[1] * AbsR[2][0] + sh1Half[2] * AbsR[1][0];
	rb = sh2Half[1] * AbsR[0][2] + sh2Half[2] * AbsR[0][1];
	if (glm::abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) 
		return 0;

	ra = sh1Half[1] * AbsR[2][1] + sh1Half[2] * AbsR[1][1];
	rb = sh2Half[0] * AbsR[0][2] + sh2Half[2] * AbsR[0][0];
	if (glm::abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) 
		return 0;

	ra = sh1Half[1] * AbsR[2][2] + sh1Half[2] * AbsR[1][2];
	rb = sh2Half[0] * AbsR[0][1] + sh2Half[1] * AbsR[0][0];
	if (glm::abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) 
		return 0;

	ra = sh1Half[0] * AbsR[2][0] + sh1Half[2] * AbsR[0][0];
	rb = sh2Half[1] * AbsR[1][2] + sh2Half[2] * AbsR[1][1];
	if (glm::abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) 
		return 0;

	ra = sh1Half[0] * AbsR[2][1] + sh1Half[2] * AbsR[0][1];
	rb = sh2Half[0] * AbsR[1][2] + sh2Half[2] * AbsR[1][0];
	if (glm::abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
		return 0;

	ra = sh1Half[0] * AbsR[2][2] + sh1Half[2] * AbsR[0][2];
	rb = sh2Half[0] * AbsR[1][1] + sh2Half[1] * AbsR[1][0];
	if (glm::abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) 
		return 0;

	ra = sh1Half[0] * AbsR[1][0] + sh1Half[1] * AbsR[0][0];
	rb = sh2Half[1] * AbsR[2][2] + sh2Half[2] * AbsR[2][1];
	if (glm::abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) 
		return 0;

	ra = sh1Half[0] * AbsR[1][1] + sh1Half[1] * AbsR[0][1];
	rb = sh2Half[0] * AbsR[2][2] + sh2Half[2] * AbsR[2][0];
	if (glm::abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) 
		return 0;

	ra = sh1Half[0] * AbsR[1][2] + sh1Half[1] * AbsR[0][2];
	rb = sh2Half[0] * AbsR[2][1] + sh2Half[1] * AbsR[2][0];
	if (glm::abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) 
		return 0;
		
	//there is no axis test that separates this two objects
	return 1;
}