#include "MyRigidBody.h"
using namespace Simplex;
//Allocation
void MyRigidBody::Init(void) {
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
void MyRigidBody::Swap(MyRigidBody& a_pOther) {
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
void MyRigidBody::Release(void) {
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
bool MyRigidBody::GetHasCollisions(void) { return m_bHasCollisions; }
void MyRigidBody::SetHasCollisions(bool a_bHasCollisions) { m_bHasCollisions = a_bHasCollisions; }
float MyRigidBody::GetRadius(void) { return m_fRadius; }
vector3 MyRigidBody::GetColorColliding(void) { return m_v3ColorColliding; }
vector3 MyRigidBody::GetColorNotColliding(void) { return m_v3ColorNotColliding; }
void MyRigidBody::SetColorColliding(vector3 a_v3Color) { m_v3ColorColliding = a_v3Color; }
void MyRigidBody::SetColorNotColliding(vector3 a_v3Color) { m_v3ColorNotColliding = a_v3Color; }
vector3 MyRigidBody::GetCenterLocal(void) { return m_v3Center; }
vector3 MyRigidBody::GetMinLocal(void) { return m_v3MinL; }
vector3 MyRigidBody::GetMaxLocal(void) { return m_v3MaxL; }
vector3 MyRigidBody::GetCenterGlobal(void) { return vector3(m_m4ToWorld * vector4(m_v3Center, 1.0f)); }
vector3 MyRigidBody::GetMinGlobal(void) { return m_v3MinG; }
vector3 MyRigidBody::GetMaxGlobal(void) { return m_v3MaxG; }
vector3 MyRigidBody::GetHalfWidth(void) { return m_v3HalfWidth; }
matrix4 MyRigidBody::GetModelMatrix(void) { return m_m4ToWorld; }
void MyRigidBody::SetModelMatrix(matrix4 a_m4ModelMatrix) {
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
	for (uint uIndex = 0; uIndex < 8; ++uIndex) {
		v3Corner[uIndex] = vector3(m_m4ToWorld * vector4(v3Corner[uIndex], 1.0f));
	}

	//Identify the max and min as the first corner
	m_v3MaxG = m_v3MinG = v3Corner[0];

	//get the new max and min for the global box
	for (uint i = 1; i < 8; ++i) {
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
MyRigidBody::MyRigidBody(std::vector<vector3> a_pointList) {
	Init();
	//Count the points of the incoming list
	uint uVertexCount = a_pointList.size();

	//If there are none just return, we have no information to create the BS from
	if (uVertexCount == 0)
		return;

	//Max and min as the first vector of the list
	m_v3MaxL = m_v3MinL = a_pointList[0];

	//Get the max and min out of the list
	for (uint i = 1; i < uVertexCount; ++i) {
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
MyRigidBody::MyRigidBody(MyRigidBody const& a_pOther) {
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
MyRigidBody& MyRigidBody::operator=(MyRigidBody const& a_pOther) {
	if (this != &a_pOther) {
		Release();
		Init();
		MyRigidBody temp(a_pOther);
		Swap(temp);
	}
	return *this;
}
MyRigidBody::~MyRigidBody() { Release(); };
//--- a_pOther Methods
void MyRigidBody::AddCollisionWith(MyRigidBody* a_pOther) {
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
void MyRigidBody::RemoveCollisionWith(MyRigidBody* a_pOther) {
	m_CollidingRBSet.erase(a_pOther);
}
void MyRigidBody::ClearCollidingList(void) {
	m_CollidingRBSet.clear();
}
bool MyRigidBody::IsColliding(MyRigidBody* const a_pOther) {
	//If one or both objects does not have collisions, return false
	if (!GetHasCollisions() || !a_pOther->GetHasCollisions())
		return false;

	//check if spheres are colliding as pre-test
	bool bColliding = (glm::distance(GetCenterGlobal(), a_pOther->GetCenterGlobal()) < m_fRadius + a_pOther->m_fRadius);

	//if they are colliding check the SAT
	if (bColliding) {
		if (SAT(a_pOther) != eSATResults::SAT_NONE)
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
void MyRigidBody::AddToRenderList(void) {
	if (m_bVisibleBS) {
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
		else
			m_pMeshMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
	}
	if (m_bVisibleOBB) {
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorColliding);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorNotColliding);
	}
	if (m_bVisibleARBB) {
		if (m_CollidingRBSet.size() > 0)
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
		else
			m_pMeshMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
	}
}

uint MyRigidBody::SAT(MyRigidBody* const a_pOther) {
	/*
	SAT by Daniel Timko

	Check all 15 axes
	Pulled a lot from the book, but I understand that if a plane can be drawn between
	two boxes, those boxes aren't colliding

	Basically, get all 15 axes (the local of both A and B, and every possible cross product) and
	project b.c - a.c onto every one.  Then, project the half widths.  If the sum of the half widths
	projection magnitudes is less than the distance projection magnitude, that axis separates the objects

	That is simplified, but yeah.  General explanation.  I'm better at math than I am at words
	*/

	struct OBB {
		vector3 c;
		vector3 u[3];
		vector3 e;
	};

	//OBB structs for both rigid bodies
	OBB a, b;

	//Get OBB centers
	a.c = GetCenterGlobal();
	b.c = a_pOther->GetCenterGlobal();

	//Get OBB half widths
	a.e = GetHalfWidth();
	b.e = a_pOther->GetHalfWidth();

	//Get OBB local axes in world space
	//Local axes of A (a.u)
	a.u[0] = glm::normalize(vector3(m_m4ToWorld * vector4(AXIS_X, 0.0f))); //Local A x axis
	a.u[1] = glm::normalize(vector3(m_m4ToWorld * vector4(AXIS_Y, 0.0f))); //Local A y axis
	a.u[2] = glm::normalize(vector3(m_m4ToWorld * vector4(AXIS_Z, 0.0f))); //Local A z axis
	//Local axes of B (b.u)
	b.u[0] = glm::normalize(vector3(a_pOther->m_m4ToWorld * vector4(AXIS_X, 0.0f))); //Local A x axis
	b.u[1] = glm::normalize(vector3(a_pOther->m_m4ToWorld * vector4(AXIS_Y, 0.0f))); //Local A y axis
	b.u[2] = glm::normalize(vector3(a_pOther->m_m4ToWorld * vector4(AXIS_Z, 0.0f))); //Local A z axis

	//Vars used for SAT
	float ra, rb;
	matrix3 R, AbsR;

	//Compute rotation matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			//Every axis from b projected onto every axis from a.  This rotation matrix will transform things from b's
			//local space to a's local space
			R[i][j] = glm::dot(a.u[i], b.u[j]);
		}
	}

	//Translation between the two OBB
	vector3 t = b.c - a.c;
	//Transform the translation into a's local coordinates by projecting each component onto a's unit axes
	t = vector3(glm::dot(t, a.u[0]), glm::dot(t, a.u[1]), glm::dot(t, a.u[2]));

	//Absolute value every dot product in the rotation matrix and make sure nothing is equal to zero to avoid null vectors
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			//Make sure that nothing is zero using epsilon
			//Also remove the sign from the dot product since we are only interested in lengths
			AbsR[i][j] = glm::abs(R[i][j]) + FLT_EPSILON;
		}
	}

	//Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++) {
		//Get half width along this axis for object A
		ra = a.e[i];
		//Get b's half width projected onto this axis
		//rb = b.half[x] * proj(b.u[x], a.u[i]) + b.half[y] * proj(b.u[y], a.u[i]) + b.half[z] * proj(b.u[z], a.u[i])
		rb = b.e[0] * AbsR[i][0] + b.e[1] * AbsR[i][1] + b.e[2] * AbsR[i][2];
		//If the length of the distance between the objects on this axis is longer than both half widths
		if (glm::abs(t[i]) > ra + rb) {
			//Seperation axis found!  Which one?  0 = a.u[x], 1 = a.u[y], 2 = a.u[z]
			if (i == 0) {
				//Return the axis of separation
				return eSATResults::SAT_AX;
			}
			else if (i == 1) {
				//Return the axis of separation
				return eSATResults::SAT_AY;
			}
			else if (i == 2) {
				//Return the axis of separation
				return eSATResults::SAT_AZ;
			}
		}
	}

	//Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++) {
		//Get a's half width projected onto this axis
		//rb = a.half[x] * proj(b.u[i], a.u[x]) + a.half[y] * proj(b.u[i], a.u[y]) + a.half[z] * proj(b.u[i], a.u[z])
		ra = a.e[0] * AbsR[0][i] + a.e[1] * AbsR[1][i] + a.e[2] * AbsR[2][i];
		//Get half width along this axis for object B
		rb = b.e[i];
		//If the length of the distance between the objects (projected onto this axis, since it is in the local space of A) 
		//is longer than both of the half widths
		if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb) {
			//Seperation axis found!  Which one? 0 = b.u[x], 1 = b.u[y], 2 = b.u[z]
			if (i == 0) {
				//Return the axis of separation
				return eSATResults::SAT_BX;
			}
			else if (i == 1) {
				//Return the axis of separation
				return eSATResults::SAT_BY;
			}
			else if (i == 2) {
				//Return the axis of separation
				return eSATResults::SAT_BZ;
			}
		}
	}

	//For every local A axis (aa)
	for (int aa = 0; aa < 3; aa++) {
		//For every local B axis (ba)
		for (int ba = 0; ba < 3; ba++) {
			//Test the cross product  aa X ba
			//Get the opposite axes for A
			int ao1 = (aa + 1) % 3;
			int ao2 = (aa + 2) % 3;
			//Get the opposite axes for B
			int bo1 = (ba + 1) % 3;
			int bo2 = (ba + 2) % 3;

			//Take A's halfWidth values not on this A axis
			//multiply each by the opposite A axis after its been projected onto this B axis
			// a.half[ao1] * proj(a.u[ao2], b.u[ba]) + a.half[ao2] * proj(a.u[ao1], b.u[ba])
			ra = a.e[ao1] * AbsR[ao2][ba] + a.e[ao2] * AbsR[ao1][ba];
			//Take b's halfWidth values not on this B axis
			//multiply each by the opposite B axis after its been projected onto this A axis
			// b.half[bo1] * proj(b.u[bo2], a.u[aa]) + b.half[bo2] * proj(b.u[bo1], a.u[aa])
			rb = b.e[bo1] * AbsR[aa][bo2] + b.e[bo2] * AbsR[aa][bo1];

			//Get t's magnitude in terms of this axis
			//Project t's components not on this A axis
			//onto the dot of the other A axes and this B axis
			float tProj = glm::abs(t[ao2] * R[ao1][ba] - t[ao1] * R[ao2][ba]);

			//If the length of the distance between the objects (projected onto this axis) 
			//is longer than both of the half widths, this is the separation axis!
			if (tProj > ra + rb) {
				//Return the axis of separation
				//Skip the 6 enum values for the object's local coordinates and then add the current axes together
				//For instance, AX x BX would be 6 since those would both be 0
				//              6 + 3*aa + ba = axis enum
				//AXxBX = 6  :: 6 + 3* 0 +  0 = 6
				//AXxBY = 7  :: 6 + 3* 0 +  1 = 7
				//AXxBZ = 8  :: 6 + 3* 0 +  2 = 8
				//AYxBX = 9  :: 6 + 3* 1 +  0 = 9
				//AYxBY = 10 :: 6 + 3* 1 +  1 = 10
				//AYxBZ = 11 :: 6 + 3* 1 +  2 = 11
				//AZxBX = 12 :: 6 + 3* 2 +  0 = 12
				//AZxBY = 13 :: 6 + 3* 2 +  1 = 13
				//AZxBZ = 14 :: 6 + 3* 2 +  2 = 14
				return (6 + 3 * aa + ba);
			}
		}
	}
	//there is no axis test that separates this two objects
	return eSATResults::SAT_NONE;
}
