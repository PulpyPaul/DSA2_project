#include "MyEntityManager.h"
using namespace Simplex;
//  MyEntityManager
MyEntityManager* MyEntityManager::m_pInstance = nullptr;
void MyEntityManager::Init(void)
{
}
void MyEntityManager::Release(void)
{
	for (uint entity = 0; entity < m_uEntityCount; entity++) {
		MyEntity* pEntity = m_entityList[entity];
		SafeDelete(pEntity);
	}
	m_uEntityCount = 0;
	m_entityList.clear();
}
MyEntityManager* MyEntityManager::GetInstance()
{
	if (m_pInstance == nullptr)
	{
		m_pInstance = new MyEntityManager();
	}
	return m_pInstance;
}
void MyEntityManager::ReleaseInstance()
{
	if (m_pInstance != nullptr) {
		delete m_pInstance;
		m_pInstance = nullptr;
	}
}
int Simplex::MyEntityManager::GetEntityIndex(String a_sUniqueID)
{
	for (uint i = 0; i < m_uEntityCount; i++) {
		if (a_sUniqueID == m_entityList[i]->GetUniqueID()) {
			return i;
		}
	}
	return -1;
}
uint Simplex::MyEntityManager::GetEntityCount() {
	return m_uEntityCount;
}
//Accessors
Model* Simplex::MyEntityManager::GetModel(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return nullptr;
	}
	if (a_uIndex >= m_uEntityCount) {
		a_uIndex = m_uEntityCount - 1;
	}
	return m_entityList[a_uIndex]->GetModel();
}
Model* Simplex::MyEntityManager::GetModel(String a_sUniqueID)
{
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		return pTemp->GetModel();
	}
	return nullptr;
}
MyRigidBody* Simplex::MyEntityManager::GetRigidBody(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return nullptr;
	}
	if (a_uIndex >= m_uEntityCount) {
		a_uIndex = m_uEntityCount - 1;
	}
	return m_entityList[a_uIndex]->GetRigidBody();
}
MyRigidBody* Simplex::MyEntityManager::GetRigidBody(String a_sUniqueID)
{
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		return pTemp->GetRigidBody();
	}
	return nullptr;
}
matrix4 Simplex::MyEntityManager::GetModelMatrix(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return IDENTITY_M4;
	}
	if (a_uIndex >= m_uEntityCount) {
		a_uIndex = m_uEntityCount - 1;
	}
	return m_entityList[a_uIndex]->GetModelMatrix();
}
matrix4 Simplex::MyEntityManager::GetModelMatrix(String a_sUniqueID)
{
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		return pTemp->GetModelMatrix();
	}
	return IDENTITY_M4;
}
void Simplex::MyEntityManager::SetModelMatrix(matrix4 a_m4ToWorld, String a_sUniqueID)
{
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		pTemp->SetModelMatrix(a_m4ToWorld);
	}
}
void Simplex::MyEntityManager::SetHasCollisions(bool a_bHasCollisions, uint a_uIndex) {
	if (m_entityList.size() == 0) {
		return;
	}
	MyEntityManager::GetEntity(a_uIndex)->GetRigidBody()->SetHasCollisions(a_bHasCollisions);
}
void Simplex::MyEntityManager::SetHasCollisions(bool a_bHasCollisions, String a_sUniqueID) {
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		pTemp->GetRigidBody()->SetHasCollisions(a_bHasCollisions);
	}
}
void Simplex::MyEntityManager::SetModelMatrix(matrix4 a_m4ToWorld, uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return;
	}
	if (a_uIndex >= m_uEntityCount) {
		a_uIndex = m_uEntityCount - 1;
	}
	m_entityList[a_uIndex]->SetModelMatrix(a_m4ToWorld);
}
//The big 3
MyEntityManager::MyEntityManager(){Init();}
MyEntityManager::MyEntityManager(MyEntityManager const& other){ }
MyEntityManager& MyEntityManager::operator=(MyEntityManager const& other) { return *this; }
MyEntityManager::~MyEntityManager(){Release();};
// other methods
void Simplex::MyEntityManager::Update(void)
{
	for (uint i = 0; i < m_uEntityCount - 1; i++) {
		for (uint j = i + 1; j < m_uEntityCount; j++) {
			m_entityList[i]->IsColliding(m_entityList[j]);
		}

		// Updates movement of darts
		if (m_entityList[i + 1]->GetDirectionMovement() != vector3(0.0f, 0.0f, 0.0f)) {
			
			// Gets the movement direction and scales the speed
			vector3 direction = m_entityList[i + 1]->GetDirectionMovement();
			direction = glm::normalize(direction);
			direction *= 0.6f;
			
			// Updates the model matrix
			matrix4 modelMatrix = m_entityList[i + 1]->GetModelMatrix();
			m_entityList[i + 1]->SetModelMatrix(modelMatrix * glm::translate(direction));
		}
	}
}
void Simplex::MyEntityManager::AddEntity(String a_sFileName, String a_sUniqueID)
{
	MyEntity* pTemp = new MyEntity(a_sFileName, a_sUniqueID);
	if (pTemp->IsInitialized()) {
		m_entityList.push_back(pTemp);
		m_uEntityCount = m_entityList.size();
	}
}
void Simplex::MyEntityManager::RemoveEntity(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return;
	}
	if (a_uIndex >= m_uEntityCount) {
		a_uIndex = m_uEntityCount - 1;
	}
	if (a_uIndex != m_uEntityCount - 1) {
		std::swap(m_entityList[a_uIndex], m_entityList[m_uEntityCount - 1]);
	}
	MyEntity* pTemp = m_entityList[m_uEntityCount - 1];
	SafeDelete(pTemp);
	m_entityList.pop_back();
	m_uEntityCount -= 1;
}
void Simplex::MyEntityManager::RemoveEntity(String a_sUniqueID)
{
	uint uIndex = GetEntityIndex(a_sUniqueID);
	RemoveEntity(uIndex);
}
String Simplex::MyEntityManager::GetUniqueID(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return "";
	}
	if (a_uIndex >= m_entityList.size()) {
		a_uIndex = m_entityList.size() - 1;
	}
	return m_entityList[a_uIndex]->GetUniqueID();
}
MyEntity* Simplex::MyEntityManager::GetEntity(uint a_uIndex)
{
	if (m_entityList.size() == 0) {
		return nullptr;
	}
	if (a_uIndex >= m_entityList.size()) {
		a_uIndex = m_entityList.size() - 1;
	}
	return m_entityList[a_uIndex];
}
void Simplex::MyEntityManager::AddEntityToRenderList(uint a_uIndex, bool a_bRigidBody)
{
	if (a_uIndex >= m_uEntityCount) {
		for (a_uIndex = 0; a_uIndex < m_uEntityCount; a_uIndex++) {
			m_entityList[a_uIndex]->AddToRenderList(a_bRigidBody);
		}
	}
	else {
		m_entityList[a_uIndex]->AddToRenderList(a_bRigidBody);
	}
}
void Simplex::MyEntityManager::AddEntityToRenderList(String a_sUniqueID, bool a_bRigidBody)
{
	MyEntity* pTemp = MyEntity::GetEntity(a_sUniqueID);
	if (pTemp) {
		pTemp->AddToRenderList(a_bRigidBody);
	}
}