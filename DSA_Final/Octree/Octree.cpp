#include "Octree.h"
#include "../Entities/MyEntityManager.h"
#include "../Physics/MyRigidBody.h"
#include <list>
#include <vector>
#include <deque>

using namespace Simplex;

//Init static vars
uint Octree::s_uOctantCount = 0;
uint Octree::s_uNextOctantID = 0;
uint Octree::s_uMaxLevel = 4;
uint Octree::s_uIdealEntityCount = 1;
int Octree::s_iDisplayOctant = -1;
Octree* Octree::s_pRoot = nullptr;
bool Octree::s_bTreeReady = false;
bool Octree::s_bTreeBuilt = false;
bool Octree::s_bShouldRebuild = false;
std::deque<uint> Octree::s_qToInsert;

void Octree::Release(void) {
	//Loop and delete all children.  This recurses
	if (!IsLeaf()) {
		for (byte a = 0; a < 8; a++) { KillChild(a); }
	}
	//Loop through entities and remove this id from their octant list
	for (uint e = 0; e < m_lEntities.size(); e++) {
		MyEntity* temp = m_pEntityMngr->GetEntity(m_lEntities[e]);
		if (temp != nullptr) {
			temp->RemoveOctant(m_uID);
		}
	}
	//If this is the root, set the root to nullptr
	if (Octree::s_pRoot == this) {
		Octree::s_pRoot = nullptr;
		Octree::s_bTreeReady = false;
		Octree::s_bTreeBuilt = false;
	}
	//Decriment the number of octants
	Octree::s_uOctantCount--;
}

void Octree::Init(void) {
	//If there is no root yet, this is the root
	if (Octree::s_pRoot == nullptr) {
		Octree::s_pRoot = this;
		Octree::s_uOctantCount = 0;
		Octree::s_uNextOctantID = 0;
	}

	m_pMeshMngr = MeshManager::GetInstance();
	m_pEntityMngr = MyEntityManager::GetInstance();

	m_uID = Octree::s_uNextOctantID++;
	m_uLevel = 0;

	Octree::s_uOctantCount++;

	m_bbRegion = BoundingBox();

	m_bActiveNodes = 0;

	m_pParent = nullptr;

	m_iCurLife = -1;
	m_iMaxLifespan = 4;
}

Octree::Octree() {
	Init();
}

Octree::Octree(BoundingBox a_bbRegion) {
	Init();
	m_bbRegion = a_bbRegion;
}

Octree::Octree(BoundingBox a_bbRegion, std::vector<uint> const &a_lObjectList) {
	Init();
	m_bbRegion = a_bbRegion;
	for (uint i = 0; i < a_lObjectList.size(); i++) {
		m_lEntities.push_back(a_lObjectList[i]);
	}
}

Octree::Octree(Octree const & other) {
	m_pMeshMngr = MeshManager::GetInstance();
	m_pEntityMngr = MyEntityManager::GetInstance();
	m_bbRegion = BoundingBox(other.m_bbRegion);
	m_lEntities = std::vector<uint>(other.m_lEntities.begin(), other.m_lEntities.end());
	m_uID = other.m_uID;
	m_uLevel = other.m_uLevel;
	m_bActiveNodes = other.m_bActiveNodes;
	m_pParent = other.m_pParent;
	for (byte c = 0; c < 8; c++) { m_pChildren[c] = other.m_pChildren[c]; }
	m_iCurLife = other.m_iCurLife;
	m_iMaxLifespan = other.m_iMaxLifespan;
}

Octree& Octree::operator=(Octree const& other) {
	if (this != &other) {
		Release();
		Init();
		Octree temp(other);
		Swap(temp);
	}
	return *this;
}

Simplex::Octree::~Octree(void) { Release(); }

void Simplex::Octree::Swap(Octree & other) {
	std::swap(m_bbRegion, other.m_bbRegion);
	std::swap(m_lEntities, other.m_lEntities);
	std::swap(m_uID, other.m_uID);
	std::swap(m_uLevel, other.m_uLevel);
	std::swap(m_bActiveNodes, other.m_bActiveNodes);
	std::swap(m_pParent, other.m_pParent);
	std::swap(m_pChildren, other.m_pChildren);
	std::swap(m_iCurLife, other.m_iCurLife);
	std::swap(m_iMaxLifespan, other.m_iMaxLifespan);
}

void Simplex::Octree::Update(void) {
	if (Octree::s_bTreeBuilt) {
		//Countdown timer for any leafs with no objects
		if (m_lEntities.size() == 0) {
			if (IsLeaf()) {
				if (m_iCurLife == -1) {
					m_iCurLife = m_iMaxLifespan;
				} else if (m_iCurLife > 0) {
					m_iCurLife--;
				}
			}
		} else {
			if (m_iCurLife != -1) {
				if (m_iMaxLifespan <= 128) {
					m_iMaxLifespan *= 2;
				}
				m_iCurLife = -1;
			}
		}

		//Recursively update every child node
		if (!IsLeaf()) {
			for (int a = 0; a < 8; a++) {
				//Null check
				if (m_pChildren[a] == nullptr) { continue; }
				m_pChildren[a]->Update();
			}
		}

		//Loop through every entity in the tree
		for (int e = 0; e < m_lEntities.size(); e++) {
			//Get the entity id
			uint eID = m_lEntities[e];
			//Get a reference to the entity
			MyEntity* thisEntity = m_pEntityMngr->GetEntity(eID);
			//If the entity has not changed, do nothing
			if (!thisEntity->GetHasChanged()) { continue; }

			//Store a reference to the Octree node the object fits in
			Octree* current = this;
			//Entity has changed.  Recalculate where it should be in the tree
			//While the current octant doesn't fully contain the object
			while (!current->m_bbRegion.Contains(thisEntity->GetRigidBody())) {
				//Set current to the parent
				if (current->m_pParent != nullptr) {
					current = current->m_pParent;
				} else {
					//There are no more parents.  This is the root node.  Break
					break;
				}
			}

			//Tell the entity that the update was successful
			thisEntity->ChangesAccepted();

			//Remove the object from this octant
			m_lEntities.erase(m_lEntities.begin() + e);
			//Recurse down and insert the entity as deep into the tree as possible
			current->Insert(eID);
		}
		//Prune dead branches if this isn't a leaf
		if (!IsLeaf()) {
			for (byte b = 0; b < 8; b++) {
				if (m_pChildren[b] == nullptr) { continue; }
				//If the node has no entities
				if (m_pChildren[b]->m_lEntities.size() == 0 && m_pChildren[b]->m_iCurLife == 0 && m_pChildren[b]->IsLeaf()) {
					KillChild(b);
				}
			}
		}
	}
}

void Octree::UpdateTree(void) {
	//If the tree has not been built
	if (!Octree::s_bTreeBuilt) {
		//Loop through all queued objects waiting to be inserted
		while (Octree::s_qToInsert.size() != 0) {
			//Insert all into the entity list and remove from front of queue
			m_lEntities.push_back(Octree::s_qToInsert.front());
			Octree::s_qToInsert.pop_front();
		}
		//Recursively build the tree
		BuildTree();
		//Give octant ids to entities that are inside them
		AssignIDToEntity();
	} else {
		while (Octree::s_qToInsert.size() != 0) {
			Insert(Octree::s_qToInsert.front());
			Octree::s_qToInsert.pop_front();
		}
	}
}

void Octree::Display(vector3 a_v3Color) {
	//Draw cube for this octant
	m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_bbRegion.center) * glm::scale(m_bbRegion.halfSize * 2.0f), a_v3Color);
	//Loop through and call display for all children
	for (uint i = 0; i < 8; i++) {
		//null check
		if (m_pChildren[i] == nullptr) { continue; }
		m_pChildren[i]->Display(a_v3Color);
	}
}

void Octree::DisplayDifferently(vector3 a_v3Color, vector3 a_v3LeafColor) {
	//Draw cube for this octant
	if (Octree::s_iDisplayOctant == m_uID || Octree::s_iDisplayOctant == -1) {
		std::cout << Octree::s_iDisplayOctant << std::endl;
		m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_bbRegion.center) * glm::scale(m_bbRegion.halfSize * 2.0f), (IsLeaf()) ? a_v3LeafColor : a_v3Color);
	}
	//Loop through and call display for all children
	for (uint i = 0; i < 8; i++) {
		//null check
		if (m_pChildren[i] == nullptr) { continue; }
		m_pChildren[i]->DisplayDifferently(a_v3Color, a_v3LeafColor);
	}
}

void Simplex::Octree::DisplayLeafs(vector3 a_v3Color) {
	if (!IsLeaf()) {
		//Loop through and call display for all children
		for (uint i = 0; i < 8; i++) {
			//null check
			if (m_pChildren[i] == nullptr) { continue; }
			m_pChildren[i]->DisplayLeafs(a_v3Color);
		}
	} else {
		m_pMeshMngr->AddWireCubeToRenderList(glm::translate(m_bbRegion.center) * glm::scale(m_bbRegion.halfSize * 2.0f), a_v3Color);
	}
}

bool Simplex::Octree::IsLeaf(void) {
	return (m_bActiveNodes == 0);
}

void Octree::KillChild(byte childIndex) {
	if (m_pChildren[childIndex] == nullptr)
		return;
	delete m_pChildren[childIndex];
	m_pChildren[childIndex] = nullptr;
	m_bActiveNodes--;
}

void Octree::BuildTree(void) {
	//If there are no dimensions, expand to the extents of all objects in the world
	if (m_bbRegion.Dimensions() == ZERO_V3) {
		GenerateExtents();
	}
	//If there are fewer than the ideal entity count in this octant, don't do anything since this node is a leaf
	if (m_lEntities.size() <= Octree::s_uIdealEntityCount) {
		return;
	}
	//If this is a max-level octant, return
	if (m_uLevel == Octree::s_uMaxLevel) {
		return;
	}

	//Store stuff from bounding box
	vector3 l_min = m_bbRegion.Min();
	vector3 l_max = m_bbRegion.Max();
	vector3 l_c = m_bbRegion.center;

	//Generate bounding boxes for all 8 children
	BoundingBox childBounds[8];
	childBounds[0] = BoundingBox(l_min, l_c);
	childBounds[1] = BoundingBox(vector3(l_c.x, l_min.y, l_min.z), vector3(l_max.x, l_c.y, l_c.z));
	childBounds[2] = BoundingBox(vector3(l_c.x, l_min.y, l_c.z), vector3(l_max.x, l_c.y, l_max.z));
	childBounds[3] = BoundingBox(vector3(l_min.x, l_min.y, l_c.z), vector3(l_c.x, l_c.y, l_max.z));
	childBounds[4] = BoundingBox(vector3(l_min.x, l_c.y, l_min.z), vector3(l_c.x, l_max.y, l_c.z));
	childBounds[5] = BoundingBox(vector3(l_c.x, l_c.y, l_min.z), vector3(l_max.x, l_max.y, l_c.z));
	childBounds[6] = BoundingBox(l_c, l_max);
	childBounds[7] = BoundingBox(vector3(l_min.x, l_c.y, l_c.z), vector3(l_c.x, l_max.y, l_max.z));

	//List of entity ids that fit in each octant
	std::vector<uint> childLists[8];
	//List of objects no longer in this octant (they're in a child now)
	std::list<uint> toRemove;
	//Loop through all objects in this octant
	for (uint e = 0; e < m_lEntities.size(); e++) {
		//Get this entity's rigidbody
		Simplex::MyRigidBody* thisRB = m_pEntityMngr->GetEntity(m_lEntities[e])->GetRigidBody();
		//loop through every bounding box we just created
		for (uint b = 0; b < 8; b++) {
			//If the entity is in the bounding box
			if (childBounds[b].Contains(thisRB)) {
				//Add it to the child octant
				childLists[b].push_back(m_lEntities[e]);
				//Add it to the "toRemove" list so it will no longer be listed in this octant
				toRemove.push_back(m_lEntities[e]);
				break;
			}
		}
	}

	//Remove duplicate items from toRemove list
	toRemove.unique();
	std::vector<uint> newEntityList;
	//Remove every moved object from this octant
	for (uint r = 0; r < m_lEntities.size(); r++) {
		bool shouldRemove = false;
		//Check the "toRemove" list for matches
		for (std::list<uint>::const_iterator i = toRemove.begin(), end = toRemove.end(); i != end; ++i) {
			shouldRemove = m_lEntities[r] == *i;
			if (shouldRemove) { break; }
		}
		//If the entity should remain in this octant
		if (!shouldRemove) {
			//Add it to the new list
			newEntityList.push_back(m_lEntities[r]);
		}
	}
	//Set the new vector to the current vector
	m_lEntities = newEntityList;

	for (uint a = 0; a < 8; a++) {
		if (childLists[a].size() != 0) {
			m_pChildren[a] = CreateChildOctant(childBounds[a], childLists[a]);
			m_pChildren[a]->BuildTree();
		}
	}

	Octree::s_bTreeBuilt = true;
	Octree::s_bTreeReady = true;
}

void Octree::Insert(uint a_uEntityIndex) {
	//If current node is a leaf node with few objects, do nothing
	if (m_lEntities.size() <= 1 && IsLeaf()) {
		m_lEntities.push_back(a_uEntityIndex);
		return;
	}
	//If current node is already the maximum depth, do nothing
	if (m_uLevel >= Octree::s_uMaxLevel) {
		m_lEntities.push_back(a_uEntityIndex);
		return;
	}

	//Store stuff from bounding box
	vector3 l_min = m_bbRegion.Min();
	vector3 l_max = m_bbRegion.Max();
	vector3 l_c = m_bbRegion.center;

	//Generate bounding boxes for all 8 children
	BoundingBox childBounds[8];
	childBounds[0] = (m_pChildren[0] != nullptr) ? m_pChildren[0]->m_bbRegion : BoundingBox(l_min, l_c);
	childBounds[1] = (m_pChildren[1] != nullptr) ? m_pChildren[1]->m_bbRegion : BoundingBox(vector3(l_c.x, l_min.y, l_min.z), vector3(l_max.x, l_c.y, l_c.z));
	childBounds[2] = (m_pChildren[2] != nullptr) ? m_pChildren[2]->m_bbRegion : BoundingBox(vector3(l_c.x, l_min.y, l_c.z), vector3(l_max.x, l_c.y, l_max.z));
	childBounds[3] = (m_pChildren[3] != nullptr) ? m_pChildren[3]->m_bbRegion : BoundingBox(vector3(l_min.x, l_min.y, l_c.z), vector3(l_c.x, l_c.y, l_max.z));
	childBounds[4] = (m_pChildren[4] != nullptr) ? m_pChildren[4]->m_bbRegion : BoundingBox(vector3(l_min.x, l_c.y, l_min.z), vector3(l_c.x, l_max.y, l_c.z));
	childBounds[5] = (m_pChildren[5] != nullptr) ? m_pChildren[5]->m_bbRegion : BoundingBox(vector3(l_c.x, l_c.y, l_min.z), vector3(l_max.x, l_max.y, l_c.z));
	childBounds[6] = (m_pChildren[6] != nullptr) ? m_pChildren[6]->m_bbRegion : BoundingBox(l_c, l_max);
	childBounds[7] = (m_pChildren[7] != nullptr) ? m_pChildren[7]->m_bbRegion : BoundingBox(vector3(l_min.x, l_c.y, l_c.z), vector3(l_c.x, l_max.y, l_max.z));

	MyEntity* temp = m_pEntityMngr->GetEntity(a_uEntityIndex);
	if (temp == nullptr) { return; }
	//Is the entity contained by the root box?
	if (m_bbRegion.Contains(temp->GetRigidBody())) {
		bool found = false;
		//Try to place object in a child octant.  If that fails, put it in current octant
		for (byte a = 0; a < 8; a++) {
			if (childBounds[a].Contains(temp->GetRigidBody())) {
				if (m_pChildren[a] != nullptr) {
					m_pChildren[a]->Insert(a_uEntityIndex);
				} else {
					m_pChildren[a] = CreateChildOctant(childBounds[a], a_uEntityIndex);
				}
				found = true;
				break;
			}
		}
		//If no child octant was found, insert item into this octant
		if (!found) {
			m_lEntities.push_back(a_uEntityIndex);
			AssignIDToEntity();
		}
	} else {
		//The item has moved outside of the tree.  We have to rebuild
		BuildTree();
	}
}

Octree* Octree::CreateChildOctant(BoundingBox bounds, std::vector<uint> entities) {
	if (entities.size() == 0) {
		return nullptr;
	}
	Octree* newOctant = new Octree(bounds, entities);
	newOctant->m_pParent = this;
	newOctant->m_uLevel = m_uLevel + 1;
	m_bActiveNodes++;
	return newOctant;
}

Octree* Octree::CreateChildOctant(BoundingBox bounds, uint entityIndex) {
	std::vector<uint> entList(1);
	entList[0] = entityIndex;
	return CreateChildOctant(bounds, entList);
}

void Simplex::Octree::AssignIDToEntity(void) {
	//Loop through all entities in this octant
	//When building the tree, this was the smallest octant that could fit the whole entity
	for (uint i = 0; i < m_lEntities.size(); i++) {
		//Give the entity IDs for every octant it intersects
		MyEntity* temp = m_pEntityMngr->GetEntity(m_lEntities[i]);
		if (temp != nullptr) {
			//Clear the IDs
			temp->ClearOctantList();
			//Assign ids
			GetIntersectingIDs(temp);
		}
	}
	//Loop through children and call their AssignIDToEntity functions
	for (uint c = 0; c < 8; c++) {
		//Null check
		if (m_pChildren[c] == nullptr) { continue; }
		m_pChildren[c]->AssignIDToEntity();
	}
}

void Octree::GetIntersectingIDs(MyEntity* entity) {
	//Loop through all children and call this function recursively
	if (!IsLeaf()) {
		for (byte a = 0; a < 8; a++) {
			if (m_pChildren[a] == nullptr) { continue; }
			m_pChildren[a]->GetIntersectingIDs(entity);
		}
	}
	//If the entity is colliding with this octant, give it this octant ID
	if (m_bbRegion.Collides(entity->GetRigidBody())) {
		entity->AddOctant(m_uID);
	}
}

// Generates the bounding box for this octant based on the objects it contains
void Octree::GenerateExtents(void) {
	vector3 min = vector3(0.0f);
	vector3 max = vector3(0.0f);
	for (uint i = 0; i < m_lEntities.size(); i++) {
		MyRigidBody* thisRB = m_pEntityMngr->GetEntity(m_lEntities[i])->GetRigidBody();
		vector3 rbMin = thisRB->GetMinGlobal();
		vector3 rbMax = thisRB->GetMaxGlobal();
		//Check all axes for min
		min.x = (rbMin.x < min.x) ? rbMin.x : min.x;
		min.y = (rbMin.y < min.y) ? rbMin.y : min.y;
		min.z = (rbMin.z < min.z) ? rbMin.z : min.z;
		//Check all axes for max
		max.x = (rbMax.x > max.x) ? rbMax.x : max.x;
		max.y = (rbMax.y > max.y) ? rbMax.y : max.y;
		max.z = (rbMax.z > max.z) ? rbMax.z : max.z;
	}
	//Get the center point
	m_bbRegion.center = (min + max) / 2.0f;
	//Get the half width
	m_bbRegion.halfSize = max - m_bbRegion.center;
}

bool Simplex::BoundingBox::Contains(Simplex::MyRigidBody * _rb) {
	vector3 min = this->Min();
	vector3 max = this->Max();
	vector3 rbMin = _rb->GetMinGlobal();
	vector3 rbMax = _rb->GetMaxGlobal();
	//AABB containment check
	return (min.x <= rbMin.x && max.x >= rbMax.x &&
			min.y <= rbMin.y && max.y >= rbMax.y &&
			min.z <= rbMin.z && max.z >= rbMax.z);
}

bool Simplex::BoundingBox::Collides(MyRigidBody * _rb) {
	vector3 min = this->Min();
	vector3 max = this->Max();
	vector3 rbMin = _rb->GetMinGlobal();
	vector3 rbMax = _rb->GetMaxGlobal();
	//AABB collision check
	return (min.x <= rbMax.x && max.x >= rbMin.x &&
			min.y <= rbMax.y && max.y >= rbMin.y &&
			min.z <= rbMax.z && max.z >= rbMin.z);
}
