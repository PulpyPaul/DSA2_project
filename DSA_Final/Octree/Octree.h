#ifndef __MYOCTREE_H_
#define __MYOCTREE_H_

#include "Simplex\Simplex.h"

//Yay for tutorials!: https://www.gamedev.net/articles/programming/general-and-gameplay-programming/introduction-to-octrees-r3529/

namespace Simplex {
	//Forward declare
	class MyRigidBody;
	class MyEntity;
	class MyEntityManager;

	struct BoundingBox {
		vector3 center = ZERO_V3;
		vector3 halfSize = ZERO_V3;

		BoundingBox(vector3 _min, vector3 _max) {
			center = (_max + _min) / 2.0f;
			halfSize = _max - center;
		}
		BoundingBox() {}
		vector3 Dimensions() { return Max() - Min(); }
		vector3 Min() { return center - halfSize; }
		vector3 Max() { return center + halfSize; }
		//AABB check for completely containing a rigidbody
		bool Contains(MyRigidBody* _rb);
		//AABB check for colliding with a rigidbody
		bool Collides(MyRigidBody* _rb);
	};

	class Octree {
		MeshManager* m_pMeshMngr;
		MyEntityManager* m_pEntityMngr;

		//Bounding box for this tree
		BoundingBox m_bbRegion;
		//List of entities in this octant
		std::vector<uint> m_lEntities;

		uint m_uID;
		uint m_uLevel;

		byte m_bActiveNodes;

		Octree* m_pParent;
		Octree* m_pChildren[8];

		int m_iCurLife;
		int m_iMaxLifespan;

	public:
		//Total number of octants
		static uint s_uOctantCount;
		//Unique ID incrementer for octants
		static uint s_uNextOctantID;
		//Max level 
		static uint s_uMaxLevel;
		//Ideal number of entiti
		static uint s_uIdealEntityCount;
		//Root octant
		static Octree* s_pRoot;
		//Tree still has objects requiring insertion
		static bool s_bTreeReady;
		//Is there an existing tree?
		static bool s_bTreeBuilt;
		//Flag the octree for rebuilding
		static bool s_bShouldRebuild;
		//Which octant ID to display
		static int s_iDisplayOctant;
		//Queue of objects waiting to be inserted
		static std::deque<uint> s_qToInsert;

		//Constructors
		Octree();
		Octree(BoundingBox a_bbRegion);
		Octree::Octree(BoundingBox a_bbRegion, std::vector<uint> const &a_lObjectList);
		//Copy constructor
		Octree(Octree const &other);
		//Copy assignment operator
		Octree& operator=(Octree const &other);
		//Destructor
		~Octree(void);
		//Swap objects
		void Swap(Octree &other);
		//Update loop.  Recursive
		void Update(void);
		//Update the tree
		void UpdateTree(void);
		//Draw this octant and call the same method on children
		void Display(vector3 a_v3Color = C_YELLOW);
		//Display leafs as a different color
		void DisplayDifferently(vector3 a_v3Color = C_YELLOW, vector3 a_v3LeafColor = C_RED);
		//Only display octants that are leafs
		void DisplayLeafs(vector3 a_v3Color = C_RED);
		//Is this octant a leaf?
		bool IsLeaf(void);
		//Recursively destroys all children
		void KillBranches(void);
		//Recursively destroy a single child octant
		void KillChild(byte childIndex);
		//Create a tree using subdivisions
		void BuildTree(void);
		//Insert entity into tree
		void Insert(uint a_uEntityIndex);
		//Traverse the tree down to the leafs and give all objects the index of the octant
		void AssignIDToEntity(void);
		//Get ids of all child octants a rigidbody intersects
		void GetIntersectingIDs(MyEntity* _rb);
		//Generates a bounding area based on the entities in the list
		void GenerateExtents(void);
	private:
		void Release(void);
		void Init(void);

		Octree* CreateChildOctant(BoundingBox bounds, std::vector<uint> entities);
		Octree* CreateChildOctant(BoundingBox bounds, uint entityIndex);
	};
};

#endif