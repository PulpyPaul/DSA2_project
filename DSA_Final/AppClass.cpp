#include "AppClass.h"
using namespace Simplex;
void Application::InitVariables(void) {
	//Set the position and target of the camera
	m_pCameraMngr->SetPositionTargetAndUp(
		vector3(0.0f, 1.0f, 13.0f), //Position
		vector3(0.0f, 1.0f, 12.0f),	//Target
		AXIS_Y);					//Up

	m_pLightMngr->SetPosition(vector3(0.0f, 3.0f, 13.0f), 1); //set the position of first light(0 is reserved for global light)

	////creeper
	//m_pCreeper = new Model();
	//m_pCreeper->Load("Minecraft\\Creeper.obj");
	//m_pCreeperRB = new MyRigidBody(m_pCreeper->GetVertexList());

	////steve
	//m_pSteve = new Model();
	//m_pSteve->Load("Minecraft\\Steve.obj");
	//m_pSteveRB = new MyRigidBody(m_pSteve->GetVertexList());

	//Floor
	floorMod = new Model();
	floorMod->Load("FinalScene\\Floor.obj");


	//Creating Ceiling
	ceilingMod = new Model();
	ceilingMod->Load("FinalScene\\Ceiling.obj");

	//Creating Walls
	for (int i = 0; i < 4; i++) {
		wallsMod.push_back(new Model());
		wallsMod[i]->Load("FinalScene\\Walls.obj");
	}

}
void Application::Update(void) {
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the ArcBall active?
	ArcBall();

	//Is the first person camera active?
	CameraRotation();


	////Set model matrix to the creeper
	//matrix4 mCreeper = glm::translate(m_v3Creeper + vector3(1.5,0.0f,0.0f)) * ToMatrix4(m_qCreeper) * ToMatrix4(m_qArcBall);
	//m_pCreeper->SetModelMatrix(mCreeper);
	//m_pCreeperRB->SetModelMatrix(mCreeper);
	//m_pMeshMngr->AddAxisToRenderList(mCreeper);

	////Set model matrix to Steve
	//matrix4 mSteve = glm::translate(vector3(2.25f, 0.0f, 0.0f)); //* glm::rotate(IDENTITY_M4, -55.0f, AXIS_Z);
	//m_pSteve->SetModelMatrix(mSteve);
	//m_pSteveRB->SetModelMatrix(mSteve);
	//m_pMeshMngr->AddAxisToRenderList(mSteve);


	//Set model matrix to the Floor
	matrix4 mFloor = glm::translate(vector3(0.0f, -1.0f, 0.0f));
	floorMod->SetModelMatrix(mFloor);

	//Set Model matrix for the ceiling
	matrix4 mCeiling = glm::translate(vector3(0.0f, 10.0f, 0.0f));
	ceilingMod->SetModelMatrix(mCeiling);

	//Create Wall
	matrix4 mWalls[4];
	mWalls[0] = glm::translate(vector3(10.0f, 0.0f, 0.0f));
	mWalls[1] = glm::translate(vector3(-10.0f, 0.0f, 0.0f));
	mWalls[2] = glm::translate(vector3(5.0f, 0.0f, 10.0f)) * glm::rotate(IDENTITY_M4, 90.0f, AXIS_Y);
	mWalls[3] = glm::translate(vector3(5.0f, 0.0f, -10.0f)) * glm::rotate(IDENTITY_M4, 90.0f, AXIS_Y);

	for (int i = 0; i < 4; i++) {
		wallsMod[i]->SetModelMatrix(mWalls[i]);
	}

	//bool bColliding = m_pCreeperRB->IsColliding(m_pSteveRB);

	//m_pCreeper->AddToRenderList();
	//m_pCreeperRB->AddToRenderList();
	//m_pSteve->AddToRenderList();
	//m_pSteveRB->AddToRenderList();

	//Add floor to render list
	floorMod->AddToRenderList();

	ceilingMod->AddToRenderList();

	//Add walls to render list
	for (int i = 0; i < wallsMod.size(); i++) {
		wallsMod[i]->AddToRenderList();
	}

	/*m_pMeshMngr->Print("Colliding: ");
	if (bColliding)
		m_pMeshMngr->PrintLine("YES!", C_RED);
	else
		m_pMeshMngr->PrintLine("no", C_YELLOW);*/
}
void Application::Display(void) {
	// Clear the screen
	ClearScreen();

	// draw a skybox
	m_pMeshMngr->AddSkyboxToRenderList();

	//render list call
	m_uRenderCallCount = m_pMeshMngr->Render();

	//clear the render list
	m_pMeshMngr->ClearRenderList();

	//draw gui
	DrawGUI();

	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}

void Application::Release(void) {

	//release the model
	SafeDelete(m_pCreeper);

	//release the rigid body for the model
	SafeDelete(m_pCreeperRB);

	//release the model
	SafeDelete(m_pSteve);

	//release the rigid body for the model
	SafeDelete(m_pSteveRB);

	//Release the floor Model
	SafeDelete(floorMod);

	//Release the Walls!
	for (int i = 0; i < wallsMod.size(); i++) {
		SafeDelete(wallsMod[i]);
		wallsMod[i] = nullptr;
	}

	//release GUI
	ShutdownGUI();
}