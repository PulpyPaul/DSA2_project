#include "AppClass.h"
using namespace Simplex;
void Application::InitVariables(void) {
	//Set the position and target of the camera
	m_pCameraMngr->SetPositionTargetAndUp(
		vector3(0.0f, 1.0f, 5.0f), //Position
		vector3(0.0f, 1.0f, 0.0f),	//Target
		AXIS_Y);					//Up

	//Add lights to the scene (light index 0 is reserved for global light)
	m_pLightMngr->SetPosition(vector3(0.0f, 3.0f, 0.0f), 1);
	m_pLightMngr->SetIntensity(10.0f, 1);

	//Create the room
	CreateRoom();

	//Make bounding volumes for all entities invisible
	for (uint i = 0; i < m_pEntityMngr->GetEntityCount(); i++) {
		m_pEntityMngr->GetRigidBody(i)->SetVisibleOBB(false);
		m_pEntityMngr->GetRigidBody(i)->SetVisibleBS(false);
		m_pEntityMngr->GetRigidBody(i)->SetVisibleARBB(false);
	}
}
void Application::Update(void) {
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the ArcBall active?
	ArcBall();

	//Is the first person camera active?
	CameraRotation();

	//Draw axes based on the floor
	m_pMeshMngr->AddAxisToRenderList(IDENTITY_M4);

	//Update the entity manager
	m_pEntityMngr->Update();

	//Add all objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
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

void Application::CreateRoom(void) {
	//This is just trial-and-error tested positions for the walls, floor, and ceiling so that the room and ground are centered at <0, 0, 0>
	//Load the floor
	m_pEntityMngr->AddEntity("FinalScene\\Floor.obj", "Floor");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(0.0f, -1.2f, 1.0f)));
	//Load the ceiling
	m_pEntityMngr->AddEntity("FinalScene\\Ceiling.obj", "Ceiling");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(0.0f, 10.0f, 1.0f)));
	//Load walls (no loop since we need to create unique IDs for each)
	//Wall 0
	m_pEntityMngr->AddEntity("FinalScene\\Walls.obj", "Wall0");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(14.0f, -0.2f, 0.0f)) * glm::scale(1.0f, 0.62f, 1.7f));
	//Wall 1
	m_pEntityMngr->AddEntity("FinalScene\\Walls.obj", "Wall1");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(-14.0f, -0.2f, 0.0f)) * glm::scale(1.0f, 0.62f, 1.7f));
	//Wall 2
	m_pEntityMngr->AddEntity("FinalScene\\Walls.obj", "Wall2");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(0.0f, -0.2f, 14.0f)) * glm::scale(1.7f, 0.62f, 1.0f) * glm::rotate(IDENTITY_M4, 90.0f, AXIS_Y));
	//Wall 3
	m_pEntityMngr->AddEntity("FinalScene\\Walls.obj", "Wall3");
	m_pEntityMngr->SetModelMatrix(glm::translate(vector3(0.0f, -0.2f, -14.0f)) * glm::scale(1.7f, 0.62f, 1.0f) * glm::rotate(IDENTITY_M4, -90.0f, AXIS_Y));
}

void Application::Release(void) {
	//release GUI
	ShutdownGUI();
}