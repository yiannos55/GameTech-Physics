#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "TestPacketReceiver.h"
#include "../CSC8503Common/GameServer.h"
#include "../CSC8503Common/GameClient.h"
//#include "NetworkedGame.h"


namespace NCL {
	namespace CSC8503 {
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);

			
		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();

			/*
			These are some of the world/object creation functions I created when testing the functionality
			in the module. Feel free to mess around with them to see different objects being created in different
			test scenarios (constraints, collision types, and so on). 
			*/
			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void BridgeConstraintTest();
			void SimpleGJKTest();

			void BuildWalls();
			void BuildBase();
			void AddApples();
			void BuildTrees();
			void BuildTrampolines();
			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();
			void LockedCameraMovement();
			void BuildSpeedBalls();

			//menu
			GameObject* AddMenuToWorld(const Vector3& position, Vector3 dimensions);
			GameObject* AddButtonToWorld(const Vector3& position, Vector3 dimensions, string name);
			void LockedMenu();
			GameObject* menu;
			bool start;
			//////////////////////
			bool collectApple(Vector3 fwdAxis);
			//////////////////////
			/////AI testing
			void MoveAI();
			void TestStateMachine();
			vector<Vector3> testNodes;
			float lerp(float v0, float v1, float t) {
				return (1 - t) * v0 + t * v1;
			}
			int i;
			float tempo = 0.0f;
			bool* found=false;
			int eventData = 0;
			//////ai 

			////networking
			void TestNetworking(bool s);
			void sentPackets(bool s);
			
			TestPacketReceiver* serverReceiver;
			TestPacketReceiver* clientReceiver;
			GameServer* server;
			GameClient* client;

			bool isNetworked;
			bool isClient;
			bool isServer;

			void Client();
			void Server();
			bool clientConnected;
			////
			GameObject* AddFloorToWorld(const Vector3& position);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);

			GameObject* AddBaseToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddTrunk(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddLeaves(const Vector3& position, float radius, float inverseMass = 10.0f);

			GameObject* AddTrampoline(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			//IT'S HAPPENING
			GameObject* AddGooseToWorld(const Vector3& position);
			GameObject* AddParkKeeperToWorld(const Vector3& position);
			GameObject* AddCharacterToWorld(const Vector3& position);
			GameObject* AddAppleToWorld(const Vector3& position);

			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;
			GameObject*         goose;
			GameObject*			HomeBase;
			GameObject*			keeper;
			GameObject*			trigger;
			NavigationGrid*		navGrid;
			NavigationPath*		outPath;
			GameObject* AddTrigger(const Vector3& position, Vector3 dimensions);
			GameObject* AddSpeedBall(const Vector3& position, float radius, float inverseMass);
			int points=0;

			bool useGravity;
			bool inSelectionMode;
			bool collected = false;

			float		forceMagnitude;

			GameObject* selectionObject = nullptr;
			GameObject* pickedUpObject = nullptr;

			///////////////////////////////////
			GameObject* targetObject = nullptr;
			///////////////////////////////////

			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	gooseMesh	= nullptr;
			OGLMesh*	keeperMesh	= nullptr;
			OGLMesh*	appleMesh	= nullptr;
			OGLMesh*	charA		= nullptr;
			OGLMesh*	charB		= nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0,10,30);

			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}
		};
	}
}

