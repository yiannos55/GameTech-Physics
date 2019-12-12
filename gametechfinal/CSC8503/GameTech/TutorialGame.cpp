#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "..//CSC8503Common/Debug.h"

#include "../CSC8503Common/PositionConstraint.h"

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame() {
	world = new GameWorld();
	renderer = new GameTechRenderer(*world);
	physics = new PhysicsSystem(*world);

	forceMagnitude = 10.0f;
	useGravity = false;
	inSelectionMode = false;

	Debug::SetRenderer(renderer);

	InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes,
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh", &cubeMesh);
	loadFunc("sphere.msh", &sphereMesh);
	loadFunc("goose.msh", &gooseMesh);
	loadFunc("CharacterA.msh", &keeperMesh);
	loadFunc("CharacterM.msh", &charA);
	loadFunc("CharacterF.msh", &charB);
	loadFunc("Apple.msh", &appleMesh);

	basicTex = (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitCamera();
	InitWorld();
}

TutorialGame::~TutorialGame() {
	delete cubeMesh;
	delete sphereMesh;
	delete gooseMesh;
	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {

	if ((lockedObject = goose) != nullptr) {
		LockedCameraMovement();
	}

	UpdateKeys();

	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(10, 40));
	}
	else {
		Debug::Print("(G)ravity off", Vector2(10, 40));
	}
	renderer->DrawString("apples colected :" + std::to_string(points), Vector2(10, 60)); // draw debug text at 10, 20

	SelectObject();
	MoveSelectedObject();
	if (collected) {
		MoveAI();
	}
	world->UpdateWorld(dt);
	renderer->Update(dt);
	physics->Update(dt);
	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
	}

	Debug::FlushRenderables();
	renderer->Render();
}

void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G)) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Matrix4 view = world->GetMainCamera()->BuildViewMatrix();
	Matrix4 camWorld = view.Inverse();
	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		goose->GetPhysicsObject()->AddTorque(Vector3(0, 2, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		goose->GetPhysicsObject()->AddTorque(Vector3(0, -2, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		goose->GetPhysicsObject()->AddForce(fwdAxis * 70);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		goose->GetPhysicsObject()->AddForce(-fwdAxis * 70);
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::SPACE)) {
		goose->GetPhysicsObject()->AddForce(Vector3(0, 50, 0));
	
	}

	trigger->GetTransform().SetParent(&lockedObject->GetTransform());
	trigger->GetTransform().SetLocalPosition(Vector3(0, 0, 0));

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F)) {

	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::E)) {
	//	collected = !collected;
			if (collectApple(fwdAxis) && (!collected)) {
				collected = true;
				targetObject->GetTransform().SetParent(&goose->GetTransform());
				targetObject->GetTransform().SetLocalPosition(Vector3(0, 2, 0));
				targetObject->GetPhysicsObject()->previousMass = targetObject->GetPhysicsObject()->GetInverseMass();
				targetObject->GetPhysicsObject()->SetInverseMass(0);
				pickedUpObject = targetObject;
			//	points++;////////////////////if apple in base's range, points ++
			}
		//else {
		//		collected = false;
		//}
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::R)) {

			collected=false;
			if (pickedUpObject) {
				for (GameObject* obj : physics->GetTrigerList()) {
					if (obj->GetName() == "base") {
						points++;
					}
				}
				pickedUpObject->GetTransform().SetParent(nullptr);
				pickedUpObject->GetPhysicsObject()->SetInverseMass(pickedUpObject->GetPhysicsObject()->previousMass);
				pickedUpObject->GetTransform().SetWorldPosition(goose->GetTransform().GetWorldPosition() + fwdAxis*2);
			//	CollisionDetection::CollisionInfo info;
				//if (CollisionDetection::ObjectIntersection(HomeBase, pickedUpObject, info)) {
				//	points++;
				//}
				pickedUpObject = nullptr;
			}
	}
}
vector<Vector3> testNodes;
float lerp(float v0, float v1, float t) {
	return (1 - t) * v0 + t * v1;
}
void TutorialGame::MoveAI() {
	NavigationGrid grid("TestGrid1.txt");
	NavigationPath outPath;
	bool found = grid.FindPath(keeper->GetTransform().GetWorldPosition(), goose->GetTransform().GetWorldPosition(), outPath);

	Vector3 pos;
	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
	}

	for (int i = 1; i < testNodes.size(); ++i) {
		Vector3 a = testNodes[i - 1];
		Vector3 b = testNodes[i];

		keeper->GetTransform().SetWorldPosition(Vector3(lerp(keeper->GetTransform().GetWorldPosition().x, a.x, 0.003f),
														lerp(keeper->GetTransform().GetWorldPosition().y, a.y, 0.003f),
														lerp(keeper->GetTransform().GetWorldPosition().z, a.z, 0.003f)));
		////Debug::DrawLine(a + Vector3(0, 3, 0), b + Vector3(0, 3, 0), Vector4(0, 1, 0, 1));
	}
}

bool TutorialGame::collectApple(Vector3 fwdAxis) {
	Vector3 testDir = fwdAxis;
	Ray detectForward(lockedObject->GetTransform().GetWorldPosition(), testDir);

	RayCollision closestCollision1;
	//Debug::DrawLine(lockedObject->GetTransform().GetWorldPosition(), testDir, Vector4(1, 0, 0, 1));

	if (world->Raycast(detectForward, closestCollision1, true) && closestCollision1.collidedAt.z < lockedObject->GetTransform().GetLocalPosition().z + 10) {
		targetObject = (GameObject*)closestCollision1.node;
		if (targetObject->GetName() == "apple") {
			//	targetObject->GetTransform().SetLocalPosition(goose->GetTransform().GetLocalPosition() + Vector3(0,5,0));
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

void  TutorialGame::LockedCameraMovement() {
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetWorldPosition();
		Vector3 camPos = objPos + lockedObject->GetConstTransform().GetWorldOrientation() * lockedOffset;

		camPos = (Matrix4::Translation(objPos) * Matrix4::Rotation(180, Vector3{ 0,1,0 }) * Matrix4::Translation(-objPos)) * camPos;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(angles.x);
		world->GetMainCamera()->SetYaw(angles.y);
	}
}


void TutorialGame::DebugObjectMovement() {
	//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM6)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 50, 0)); \
		}
	}
}

/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around.

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		renderer->DrawString("Press Q to change to camera mode!", Vector2(10, 0));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());


			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;
				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				//	Debug::DrawLine(selectionObject->GetTransform().GetLocalPosition(), selectionObject->GetTransform().GetLocalPosition() +Vector3(0,0,10), Vector4(1, 0, 0, 1));

					//////////////////////////////
					//Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!
					//Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);

					/*Vector3 testDir = Vector3(0,0,1);
					Ray detectForward (lockedObject->GetTransform().GetWorldPosition(), testDir);

					RayCollision closestCollision1;
					Debug::DrawLine(lockedObject->GetTransform().GetWorldPosition(), testDir, Vector4(1, 0, 0, 1));

					if (world->Raycast(detectForward, closestCollision1, true)&& closestCollision1.collidedAt.z < lockedObject->GetTransform().GetLocalPosition().z+10) {
						targetObject = (GameObject*)closestCollision1.node;
						targetObject->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));
						return true;
					}
					else {
						return false;
					}*/
					///////////////////////////

				return true;
			}
			else {
				return false;
			}
		}
		if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
			if (selectionObject) {
				if (lockedObject == selectionObject) {
					lockedObject = nullptr;
				}
				else {
					lockedObject = selectionObject;
				}
			}
		}
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(10, 0));
	}
	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void TutorialGame::MoveSelectedObject() {
	renderer->DrawString("click Force:" + std::to_string(forceMagnitude), Vector2(10, 20)); // draw debug text at 10, 20
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject) {
		return; // nothing was selected
	}
	// push the selected object
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());
		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
			}
		}
	}
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.5f);
	world->GetMainCamera()->SetFarPlane(5000.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(-60, 40, 60));
	//lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	//InitMixedGridWorld(10, 10, 3.5f, 3.5f);
	goose = AddGooseToWorld(Vector3(270, 5, 270));
	AddAppleToWorld(Vector3(250, 5, 250));
	AddAppleToWorld(Vector3(230, 5, 230));

	BuildWalls(); //walls
	BuildBase();
	BuildTrampolines();
	BuildTrees();
	keeper = AddParkKeeperToWorld(Vector3(80, 4, 80));

	AddFloorToWorld(Vector3(0, 0, 0));
	BridgeConstraintTest();
}



//From here on it's functions to add in objects to the world!
GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("cube");

	AABBVolume* volume = new AABBVolume(dimensions);
	//OBBVolume* volume = new OBBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddTrigger(const Vector3& position, Vector3 dimensions) {
	GameObject* cube = new GameObject("trigger");

	AABBVolume* volume = new AABBVolume(dimensions);
	
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, nullptr, basicShader));

	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetPhysicsObject()->SetInverseMass(0);
	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::BuildWalls() {
	AddCubeToWorld(Vector3(0, 100, -300), Vector3(300, 100, 10), 0);
	AddCubeToWorld(Vector3(0, 100, 300), Vector3(300, 100, 10), 0);
	AddCubeToWorld(Vector3(300, 100, 0), Vector3(10, 100, 300), 0);
	AddCubeToWorld(Vector3(-300, 100, 0), Vector3(10, 100, 300), 0);


	//////////////////add maze around AI
	AddCubeToWorld(Vector3(50, 5, 100), Vector3(50, 3, 3), 0);
	AddCubeToWorld(Vector3(50, 5, 0), Vector3(50, 3, 3), 0);
	AddCubeToWorld(Vector3(0, 5, 50), Vector3(3, 3, 50), 0);
	AddCubeToWorld(Vector3(100, 5, 50), Vector3(3, 3, 50), 0);
	//////////////////////
}

void TutorialGame::BuildTrees() {

	
	AddTrunk(Vector3(0, 6, 0), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(0, 18, 0), 3, 0);
	AddLeaves(Vector3(-1, 15, -1), 4, 0);
	AddLeaves(Vector3(1, 15, 1), 4, 0);

	AddTrunk(Vector3(15, 6, 15), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(15, 18, 15), 3, 0);
	AddLeaves(Vector3(14, 15, 14), 4, 0);
	AddLeaves(Vector3(16, 15, 16), 4, 0);

	AddTrunk(Vector3(-20, 6, 20), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(-20, 18, 20), 3, 0);
	AddLeaves(Vector3(-21, 15, 19), 4, 0);
	AddLeaves(Vector3(-19, 15, 21), 4, 0);

	AddTrunk(Vector3(30, 6, 30), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(30, 18, 30), 3, 0);
	AddLeaves(Vector3(29, 15, 29), 4, 0);
	AddLeaves(Vector3(31, 15, 31), 4, 0);

	AddTrunk(Vector3(-20, 6, 40), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(-20, 18, 40), 3, 0);
	AddLeaves(Vector3(-21, 15, 39), 4, 0);
	AddLeaves(Vector3(-19, 15, 41), 4, 0);

	AddTrunk(Vector3(-50, 6, 70), Vector3(1, 6, 1), 0);
	AddLeaves(Vector3(-50, 18, 70), 3, 0);
	AddLeaves(Vector3(-51, 15, 69), 4, 0);
	AddLeaves(Vector3(-49, 15, 71), 4, 0);

}
void TutorialGame::BuildBase() {
	//AddBaseToWorld(Vector3(-270, 1, -270), Vector3(30, 2, 30), 0);
	AddBaseToWorld(Vector3(270, 1, 270), Vector3(30, 2, 30), 0);
}

void TutorialGame::BuildTrampolines() {
	AddTrampoline(Vector3(-100, 0.5, 0), Vector3(5, 2, 5), 0);
	AddTrampoline(Vector3(50, 0.5, 50), Vector3(5, 2, 5), 0);
}
/*

A single function to add a large immoveable cube to the bottom of our world

*/

GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject("floor");

	Vector3 floorSize = Vector3(300, 2, 300);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetWorldScale(floorSize);
	floor->GetTransform().SetWorldPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject("sphere");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);
	sphere->GetTransform().SetWorldScale(sphereSize);
	sphere->GetTransform().SetWorldPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);

	return sphere;
}


GameObject* TutorialGame::AddTrampoline(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("trampoline");

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetRenderObject()->SetColour(Vector4(1, 0.1f, 0.1f, 1));
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddBaseToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* HomeBase = new GameObject("base");

	AABBVolume* volume = new AABBVolume(dimensions);

	HomeBase->SetBoundingVolume((CollisionVolume*)volume);

	HomeBase->GetTransform().SetWorldPosition(position);
	HomeBase->GetTransform().SetWorldScale(dimensions);

	HomeBase->SetRenderObject(new RenderObject(&HomeBase->GetTransform(), cubeMesh, basicTex, basicShader));
	HomeBase->SetPhysicsObject(new PhysicsObject(&HomeBase->GetTransform(), HomeBase->GetBoundingVolume()));
	HomeBase->GetRenderObject()->SetColour(Vector4(0.2, 0.6, 0.5, 1));
	HomeBase->GetPhysicsObject()->SetInverseMass(inverseMass);
	HomeBase->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(HomeBase);

	return HomeBase;
}
GameObject* TutorialGame::AddTrunk(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("trunks");

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetRenderObject()->SetColour(Vector4(0.59, 0.29, 0, 1));
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}
GameObject* TutorialGame::AddLeaves(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject("sphere");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);
	sphere->GetTransform().SetWorldScale(sphereSize);
	sphere->GetTransform().SetWorldPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));
	sphere->GetRenderObject()->SetColour(Vector4(0.2, 1, 0.5, 1));
	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);

	return sphere;
}
GameObject* TutorialGame::AddGooseToWorld(const Vector3& position)
{
	float size = 1.0f;
	float inverseMass = 1.0f;

	GameObject* goose = new GameObject("goose");

	SphereVolume* volume = new SphereVolume(size);
	goose->SetBoundingVolume((CollisionVolume*)volume);

	goose->GetTransform().SetWorldScale(Vector3(size, size, size));
	goose->GetTransform().SetWorldPosition(position);

	goose->SetRenderObject(new RenderObject(&goose->GetTransform(), gooseMesh, nullptr, basicShader));
	goose->SetPhysicsObject(new PhysicsObject(&goose->GetTransform(), goose->GetBoundingVolume()));

	goose->GetPhysicsObject()->SetInverseMass(inverseMass);
	goose->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(goose);
	trigger = AddTrigger(position, Vector3(3, 3, 3));
	return (goose);
}

GameObject* TutorialGame::AddParkKeeperToWorld(const Vector3& position)
{
	float meshSize = 4.0f;
	float inverseMass = 0.5f;

	GameObject* keeper = new GameObject("keeper");

	AABBVolume* volume = new AABBVolume(Vector3(0.3, 0.9f, 0.3) * meshSize);
	keeper->SetBoundingVolume((CollisionVolume*)volume);

	keeper->GetTransform().SetWorldScale(Vector3(meshSize, meshSize, meshSize));
	keeper->GetTransform().SetWorldPosition(position);

	keeper->SetRenderObject(new RenderObject(&keeper->GetTransform(), keeperMesh, nullptr, basicShader));
	keeper->SetPhysicsObject(new PhysicsObject(&keeper->GetTransform(), keeper->GetBoundingVolume()));

	keeper->GetPhysicsObject()->SetInverseMass(inverseMass);
	keeper->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(keeper);

	return keeper;
}

GameObject* TutorialGame::AddCharacterToWorld(const Vector3& position) {
	float meshSize = 4.0f;
	float inverseMass = 0.5f;

	auto pos = keeperMesh->GetPositionData();

	Vector3 minVal = pos[0];
	Vector3 maxVal = pos[0];

	for (auto& i : pos) {
		maxVal.y = max(maxVal.y, i.y);
		minVal.y = min(minVal.y, i.y);
	}

	GameObject* character = new GameObject();

	float r = rand() / (float)RAND_MAX;


	AABBVolume* volume = new AABBVolume(Vector3(0.3, 0.9f, 0.3) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform().SetWorldScale(Vector3(meshSize, meshSize, meshSize));
	character->GetTransform().SetWorldPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), r > 0.5f ? charA : charB, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddAppleToWorld(const Vector3& position) {
	GameObject* apple = new GameObject("apple");

	SphereVolume* volume = new SphereVolume(0.8f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform().SetWorldScale(Vector3(5, 5, 5));
	apple->GetTransform().SetWorldPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), appleMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));
	apple->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0));
}

//void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
//	float sphereRadius = 1.0f;
//	Vector3 cubeDims = Vector3(1, 1, 1);
//
//	for (int x = 0; x < numCols; ++x) {
//		for (int z = 0; z < numRows; ++z) {
//			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
//
//			if (rand() % 2) {
//				AddCubeToWorld(position, cubeDims);
//			}
//			else {
//				AddSphereToWorld(position, sphereRadius);
//			}
//		}
//	}
//	//AddFloorToWorld(Vector3(0, -10, 0));
//}

//void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
//	for (int x = 1; x < numCols+1; ++x) {
//		for (int z = 1; z < numRows+1; ++z) {
//			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
//			AddCubeToWorld(position, cubeDims, 1.0f);
//		}
//	}
//	AddFloorToWorld(Vector3(0, -2, 0));
//}

void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(3, 3, 3);

	float	invCubeMass = 5;
	int		numLinks = 4;
	float	maxDistance = 10;
	float	cubeDistance = 10;

	Vector3 startPos = Vector3(-20, 10, -20);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), Vector3(2,10,2), 0);

	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), Vector3(2, 10, 2), 0);

	GameObject* previous = start;

	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}

	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

void TutorialGame::SimpleGJKTest() {
	Vector3 dimensions = Vector3(5, 5, 5);
	Vector3 floorDimensions = Vector3(100, 2, 100);

	GameObject* fallingCube = AddCubeToWorld(Vector3(0, 20, 0), dimensions, 10.0f);
	GameObject* newFloor = AddCubeToWorld(Vector3(0, 0, 0), floorDimensions, 0.0f);

	delete fallingCube->GetBoundingVolume();
	delete newFloor->GetBoundingVolume();

	fallingCube->SetBoundingVolume((CollisionVolume*)new OBBVolume(dimensions));
	newFloor->SetBoundingVolume((CollisionVolume*)new OBBVolume(floorDimensions));

}

