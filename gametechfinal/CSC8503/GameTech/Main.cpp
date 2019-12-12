#include "../../Common/Window.h"

#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

#include "../CSC8503Common/GameServer.h"
#include "../CSC8503Common/GameClient.h"

#include "../CSC8503Common/NavigationGrid.h"

#include "TutorialGame.h"
#include "NetworkedGame.h"

using namespace NCL;
using namespace CSC8503;

struct dataPassing {
	void* actualObject;
	float dt;
	Window* w;
	int* eventID;
};


void TestStateMachine(Window*w, TutorialGame* g ){
	StateMachine* testMachine = new StateMachine();


	StateFunc AFunc = [](void* g) {
		dataPassing* realData = (dataPassing*)g;
		float dt = realData->dt;

		if (dt > 1.0f) {
			std::cout << "Skipping large time delta" << std::endl;
			return; //must have hit a breakpoint or something to have a 1 second frame time!
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			realData->w->ShowConsole(true);
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			realData->w->ShowConsole(false);
		}

		//DisplayPathfinding();

		realData->w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));

		((TutorialGame*)realData->actualObject)->UpdateGame(dt);
		//if (((TutorialGame*)realData->actualObject)->collected) {
		//	*realData->eventID = 200;
		//}
		//*realData)++;
		//std::cout << "in state A" << std::endl;
	};
	StateFunc BFunc = [](void* data) {
		//int* realData = (int*)data;
		//(*realData)--;
		std::cout << "in state B" << std::endl;
	};
	int eventData = 0;

	dataPassing mainGame{ g, 0, w, &eventData };
	dataPassing dummyStuff{ nullptr , 0, nullptr,  &eventData };
	GenericState* stateA = new GenericState(AFunc, (void*)& mainGame); // tutorialGame object
	GenericState* stateB = new GenericState(BFunc, (void*)& dummyStuff);
	testMachine->AddState(stateA);
	testMachine->AddState(stateB);

	GenericTransition<int&, int>* transitionA =
		new GenericTransition<int&, int>(GenericTransition<int&, int>::GreaterThanTransition, eventData, 10, stateA, stateB); // if greater than10, a to b

	GenericTransition<int&, int>* transitionB =
		new GenericTransition<int&, int>(GenericTransition<int&, int>::EqualsTransition, eventData, 0, stateB, stateA); // if == 0, b to a

	testMachine->AddTransition(transitionA);
	testMachine->AddTransition(transitionB);

	while (w->UpdateWindow() && !Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) {
		// for all the possible states, that is just an example
		float dt = w->GetTimer()->GetTimeDeltaSeconds();
		mainGame.dt = dt;

		/*if (dt > 1.0f) {
			std::cout << "Skipping large time delta" << std::endl;
			continue; //must have hit a breakpoint or something to have a 1 second frame time!
		}*/


		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			// Set event data accordingly....  , eventData = ???
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			// Set event data accordingly....   eventData = ???
		}
		testMachine->Update(); //run the state machine
	}
	delete testMachine;
}

void TestNetworking() {
}

//vector<Vector3> testNodes;
//
//void TestPathfinding() {
//	NavigationGrid grid("TestGrid1.txt");
//
//	NavigationPath outPath;
//
//	Vector3 startPos(80, 0, 10);
//	Vector3 endPos(80, 0, 80);
//
//	bool found = grid.FindPath(startPos, endPos, outPath);
//
//	Vector3 pos;
//	while (outPath.PopWaypoint(pos)) {
//		testNodes.push_back(pos);
//	}
//}

//void DisplayPathfinding() {
//	for (int i = 1; i < testNodes.size(); ++i) {
//		Vector3 a = testNodes[i - 1];
//		Vector3 b = testNodes[i];
//
//		Debug::DrawLine(a + Vector3(0,10,0), b + Vector3(0, 10, 0), Vector4(0, 1, 0, 1));
//	}
//}



/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead. 

This time, we've added some extra functionality to the window class - we can
hide or show the 

*/
int main() {
	Window*w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720);

	if (!w->HasInitialised()) {
		return -1;
	}	

	//TestNetworking();
	//TestPathfinding();
	
	w->ShowOSPointer(false);
	w->LockMouseToWindow(true);
	TutorialGame* g = new TutorialGame();
	TestStateMachine(w, g);

#if 0

	while (w->UpdateWindow() && !Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) {
		float dt = w->GetTimer()->GetTimeDeltaSeconds();

		if (dt > 1.0f) {
			std::cout << "Skipping large time delta" << std::endl;
			continue; //must have hit a breakpoint or something to have a 1 second frame time!
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			w->ShowConsole(true);
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			w->ShowConsole(false);
		}

		//DisplayPathfinding();

		w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));

		g->UpdateGame(dt);
	}
	Window::DestroyGameWindow();
#else
	//TestStateMachine(w, g);
#endif
}