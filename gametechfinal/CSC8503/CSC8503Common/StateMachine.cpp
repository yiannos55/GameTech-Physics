#include "StateMachine.h"
#include "State.h"
#include "StateTransition.h"

using namespace NCL::CSC8503;

StateMachine::StateMachine()
{
	activeState = nullptr;
}

StateMachine::~StateMachine()
{

}

void StateMachine::AddState(State* s) {
	allStates.emplace_back(s);
	if (activeState == nullptr) {
		activeState = s;
	}
}

void StateMachine::AddTransition(StateTransition* t) {
	allTransitions.insert(std::make_pair(t->GetSourceState(), t));
}

void StateMachine::Update() {
	if (activeState) {
		activeState->Update();
		//get the transition set starting from this state node
		std::pair<TransitionIterator, TransitionIterator> range = allTransitions.equal_range(activeState);
		//iterate throught them all
		for (auto& i = range.first; i != range.second; ++i) {
			if (i->second->CanTransition()) { //some transition is true
				State* newState = i->second->GetDestinationState();
				activeState = newState;
			}
		}
	}
}