#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "GameObject.h"
#include "CollisionDetection.h"
#include "../../Common/Quaternion.h"

#include "Constraint.h"
#include "../CSC8503Common/PositionConstraint.h"

#include "Debug.h"

#include <functional>
using namespace NCL;
using namespace CSC8503;

PhysicsSystem::PhysicsSystem(GameWorld& g) : gameWorld(g)	{
	applyGravity	= false;
	useBroadPhase	= true;	
	dTOffset		= 0.0f;
	globalDamping	= 0.95f;
	SetGravity(Vector3(0.0f, -15.8f, 0.0f));
}

PhysicsSystem::~PhysicsSystem()	{
}

void PhysicsSystem::SetGravity(const Vector3& g) {
	gravity = g;
}

/*

If the 'game' is ever reset, the PhysicsSystem must be
'cleared' to remove any old collisions that might still
be hanging around in the collision list. If your engine
is expanded to allow objects to be removed from the world,
you'll need to iterate through this collisions list to remove
any collisions they are in.

*/
void PhysicsSystem::Clear() {
	allCollisions.clear();
}

/*

This is the core of the physics engine update

*/

//int constraintIterationCount = 10;
//
//void PhysicsSystem::Update(float dt) {
//	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::I)) {
//		constraintIterationCount--;
//		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
//	}
//	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::O)) {
//		constraintIterationCount++;
//		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
//	}
//	triggers.clear();
//	dTOffset += dt; //We accumulate time delta here - there might be remainders from previous frame!
//
//	float idealFrameRate = 60;							//this is the framerate we'd like to maintain kplzthx
//	float idealIterations = idealFrameRate * 2;			//we want n per second;
//
//	int perFrameIts = (int)(idealIterations / idealFrameRate);
//
//	float iterationDt = 1.0f / idealIterations;				//So each iteration we'll get an advance of this time
//	float currentFrameRate = 1.0f / dt;							//what's our current fps?
//
//	int realIterations = (int)(dTOffset / iterationDt);				//how many iterations of the desired dt can we actually fit in our timestep?
//
//	if (currentFrameRate < idealFrameRate * 0.5f) {
//		iterationDt = dTOffset; //run one big update if the framerate tanks
//	}
//	else if (realIterations > perFrameIts + 1) { //+1 as we sometimes accumulate an extra frame to take up the leftover time
//		//UH OH, we're accumulating too much time from somewhere, half the iteration count
//		//Probably caused by not being able to quite maintain the fps
//		iterationDt *= 2;
//	}
//
//	if (useBroadPhase) {
//		UpdateObjectAABBs();
//	}
//	int iteratorCount = 0;
//	while (dTOffset >= iterationDt) {
//		IntegrateAccel(iterationDt); //Update accelerations from external forces
//		if (useBroadPhase) {
//			BroadPhase();
//			NarrowPhase();
//		}
//		else {
//			BasicCollisionDetection();
//		}
//
//		//This is our simple iterative solver - 
//		//we just run things multiple times, slowly moving things forward
//		//and then rechecking that the constraints have been met		
//		float constraintDt = iterationDt / (float)constraintIterationCount;
//		for (int i = 0; i < constraintIterationCount; ++i) {
//			UpdateConstraints(constraintDt);
//		}
//		IntegrateVelocity(iterationDt); //update positions from new velocity changes
//
//		dTOffset -= iterationDt;
//		iteratorCount++;
//	}
//
//	ClearForces();	//Once we've finished with the forces, reset them to zero
//
//	UpdateCollisionList(); //Remove any old collisions
//}


void PhysicsSystem::Update(float dt) {
	GameTimer testTimer;
	triggers.clear();

	testTimer.GetTimeDeltaSeconds();

	frameDT = dt;

	dTOffset += dt; //We accumulate time delta here - there might be remainders from previous frame!

	float iterationDt = 1.0f / 120.0f; //Ideally we'll have 120 physics updates a second 

	if (dTOffset > 8 * iterationDt) { //the physics engine cant catch up!
		iterationDt = 1.0f / 15.0f; //it'll just have to run bigger timesteps...
		//std::cout << "Setting physics iterations to 15" << iterationDt << std::endl;
	}
	else if (dTOffset > 4  * iterationDt) { //the physics engine cant catch up!
		iterationDt = 1.0f / 30.0f; //it'll just have to run bigger timesteps...
		//std::cout << "Setting iteration dt to 4 case " << iterationDt << std::endl;
	}
	else if (dTOffset > 2* iterationDt) { //the physics engine cant catch up!
		iterationDt = 1.0f / 60.0f; //it'll just have to run bigger timesteps...
		//std::cout << "Setting iteration dt to 2 case " << iterationDt << std::endl;
	}
	else {
		//std::cout << "Running normal update " << iterationDt << std::endl;
	}

	int constraintIterationCount = 10;
	iterationDt = dt;

	if (useBroadPhase) {
		UpdateObjectAABBs();
	}

	while(dTOffset > iterationDt *0.5) {
		IntegrateAccel(iterationDt); //Update accelerations from external forces
		if (useBroadPhase) {
			BroadPhase();
			NarrowPhase();
		}
		else {
			BasicCollisionDetection();
		}
		//This is our simple iterative solver - 
		//we just run things multiple times, slowly moving things forward
		//and then rechecking that the constraints have been met		
		float constraintDt = iterationDt /  (float)constraintIterationCount;

		for (int i = 0; i < constraintIterationCount; ++i) {
			UpdateConstraints(constraintDt);	
		}
		
		IntegrateVelocity(iterationDt); //update positions from new velocity changes

		dTOffset -= iterationDt; 
	}
	ClearForces();	//Once we've finished with the forces, reset them to zero
	UpdateCollisionList(); //Remove any old collisions
	//std::cout << iteratorCount << " , " << iterationDt << std::endl;
	float time = testTimer.GetTimeDeltaSeconds();
	//std::cout << "Physics time taken: " << time << std::endl;
}

/*
Later on we're going to need to keep track of collisions
across multiple frames, so we store them in a set.

The first time they are added, we tell the objects they are colliding.
The frame they are to be removed, we tell them they're no longer colliding.

From this simple mechanism, we we build up gameplay interactions inside the
OnCollisionBegin / OnCollisionEnd functions (removing health when hit by a 
rocket launcher, gaining a point when the player hits the gold coin, and so on).
*/
void PhysicsSystem::UpdateCollisionList() {
	for (std::set<CollisionDetection::CollisionInfo>::iterator i = allCollisions.begin(); i != allCollisions.end(); ) {
		if ((*i).framesLeft == numCollisionFrames) {
			i->a->OnCollisionBegin(i->b);
			i->b->OnCollisionBegin(i->a);
		}
		(*i).framesLeft = (*i).framesLeft - 1;
		if ((*i).framesLeft < 0) {
			i->a->OnCollisionEnd(i->b);
			i->b->OnCollisionEnd(i->a);
			i = allCollisions.erase(i);
		}
		else {
			++i;
		}
	}
}

void PhysicsSystem::UpdateObjectAABBs() {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		(*i)->UpdateBroadphaseAABB();
	}
}

/*

This is how we'll be doing collision detection in tutorial 4.
We step thorugh every pair of objects once (the inner for loop offset 
ensures this), and determine whether they collide, and if so, add them
to the collision set for later processing. The set will guarantee that
a particular pair will only be added once, so objects colliding for
multiple frames won't flood the set with duplicates.
*/
void PhysicsSystem::BasicCollisionDetection() {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		if ((*i)->GetPhysicsObject() == nullptr) {
			continue;
		}
		for (auto j = i + 1; j != last; ++j) {
			if ((*j)->GetPhysicsObject() == nullptr) {
				continue;
			}
		
			CollisionDetection::CollisionInfo info;

			if (CollisionDetection::ObjectIntersection(*i, *j, info)) {

				if ((*i)->GetName() == "trigger" || (*j)->GetName() == "trigger") {

					info.framesLeft = numCollisionFrames;
					if ((*i)->GetName() == "trigger") {
						triggers.insert(*j);
					}
					else {
						triggers.insert(*i);
					}
				}
				if ((*i)->GetName() == "speedBall" || (*j)->GetName() == "speedBall") {

					info.framesLeft = numCollisionFrames;
					if ((*i)->GetName() == "speedBall") {
						triggers.insert(*j);
					}
					else {
						triggers.insert(*i);
					}
				}
				if ((*i)->GetName() == "keeper" || (*j)->GetName() == "keeper") {
					info.framesLeft = numCollisionFrames;
					if ((*i)->GetName() == "speedBall") {
						triggers.insert(*j);
					}
					else {
						triggers.insert(*i);
					}
				}
				else {
					ImpulseResolveCollision(*info.a, *info.b, info.point);
					info.framesLeft = numCollisionFrames;
					allCollisions.insert(info);
				}
			}

		}
	}
}

//void PhysicsSystem::SpringResolve(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const {
//	PhysicsObject* physA = a.GetPhysicsObject();
//	PhysicsObject* physB = b.GetPhysicsObject();
//	float totalMass = physA->GetInverseMass() + physB->GetInverseMass();
//
//	if (totalMass == 0.0f) {
//		return;
//	}
//}
/*

In tutorial 5, we start determining the correct response to a collision,
so that objects separate back out. 

*/
void PhysicsSystem::ImpulseResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const {

	PhysicsObject* physA = a.GetPhysicsObject();
	PhysicsObject* physB = b.GetPhysicsObject();
	
	Transform& transformA = a.GetTransform();
	Transform& transformB = b.GetTransform();

	float totalMass = physA->GetInverseMass() + physB->GetInverseMass();

	if (totalMass == 0.0f) {
		return;
	}

	//separate them out using projection
	transformA.SetWorldPosition(transformA.GetWorldPosition() - (p.normal * p.penetration * (physA->GetInverseMass() / totalMass)));

	transformB.SetWorldPosition(transformB.GetWorldPosition() + (p.normal * p.penetration * (physB->GetInverseMass() / totalMass)));

	Vector3 relativeA = p.localA;
	Vector3 relativeB = p.localB;

	Vector3 angVelocityA = Vector3::Cross(physA->GetAngularVelocity(), relativeA);
	Vector3 angVelocityB = Vector3::Cross(physB->GetAngularVelocity(), relativeB);

	Vector3 fullVelocityA = physA->GetLinearVelocity() + angVelocityA;
	Vector3 fullVelocityB = physB->GetLinearVelocity() + angVelocityB;
		
	Vector3 contactVelocity = fullVelocityB - fullVelocityA;

	float impulseForce = Vector3::Dot(contactVelocity, p.normal)*((physA->GetElasticity())*(physB->GetElasticity()));

	if (impulseForce > 0) {
		return;
	}

	//work out the effect of inertia 
	Vector3 inertiaA =	Vector3::Cross(physA->GetInertiaTensor() * Vector3::Cross(relativeA, p.normal), relativeA);
	Vector3 inertiaB =	Vector3::Cross(physB->GetInertiaTensor() * 
						Vector3::Cross(relativeB, p.normal), relativeB);

	float angularEffect = Vector3::Dot(inertiaA + inertiaB, p.normal);

	float cRestitution = 0.66f; //disperse some kinectic energy

	float j = (-(1.0f + cRestitution) * impulseForce) / (totalMass + angularEffect);

	Vector3 fullImpulse = p.normal * j;

	physA->ApplyLinearImpulse(-fullImpulse);
	physB->ApplyLinearImpulse(fullImpulse);

	physA->ApplyAngularImpulse(Vector3::Cross(relativeA, -fullImpulse));
	physB->ApplyAngularImpulse(Vector3::Cross(relativeB,  fullImpulse));
}

/*

Later, we replace the BasicCollisionDetection method with a broadphase
and a narrowphase collision detection method. In the broad phase, we
split the world up using an acceleration structure, so that we can only
compare the collisions that we absolutely need to. 

*/

void PhysicsSystem::BroadPhase() {
	broadphaseCollisions.clear();
	QuadTree<GameObject*> tree(Vector2(1024, 1024), 7, 6);
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;

	gameWorld.GetObjectIterators(first, last);
	for (auto i = first; i != last; ++i) {
		Vector3 halfSizes;
		if (!(*i)->GetBroadphaseAABB(halfSizes)) {
			continue;
		}
		Vector3 pos = (*i)->GetConstTransform().GetWorldPosition();
		tree.Insert(*i, pos, halfSizes);
	}

	tree.OperateOnContents([&](std::list<QuadTreeEntry<GameObject*>>& data) {
		CollisionDetection::CollisionInfo info;
		for (auto i = data.begin(); i != data.end(); ++i) {
			for (auto j = std::next(i); j != data.end(); ++j) {
				info.a = min((*i).object, (*j).object);
				info.b = max((*i).object, (*j).object);

				broadphaseCollisions.insert(info);
			}
		}
	});
}

/*

The broadphase will now only give us likely collisions, so we can now go through them,
and work out if they are truly colliding, and if so, add them into the main collision list
*/
void PhysicsSystem::NarrowPhase() {
	for (std::set<CollisionDetection::CollisionInfo>::iterator i = broadphaseCollisions.begin(); i != broadphaseCollisions.end(); ++i) {
		CollisionDetection::CollisionInfo info = *i;
		if (CollisionDetection::ObjectIntersection(info.a, info.b, info)) {
			if (info.a->GetName() == "trigger" || info.b->GetName() == "trigger") {
				
				info.framesLeft = numCollisionFrames;
				if (info.a->GetName() == "trigger") {
					triggers.insert(info.b);
				}
				else {
					triggers.insert(info.a);
				}
			}
			info.framesLeft = numCollisionFrames;
			ImpulseResolveCollision(*info.a, *info.b, info.point);
			allCollisions.insert(info);
		}
	}

}

/*
Integration of acceleration and velocity is split up, so that we can
move objects multiple times during the course of a PhysicsUpdate,
without worrying about repeated forces accumulating etc. 

This function will update both linear and angular acceleration,
based on any forces that have been accumulated in the objects during
the course of the previous game frame.
*/
void PhysicsSystem::IntegrateAccel(float dt) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		PhysicsObject* object = (*i)->GetPhysicsObject();
		if (object == nullptr) {
			continue; // no physics object for this game object
		}
		float inverseMass = object->GetInverseMass();

		Vector3 linearVel = object->GetLinearVelocity();
		Vector3 force = object->GetForce();
		Vector3 accel = force * inverseMass;

		if (applyGravity && inverseMass > 0) {
			accel += gravity; //dont move infinitely heavy things
		}
		linearVel += accel * dt; // integrate accel!
		object->SetLinearVelocity(linearVel);

		// angular stuff
		Vector3 torque = object->GetTorque();
		Vector3 angVel = object->GetAngularVelocity();

		object->UpdateInertiaTensor(); // update tensor vs orientation

		Vector3 angAccel = object->GetInertiaTensor() * torque;

		angVel += angAccel * dt; //integrate angular accel
		object->SetAngularVelocity(angVel);
	}
}
/*
This function integrates linear and angular velocity into
position and orientation. It may be called multiple times
throughout a physics update, to slowly move the objects through
the world, looking for collisions.
*/
void PhysicsSystem::IntegrateVelocity(float dt) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);
	float dampingFactor = 1.0f - 0.95f;
	float frameDamping = powf(dampingFactor, dt);

	for (auto i = first; i != last; ++i) {
		PhysicsObject* object = (*i)->GetPhysicsObject();
		if (object == nullptr) {
			continue;
		}
		Transform& transform = (*i)->GetTransform();
		//position stuff
		Vector3 position = transform.GetLocalPosition();
		Vector3 linearVel = object->GetLinearVelocity();
		position += linearVel * dt;
		transform.SetLocalPosition(position);
		//transform.SetWorldPosition(position);

		//linear Damping
		linearVel = linearVel * frameDamping;
		object->SetLinearVelocity(linearVel);

		//orientation stuff
		Quaternion orientation = transform.GetLocalOrientation();
		Vector3 angVel = object->GetAngularVelocity();

		orientation = orientation + (Quaternion(angVel * dt * 0.5f, 0.0f) * orientation);
		orientation.Normalise();

		transform.SetLocalOrientation(orientation);

		//damp the angular velocity too
		angVel = angVel * frameDamping;
		object->SetAngularVelocity(angVel);
	}
}

/*
Once we're finished with a physics update, we have to
clear out any accumulated forces, ready to receive new
ones in the next 'game' frame.
*/
void PhysicsSystem::ClearForces() {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		//Clear our object's forces for the next frame
		(*i)->GetPhysicsObject()->ClearForces();
	}
}


/*

As part of the final physics tutorials, we add in the ability
to constrain objects based on some extra calculation, allowing
us to model springs and ropes etc. 

*/
void PhysicsSystem::UpdateConstraints(float dt) {
	std::vector<Constraint*>::const_iterator first;
	std::vector<Constraint*>::const_iterator last;
	gameWorld.GetConstraintIterators(first, last);
	for (auto i = first; i != last; ++i) {
		(*i)->UpdateConstraint(dt);
	}
}