/*
	Copyright 2011-2020 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein
	
	a3_Kinematics.h
	Hierarchical kinematics solvers.
*/

#ifndef __ANIMAL3D_KINEMATICS_H
#define __ANIMAL3D_KINEMATICS_H


#include "a3_HierarchyState.h"


//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
typedef struct a3_SphereCollider	a3_SphereCollider;
typedef struct a3_PlaneCollider		a3_PlaneCollider;
typedef struct a3_Rigidbody			a3_Rigidbody;
typedef struct a3_SphereManager		a3_SphereManager;
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// physics - rigidbody and collisions

// single sphere collider at a point in space
struct a3_SphereCollider
{
	a3vec3 position;
	a3real radius;

	a3_Rigidbody * rigidbody;
};

// holds all of the spheres
struct a3_SphereManager
{
	a3_SphereCollider* sphere;
	a3i32 numSpheres;

	a3boolean gravity;
};

// collider for a plane at a point in space with a normal vector
struct a3_PlaneCollider
{
	a3vec3 position;
	a3vec4 normal;
	a3real bounce;
};

// rigidbody to control physics movementt
struct a3_Rigidbody
{
	a3vec3 velocity, acceleration;
};

// initialize sphere collider
a3i32 a3SphereColliderCreate(a3_SphereCollider *collider_out, a3vec3 position, a3real radius);

// initialize sphere manager
a3i32 a3SphereManagerCreate(a3_SphereManager* manager_out, const a3i32 numSpheres);

// free data
a3ret a3SphereManagerRelease(a3_SphereManager* manager);

// initialize plane collider
a3i32 a3PlaneColliderCreate(a3_PlaneCollider* collider_out, a3vec3 position, a3vec4 normal, a3real bounce);

// initialize rigidbody 
//a3i32 a3RigidbodyCreate(a3_Rigidbody *rigidbody_out, a3vec3 velocity, a3vec3 acceleration);

// initialize rigidbody  with zero values
a3i32 a3RigidbodyCreate(a3_Rigidbody *rigidbody_out);

// update rigidbody
a3i32 a3RigidbodyUpdate(a3_Rigidbody *rigidbody_out, a3real dt);

// update sphere manager
a3i32 a3SphereUpdate(a3_SphereManager* manager_out, a3real dt);

// toggle gravity
a3i32 a3GravitySet(a3_SphereManager* manager_out, a3boolean doGravity, a3real gravity);

// check sphere/sphere collision
a3i32 a3SphereSphereCollide(a3_SphereCollider *sphere, a3_SphereCollider *sphereTwo);

// check sphere/plane collision
a3i32 a3SpherePlaneCollide(a3_SphereCollider *sphere, a3_PlaneCollider *plane);

//-----------------------------------------------------------------------------

// general forward kinematics: 
// given local transforms for hierarchy nodes, calculate object-space: 
//		if not root, 
//			object-space node = object-space parent * local-space node
//		else
//			object-space node = local-space node

// forward kinematics solver given an initialized hierarchy state
a3i32 a3kinematicsSolveForward(const a3_HierarchyState *hierarchyState);

// forward kinematics solver starting at a specified joint
a3i32 a3kinematicsSolveForwardPartial(const a3_HierarchyState *hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount);

// single node FK solver
void a3kinematicsSolveForwardSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex);

a3i32 a3kinematicsSolveInverseRagdoll(const a3_HierarchyState* hierarchyState);
a3i32 a3kinematicsSolveInverseRagdollPartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount);
a3i32 a3kinematicsSolveInverseRagdollArm(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex);
a3i32 a3kinematicsSolveInverseRagdollLeg(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex);


//-----------------------------------------------------------------------------

// general inverse kinematics: 
// given object transforms for hierarchy nodes, calculate local-space: 
//		if not root, 
//			local-space node = inverse object-space parent * object-space node
//		else
//			local-space node = object-space node

// inverse kinematics solver given an initialized hierarchy state
a3i32 a3kinematicsSolveInverse(const a3_HierarchyState *hierarchyState);

// inverse kinematics solver starting at a specified joint
a3i32 a3kinematicsSolveInversePartial(const a3_HierarchyState *hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount);

// single node IK solver
void a3kinematicsSolveInverseSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex);


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// locomotion algorithms:
// euler integration: implement euler's method of integration for the target var, given its derivative
//	-> begins with current state of some variable x
//	-> adds its current derivative (rate of change at time t)
//	-> scaled by the current differential (delta-time or dt)
a3real a3EulerIntegration(a3real x, a3real dx_dt, a3real dt);


// kinematic integration: Implement kinematic integration for the target variable, given its first and second derivatives
//	-> begins with current state of some variable x
//	-> adds its current derivative (rate of change at time t) scaled by the current differential (delta-time or dt)
//	-> add its currnet second derivative (rate of change of rate of change at time t) scaled by half of the squared differential (dt) 
a3real a3KinematicIntegration(a3real x, a3real dx_dt, a3real d2x_dt2, a3real dt);


// interpolation-based integration: implement euler's method of integration for the target var, given its derivative
//	-> begins with current state of some variable x
//	-> adds the dif between the target and the current values
//	-> scaled by the interpolation parameter (between 0 and 1)
a3real a3InterpBasedIntegration(a3real x, a3real xc, a3real u);

//-----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_Kinematics.inl"


#endif	// !__ANIMAL3D_KINEMATICS_H