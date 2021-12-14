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
	
	a3_Kinematics.inl
	Implementation of kinematics solvers.
*/


#ifdef __ANIMAL3D_KINEMATICS_H
#ifndef __ANIMAL3D_KINEMATICS_INL
#define __ANIMAL3D_KINEMATICS_INL


//-----------------------------------------------------------------------------

// update rigidbody
inline a3i32 a3RigidbodyUpdate(a3_Rigidbody* rigidbody_out)
{
	// change rigidbody velocity using its acceleration
	a3real3Sum(rigidbody_out->velocity.v, rigidbody_out->velocity.v, rigidbody_out->acceleration.v);

	return -1;
}


// check sphere/sphere collision
inline a3i32 a3SphereSphereCollide(a3_SphereCollider* sphere, a3_SphereCollider* sphereTwo)
{
	a3real radiiSum = sphere->radius + sphereTwo->radius; // combined length
	a3vec3 collisionVec;
	a3real3Diff(collisionVec.v, sphereTwo->position.v, sphere->position.v); // vector from sphere to sphereTwo

	if (a3real3LengthSquared(collisionVec.v) <= (radiiSum * radiiSum))
	{
		// colliding

	}
	return -1;
}

// formula for reflected vector taken from: https://www.3dkingdoms.com/weekly/weekly.php?a=2 

// check sphere/plane collision
inline a3i32 a3SpherePlaneCollide(a3_SphereCollider* sphere, a3_PlaneCollider* plane)
{
	a3vec3 sphereToPlane, projVec;
	a3real3Diff(sphereToPlane.v, sphere->position.v, plane->position.v); // vector from the sphere's pos to the plane's pos
	a3real3Projected(projVec.v, sphereToPlane.v, plane->normal.v); // gives vector from plane to sphere parrallel to the plane's normal

	if (a3real3LengthSquared(projVec.v) <= sphere->radius * sphere->radius)
	{
		// colliding -> reflect velocity of sphere
		// R = b * (-2*(V dot N) * N + V)
		a3vec3 newVel;
		newVel = plane->normal.xyz;
		a3real3MulS(newVel.v, a3real3Dot(sphere->rigidbody->velocity.v, plane->normal.v)); // (V dot N) * N
		a3real3Sub(newVel.v, sphere->rigidbody->velocity.v); //+ V
		a3real3MulS(newVel.v, -2);// * -2
		a3real3MulS(newVel.v, plane->bounce); // * bounciness

		// adding to velocity
		a3real3Add(sphere->rigidbody->velocity.v, newVel.v);
	}

	return -1;
}

//-----------------------------------------------------------------------------

// single FK solver
inline void a3kinematicsSolveForwardSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
	a3real4x4Product(hierarchyState->objectSpace->pose[index].transformMat.m,
		hierarchyState->objectSpace->pose[parentIndex].transformMat.m,
		hierarchyState->localSpace->pose[index].transformMat.m);
}

// partial FK solver
inline a3i32 a3kinematicsSolveForwardPartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// ****TO-DO: implement forward kinematics algorithm
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- object matrix = parent object matrix * local matrix
		//		- else
		//			- copy local matrix to object matrix
		const a3_HierarchyNode* itr = hierarchyState->hierarchy->nodes + firstIndex;
		const a3_HierarchyNode* const end = itr + nodeCount;
		for (; itr < end; ++itr)
		{
			if (itr->parentIndex >= 0)
				a3kinematicsSolveForwardSingle(hierarchyState, itr->index, itr->parentIndex);
			else
				hierarchyState->objectSpace->pose[itr->index] = hierarchyState->localSpace->pose[itr->index];
		}
		return (a3i32)(end - itr);
	}
	return -1;
}


//-----------------------------------------------------------------------------

// single IK solver
inline void a3kinematicsSolveInverseSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
	a3real4x4Product(hierarchyState->localSpace->pose[index].transformMat.m,
		hierarchyState->objectSpaceInv->pose[parentIndex].transformMat.m,
		hierarchyState->objectSpace->pose[index].transformMat.m);
}

// partial IK solver
inline a3i32 a3kinematicsSolveInversePartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// ****TO-DO: implement inverse kinematics algorithm
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- local matrix = inverse parent object matrix * object matrix
		//		- else
		//			- copy object matrix to local matrix
		const a3_HierarchyNode* itr = hierarchyState->hierarchy->nodes + firstIndex;
		const a3_HierarchyNode* const end = itr + nodeCount;
		for (; itr < end; ++itr)
		{
			if (itr->parentIndex >= 0)
				a3kinematicsSolveInverseSingle(hierarchyState, itr->index, itr->parentIndex);
			else
				hierarchyState->localSpace->pose[itr->index] = hierarchyState->objectSpace->pose[itr->index];
		}
		return (a3i32)(end - itr);
	}
	return -1;
}


//-----------------------------------------------------------------------------

// FK solver
inline a3i32 a3kinematicsSolveForward(const a3_HierarchyState *hierarchyState)
{
	if (hierarchyState && hierarchyState->hierarchy)
		return a3kinematicsSolveForwardPartial(hierarchyState, 0, hierarchyState->hierarchy->numNodes);
	return -1;
}


//-----------------------------------------------------------------------------

// IK solver
inline a3i32 a3kinematicsSolveInverse(const a3_HierarchyState *hierarchyState)
{
	if (hierarchyState && hierarchyState->hierarchy)
		return a3kinematicsSolveInversePartial(hierarchyState, 0, hierarchyState->hierarchy->numNodes);
	return -1;
}


//-----------------------------------------------------------------------------

// EULER UML
//  +fIntegrateEuler(x : ftype, dx_dt : ftype, dt : float) : ftype
inline a3real a3EulerIntegration(a3real x, a3real dx_dt, a3real dt)
{
	//a3real result = x + (dx_dt / dt)*dt;
	a3real result = x + (dx_dt)*dt;
	return result;
}

// KINEMATIC UML
//  +fIntegrateKinematic(x : ftype, dx_dt : ftype, d2x_dt2 : ftype, dt : float) : ftype
inline a3real a3KinematicIntegration(a3real x, a3real dx_dt, a3real d2x_dt2, a3real dt)
{
	//a3real dx = a3EulerIntegration(x, d2x_dt2, dt);
	a3real result = a3EulerIntegration(x, dx_dt, dt) + (d2x_dt2) * ((dt * dt) / a3real_two);
	return result;
}


// INTERP UML
// +fIntegrateInterpolated(x : ftype, xc : ftype, u : float) : ftype
inline a3real a3InterpBasedIntegration(a3real x, a3real xc, a3real u)
{
	a3real result = a3lerp(x, xc, u);
	return result;
}

//-----------------------------------------------------------------------------
#endif	// !__ANIMAL3D_KINEMATICS_INL
#endif	// __ANIMAL3D_KINEMATICS_H