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
	
	a3_Kinematics.c
	Implementation of kinematics solvers.
*/

#include "../a3_Kinematics.h"
#include <A3_DEMO/_animation/a3_KeyframeAnimationController.h>


//-----------------------------------------------------------------------------
// initialize sphere collider
a3i32 a3SphereColliderCreate(a3_SphereCollider* collider_out, a3vec3 position, a3real radius)
{
	collider_out->position = position;
	collider_out->radius = radius;
	a3RigidbodyCreate(collider_out->rigidbody);
	return -1;
}

// initialize plane collider
a3i32 a3PlaneColliderCreate(a3_PlaneCollider* collider_out, a3vec3 position, a3vec4 normal, a3real bounce)
{
	collider_out->position = position;
	collider_out->normal = normal;
	collider_out->bounce = bounce;
	return -1;
}

// initialize rigidbody  with zero values
a3i32 a3RigidbodyCreate(a3_Rigidbody* rigidbody_out)
{
	rigidbody_out->velocity = a3vec3_zero;
	rigidbody_out->acceleration = a3vec3_zero;

	rigidbody_out->gravity = false;
	return -1;
}


//-----------------------------------------------------------------------------
