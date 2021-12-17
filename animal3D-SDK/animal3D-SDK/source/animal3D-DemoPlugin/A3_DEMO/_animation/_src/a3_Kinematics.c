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
#include <stdlib.h>


//-----------------------------------------------------------------------------
// initialize sphere collider
a3i32 a3SphereColliderCreate(a3_SphereCollider* collider_out, a3vec3 position, a3real radius)
{
	collider_out->position = position;
	collider_out->radius = radius;
	a3RigidbodyCreate(collider_out);
	return -1;
}

// initialize sphere manager
a3i32 a3SphereManagerCreate(a3_SphereManager* manager_out, const a3i32 numSpheres, a3_HierarchyState* hierarchyState)
{
	if (manager_out && numSpheres)
	{
		if (!manager_out->sphere)
		{
			const a3ui32 dataSize = sizeof(a3_SphereCollider) * numSpheres;
			manager_out->sphere = (a3_SphereCollider*)malloc(dataSize);
			//memset(manager_out->numSpheres, 0, dataSize);
			manager_out->numSpheres = numSpheres;
			manager_out->gravity = false;


			for (a3i32 i = 0; i < numSpheres; i++)
			{
				a3vec3 pos = (manager_out->sphere + i)->position = hierarchyState->objectSpace->pose[i].translate.xyz; // unsure if obj space will work
				a3SphereColliderCreate(manager_out->sphere + i, pos, 1);
			}

			return 1;
		}
	}
	return -1;
}

// free data
a3ret a3SphereManagerRelease(a3_SphereManager* manager)
{
	if (manager)
	{
		if (manager->sphere)
		{
			free(manager->sphere);
			manager->sphere = 0;
			manager->numSpheres = 0;
			manager->gravity = false;
			return 1;
		}
	}
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
a3i32 a3RigidbodyCreate(a3_SphereCollider* collider_out)
{
	const a3ui32 rbSize = sizeof(a3_Rigidbody);
	collider_out->rigidbody = (a3_Rigidbody*)malloc(rbSize);

	collider_out->rigidbody->velocity = a3vec3_zero;
	collider_out->rigidbody->acceleration = a3vec3_zero;
	return -1;
}


//-----------------------------------------------------------------------------
