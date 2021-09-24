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
	
	a3_SpatialPose.inl
	Implementation of inline spatial pose operations.
*/


#ifdef __ANIMAL3D_SPATIALPOSE_H
#ifndef __ANIMAL3D_SPATIALPOSE_INL
#define __ANIMAL3D_SPATIALPOSE_INL


//-----------------------------------------------------------------------------

// set rotation values for a single node pose
inline a3i32 a3spatialPoseSetRotation(a3_SpatialPose* spatialPose, const a3f32 rx_degrees, const a3f32 ry_degrees, const a3f32 rz_degrees)
{
	if (spatialPose)
	{
		spatialPose->rotation.x = rx_degrees;
		spatialPose->rotation.y = ry_degrees;
		spatialPose->rotation.z = rz_degrees;
	}
	return -1;
}

// scale
inline a3i32 a3spatialPoseSetScale(a3_SpatialPose* spatialPose, const a3f32 sx, const a3f32 sy, const a3f32 sz)
{
	if (spatialPose)
	{
		spatialPose->scale.x = sx;
		spatialPose->scale.y = sy;
		spatialPose->scale.z = sz;
	}
	return -1;
}

// translation
inline a3i32 a3spatialPoseSetTranslation(a3_SpatialPose* spatialPose, const a3f32 tx, const a3f32 ty, const a3f32 tz)
{
	if (spatialPose)
	{
		spatialPose->translation.x = tx;
		spatialPose->translation.y = ty;
		spatialPose->translation.z = tz;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// reset single node pose
inline a3i32 a3spatialPoseReset(a3_SpatialPose* spatialPose)
{
	if (spatialPose)
	{
		spatialPose->transform = a3mat4_identity;
		spatialPose->rotation = a3vec3_zero;
		spatialPose->translation = a3vec3_zero;
		spatialPose->scale = a3vec3_one;

		// done
		return 0;
	}
	return -1;
}

// convert single node pose to matrix
inline a3i32 a3spatialPoseConvert(a3mat4* mat_out, const a3_SpatialPose* spatialPose_in, const a3_SpatialPoseChannel channel, const a3_SpatialPoseEulerOrder order)
{
	if (mat_out && spatialPose_in)
	{
		// RST -> mat4
		// M = T * ((R2 * R1 * R0) * S)
		
		//		|		  tx|
		// M =  |   RS	  ty|
		//		|		  tz|
		//		|0  0  0   1|

		// scale matrix
		a3mat3* tempScale;
		a3real3x3Set(tempScale->m, spatialPose_in->scale.x, 0, 0,
							0, spatialPose_in->scale.y, 0,
							0, 0, spatialPose_in->scale.z);

		// rotation matrices
		a3mat3* tempXRot, * tempYRot, * tempZRot;
		a3real3x3Set(tempXRot->m, 1, 0, 0,
						    0, a3cosd(spatialPose_in->rotation.x), -a3sind(spatialPose_in->rotation.x),
						    0, a3sind(spatialPose_in->rotation.x), a3cosd(spatialPose_in->rotation.x) );

		a3real3x3Set(tempYRot->m, a3cosd(spatialPose_in->rotation.y), 0, a3sind(spatialPose_in->rotation.y),
							0,									1,	0,
							-a3sind(spatialPose_in->rotation.y), 0, a3cosd(spatialPose_in->rotation.y) );

		a3real3x3Set(tempZRot->m, a3cosd(spatialPose_in->rotation.z), -a3sind(spatialPose_in->rotation.z), 0,
							a3sind(spatialPose_in->rotation.z), a3cosd(spatialPose_in->rotation.z),  0,
							0,									0,									 1 );

		a3mat3 * tempScaleRot;
		a3real3x3Product(tempZRot->m, tempZRot->m, tempYRot->m);
		a3real3x3Product(tempZRot->m, tempZRot->m, tempXRot->m);
		a3real3x3Product(tempScaleRot->m, tempZRot->m, tempScale->m);

		//a3real4x4SetIdentity(mat_out->m);
		a3real4x4SetReal3x3(mat_out->m, tempScaleRot->m);
		mat_out->m03 = spatialPose_in->translation.x;
		mat_out->m13 = spatialPose_in->translation.y;
		mat_out->m23 = spatialPose_in->translation.y;

		
		//a3real4x4MulTransform(mat_out->m, tempScaleRot->m);
	}
	return -1;
}

// copy operation for single node pose
inline a3i32 a3spatialPoseCopy(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_in)
{
	if (spatialPose_out && spatialPose_in)
	{
		spatialPose_out->rotation = spatialPose_in->rotation;
		spatialPose_out->scale = spatialPose_in->scale;
		spatialPose_out->translation = spatialPose_in->translation;
	}
	return -1;
}

// concatenate/combine
inline a3i32 a3spatialPoseConcat(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_lh, const a3_SpatialPose* spatialPose_rh)
{
	if (spatialPose_out && spatialPose_lh && spatialPose_rh)
	{
		//spatialPose_out->transform; // no >:(
		// Euler: validate(lh + rh) - > constrain sum to rotational domain
		spatialPose_out->rotation.x = a3trigValid_sind(spatialPose_lh->rotation.x + spatialPose_rh->rotation.x);
		spatialPose_out->rotation.y = a3trigValid_sind(spatialPose_lh->rotation.y + spatialPose_rh->rotation.y);
		spatialPose_out->rotation.z = a3trigValid_sind(spatialPose_lh->rotation.z + spatialPose_rh->rotation.z);

		// Scale: comp(lh * rh)  -> component wise multiplication
		spatialPose_out->scale.x = spatialPose_lh->scale.x * spatialPose_rh->scale.x; 
		spatialPose_out->scale.y = spatialPose_lh->scale.y * spatialPose_rh->scale.y; 
		spatialPose_out->scale.z = spatialPose_lh->scale.z * spatialPose_rh->scale.z; 
		
		// Translate: addition (lh + rh)
		spatialPose_out->translation.x = spatialPose_lh->translation.x + spatialPose_rh->translation.x; 
		spatialPose_out->translation.y = spatialPose_lh->translation.y + spatialPose_rh->translation.y; 
		spatialPose_out->translation.z = spatialPose_lh->translation.z + spatialPose_rh->translation.z; 
		return 0;
	}
	return -1;
}

// lerp
inline a3i32 a3spatialPoseLerp(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose0, const a3_SpatialPose* spatialPose1, const a3real u)
{
	if (spatialPose_out && spatialPose0 && spatialPose1)
	{
		//spatialPose_out->transform; // no >:(
		// Euler: lerp (p0, p1, u) -> (p1 - p0)u + p0
		a3real3Lerp(spatialPose_out->rotation.v, spatialPose0->rotation.v, spatialPose1->rotation.v, u);

		// lerp is okay... but really... exp_lerp() -> (p1(p0^-1))^u * p0
		a3real3Lerp(spatialPose_out->scale.v, spatialPose0->scale.v, spatialPose1->scale.v, u);

		// lerp (p0, p1, u)
		a3real3Lerp(spatialPose_out->translation.v, spatialPose0->translation.v, spatialPose1->translation.v, u);

		return 0;
	}
	return -1;
}

//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_SPATIALPOSE_INL
#endif	// __ANIMAL3D_SPATIALPOSE_H