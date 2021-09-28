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
		spatialPose->rotate_euler.x = rx_degrees;
		spatialPose->rotate_euler.y = ry_degrees;
		spatialPose->rotate_euler.z = rz_degrees;
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
		spatialPose->rotate_quat = a3vec4_w;		// mul
		spatialPose->rotate_euler = a3vec4_zero;
		spatialPose->translation = a3vec4_zero;
		spatialPose->scale = a3vec4_one;

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
						    0, a3cosd(spatialPose_in->rotate_euler.x), -a3sind(spatialPose_in->rotate_euler.x),
						    0, a3sind(spatialPose_in->rotate_euler.x), a3cosd(spatialPose_in->rotate_euler.x) );

		a3real3x3Set(tempYRot->m, a3cosd(spatialPose_in->rotate_euler.y), 0, a3sind(spatialPose_in->rotate_euler.y),
							0,									1,	0,
							-a3sind(spatialPose_in->rotate_euler.y), 0, a3cosd(spatialPose_in->rotate_euler.y) );

		a3real3x3Set(tempZRot->m, a3cosd(spatialPose_in->rotate_euler.z), -a3sind(spatialPose_in->rotate_euler.z), 0,
							a3sind(spatialPose_in->rotate_euler.z), a3cosd(spatialPose_in->rotate_euler.z),  0,
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
		spatialPose_out->rotate_euler = spatialPose_in->rotate_euler;
		spatialPose_out->scale = spatialPose_in->scale;
		spatialPose_out->translation = spatialPose_in->translation;
	}
	return -1;
}

// concatenate/combine
inline a3i32 a3spatialPoseConcat(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_lh, const a3_SpatialPose* spatialPose_rh, const a3boolean usingQuaternions)
{
	if (spatialPose_out && spatialPose_lh && spatialPose_rh)
	{
		//spatialPose_out->transform; // no >:(
		if (usingQuaternions)
		{
			// Quat: (lh * rh) = (w_l + v_l)(w_r + v_r)
			//				   = (w_l*w_r - v_l . v_r) + (w_l*v_r + w_r*v_l + v_l x v_r)
		}
		
		else
		{
			// Euler: validate(lh + rh) - > constrain sum to rotational domain
			spatialPose_out->rotate_euler.x = a3trigValid_sind(spatialPose_lh->rotate_euler.x + spatialPose_rh->rotate_euler.x);
			spatialPose_out->rotate_euler.y = a3trigValid_sind(spatialPose_lh->rotate_euler.y + spatialPose_rh->rotate_euler.y);
			spatialPose_out->rotate_euler.z = a3trigValid_sind(spatialPose_lh->rotate_euler.z + spatialPose_rh->rotate_euler.z);
		}
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
inline a3i32 a3spatialPoseLerp(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose0, const a3_SpatialPose* spatialPose1, const a3real u, const a3boolean usingQuaternions)
{
	if (spatialPose_out && spatialPose0 && spatialPose1)
	{
		//spatialPose_out->transform; // no >:(
		if (usingQuaternions)
		{
			// Quat: (3 opts) slerp(q0, q1, u)
			// = (sin([1-t]y0q0 + sin([t]y)q1) / sin(y)
			// y = acos(q0 . q1)
			// (2) lerp - non-unit length -> uniform scale, s = |q|^2
			// (3) nlerp = normalize(lerp)
		}

		else
		{
			// Euler: lerp (p0, p1, u) -> (p1 - p0)u + p0
			a3real3Lerp(spatialPose_out->rotate_euler.v, spatialPose0->rotate_euler.v, spatialPose1->rotate_euler.v, u);
		}

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