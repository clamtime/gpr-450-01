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
	
	a3_HierarchyStateBlend.inl
	Implementation of inline hierarchical blend operations.
*/


#ifdef __ANIMAL3D_HIERARCHYSTATEBLEND_H
#ifndef __ANIMAL3D_HIERARCHYSTATEBLEND_INL
#define __ANIMAL3D_HIERARCHYSTATEBLEND_INL


//-----------------------------------------------------------------------------

inline a3_BlendConstruct a3BlendConstruct(a3_SpatialPose* out, a3vec4 const r, a3vec4 const s, a3vec4 const t)
{
	out->angles = r;
	out->scale = s;
	out->translation = t;
}

inline a3_BlendCopy a3BlendCopy(a3_SpatialPose* lhsout, a3_SpatialPose const* rhs)
{
	lhsout->angles	     = rhs->angles;
	lhsout->orientation = rhs->orientation;
	lhsout->scale       = rhs->scale;
	lhsout->transform   = rhs->transform;
	lhsout->translation = rhs->translation;
}

inline a3_BlendNegate a3BlendNegate(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Sub(out->angles.v, rhs->angles.v);
	a3real4Sub(out->orientation.v, rhs->orientation.v);
	a3real4Sub(out->scale.v, rhs->scale.v);
	//a3real4Sub(out->transform.v, rhs->transform.v);
	a3real4Sub(out->translation.v, rhs->translation.v);
}

inline a3_BlendConcat a3BlendConcat(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Add(out->angles.v, rhs->angles.v);
	a3real4Add(out->orientation.v, rhs->orientation.v);
	a3real4ProductComp(out->scale.v, lhs->scale.v, rhs->scale.v);
	//a3real4Add(out->transform.v, rhs->transform.v);
	a3real4Add(out->translation.v, rhs->translation.v);
}

inline a3_BlendDeconcat a3BlendDeconcat(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Sub(out->angles.v, rhs->angles.v);
	a3real4Sub(out->orientation.v, rhs->orientation.v);
	a3real4QuotientComp(out->scale.v, lhs->scale.v, rhs->scale.v);
	//a3real4Sub(out->transform.v, rhs->transform.v);
	a3real4Sub(out->translation.v, rhs->translation.v);
}

inline a3vec4 a3vec4Lerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u)
{
	// implement linear interpolation
	a3real4Lerp(v_out->v, v0->v, v1->v, u);
}

inline a3vec4 a3Vec4LogLerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u)
{
	// implement log interpolation
	// (v1*v0^-1)^u * v0
	
	//return v0;
}

inline a3vec4 a3vec4Slerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u)
{
	// implement spherical linear interpolation
	a3real4Slerp(v_out->v, v0->v, v1->v, u);
	//return v0;
}

inline a3vec4 a3vec4Nlerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u)
{
	// implement normalized linear interpolation
	a3real4NLerp(v_out->v, v0->v, v1->v, u);
	//return v0;
}

//-----------------------------------------------------------------------------
// pointer-based reset/identity operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out)
{
	pose_out->transform = a3mat4_identity;
	pose_out->scale = a3vec4_one;
	pose_out->angles = a3vec4_zero;
	pose_out->translation = a3vec4_zero;
	// ...

	// done
	return pose_out;
}

// pointer-based nearest operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	if (u < 0.5)
	{
		// copy p0
		a3BlendCopy(pose_out, pose0);
	}
	else
	{
		// copy p1
		a3BlendCopy(pose_out, pose1);
	}
	return pose_out;
}

// pointer-based LERP operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	a3vec4Nlerp(&pose_out->translation, &pose0->translation, &pose1->translation, u);
	a3vec4Slerp(&pose_out->angles, &pose0->angles, &pose1->angles, u); // not quite right but i'm unsure on log lerp right now
	a3vec4Nlerp(&pose_out->scale, &pose0->scale, &pose1->scale, u);
	// done
	return pose_out;
}

inline a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* poseP, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* poseN, a3real const u)
{
	// not sure if this is correct but its the only way that makes sense to me right now
	a3real4CatmullRom(pose_out->translation.v, poseP->translation.v, pose0->translation.v, pose1->translation.v, poseN->translation.v, u);
	a3real4CatmullRom(pose_out->scale.v, poseP->scale.v, pose0->scale.v, pose1->scale.v, poseN->scale.v, u);
	a3real4CatmullRom(pose_out->angles.v, poseP->angles.v, pose0->angles.v, pose1->angles.v, poseN->angles.v, u);
	//done
	return pose_out;
}

inline a3_BiLerp* a3Bilerp(a3_SpatialPose* out, a3_SpatialPose const* p00, a3_SpatialPose const* p01,
	a3_SpatialPose const* p10, a3_SpatialPose const* p11, a3real const u1, a3real const u2, a3real const u3)
{
	a3real4Bilerp(out->angles.v, p00->angles.v, p01->angles.v, p10->angles.v, p11->angles.v, u1, u2);
	a3real4Bilerp(out->orientation.v, p00->orientation.v, p01->orientation.v, p10->orientation.v, p11->orientation.v, u1, u2);
	a3real4Bilerp(out->scale.v, p00->scale.v, p01->scale.v, p10->scale.v, p11->scale.v, u1, u2);
	a3real4Bilerp(out->translation.v, p00->translation.v, p01->translation.v, p10->translation.v, p11->translation.v, u1, u2);
}


//-----------------------------------------------------------------------------

// data-based reset/identity
inline a3_SpatialPose a3spatialPoseDOpIdentity()
{
	a3_SpatialPose const result = { a3mat4_identity /*, ...*/ };
	return result;
}

// data-based LERP
inline a3_SpatialPose a3spatialPoseDOpLERP(a3_SpatialPose const pose0, a3_SpatialPose const pose1, a3real const u)
{
	a3_SpatialPose result = { 0 };
	// ...

	// done
	return result;
}


//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for hierarchical pose
inline a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out)
{
	a3spatialPoseOpIdentity(pose_out->pose);
	
	// done
	return pose_out;
}

// pointer-based LERP operation for hierarchical pose
inline a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u)
{
	a3spatialPoseOpLERP(pose_out->pose, pose0->pose, pose1->pose, u);
	// done
	return pose_out;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_INL
#endif	// __ANIMAL3D_HIERARCHYSTATEBLEND_H