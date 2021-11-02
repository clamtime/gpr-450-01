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


// HIERARCHICAL
inline a3_BlendConstructHierarchy a3BlendConstructHierarchy(a3_HierarchyPose* out, a3vec4 const* r, a3vec4 const* s, a3vec4 const* t, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		(out->pose + i)->angles      = *r;
		(out->pose + i)->scale       = *s;
		(out->pose + i)->translation = *t;
	}
}

inline a3_BlendCopyHierarchy a3BlendCopyHierarchy(a3_HierarchyPose* lhsout, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		(lhsout->pose + i)->angles      = (rhs->pose + i)->angles;
		(lhsout->pose + i)->orientation = (rhs->pose + i)->orientation;
		(lhsout->pose + i)->scale       = (rhs->pose + i)->scale;
		(lhsout->pose + i)->translation = (rhs->pose + i)->translation;
		(lhsout->pose + i)->transform   = (rhs->pose + i)->transform;
	}
}

inline a3_BlendNegateHierarchy a3BlendNegateHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Sub((out->pose + i)->angles.v     , (rhs->pose + i)->angles.v);
		a3real4Sub((out->pose + i)->orientation.v, (rhs->pose + i)->orientation.v);
		a3real4Sub((out->pose + i)->scale.v      , (rhs->pose + i)->scale.v);
		//a3real4Sub(out->transform.v, rhs->transform.v);
		a3real4Sub((out->pose + i)->translation.v, (rhs->pose + i)->translation.v);
	}
}

inline a3_BlendConcatHierarchy a3BlendConcatHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Add((out->pose + i)->angles.v, (rhs->pose + i)->angles.v);
		a3real4Add((out->pose + i)->orientation.v, (rhs->pose + i)->orientation.v);
		a3real4ProductComp((out->pose + i)->scale.v, (lhs->pose + i)->scale.v, (rhs->pose + i)->scale.v);
		a3real4Add((out->pose + i)->translation.v, (rhs->pose + i)->translation.v);

		
	}
}

inline a3_BlendDeconcatHierarchy a3BlendDeconcatHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Sub((out->pose + i)->angles.v, (rhs->pose + i)->angles.v);
		a3real4Sub((out->pose + i)->orientation.v, (rhs->pose + i)->orientation.v);
		a3real4QuotientComp((out->pose + i)->scale.v, (lhs->pose + i)->scale.v, (rhs->pose + i)->scale.v);
		a3real4Sub((out->pose + i)->translation.v, (rhs->pose + i)->translation.v);
	}
}

inline a3_BiLerpHierarchy* a3BilerpHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p00, a3_HierarchyPose const* p01,
	a3_HierarchyPose const* p10, a3_HierarchyPose const* p11, a3real const u1, a3real const u2, a3real const u3, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3real4Bilerp((out->pose + i)->angles.v     , (p00->pose + i)->angles.v		, (p01->pose + i)->angles.v		, (p10->pose + i)->angles.v		, (p11->pose + i)->angles.v, u1, u2);
		a3real4Bilerp((out->pose + i)->orientation.v, (p00->pose + i)->orientation.v, (p01->pose + i)->orientation.v, (p10->pose + i)->orientation.v, (p11->pose + i)->orientation.v, u1, u2);
		a3real4Bilerp((out->pose + i)->scale.v      , (p00->pose + i)->scale.v      , (p01->pose + i)->scale.v		, (p10->pose + i)->scale.v		, (p11->pose + i)->scale.v, u1, u2);
		a3real4Bilerp((out->pose + i)->translation.v, (p00->pose + i)->translation.v, (p01->pose + i)->translation.v, (p10->pose + i)->translation.v, (p11->pose + i)->translation.v, u1, u2);
	}
}


// proj3 spatial pose
inline a3_Smoothstep a3Smoothstep(a3_SpatialPose* out, a3_SpatialPose const* p0, a3_SpatialPose const* p1, a3real u)
{
	// from wikipedia / AMD
	//x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0); 
	//return x * x * (3 - 2 * x);


    // a3BlendDeconcat(a3BlendDeconcatReal(p0, p0, u), a3Descale(p0, p0, u), a3BlendDeconcat(p1, p1, p0));
}

inline a3_Descale a3Descale(a3_SpatialPose* out, a3_SpatialPose const* p, a3real const u)
{
	a3BlendCopy(out, p);

	out->angles.v0 -= u;
	out->angles.v1 -= u;
	out->angles.v2 -= u;
	out->angles.v3 -= u;

	out->orientation.v0 -= u;
	out->orientation.v1 -= u;
	out->orientation.v2 -= u;
	out->orientation.v3 -= u;


	out->scale.v0 /= u;
	out->scale.v1 /= u;
	out->scale.v2 /= u;
	out->scale.v3 /= u;

	out->translation.v0 -= u;
	out->translation.v1 -= u;
	out->translation.v2 -= u;
	out->translation.v3 -= u;
}

inline a3_Convert a3Convert(a3_SpatialPose* out, a3_SpatialPose const* p)
{
	//a3real4x4ConcatL(out->transform.m, p->orientation.v);
	//a3real4x4ConcatL(out->transform.m, p->scale.v);
	//a3real4x4ConcatL(out->transform.m, p->translation.v);
}

inline a3_Revert a3Revert(a3_SpatialPose* out, a3_SpatialPose const* p)
{
	//a3real4x4Deconcat(out->transform.m, p->orientation.v);
	//a3real4x4Deconcat(out->transform.m, p->scale.v);
	//a3real4x4Deconcat(out->transform.m, p->translation.v);
}

// proj3 hierarchical pose
inline a3_SmoothstepHierarchy a3SmoothstepHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p0, a3_HierarchyPose const* p1, a3real u, a3real nodeCount)
{
	// from wikipedia / AMD
	//x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0); 
	//return x * x * (3 - 2 * x);

	for (a3index i = 0; i < nodeCount; i++)
	{
		// a3BlendDeconcat(a3BlendDeconcatReal(p0->pose + i, p0->pose + i, u), a3Descale(p0->pose+i, p0->pose+i, u), a3BlendDeconcat(p1->pose + i, p1->pose + i, p0->pose + i));

	}
}

inline a3_DescaleHierarchy a3DescaleHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real const u, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopy(out->pose + i, p->pose + i);

		(out->pose + i)->angles.v0 -= u;
		(out->pose + i)->angles.v1 -= u;
		(out->pose + i)->angles.v2 -= u;
		(out->pose + i)->angles.v3 -= u;
		
		(out->pose + i)->orientation.v0 -= u;
		(out->pose + i)->orientation.v1 -= u;
		(out->pose + i)->orientation.v2 -= u;
		(out->pose + i)->orientation.v3 -= u;
		
		
		(out->pose + i)->scale.v0 /= u;
		(out->pose + i)->scale.v1 /= u;
		(out->pose + i)->scale.v2 /= u;
		(out->pose + i)->scale.v3 /= u;
		
		(out->pose + i)->translation.v0 -= u;
		(out->pose + i)->translation.v1 -= u;
		(out->pose + i)->translation.v2 -= u;
		(out->pose + i)->translation.v3 -= u;
	}
}

inline a3_ConvertHierarchy a3ConvertHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->orientation.v);
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->scale.v);
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->translation.v);
	}
}

inline a3_RevertHierarchy a3RevertHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->orientation.v);
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->scale.v);
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->translation.v);
	}
}


inline a3_FK a3FK(a3mat4* out, a3_HierarchyPose p, a3mat4 objectSpace, a3mat4 localSpace, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; ++i)
	{
		// if (p.pose + i) -> some sort of parent index 
			// parent object matrix * current object matrix
		// else
			// copy current local matrix to current object matrix
	}
}
inline a3_IK a3IK(a3mat4* out, a3_HierarchyPose p, a3mat4 objectSpace, a3mat4 localSpace, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; ++i)
	{
		// if (p.pose + i) -> some sort of parent index 
			// invert parent object matrix * current object matrix
		// else
			// copy current object matrix to current local matrix
	}
}

// end proj3 step1 funcs


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