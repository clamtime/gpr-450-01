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
	out->rotate = r;
	out->scale = s;
	out->translate = t;
}

inline a3_BlendCopy a3BlendCopy(a3_SpatialPose* lhsout, a3_SpatialPose const* rhs)
{
	lhsout->rotate = rhs->rotate;
	lhsout->user = rhs->user;
	lhsout->scale = rhs->scale;
	lhsout->transformMat = rhs->transformMat;
	lhsout->translate = rhs->translate;
}

inline a3_BlendNegate a3BlendNegate(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Sub(out->rotate.v, rhs->rotate.v);
	a3real4Sub(out->user.v, rhs->user.v);
	a3real4Sub(out->scale.v, rhs->scale.v);
	//a3real4Sub(out->transform.v, rhs->transform.v);
	a3real4Sub(out->translate.v, rhs->translate.v);
}

inline a3_BlendConcat a3BlendConcat(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Add(out->rotate.v, rhs->rotate.v);
	a3real4Add(out->user.v, rhs->user.v);
	a3real4ProductComp(out->scale.v, lhs->scale.v, rhs->scale.v);
	//a3real4Add(out->transform.v, rhs->transform.v);
	a3real4Add(out->translate.v, rhs->translate.v);
}

inline a3_BlendDeconcat a3BlendDeconcat(a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs)
{
	a3BlendCopy(out, lhs);

	a3real4Sub(out->rotate.v, rhs->rotate.v);
	a3real4Sub(out->user.v, rhs->user.v);
	a3real4QuotientComp(out->scale.v, lhs->scale.v, rhs->scale.v);
	//a3real4Sub(out->transform.v, rhs->transform.v);
	a3real4Sub(out->translate.v, rhs->translate.v);
}


// HIERARCHICAL
inline a3_BlendConstructHierarchy a3BlendConstructHierarchy(a3_HierarchyPose* out, a3vec4 const* r, a3vec4 const* s, a3vec4 const* t, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		(out->pose + i)->rotate = *r;
		(out->pose + i)->scale = *s;
		(out->pose + i)->translate = *t;
	}
}

inline a3_BlendCopyHierarchy a3BlendCopyHierarchy(a3_HierarchyPose* lhsout, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		(lhsout->pose + i)->rotate = (rhs->pose + i)->rotate;
		(lhsout->pose + i)->user = (rhs->pose + i)->user;
		(lhsout->pose + i)->scale = (rhs->pose + i)->scale;
		(lhsout->pose + i)->translate = (rhs->pose + i)->translate;
		(lhsout->pose + i)->transformMat = (rhs->pose + i)->transformMat;
	}
}

inline a3_BlendNegateHierarchy a3BlendNegateHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Sub((out->pose + i)->rotate.v, (rhs->pose + i)->rotate.v);
		a3real4Sub((out->pose + i)->user.v, (rhs->pose + i)->user.v);
		a3real4Sub((out->pose + i)->scale.v, (rhs->pose + i)->scale.v);
		//a3real4Sub(out->transform.v, rhs->transform.v);
		a3real4Sub((out->pose + i)->translate.v, (rhs->pose + i)->translate.v);
	}
}

inline a3_BlendConcatHierarchy a3BlendConcatHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Add((out->pose + i)->rotate.v, (rhs->pose + i)->rotate.v);
		a3real4Add((out->pose + i)->user.v, (rhs->pose + i)->user.v);
		a3real4ProductComp((out->pose + i)->scale.v, (lhs->pose + i)->scale.v, (rhs->pose + i)->scale.v);
		a3real4Add((out->pose + i)->translate.v, (rhs->pose + i)->translate.v);


	}
}

inline a3_BlendDeconcatHierarchy a3BlendDeconcatHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3BlendCopyHierarchy(out + i, lhs + i, nodeCount);

		a3real4Sub((out->pose + i)->rotate.v, (rhs->pose + i)->rotate.v);
		a3real4Sub((out->pose + i)->user.v, (rhs->pose + i)->user.v);
		a3real4QuotientComp((out->pose + i)->scale.v, (lhs->pose + i)->scale.v, (rhs->pose + i)->scale.v);
		a3real4Sub((out->pose + i)->translate.v, (rhs->pose + i)->translate.v);
	}
}

inline a3_BiLerpHierarchy* a3BilerpHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p00, a3_HierarchyPose const* p01,
	a3_HierarchyPose const* p10, a3_HierarchyPose const* p11, a3real const u1, a3real const u2, a3real const u3, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		a3real4Bilerp((out->pose + i)->rotate.v, (p00->pose + i)->rotate.v, (p01->pose + i)->rotate.v, (p10->pose + i)->rotate.v, (p11->pose + i)->rotate.v, u1, u2);
		a3real4Bilerp((out->pose + i)->user.v, (p00->pose + i)->user.v, (p01->pose + i)->user.v, (p10->pose + i)->user.v, (p11->pose + i)->user.v, u1, u2);
		a3real4Bilerp((out->pose + i)->scale.v, (p00->pose + i)->scale.v, (p01->pose + i)->scale.v, (p10->pose + i)->scale.v, (p11->pose + i)->scale.v, u1, u2);
		a3real4Bilerp((out->pose + i)->translate.v, (p00->pose + i)->translate.v, (p01->pose + i)->translate.v, (p10->pose + i)->translate.v, (p11->pose + i)->translate.v, u1, u2);
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

	out->rotate.v0 -= u;
	out->rotate.v1 -= u;
	out->rotate.v2 -= u;
	out->rotate.v3 -= u;

	out->user.v0 -= u;
	out->user.v1 -= u;
	out->user.v2 -= u;
	out->user.v3 -= u;


	out->scale.v0 /= u;
	out->scale.v1 /= u;
	out->scale.v2 /= u;
	out->scale.v3 /= u;

	out->translate.v0 -= u;
	out->translate.v1 -= u;
	out->translate.v2 -= u;
	out->translate.v3 -= u;
}

inline a3_Convert a3Convert(a3_SpatialPose* out, a3_SpatialPose const* p)
{
	//a3real4x4ConcatL(out->transform.m, p->user.v);
	//a3real4x4ConcatL(out->transform.m, p->scale.v);
	//a3real4x4ConcatL(out->transform.m, p->translate.v);
}

inline a3_Revert a3Revert(a3_SpatialPose* out, a3_SpatialPose const* p)
{
	//a3real4x4Deconcat(out->transform.m, p->user.v);
	//a3real4x4Deconcat(out->transform.m, p->scale.v);
	//a3real4x4Deconcat(out->transform.m, p->translate.v);
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

		(out->pose + i)->rotate.v0 -= u;
		(out->pose + i)->rotate.v1 -= u;
		(out->pose + i)->rotate.v2 -= u;
		(out->pose + i)->rotate.v3 -= u;

		(out->pose + i)->user.v0 -= u;
		(out->pose + i)->user.v1 -= u;
		(out->pose + i)->user.v2 -= u;
		(out->pose + i)->user.v3 -= u;


		(out->pose + i)->scale.v0 /= u;
		(out->pose + i)->scale.v1 /= u;
		(out->pose + i)->scale.v2 /= u;
		(out->pose + i)->scale.v3 /= u;

		(out->pose + i)->translate.v0 -= u;
		(out->pose + i)->translate.v1 -= u;
		(out->pose + i)->translate.v2 -= u;
		(out->pose + i)->translate.v3 -= u;
	}
}

inline a3_ConvertHierarchy a3ConvertHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->user.v);
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->scale.v);
		//a3real4x4ConcatL((out->pose + i)->transform.m, (p->pose + i)->translate.v);
	}
}

inline a3_RevertHierarchy a3RevertHierarchy(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount)
{
	for (a3index i = 0; i < nodeCount; i++)
	{
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->user.v);
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->scale.v);
		//a3real4x4DeconcatL((out->pose + i)->transform.m, (p->pose + i)->translate.v);
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
	pose_out->transformMat = a3mat4_identity;
	pose_out->scale = a3vec4_one;
	pose_out->rotate = a3vec4_zero;
	pose_out->translate = a3vec4_zero;
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
	a3vec4Nlerp(&pose_out->translate, &pose0->translate, &pose1->translate, u);
	a3vec4Slerp(&pose_out->rotate, &pose0->rotate, &pose1->rotate, u); // not quite right but i'm unsure on log lerp right now
	a3vec4Nlerp(&pose_out->scale, &pose0->scale, &pose1->scale, u);
	// done
	return pose_out;
}

inline a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* poseP, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* poseN, a3real const u)
{
	// not sure if this is correct but its the only way that makes sense to me right now
	a3real4CatmullRom(pose_out->translate.v, poseP->translate.v, pose0->translate.v, pose1->translate.v, poseN->translate.v, u);
	a3real4CatmullRom(pose_out->scale.v, poseP->scale.v, pose0->scale.v, pose1->scale.v, poseN->scale.v, u);
	a3real4CatmullRom(pose_out->rotate.v, poseP->rotate.v, pose0->rotate.v, pose1->rotate.v, poseN->rotate.v, u);
	//done
	return pose_out;
}

inline a3_BiLerp* a3Bilerp(a3_SpatialPose* out, a3_SpatialPose const* p00, a3_SpatialPose const* p01,
	a3_SpatialPose const* p10, a3_SpatialPose const* p11, a3real const u1, a3real const u2, a3real const u3)
{
	a3real4Bilerp(out->rotate.v, p00->rotate.v, p01->rotate.v, p10->rotate.v, p11->rotate.v, u1, u2);
	a3real4Bilerp(out->user.v, p00->user.v, p01->user.v, p10->user.v, p11->user.v, u1, u2);
	a3real4Bilerp(out->scale.v, p00->scale.v, p01->scale.v, p10->scale.v, p11->scale.v, u1, u2);
	a3real4Bilerp(out->translate.v, p00->translate.v, p01->translate.v, p10->translate.v, p11->translate.v, u1, u2);
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