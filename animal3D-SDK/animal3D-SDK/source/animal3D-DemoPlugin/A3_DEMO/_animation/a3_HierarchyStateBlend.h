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
	
	a3_HierarchyStateBlend.h
	Hierarchy blend operations.
*/

#ifndef __ANIMAL3D_HIERARCHYSTATEBLEND_H
#define __ANIMAL3D_HIERARCHYSTATEBLEND_H


#include "a3_HierarchyState.h"

#include "a3_Kinematics.h"


#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus

#endif	// __cplusplus


// ROBUST BLEND NODE for any op
typedef a3_SpatialPose*(*a3_SpatialPoseBlendOp)(a3_SpatialPose* p_out, a3_SpatialPose const* ctrl[], a3real const param[]);

typedef struct a3_SpatialPoseBlendNode
{
	a3_SpatialPoseBlendOp op;
	a3_SpatialPose* p_out;
	a3_SpatialPose const* p_ctrl[8];
	a3real const* param[8];
} a3_SpatialPoseBlendNode;


inline a3_SpatialPoseBlendNode* a3spatialPoseBlendNodeCall(a3_SpatialPoseBlendNode* b)
{
	//b->op(b->p_out, b->p_ctrl, b->param);
	return b;
}

// e.g. lerp
inline a3_SpatialPose* a3_SpatialPoseBlendLerp(a3_SpatialPose* p_out, a3_SpatialPose const* ctrl[2], a3real const param[1])
{
	// the formula: p0 + (p1 - p0)*u
	a3_SpatialPose const* p0 = ctrl[0];
	a3_SpatialPose const* p1 = ctrl[1];
	a3real const u = param[0];
	a3spatialPoseLerp(p_out, p0, p1, u);
	return p_out;
}

// add
inline a3_SpatialPose* a3_SpatialPoseBlendConcat(a3_SpatialPose* p_out, a3_SpatialPose const* ctrl[2])
{
	a3_SpatialPose const* p0 = ctrl[0];
	a3_SpatialPose const* p1 = ctrl[1];
	a3spatialPoseConcat(p_out, p0, p1);
	return p_out;
}

// scale
inline a3_SpatialPose* a3_SpatialPoseBlendScale(a3_SpatialPose* p_out, a3_SpatialPose const* p_base, a3_SpatialPose const* ctrl[1])
{
	a3_SpatialPose const* p0 = ctrl[0];
	a3spatialPoseConcat(p_out, p_base, p0);
	return p_out;
}


//// object based
//typedef a3_SpatialPose(*a3_SpatialPoseBlendOpLerp)(a3_SpatialPose const p0,	a3_SpatialPose const p1, a3real const u);
//
//// pointer based
//typedef a3_SpatialPose* (*a3_SpatialPoseBlendOpLerp)(a3_SpatialPose* p_out, a3_SpatialPose const* p0, a3_SpatialPose const* p1, a3real const u);


// SPATIAL POSE
typedef a3_SpatialPose(*a3_BlendConstruct)(a3_SpatialPose* out,			a3vec4 const* r,		   a3vec4 const* s, a3vec4 const* t);
typedef a3_SpatialPose(*a3_BlendCopy)(     a3_SpatialPose* lhsout, a3_SpatialPose const* rhs);
typedef a3_SpatialPose(*a3_BlendNegate)(   a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs);
typedef a3_SpatialPose(*a3_BlendConcat)(   a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs);
typedef a3_SpatialPose(*a3_BlendDeconcat)( a3_SpatialPose* out, a3_SpatialPose const* lhs, a3_SpatialPose const* rhs);
typedef a3_SpatialPose(*a3_BlendScale)(/*?*/);
typedef a3_SpatialPose(*a3_BiLerp)(a3_SpatialPose* out, a3_SpatialPose const* p00, a3_SpatialPose const* p01, 
	a3_SpatialPose const* p10, a3_SpatialPose const* p11, a3real const u1, a3real const u2, a3real const u3);

// HIERARCHICAL 
typedef a3_HierarchyPose(*a3_BlendConstructHierarchy)(a3_HierarchyPose* out, a3vec4 const* r, a3vec4 const* s, a3vec4 const* t, a3real nodeCount);
typedef a3_HierarchyPose(*a3_BlendCopyHierarchy)     (a3_HierarchyPose* lhsout, a3_HierarchyPose const* rhs, a3real nodeCount);
typedef a3_HierarchyPose(*a3_BlendNegateHierarchy)   (a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount);
typedef a3_HierarchyPose(*a3_BlendConcatHierarchy)   (a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount);
typedef a3_HierarchyPose(*a3_BlendDeconcatHierarchy) (a3_HierarchyPose* out, a3_HierarchyPose const* lhs, a3_HierarchyPose const* rhs, a3real nodeCount);
typedef a3_HierarchyPose(*a3_BlendScaleHierarchy)    (/*?*/);
typedef a3_HierarchyPose(*a3_BiLerpHierarchy)        (a3_HierarchyPose* out, a3_HierarchyPose const* p00, a3_HierarchyPose const* p01,
	a3_HierarchyPose const* p10, a3_HierarchyPose const* p11, a3real const u1, a3real const u2, a3real const u3, a3real nodeCount);

// SPATIAL POSE PROJ3
typedef a3_SpatialPose(*a3_Smoothstep)(a3_SpatialPose* out, a3_SpatialPose const* p0, a3_SpatialPose const* p1, a3real u);
typedef a3_SpatialPose(*a3_Descale)(a3_SpatialPose* out, a3_SpatialPose const* p, a3real const u);
typedef a3_SpatialPose(*a3_Convert)(a3_SpatialPose* out, a3_SpatialPose const* p);
typedef a3_SpatialPose(*a3_Revert)(a3_SpatialPose* out, a3_SpatialPose const* p);

// HIERARCHICAL PROJ3
typedef a3_HierarchyPose(*a3_SmoothstepHierarchy)(a3_HierarchyPose* out, a3_HierarchyPose const* p0, a3_HierarchyPose const* p1, a3real u, a3real nodeCount);
typedef a3_HierarchyPose(*a3_DescaleHierarchy)(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real const u, a3real nodeCount);
typedef a3_HierarchyPose(*a3_ConvertHierarchy)(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount);
typedef a3_HierarchyPose(*a3_RevertHierarchy)(a3_HierarchyPose* out, a3_HierarchyPose const* p, a3real nodeCount);

typedef a3_HierarchyPose(*a3_FK)(a3mat4* out, a3_HierarchyPose p, a3mat4 objectSpace, a3mat4 localSpace, a3real nodeCount);
typedef a3_HierarchyPose(*a3_IK)(a3mat4* out, a3_HierarchyPose p, a3mat4 objectSpace, a3mat4 localSpace, a3real nodeCount);




// blend operation function pointer

typedef a3vec4(*a3_BlendOpLerp)(a3vec4 const v0, a3vec4 const v1, a3real const u);
typedef struct a3_SpatialPoseBlendOpLerp
{
	a3_BlendOpLerp opOrientation, opAngles, opScale, opTranslation;
} a3_SpatialPoseBlendOpLerp;

// linear interpolation
 a3vec4 a3vec4Lerp(a3vec4 *const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u);

// log interpolation
 a3vec4 a3Vec4LogLerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u);

// spherical linear interpolation
a3vec4 a3vec4Slerp(a3vec4* const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u);

// implement normalized linear interpolation
a3vec4 a3vec4Nlerp(a3vec4 *const v_out, a3vec4 const* v0, a3vec4 const* v1, a3real const u);

//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out);

// pointer-based LERP operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u);

// pointer-based Cubic operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* poseP, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* poseN, a3real const u);


//-----------------------------------------------------------------------------

// data-based reset/identity
a3_SpatialPose a3spatialPoseDOpIdentity();

// data-based LERP
a3_SpatialPose a3spatialPoseDOpLERP(a3_SpatialPose const pose0, a3_SpatialPose const pose1, a3real const u);


//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for hierarchical pose
a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out);

// pointer-based LERP operation for hierarchical pose
a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u);


//-----------------------------------------------------------------------------




#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_HierarchyStateBlend.inl"


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_H