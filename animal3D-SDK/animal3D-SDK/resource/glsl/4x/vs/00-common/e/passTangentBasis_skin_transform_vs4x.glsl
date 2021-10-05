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
	
	passTangentBasis_skin_transform_vs4x.glsl
	Calculate and pass tangent basis with skinning.
*/

#version 450

layout (location = 0) in vec4 aPosition;
layout (location = 2) in vec4 aNormal;
layout (location = 8) in vec4 aTexcoord;
layout (location = 10) in vec4 aTangent;
layout (location = 11) in vec4 aBitangent;

// skinning attributes
// w = weights (1)
// j = joints (7)
// rigid: 1 infl at 100% wt
//layout (location = 7) in int aBlendIndex; 
// smooth: 4 infl at wts
layout (location = 7) in ivec4 aBlendIndex;
layout (location = 1) in vec4 aBlendWeight;


#define MAX_JOINTS 128

#define dquat mat2x4

uniform mat4 uP;
uniform mat4 uMV, uMV_nrm;
uniform mat4 uAtlas;

uniform ubTransformBlend
{
	mat4 uSkinMat[MAX_JOINTS]; // s
	dquat uSkinDQ[MAX_JOINTS]; // q
};

out vbVertexData {
	mat4 vTangentBasis_view;
	vec4 vTexcoord_atlas;
};

flat out int vVertexID;
flat out int vInstanceID;

vec4 skinRigidLinear(in vec4 v, in int j)
{
// v' = s_j * v
	return (uSkinMat[j] * v);
}

vec4 skinSmoothLinear(in vec4 v, in ivec4 j, in vec4 w)
{
	vec4 v_out = vec4(0.0);
	// v' = sum (w_i * s_j_i * v)
	v_out += (w[0] * uSkinMat[j[0]] * v);
	v_out += (w[1] * uSkinMat[j[1]] * v);
	v_out += (w[2] * uSkinMat[j[2]] * v);
	v_out += (w[3] * uSkinMat[j[3]] * v);
	return v_out;
}

mat4 convDQ2Mat4(in dquat dq)
{
	// TO-DO: Implement me
	mat4 m_out = mat4 (1.0);
	return m_out;
}

mat4 skinSmoothDQDLB(in ivec4 j, in vec4 w)
{
	// q' = sum [i = 0..3](w_i * q_j_i)
	// m' = convert (q')
	dquat dq = dquat(0.0);
		// do skinning formula
	return convDQ2Mat4(dq / length(dq[0]));
}

void main()
{
	// DUMMY OUTPUT: directly assign input position to output position
//	gl_Position = aPosition;

	vTangentBasis_view = uMV_nrm * mat4(
//		skinRigidLinear(vec4(aTangent.xyz, 0.0), aBlendIndex[0]),
//		skinRigidLinear(vec4(aBitangent.xyz, 0.0), aBlendIndex[0]),
//		skinRigidLinear(vec4(aNormal.xyz, 0.0), aBlendIndex[0]),
		skinSmoothLinear(vec4(aTangent.xyz, 0.0), aBlendIndex, aBlendWeight),
		skinSmoothLinear(vec4(aBitangent.xyz, 0.0), aBlendIndex, aBlendWeight),
		skinSmoothLinear(vec4(aNormal.xyz, 0.0), aBlendIndex, aBlendWeight),
		vec4(0.0));
	vTangentBasis_view[3] = uMV *
		//skinRigidLinear(aPosition, aBlendIndex[0]);
		skinSmoothLinear(aPosition, aBlendIndex, aBlendWeight);
	gl_Position = uP * vTangentBasis_view[3];
	
	vTexcoord_atlas = uAtlas * aTexcoord;

	vVertexID = gl_VertexID;
	vInstanceID = gl_InstanceID;
}
