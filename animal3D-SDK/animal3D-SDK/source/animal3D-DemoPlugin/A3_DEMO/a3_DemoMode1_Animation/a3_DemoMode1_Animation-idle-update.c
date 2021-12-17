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

	a3_DemoMode1_Animation-idle-update.c
	Demo mode implementations: animation scene.

	********************************************
	*** UPDATE FOR ANIMATION SCENE MODE      ***
	********************************************
*/

//-----------------------------------------------------------------------------

#include "../a3_DemoMode1_Animation.h"

//typedef struct a3_DemoState a3_DemoState;
#include "../a3_DemoState.h"

#include "../_a3_demo_utilities/a3_DemoMacros.h"
#include <stdio.h>

//-----------------------------------------------------------------------------
// UTILS

inline a3real4r a3demo_mat2quat_safe(a3real4 q, a3real4x4 const m)
{
	// ****TO-DO: 
	//	-> convert rotation part of matrix to quaternion
	//	-> NOTE: this is for testing dual quaternion skinning only; 
	//		quaternion data would normally be computed with poses

	a3real4SetReal4(q, a3vec4_w.v);

	// done
	return q;
}

inline a3real4x2r a3demo_mat2dquat_safe(a3real4x2 Q, a3real4x4 const m)
{
	// ****TO-DO: 
	//	-> convert matrix to dual quaternion
	//	-> NOTE: this is for testing dual quaternion skinning only; 
	//		quaternion data would normally be computed with poses

	a3demo_mat2quat_safe(Q[0], m);
	a3real4SetReal4(Q[1], a3vec4_zero.v);

	// done
	return Q;
}


//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// UPDATE

void a3demo_update_objects(a3f64 const dt, a3_DemoSceneObject* sceneObjectBase,
	a3ui32 count, a3boolean useZYX, a3boolean applyScale);
void a3demo_update_defaultAnimation(a3_DemoState* demoState, a3f64 const dt,
	a3_DemoSceneObject* sceneObjectBase, a3ui32 count, a3ui32 axis);
void a3demo_update_bindSkybox(a3_DemoSceneObject* obj_camera, a3_DemoSceneObject* obj_skybox);
void a3demo_update_pointLight(a3_DemoSceneObject* obj_camera, a3_DemoPointLight* pointLightBase, a3ui32 count);

void a3demo_applyScale_internal(a3_DemoSceneObject* sceneObject, a3real4x4p s);

void a3animation_update_graphics(a3_DemoState* demoState, a3_DemoMode1_Animation* demoMode,
	a3_DemoModelMatrixStack const* matrixStack, a3_HierarchyState const* activeHS)
{
	// active camera
	a3_DemoProjector const* activeCamera = demoMode->projector + demoMode->activeCamera;
	a3_DemoSceneObject const* activeCameraObject = activeCamera->sceneObject;

	// temp scale mat
	a3mat4 scaleMat = a3mat4_identity;
	a3addressdiff const skeletonIndex = demoMode->obj_skeleton - demoMode->object_scene;
	a3ui32 const mvp_size = demoMode->hierarchy_skel->numNodes * sizeof(a3mat4);
	a3ui32 const t_skin_size = sizeof(demoMode->t_skin);
	a3ui32 const dq_skin_size = sizeof(demoMode->dq_skin);
	a3mat4 const mvp_obj = matrixStack[skeletonIndex].modelViewProjectionMat;
	a3mat4* mvp_joint, * mvp_bone, * t_skin;
	a3dualquat* dq_skin;
	a3index i;
	a3i32 p;

	// update joint and bone transforms
	for (i = 0; i < demoMode->hierarchy_skel->numNodes; ++i)
	{
		mvp_joint = demoMode->mvp_joint + i;
		mvp_bone = demoMode->mvp_bone + i;
		t_skin = demoMode->t_skin + i;
		dq_skin = demoMode->dq_skin + i;

		// joint transform
		a3real4x4SetScale(scaleMat.m, a3real_sixth);
		a3real4x4Concat(activeHS->objectSpace->pose[i].transformMat.m, scaleMat.m);
		a3real4x4Product(mvp_joint->m, mvp_obj.m, scaleMat.m);

		// bone transform
		p = demoMode->hierarchy_skel->nodes[i].parentIndex;
		if (p >= 0)
		{
			// position is parent joint's position
			scaleMat.v3 = activeHS->objectSpace->pose[p].transformMat.v3;

			// direction basis is from parent to current
			a3real3Diff(scaleMat.v2.v,
				activeHS->objectSpace->pose[i].transformMat.v3.v, scaleMat.v3.v);

			// right basis is cross of some upward vector and direction
			// select 'z' for up if either of the other dimensions is set
			a3real3MulS(a3real3CrossUnit(scaleMat.v0.v,
				a3real2LengthSquared(scaleMat.v2.v) > a3real_zero
				? a3vec3_z.v : a3vec3_y.v, scaleMat.v2.v), a3real_sixth);

			// up basis is cross of direction and right
			a3real3MulS(a3real3CrossUnit(scaleMat.v1.v,
				scaleMat.v2.v, scaleMat.v0.v), a3real_sixth);
		}
		else
		{
			// if we are a root joint, make bone invisible
			a3real4x4SetScale(scaleMat.m, a3real_zero);
		}
		a3real4x4Product(mvp_bone->m, mvp_obj.m, scaleMat.m);

		// get base to current object-space
		*t_skin = activeHS->objectSpaceBindToCurrent->pose[i].transformMat;

		// calculate DQ
		a3demo_mat2dquat_safe(dq_skin->Q, t_skin->m);
	}

	// upload
	a3bufferRefill(demoState->ubo_transformMVP, 0, mvp_size, demoMode->mvp_joint);
	a3bufferRefill(demoState->ubo_transformMVPB, 0, mvp_size, demoMode->mvp_bone);
	a3bufferRefill(demoState->ubo_transformBlend, 0, t_skin_size, demoMode->t_skin);
	a3bufferRefillOffset(demoState->ubo_transformBlend, 0, t_skin_size, dq_skin_size, demoMode->dq_skin);
}

void a3animation_update_fk(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// FK pipeline
		a3hierarchyPoseConcat(activeHS->localSpace,	// local: goal to calculate
			activeHS->animPose, // holds current sample pose
			baseHS->localSpace, // holds base pose (animPose is all identity poses)
			activeHS->hierarchy->numNodes);
		a3hierarchyPoseConvert(activeHS->localSpace,
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order);
		a3kinematicsSolveForward(activeHS);
	}
}

void a3animation_update_ik(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// IK pipeline
		// ****DONE: direct opposite of FK
		// Convert?
		a3hierarchyPoseRestore(activeHS->localSpace,
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order);
		// Deconcat
		a3hierarchyPoseDeconcat(activeHS->localSpace,	// local: goal to calculate
			activeHS->animPose, // holds current sample pose
			baseHS->localSpace, // holds base pose (animPose is all identity poses)
			activeHS->hierarchy->numNodes);
		// Solve Inverse
		a3i32 nodeIterator = a3kinematicsSolveInverse(activeHS);
		a3kinematicsSolveForward(activeHS);
		//(activeHS+nodeIterator)->localSpace
	}
}

void a3animation_update_skin(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS)
{
	if (activeHS->hierarchy == baseHS->hierarchy)
	{
		// FK pipeline extended for skinning and other applications
		a3hierarchyStateUpdateLocalInverse(activeHS);
		a3hierarchyStateUpdateObjectInverse(activeHS);
		a3hierarchyStateUpdateObjectBindToCurrent(activeHS, baseHS);
	}
}

void a3animation_update_applyEffectors(a3_DemoMode1_Animation* demoMode,
	a3_HierarchyState* activeHS, a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		a3_DemoSceneObject* sceneObject = demoMode->obj_skeleton;
		a3ui32 j = sceneObject->sceneGraphIndex;

		// need to properly transform joints to their parent frame and vice-versa
		a3mat4 const controlToSkeleton = demoMode->sceneGraphState->localSpaceInv->pose[j].transformMat;
		a3vec4 controlLocator_neckLookat, controlLocator_wristEffector, controlLocator_wristConstraint, controlLocator_wristBase;
		a3mat4 jointTransform_neck = a3mat4_identity, jointTransform_wrist = a3mat4_identity, jointTransform_elbow = a3mat4_identity, jointTransform_shoulder = a3mat4_identity;
		a3ui32 j_neck, j_wrist, j_elbow, j_shoulder;

		// NECK LOOK-AT 
		{
			// look-at effector
			sceneObject = demoMode->obj_skeleton_neckLookat_ctrl;
			a3real4Real4x4Product(controlLocator_neckLookat.v, controlToSkeleton.m,
				demoMode->sceneGraphState->localSpace->pose[sceneObject->sceneGraphIndex].transformMat.v3.v);
			j = j_neck = a3hierarchyGetNodeIndex(activeHS->hierarchy, "mixamorig:Neck");
			jointTransform_neck = activeHS->objectSpace->pose[j].transformMat;

			// ****TO-DO: 
			// make "look-at" matrix
			// in this example, +Z is towards locator, +Y is up
			// z = v / |v|
			// x = 

			a3vec3 neckPos, xRot, yRot, zRot;
			neckPos = jointTransform_neck.v3.xyz;
			a3real3Diff(zRot.v, controlLocator_neckLookat.v, neckPos.v); // vector from joint to effector
			a3real3Normalize(zRot.v);
			a3real3CrossUnit(xRot.v, a3vec3_y.v, zRot.v);
			a3real3CrossUnit(yRot.v, zRot.v, xRot.v);
			activeHS->objectSpace->pose[j].transformMat.v0.xyz = xRot;
			activeHS->objectSpace->pose[j].transformMat.v1.xyz = yRot;
			activeHS->objectSpace->pose[j].transformMat.v2.xyz = zRot;

			// ****TO-DO: 
			// reassign resolved transforms to OBJECT-SPACE matrices
			// resolve local and animation pose for affected joint
			//	(instead of doing IK for whole skeleton when only one joint has changed)
			a3real4x4TransformInverse(activeHS->objectSpaceInv->pose[j].transformMat.m, activeHS->objectSpace->pose[j].transformMat.m);
			a3kinematicsSolveInverseSingle(activeHS, j, activeHS->hierarchy->nodes[j].parentIndex);
			a3spatialPoseRestore(activeHS->localSpace->pose + j, poseGroup->channel[j], poseGroup->order[j]);
			a3spatialPoseDeconcat(activeHS->animPose->pose + j, activeHS->localSpace->pose + j, baseHS->localSpace->pose + j);


			/*// do i need to get a rotation matrix before this? this feels right but im unsure
			a3real4Normalize(controlLocator_neckLookat.v);
			a3vec3 x = a3vec3_zero;
			a3vec3 y = a3vec3_zero;
			a3vec3 z = controlLocator_neckLookat.xyz;
			a3real3Cross(x.v, z.v, a3vec3_y.v);
			//a3real4Normalize(x.v);
			a3real3Cross(y.v, x.v, z.v);

			a3vec4 t = jointTransform_neck.v3;
			//a3mat3 rotMat = { *x.v, *y.v, *z.v };

			a3mat4 lookAt = a3mat4_identity;
			lookAt.v3 = t;
			lookAt.v0.xyz = x;
			lookAt.v1.xyz = y;
			lookAt.v2.xyz = z;*/

			

			/*activeHS->objectSpace->pose[j].transformMat = lookAt;
			a3animation_update_ik(activeHS, baseHS, poseGroup); //unsure if this is all thats needed to be done?
			a3animation_update_fk(activeHS, baseHS, poseGroup);*/

		}

		// RIGHT ARM REACH
		{
			// right wrist effector
			sceneObject = demoMode->obj_skeleton_wristEffector_r_ctrl;
			a3real4Real4x4Product(controlLocator_wristEffector.v, controlToSkeleton.m,
				demoMode->sceneGraphState->localSpace->pose[sceneObject->sceneGraphIndex].transformMat.v3.v);
			j = j_wrist = a3hierarchyGetNodeIndex(activeHS->hierarchy, "mixamorig:RightHand");
			jointTransform_wrist = activeHS->objectSpace->pose[j].transformMat;

			// right wrist constraint
			sceneObject = demoMode->obj_skeleton_wristConstraint_r_ctrl;
			a3real4Real4x4Product(controlLocator_wristConstraint.v, controlToSkeleton.m,
				demoMode->sceneGraphState->localSpace->pose[sceneObject->sceneGraphIndex].transformMat.v3.v);
			j = j_elbow = a3hierarchyGetNodeIndex(activeHS->hierarchy, "mixamorig:RightForeArm");
			jointTransform_elbow = activeHS->objectSpace->pose[j].transformMat;

			// right wrist base
			j = j_shoulder = a3hierarchyGetNodeIndex(activeHS->hierarchy, "mixamorig:RightArm");
			jointTransform_shoulder = activeHS->objectSpace->pose[j].transformMat;
			controlLocator_wristBase = jointTransform_shoulder.v3;


			// ****TO-DO: 
			// solve positions and orientations for joints
			// in this example, +X points away from child, +Y is normal
			// 1) check if solution exists - done
			//	-> get vector between base and end effector; if it extends max length, straighten limb
			//	-> position of end effector's target is at the minimum possible distance along this vector
			

			// can we solve ik?
			a3real effectorDist = a3real3Distance(controlLocator_wristEffector.v, controlLocator_wristBase.v);
			a3real chainLength = a3real3Distance(jointTransform_shoulder.v3.xyz.v, jointTransform_elbow.v3.xyz.v) + 
				a3real3Distance(jointTransform_elbow.v3.xyz.v, jointTransform_wrist.v3.xyz.v);
			
			a3vec4 shoulderToEffector = jointTransform_shoulder.v3;

			a3real4Sub(shoulderToEffector.v, controlLocator_wristEffector.v);
			a3real4 sTENorm = { shoulderToEffector.v0, shoulderToEffector.v1, shoulderToEffector.v2, shoulderToEffector.v3 };
			a3real4Normalize(sTENorm);

			if (effectorDist > chainLength)
			{
				// make arm straight
				
				a3real shoulderElbowDist = a3real4Distance(jointTransform_shoulder.v3.xyz.v, jointTransform_elbow.v3.xyz.v);
				a3real4r elbowPos = sTENorm;
				a3real4MulS(elbowPos, shoulderElbowDist);
				a3real shoulderWristDist = a3real4Distance(jointTransform_shoulder.v3.xyz.v, jointTransform_wrist.v3.xyz.v);
				a3real4r wristPos = sTENorm;
				a3real4MulS(wristPos, shoulderWristDist);


				// dir vector -> vector from shoulder to effector
				// elbow pos -> normalized dir vector * mag of shoulder to elbow
				// wrist pos -> normalized dir vector * mag of shoulder to wrist

				//activeHS->localSpace->pose;
				//activeHS->animPose->pose[j_wrist].transformMat = demoMode->obj_skeleton_wristEffector_r_ctrl->modelMat;
				//a3kinematicsSolveInverse(activeHS);
			}
			else
			{
				// solve for ik
				// 
				// wrist pos -> pos of effector
				//a3vec4* wristPos = demoMode->obj_skeleton_wristEffector_r_ctrl->modelMat.v;
				// c = constraint pos - shoulder pos (unsure what exactly c is tho)
				//a3vec4* c = demoMode->obj_skeleton_wristConstraint_r_ctrl->modelMat.v;
				//a3real4Sub(c->v, jointTransform_shoulder.v->xyz.v);
				// d -> shoulder to effector (should already have this)
				// n -> cross (c, d) (also unsure of what n does exactly but good to have)
				//a3real* n = 0;
				//a3real3Cross(n, c->v, shoulderToEffector.v);
				//a3kinematicsSolveInversePartial(activeHS, j_shoulder, activeHS->hierarchy->numNodes);
			}


			// ****TO-DO: 
			// reassign resolved transforms to OBJECT-SPACE matrices
			// work from root to leaf too get correct transformations

		}
	}
}

void a3animation_update_animation(a3_DemoMode1_Animation* demoMode, a3f64 const dt,
	a3boolean const updateIK)
{
	a3_HierarchyState* activeHS_fk = demoMode->hierarchyState_skel_fk;
	a3_HierarchyState* activeHS_ik = demoMode->hierarchyState_skel_ik;
	a3_HierarchyState* activeHS = demoMode->hierarchyState_skel_final;
	a3_HierarchyState const* baseHS = demoMode->hierarchyState_skel_base;
	a3_HierarchyPoseGroup const* poseGroup = demoMode->hierarchyPoseGroup_skel;

	// switch controller to see different states
	// A is idle, arms down; B is skin test, arms out
	a3_ClipController* clipCtrl_fk = demoMode->clipCtrlA;
	a3ui32 sampleIndex0, sampleIndex1;

	// resolve FK state
	// update clip controller, keyframe lerp
	a3clipControllerUpdate(clipCtrl_fk, dt);
	sampleIndex0 = demoMode->clipPool->keyframe[clipCtrl_fk->keyframeIndex].sampleIndex0;
	sampleIndex1 = demoMode->clipPool->keyframe[clipCtrl_fk->keyframeIndex].sampleIndex1;
	a3hierarchyPoseLerp(activeHS_fk->animPose,
		poseGroup->hpose + sampleIndex0, poseGroup->hpose + sampleIndex1,
		(a3real)clipCtrl_fk->keyframeParam, activeHS_fk->hierarchy->numNodes);
	// run FK pipeline
	a3animation_update_fk(activeHS_fk, baseHS, poseGroup);

	// resolve IK state
	// copy FK to IK
	a3hierarchyPoseCopy(
		activeHS_ik->animPose,	// dst: IK anim
		activeHS_fk->animPose,	// src: FK anim
		//baseHS->animPose,	// src: base anim
		activeHS_ik->hierarchy->numNodes);
	// run FK
	a3animation_update_fk(activeHS_ik, baseHS, poseGroup);
	if (updateIK)
	{
		// invert object-space
		a3hierarchyStateUpdateObjectInverse(activeHS_ik);
		// run solvers
		a3animation_update_applyEffectors(demoMode, activeHS_ik, baseHS, poseGroup);
		// run full IK pipeline (if not resolving with effectors)
		//a3animation_update_ik(activeHS_ik, baseHS, poseGroup);
	}

	// blend FK/IK to final
	// testing: copy source to final
	a3hierarchyPoseCopy(activeHS->animPose,	// dst: final anim
		//activeHS_fk->animPose,	// src: FK anim
		activeHS_ik->animPose,	// src: IK anim
		//baseHS->animPose,	// src: base anim (identity)
		activeHS->hierarchy->numNodes);
	// run FK pipeline (skinning optional)
	a3animation_update_fk(activeHS, baseHS, poseGroup);
	a3animation_update_skin(activeHS, baseHS);
}


void a3animation_update_sceneGraph(a3_DemoMode1_Animation* demoMode, a3f64 const dt)
{
	a3ui32 i;
	a3mat4 scaleMat = a3mat4_identity;

	a3demo_update_objects(dt, demoMode->object_scene, animationMaxCount_sceneObject, 0, 0);
	a3demo_update_objects(dt, demoMode->obj_camera_main, 1, 1, 0);

	a3demo_updateProjectorViewProjectionMat(demoMode->proj_camera_main);

	// apply scales to objects
	for (i = 0; i < animationMaxCount_sceneObject; ++i)
	{
		a3demo_applyScale_internal(demoMode->object_scene + i, scaleMat.m);
	}

	// update skybox
	a3demo_update_bindSkybox(demoMode->obj_camera_main, demoMode->obj_skybox);

	for (i = 0; i < animationMaxCount_sceneObject; ++i)
		demoMode->sceneGraphState->localSpace->pose[i].transformMat = demoMode->object_scene[i].modelMat;
	a3kinematicsSolveForward(demoMode->sceneGraphState);
	a3hierarchyStateUpdateLocalInverse(demoMode->sceneGraphState);
	a3hierarchyStateUpdateObjectInverse(demoMode->sceneGraphState);
}

void a3animation_update(a3_DemoState* demoState, a3_DemoMode1_Animation* demoMode, a3f64 const dt)
{
	a3ui32 i;
	a3real const dtr = (a3real)dt;
	a3_DemoModelMatrixStack matrixStack[animationMaxCount_sceneObject];

	// active camera
	a3_DemoProjector const* activeCamera = demoMode->projector + demoMode->activeCamera;
	a3_DemoSceneObject const* activeCameraObject = activeCamera->sceneObject;

	// skeletal
	if (demoState->updateAnimation)
		a3animation_update_animation(demoMode, dt, 1);

	// update scene graph local transforms
	a3animation_update_sceneGraph(demoMode, dt);

	// update matrix stack data using scene graph
	for (i = 0; i < animationMaxCount_sceneObject; ++i)
	{
		a3demo_updateModelMatrixStack(matrixStack + i,
			activeCamera->projectionMat.m,
			demoMode->sceneGraphState->objectSpace->pose[demoMode->obj_camera_main->sceneGraphIndex].transformMat.m,
			demoMode->sceneGraphState->objectSpaceInv->pose[demoMode->obj_camera_main->sceneGraphIndex].transformMat.m,
			demoMode->sceneGraphState->objectSpace->pose[demoMode->object_scene[i].sceneGraphIndex].transformMat.m,
			a3mat4_identity.m);
	}

	// prepare and upload graphics data
	a3animation_update_graphics(demoState, demoMode, matrixStack, demoMode->hierarchyState_skel_final);

	// testing: reset IK effectors to lock them to FK result
	{
		//void a3animation_load_resetEffectors(a3_DemoMode1_Animation * demoMode,
		//	a3_HierarchyState * hierarchyState, a3_HierarchyPoseGroup const* poseGroup);
		//a3animation_load_resetEffectors(demoMode,
		//	demoMode->hierarchyState_skel_final, demoMode->hierarchyPoseGroup_skel);
	}

	// ****DONE:
	// process input
	demoMode->pos = demoMode->obj_skeleton_ctrl->position.xy;
	demoMode->rot = a3deg2rad(-a3trigValid_asin(demoMode->obj_skeleton_ctrl->euler.z));


	/*switch (demoMode->ctrl_position)
	{
	case animation_input_direct:
		demoMode->pos.x = (a3real)demoMode->axis_l[0];
		demoMode->pos.y = (a3real)demoMode->axis_l[1];
		break;

	case animation_input_euler:
		demoMode->vel.x = (a3real)demoMode->axis_l[0];
		demoMode->vel.y = (a3real)demoMode->axis_l[1];

		demoMode->pos.x = a3EulerIntegration(demoMode->pos.x, demoMode->vel.x, dtr);
		demoMode->pos.y = a3EulerIntegration(demoMode->pos.y, demoMode->vel.y, dtr);
		break;

		/*case animation_input_kinematic:
			// not quite working right
			demoMode->acc.x = (a3real)demoMode->axis_l[0];
			demoMode->acc.y = (a3real)demoMode->axis_l[1];

			demoMode->pos.x = a3KinematicIntegration(demoMode->pos.x, demoMode->vel.x, demoMode->acc.x, dtr);
			demoMode->pos.y = a3KinematicIntegration(demoMode->pos.y, demoMode->vel.y, demoMode->acc.y, dtr);

			demoMode->vel.x = a3EulerIntegration(demoMode->vel.x, demoMode->acc.x, dtr);
			demoMode->vel.y = a3EulerIntegration(demoMode->vel.y, demoMode->acc.y, dtr);
			break;*/
	}

	/*switch (demoMode->ctrl_rotation)
	{
	case animation_input_direct:
		demoMode->rot += ((a3real)demoMode->axis_r[0] * (a3real_twopi - a3real_pi));
		break;
	case animation_input_euler:
		demoMode->velr += ((a3real)demoMode->axis_r[0] * (a3real_twopi - a3real_pi));
		demoMode->rot = a3EulerIntegration(demoMode->rot, demoMode->velr, dtr);
		break;
	case animation_input_kinematic:
		demoMode->accr += ((a3real)demoMode->axis_r[0] * (a3real_twopi - a3real_pi));
		demoMode->rot = a3KinematicIntegration(demoMode->rot, demoMode->velr, demoMode->accr, dtr);
		break;

	}

	switch (demoMode->ctrl_target)
	{
	case animation_ctrl_character:
		// apply input
		demoMode->obj_skeleton_ctrl->position.x = +(demoMode->pos.x);
		demoMode->obj_skeleton_ctrl->position.y = +(demoMode->pos.y);
		demoMode->obj_skeleton_ctrl->euler.z = -a3trigValid_sind(demoMode->rot);
		break;
		*/
	//}

//
//}


//-----------------------------------------------------------------------------
