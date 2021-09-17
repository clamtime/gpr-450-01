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
	
	a3_KeyframeAnimationController.c
	Implementation of keyframe animation controller.
*/

#include "../a3_KeyframeAnimationController.h"

#include <string.h>


//-----------------------------------------------------------------------------

// initialize clip controller
a3i32 a3clipControllerInit(a3_ClipController* clipCtrl_out, const a3byte ctrlName[a3keyframeAnimation_nameLenMax], const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool, const a3ui32 terminusAction)
{
	*clipCtrl_out->name = ctrlName[a3keyframeAnimation_nameLenMax];
	*clipCtrl_out->clipPool = *clipPool;
	clipCtrl_out->currentClipIndex = clipIndex_pool;
	clipCtrl_out->terminusAction = terminusAction;
	return -1;
}

a3i32 a3clipControllerEvaluate(a3_ClipController const* clipCtrl, a3_Sample* sample_out)
{
	if (clipCtrl && clipCtrl->clipPtr && sample_out)
	{
		// 0: no interpolation
		//*sample_out = clipCtrl->keyframePtr0->sample;

		// 1: nearest
		// if (u < 0.5) then k0, else k1

		// 2: lerp
		// k = k0 + (k1 - k0)u
		sample_out->time = clipCtrl->keyframeTime;
		sample_out->value = a3lerp(
			clipCtrl->keyframePtr0->sample.value,
			clipCtrl->keyframePtr1->sample.value,
			clipCtrl->keyframeParameter);

		// 3: catmull-rom/cubic Hermite
		//    CatmullRom(kP, k0, k1, kN, u)
		//        kP: keyframe before k0
		//        kN: keyframe after k1
		sample_out->time = clipCtrl->keyframeTime;
		sample_out->value = a3CatmullRom(clipCtrl->keyframePtrP->sample.value, clipCtrl->keyframePtr0->sample.value,
			clipCtrl->keyframePtr1->sample.value, clipCtrl->keyframePtrN->sample.value, clipCtrl->keyframeParameter);
	}
	return -1;
}

//-----------------------------------------------------------------------------
