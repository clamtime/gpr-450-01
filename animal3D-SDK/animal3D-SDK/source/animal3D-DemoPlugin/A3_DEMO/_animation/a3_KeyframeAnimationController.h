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
	
	a3_KeyframeAnimationController.h
	Keyframe animation clip controller. Basically a frame index manager. Very 
	limited in what one can do with this; could potentially be so much more.
*/

#ifndef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H
#define __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H


#include "a3_KeyframeAnimation.h"


//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
typedef struct a3_ClipController			a3_ClipController;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

// clip controller
// metaphor: playhead
struct a3_ClipController
{
	// name of controller
	a3byte name[a3keyframeAnimation_nameLenMax];

	// current clip being played (if index should it be a3ui32?)
	a3ui32 currentClipIndex;
	a3index clipIndex_pool; // diff way

	// current time relative to start of clip [0, clip duration)
	a3real clipTime;

	// clip parameter, normalized clipTime [0, 1)
	a3real clipParameter;

	// current keyframe being played
	a3index currentKeyframeIndex, nextKeyframeIndex;
	a3index keyframeIndex0_clip, keyframeIndex1_clip;

	// current time relative to start of keyframe [0, keyframe duration)
	a3real keyframeTime;

	// keyframe parameter, normalized keyframeTime [0, 1)
	a3real keyframeParameter;

	// +1 forward, 0 pause, -1 reverse
	a3ui32 playbackDirection;

	// +1 loop, 0 stop, -1 ping-pong
	a3ui32 terminusAction;

	// clip pool to play from
	a3_ClipPool* clipPool;

	a3_Clip const* clipPtr;
	a3_Keyframe const* keyframePtr0, * keyframePtr1;
};


//-----------------------------------------------------------------------------

// initialize clip controller
a3i32 a3clipControllerInit(a3_ClipController* clipCtrl_out, const a3byte ctrlName[a3keyframeAnimation_nameLenMax], const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool, const a3ui32 terminusAction);

// update clip controller
a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt);

// set clip to play
a3i32 a3clipControllerSetClip(a3_ClipController* clipCtrl, const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool);

// evaluate the current value at time
a3i32 a3clipControllerEvaluate(a3_ClipController const* clipCtrl, a3_Sample* sample_out);

//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_KeyframeAnimationController.inl"


#endif	// !__ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H