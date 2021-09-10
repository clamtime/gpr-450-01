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
	inline definitions for keyframe animation controller.
*/

#ifdef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H
#ifndef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#define __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL


//-----------------------------------------------------------------------------

// update clip controller
inline a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt)
{
	a3_Clip * currentClip = clipCtrl->clipPool->clip + clipCtrl->currentClipIndex;

	// pre-resolution: apply the time update -> inside switch statement
	// resolve time - 7 cases
	switch (clipCtrl->playbackDirection)
	{
	// playback is PAUSED
	case 0:
		// no change in time
		break;

	// playback is FORWARD
	case 1:
		clipCtrl->keyframeTime += dt;
		clipCtrl->clipTime += dt;

		if (clipCtrl->clipTime >= currentClip->duration)
		{
			// end of clip
			if (clipCtrl->terminusAction == 0)// stop
				clipCtrl->playbackDirection = 0; 
			else if (clipCtrl->terminusAction == 1)// loop
			{
				clipCtrl->keyframeTime -= (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration;
				clipCtrl->clipTime -= currentClip->duration;
				clipCtrl->currentKeyframeIndex = currentClip->firstKeyframeIndex;
			}
			else if (clipCtrl->terminusAction == -1)// reverse
			{
				clipCtrl->playbackDirection = -1; 
				clipCtrl->keyframeTime -= (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration;
				clipCtrl->keyframeTime = (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration - clipCtrl->keyframeTime;
				clipCtrl->clipTime = currentClip->duration - (clipCtrl->clipTime - currentClip->duration);
			}
		}
		else if (clipCtrl->keyframeTime >= (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration)
		{
			// move to next keyframe
			clipCtrl->keyframeTime -= (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration;
			clipCtrl->currentKeyframeIndex++;
		}
		else
		{
			// continue playing, same keyframe
		}
		break;

	// playback is REVERSE
	case -1:
		clipCtrl->keyframeTime -= dt;
		clipCtrl->clipTime -= dt;

		if (clipCtrl->clipTime < 0)
		{
			// beginning of clip
			if (clipCtrl->terminusAction == 0)// stop
				clipCtrl->playbackDirection = 0;
			else if (clipCtrl->terminusAction == 1)// loop
			{
				clipCtrl->keyframeTime += (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration;
				clipCtrl->clipTime += currentClip->duration;
				clipCtrl->currentKeyframeIndex = currentClip->lastKeyframeIndex;
			}
			else if (clipCtrl->terminusAction == -1)// reverse
			{
				clipCtrl->playbackDirection = 1;
				clipCtrl->keyframeTime *= -1; // gets the amound of time over and makes it positive
				clipCtrl->clipTime *= -1;
			}
		}
		else if (clipCtrl->keyframeTime < 0)
		{
			// move to previous keyframe
			clipCtrl->keyframeTime += (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->duration;
			clipCtrl->currentKeyframeIndex--;
		}
		else
		{
			// continue playing, same keyframe
		}
		break;

	}

	// post-resolution - normalize time/parameters
	clipCtrl->keyframeParameter = clipCtrl->keyframeTime * (currentClip->keyframePool->keyframe + clipCtrl->currentKeyframeIndex)->durationInv;
	clipCtrl->clipParameter = clipCtrl->clipTime * currentClip->durationInv;
	
	return -1;
}

// set clip to play
inline a3i32 a3clipControllerSetClip(a3_ClipController* clipCtrl, const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool)
{
	clipCtrl->currentClipIndex = clipIndex_pool;

	return -1;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#endif	// __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H