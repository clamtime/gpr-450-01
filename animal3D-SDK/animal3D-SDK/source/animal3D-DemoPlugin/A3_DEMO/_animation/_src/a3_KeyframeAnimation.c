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
	
	a3_KeyframeAnimation.c
	Implementation of keyframe animation interfaces.
*/

#include "../a3_KeyframeAnimation.h"

#include <stdlib.h>
#include <string.h>


// macros to help with names
#define A3_CLIP_DEFAULTNAME		("unnamed clip")
#define A3_CLIP_SEARCHNAME		((clipName && *clipName) ? clipName : A3_CLIP_DEFAULTNAME)


//-----------------------------------------------------------------------------

// allocate keyframe pool
a3i32 a3keyframePoolCreate(a3_KeyframePool* keyframePool_out, const a3ui32 count)
{
	// allocating space for the array
	keyframePool_out->count = count;
	keyframePool_out->keyframe = (a3_Keyframe*)calloc(count, sizeof(a3_Keyframe));
	
	for (a3ui32 i = 0; i < count; i++)
	{
		// initializing each element of the array
		a3keyframeInit(keyframePool_out->keyframe + i, 0.5, 30 + i, i);
	}

	return -1;
}

// release keyframe pool
a3i32 a3keyframePoolRelease(a3_KeyframePool* keyframePool)
{
	free(keyframePool->keyframe);
	return -1;
}

// initialize keyframe
a3i32 a3keyframeInit(a3_Keyframe* keyframe_out, const a3real duration, const a3ui32 value_x, const a3ui32 index)
{
	// setting the duration and duration inverse
	keyframe_out->duration = duration;
	keyframe_out->durationInv = 1 / duration;

	// setting the data
	keyframe_out->data = value_x;

	//setting the index
	keyframe_out->index = index;

	return -1;
}


// allocate clip pool
a3i32 a3clipPoolCreate(a3_ClipPool* clipPool_out, const a3ui32 count, a3_Keyframe* keyframePool)
{
	// allocating space for the array
	clipPool_out->count = count;
	clipPool_out->clip = (a3_Clip*)calloc(count, sizeof(a3_Clip));

	for (a3ui32 i = 0; i < count; i++)
	{
		a3_Clip* currentClip = clipPool_out->clip + i;
		// initializing each element of the array
		a3clipInit(currentClip, "Clip " + i, keyframePool, 0, 20);
	}

	return -1;
}

// release clip pool
a3i32 a3clipPoolRelease(a3_ClipPool* clipPool)
{
	free(clipPool->clip);
	return -1;
}

// initialize clip with first and last indices
a3i32 a3clipInit(a3_Clip* clip_out, const a3byte clipName[a3keyframeAnimation_nameLenMax], const a3_KeyframePool* keyframePool, const a3ui32 firstKeyframeIndex, const a3ui32 finalKeyframeIndex)
{
	clip_out->index = 0;
	*clip_out->name = clipName[a3keyframeAnimation_nameLenMax];
	*clip_out->keyframePool = *keyframePool;
	clip_out->firstKeyframeIndex = firstKeyframeIndex;
	clip_out->lastKeyframeIndex  = finalKeyframeIndex;
	return -1;
}

// get clip index from pool
a3i32 a3clipGetIndexInPool(const a3_ClipPool* clipPool, const a3byte clipName[a3keyframeAnimation_nameLenMax])
{
	for (a3ui32 i = 0; i < clipPool->count; i++)
	{
		a3_Clip * currentClip = clipPool->clip + i;
		if (currentClip->name == clipName)
		{
			return i;
		}
	}
	return -1;
}


//-----------------------------------------------------------------------------
