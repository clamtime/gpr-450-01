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
	
	a3_HierarchyState.c
	Implementation of transform hierarchy state.
*/

#include "../a3_HierarchyState.h"

#include <stdlib.h>
#include <string.h>


//-----------------------------------------------------------------------------

// initialize pose set given an initialized hierarchy and key pose count
a3i32 a3hierarchyPoseGroupCreate(a3_HierarchyPoseGroup *poseGroup_out, const a3_Hierarchy *hierarchy, const a3ui32 poseCount)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (poseGroup_out && hierarchy && !poseGroup_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		const a3ui32 dataSize = sizeof(a3_HierarchyPose) * poseCount;

		// allocate everything (one malloc)
		//??? = (...)malloc(sz);
		poseGroup_out->hPose = (a3_HierarchyPose*)malloc(dataSize);

		// set pointers
		poseGroup_out->hierarchy = hierarchy;
		poseGroup_out->poseCount = poseCount;
		poseGroup_out->spatialPoseCount = poseCount * hierarchy->numNodes;
		a3hierarchyPoseReset(poseGroup_out->hPose, poseCount);

		// reset all data
		// ????
		for (a3ui32 i = 0; i < poseGroup_out->poseCount; i++)
		{
			a3spatialPoseReset(poseGroup_out->hPose[i].spatialPose);
		}


		// done
		return 1;
	}
	return -1;
}

// release pose set
a3i32 a3hierarchyPoseGroupRelease(a3_HierarchyPoseGroup *poseGroup)
{
	// validate param exists and is initialized
	if (poseGroup && poseGroup->hierarchy)
	{
		// release everything (one free)
		//free(???);
		free(poseGroup->hPose);

		// reset pointers
		poseGroup->hierarchy = 0;
		poseGroup->hPose = 0;
		poseGroup->poseCount = 0;

		// done
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// initialize hierarchy state given an initialized hierarchy
a3i32 a3hierarchyStateCreate(a3_HierarchyState *state_out, const a3_Hierarchy *hierarchy)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (state_out && hierarchy && !state_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		const a3ui32 dataSize = sizeof(a3_HierarchyPoseGroup) * 4;

		// allocate everything (one malloc)
		//??? = (...)malloc(sz);
		state_out->poseGroup = (a3_HierarchyPoseGroup*)malloc(dataSize);

		// set pointers
		state_out->hierarchy = hierarchy;

		// reset all data
		

		// done
		return 1;
	}
	return -1;
}

// release hierarchy state
a3i32 a3hierarchyStateRelease(a3_HierarchyState *state)
{
	// validate param exists and is initialized
	if (state && state->hierarchy)
	{
		// release everything (one free)
		//free(???);
		free(state->poseGroup);

		// reset pointers
		state->hierarchy = 0;
		state->poseGroup = 0;
		state->samplePose = 0;
		state->localSpacePose = 0;
		state->objectSpacePose = 0;

		// done
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// load HTR file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadHTR(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->poseCount && hierarchy_out && !hierarchy_out->numNodes && resourceFilePath && *resourceFilePath)
	{
		//a3_FileStream * fileStream;
		//a3fileStreamOpenRead(fileStream, resourceFilePath);

		// get data from file into hierarchy
		// num segments -> number of necessary nodes
		// num frames -> number of poses
		// euler rotation order -> order to read the euler angles into the rotation matrix in (xyz, zyx, etc)
		// scale factor -> global scale data
		
		// Segment names & hierarchy 
		//	-> create nodes with the respective names of strings and connect them to the parent nodes

		// BasePosition
		//	-> starting with the base pose, record transformation data for each node
		//	-> order: segment name, translation, rotation, segment length
		//	-> translation data appears in xyz, convert to proper euler angle configuration depending on what was specified earlier
	}
	return -1;
}

// load BVH file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadBVH(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->poseCount && hierarchy_out && !hierarchy_out->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

// save HTR file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveHTR(a3_HierarchyPoseGroup const* poseGroup, a3_Hierarchy* hierarchy, const a3byte* resourceFilePath)
{
	if (poseGroup && !poseGroup->poseCount && hierarchy && !hierarchy->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

// save BVH file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveBVH(a3_HierarchyPoseGroup const* poseGroup, a3_Hierarchy* hierarchy, const a3byte* resourceFilePath)
{
	if (poseGroup && !poseGroup->poseCount && hierarchy && !hierarchy->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

//-----------------------------------------------------------------------------
