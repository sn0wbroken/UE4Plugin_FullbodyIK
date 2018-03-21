#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "CommonAnimTypes.h"
#include "FullbodyIKSetting.h"
#include "AnimNode_FullbodyIKPractice.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct FULLBODYIK_API FAnimNode_FullbodyIKPractice : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()
};
