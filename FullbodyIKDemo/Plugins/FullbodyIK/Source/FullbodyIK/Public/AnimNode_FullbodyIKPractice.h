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

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category=IK)
	TArray<FName> IkEndBoneNames;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category=IK)
	UFullbodyIKSetting* Setting;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category=IK)
	int32 EffectorCountMax;

	FAnimNode_FullbodyIKPractice();

	// FAnimNode_Base interface
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	struct FBuffer
	{
	public:
		FBuffer()
			: Elements(nullptr)
			, SizeX(0)
			, SizeY(0)
		{
		}

		FBuffer(float* InElements, int32 InSizeX)
			: Elements(InElements)
			, SizeX(InSizeX)
			, SizeY(1)
		{
			Elements = new (InElements) float(sizeof(float) * SizeX * SizeY);
		}

		FBuffer(float* InElements, int32 InSizeY, int32 InSizeX)
			: Elements(InElements)
			, SizeX(InSizeX)
			, SizeY(InSizeY)
		{
			Elements = new (InElements) float(sizeof(float) * SizeX * SizeY);
		}

		void Reset()
		{
			FMemory::Memzero(Elements, sizeof(float) * SizeX * SizeY);
		}

		float& Ref(int32 X)
		{
			return Elements[X];
		}

		float& Ref(int32 Y, int32 X)
		{
			return Elements[Y * SizeX + X];
		}

		float* Ptr()
		{
			return Elements;
		}

	private:
		float* Elements;
		int32 SizeX;
		int32 SizeY;
	};

	struct FSolverInternal
	{
	public:
		FSolverInternal()
		{
		}

		int32 BoneIndex;
		int32 ParentBoneIndex;
		int32 BoneIndicesIndex;
		FTransform LocalTransform;
		FTransform ComponentTransform;
		FTransform InitLocalTransform;
		FTransform InitComponentTransform;
		bool bTranslation;
		bool bLimited;
		FFullbodyIKSolverAxis X;
		FFullbodyIKSolverAxis Y;
		FFullbodyIKSolverAxis Z;
	};

	static const int32 AXIS_COUNT = 3;

	TArray<int32> BoneIndices;
	int32 BoneCount;
	int32 BoneAxisCount;

	TMap<int32, FSolverInternal> SolverInternals;
	TMap<int32, TArray<int32>> SolverTree;

	FFullbodyIKSolver GetSolver(FName BoneName) const;
};
