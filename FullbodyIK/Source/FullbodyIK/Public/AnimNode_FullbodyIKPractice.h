#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "CommonAnimTypes.h"
#include "FullbodyIKSetting.h"
#include "AnimNode_FullbodyIKPractice.generated.h"

UENUM(BlueprintType)
enum class EFullbodyIkEffectorTypePractice : uint8
{
	KeepLocation,
	KeepRotation,
	KeepLocationAndRotation,
	FollowOriginalLocation,
	FollowOriginalRotation,
	FollowOriginalLocationAndRotation,
};

USTRUCT(BlueprintType)
struct FULLBODYIK_API FAnimNode_FullbodyIkEffectorPractice
{
	GENERATED_BODY()

public:
	FAnimNode_FullbodyIkEffectorPractice()
		: EffectorType(EFullbodyIkEffectorTypePractice::KeepLocation)
		, EffectorBoneName(NAME_None)
		, RootBoneName(NAME_None)
		, Location(FVector::ZeroVector)
		, Rotation(FRotator::ZeroRotator)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	EFullbodyIkEffectorTypePractice EffectorType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	FName EffectorBoneName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	FName RootBoneName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	FVector Location;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	FRotator Rotation;
};

USTRUCT(BlueprintType)
struct FULLBODYIK_API FAnimNode_FullbodyIkEffectorsPractice
{
	GENERATED_BODY()

public:
	FAnimNode_FullbodyIkEffectorsPractice()
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector)
	TArray<FAnimNode_FullbodyIkEffectorPractice> Effectors;
};

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

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=IK)
	FVector CenterOfMass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=EndEffector, meta=(PinShownByDefault))
	FAnimNode_FullbodyIkEffectorsPractice Effectors;

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
			: BoneIndex(INDEX_NONE)
			, ParentBoneIndex(INDEX_NONE)
			, BoneIndicesIndex(INDEX_NONE)
			, LocalTransform(FTransform::Identity)
			, ComponentTransform(FTransform::Identity)
			, InitLocalTransform(FTransform::Identity)
			, InitComponentTransform(FTransform::Identity)
			, bTranslation(false)
			, bLimited(false)
			, Mass(1.0f)
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
		float Mass;
		FFullbodyIKSolverAxis X;
		FFullbodyIKSolverAxis Y;
		FFullbodyIKSolverAxis Z;
	};

	struct FEffectorInternal
	{
	public:
		FEffectorInternal()
			: EffectorType(EFullbodyIkEffectorTypePractice::KeepLocation)
			, EffectorBoneIndex(INDEX_NONE)
			, RootBoneIndex(INDEX_NONE)
			, ParentBoneIndex(INDEX_NONE)
			, Location(FVector::ZeroVector)
			, Rotation(FRotator::ZeroRotator)
		{
		}

		EFullbodyIkEffectorTypePractice EffectorType;
		int32 EffectorBoneIndex;
		int32 RootBoneIndex;
		int32 ParentBoneIndex;
		FVector Location;
		FRotator Rotation;
	};

	FFullbodyIKSolver GetSolver(FName BoneName) const;

	FTransform GetWorldSpaceBoneTransform(const int32& BoneIndex) const;
	FVector GetWorldSpaceBoneLocation(const int32& BoneIndex) const;
	FQuat GetWorldSpaceBoneRotation(const int32& BoneIndex) const;
	FTransform GetLocalSpaceBoneTransform(const int32& BoneIndex) const;
	FVector GetLocalSpaceBoneLocation(const int32& BoneIndex) const;
	FQuat GetLocalSpaceBoneRotation(const int32& BoneIndex) const;

	void CalcJacobian(const FEffectorInternal& EffectorInternal, float* Jacobian);

	void SolveSolver(
		int32 BoneIndex,
		const FTransform& ParentComponentTransform,
		const TFunction<void(int32, FVector&, FVector&)>& LocationOffsetProcess,
		const TFunction<void(int32, FRotator&, FRotator&)>& RotationOffsetProcess);

	void UpdateCenterOfMass();

	static const int32 AXIS_COUNT = 3;

	TArray<int32> BoneIndices;
	int32 BoneCount;
	int32 BoneAxisCount;

	TMap<int32, FSolverInternal> SolverInternals;
	TMap<int32, TArray<int32>> SolverTree;

	UObject* CachedAnimInstanceObject;
	FTransform CachedComponentTransform;

	TArray<float> ElementsJ;
	TArray<float> ElementsJt;
	TArray<float> ElementsJtJ;
	TArray<float> ElementsJtJi;
	TArray<float> ElementsJp;
	TArray<float> ElementsW0;
	TArray<float> ElementsWi;
	TArray<float> ElementsJtWi;
	TArray<float> ElementsJtWiJ;
	TArray<float> ElementsJtWiJi;
	TArray<float> ElementsJtWiJiJt;
	TArray<float> ElementsJwp;
	TArray<float> ElementsRt1;
	TArray<float> ElementsEta;
	TArray<float> ElementsEtaJ;
	TArray<float> ElementsEtaJJp;
	TArray<float> ElementsRt2;
};
