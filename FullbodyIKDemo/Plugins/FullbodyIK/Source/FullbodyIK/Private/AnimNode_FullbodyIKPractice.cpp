#include "AnimNode_FullbodyIKPractice.h"
#include "Engine/Engine.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include "SceneManagement.h"
#include "AnimInstanceInterface_FullbodyIK.h"
#include "DrawDebugHelpers.h"

namespace {
FORCEINLINE float SinDegree(float Degree) { return FMath::Sin(FMath::DegreesToRadians(Degree)); }
FORCEINLINE float CosDegree(float Degree) { return FMath::Cos(FMath::DegreesToRadians(Degree)); }

FORCEINLINE FMatrix RotX(float Roll)
{
	return FMatrix(
		FPlane(1, 0, 0, 0),
		FPlane(0, CosDegree(Roll), -SinDegree(Roll), 0),
		FPlane(0, SinDegree(Roll), CosDegree(Roll), 0),
		FPlane(0, 0, 0, 1)
	);
}

FORCEINLINE FMatrix RotY(float Pitch)
{
	return FMatrix(
		FPlane(CosDegree(Pitch), 0, SinDegree(Pitch), 0),
		FPlane(0, 1, 0, 0),
		FPlane(-SinDegree(Pitch), 0, CosDegree(Pitch), 0),
		FPlane(0, 0, 0, 1)
	);
}

FORCEINLINE FMatrix RotZ(float Yaw)
{
	return FMatrix(
		FPlane(CosDegree(Yaw), SinDegree(Yaw), 0, 0),
		FPlane(-SinDegree(Yaw), CosDegree(Yaw), 0, 0),
		FPlane(0, 0, 1, 0),
		FPlane(0, 0, 0, 1)
	);
}

FORCEINLINE FMatrix DiffRotX(float Roll)
{
	return FMatrix(
		FPlane(0, 0, 0, 0),
		FPlane(0, -SinDegree(Roll), -CosDegree(Roll), 0),
		FPlane(0, CosDegree(Roll), -SinDegree(Roll), 0),
		FPlane(0, 0, 0, 0)
	);
}

FORCEINLINE FMatrix DiffRotY(float Pitch)
{
	return FMatrix(
		FPlane(-SinDegree(Pitch), 0, CosDegree(Pitch), 0),
		FPlane(0, 0, 0, 0),
		FPlane(-CosDegree(Pitch), 0, -SinDegree(Pitch), 0),
		FPlane(0, 0, 0, 0)
	);
}

FORCEINLINE FMatrix DiffRotZ(float Yaw)
{
	return FMatrix(
		FPlane(-SinDegree(Yaw), CosDegree(Yaw), 0, 0),
		FPlane(-CosDegree(Yaw), -SinDegree(Yaw), 0, 0),
		FPlane(0, 0, 0, 0),
		FPlane(0, 0, 0, 0)
	);
}

FORCEINLINE float MatrixInverse3(float* DstMatrix, const float* SrcMatrix)
{
	float Det =
		  SrcMatrix[0 * 3 + 0] * SrcMatrix[1 * 3 + 1] * SrcMatrix[2 * 3 + 2]
		+ SrcMatrix[1 * 3 + 0] * SrcMatrix[2 * 3 + 1] * SrcMatrix[0 * 3 + 2]
		+ SrcMatrix[2 * 3 + 0] * SrcMatrix[0 * 3 + 1] * SrcMatrix[1 * 3 + 2]
		- SrcMatrix[0 * 3 + 0] * SrcMatrix[2 * 3 + 1] * SrcMatrix[1 * 3 + 2]
		- SrcMatrix[2 * 3 + 0] * SrcMatrix[1 * 3 + 1] * SrcMatrix[0 * 3 + 2]
		- SrcMatrix[1 * 3 + 0] * SrcMatrix[0 * 3 + 1] * SrcMatrix[2 * 3 + 2];

	if (Det == 0)
	{
		return Det;
	}

	DstMatrix[0 * 3 + 0] = (SrcMatrix[1 * 3 + 1] * SrcMatrix[2 * 3 + 2] - SrcMatrix[1 * 3 + 2] * SrcMatrix[2 * 3 + 1]) / Det;
	DstMatrix[0 * 3 + 1] = (SrcMatrix[0 * 3 + 2] * SrcMatrix[2 * 3 + 1] - SrcMatrix[0 * 3 + 1] * SrcMatrix[2 * 3 + 2]) / Det;
	DstMatrix[0 * 3 + 2] = (SrcMatrix[0 * 3 + 1] * SrcMatrix[1 * 3 + 2] - SrcMatrix[0 * 3 + 2] * SrcMatrix[1 * 3 + 1]) / Det;

	DstMatrix[1 * 3 + 0] = (SrcMatrix[1 * 3 + 2] * SrcMatrix[2 * 3 + 0] - SrcMatrix[1 * 3 + 0] * SrcMatrix[2 * 3 + 2]) / Det;
	DstMatrix[1 * 3 + 1] = (SrcMatrix[0 * 3 + 0] * SrcMatrix[2 * 3 + 2] - SrcMatrix[0 * 3 + 2] * SrcMatrix[2 * 3 + 0]) / Det;
	DstMatrix[1 * 3 + 2] = (SrcMatrix[0 * 3 + 2] * SrcMatrix[1 * 3 + 0] - SrcMatrix[0 * 3 + 0] * SrcMatrix[1 * 3 + 2]) / Det;

	DstMatrix[2 * 3 + 0] = (SrcMatrix[1 * 3 + 0] * SrcMatrix[2 * 3 + 1] - SrcMatrix[1 * 3 + 1] * SrcMatrix[2 * 3 + 0]) / Det;
	DstMatrix[2 * 3 + 1] = (SrcMatrix[0 * 3 + 1] * SrcMatrix[2 * 3 + 0] - SrcMatrix[0 * 3 + 0] * SrcMatrix[2 * 3 + 1]) / Det;
	DstMatrix[2 * 3 + 2] = (SrcMatrix[0 * 3 + 0] * SrcMatrix[1 * 3 + 1] - SrcMatrix[0 * 3 + 1] * SrcMatrix[1 * 3 + 0]) / Det;

	return Det;
}

FORCEINLINE float MatrixInverse4(float* DstMatrix, const float* SrcMatrix)
{
	float Det =
			SrcMatrix[0 * 4 + 0] * (
				SrcMatrix[1 * 4 + 1] * (SrcMatrix[2 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[2 * 4 + 3] * SrcMatrix[3 * 4 + 2]) -
				SrcMatrix[2 * 4 + 1] * (SrcMatrix[1 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[1 * 4 + 3] * SrcMatrix[3 * 4 + 2]) +
				SrcMatrix[3 * 4 + 1] * (SrcMatrix[1 * 4 + 2] * SrcMatrix[2 * 4 + 3] - SrcMatrix[1 * 4 + 3] * SrcMatrix[2 * 4 + 2])
				) -
			SrcMatrix[1 * 4 + 0] * (
				SrcMatrix[0 * 4 + 1] * (SrcMatrix[2 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[2 * 4 + 3] * SrcMatrix[3 * 4 + 2]) -
				SrcMatrix[2 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[3 * 4 + 2]) +
				SrcMatrix[3 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[2 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[2 * 4 + 2])
				) +
			SrcMatrix[2 * 4 + 0] * (
				SrcMatrix[0 * 4 + 1] * (SrcMatrix[1 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[1 * 4 + 3] * SrcMatrix[3 * 4 + 2]) -
				SrcMatrix[1 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[3 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[3 * 4 + 2]) +
				SrcMatrix[3 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[1 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[1 * 4 + 2])
				) -
			SrcMatrix[3 * 4 + 0] * (
				SrcMatrix[0 * 4 + 1] * (SrcMatrix[1 * 4 + 2] * SrcMatrix[2 * 4 + 3] - SrcMatrix[1 * 4 + 3] * SrcMatrix[2 * 4 + 2]) -
				SrcMatrix[1 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[2 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[2 * 4 + 2]) +
				SrcMatrix[2 * 4 + 1] * (SrcMatrix[0 * 4 + 2] * SrcMatrix[1 * 4 + 3] - SrcMatrix[0 * 4 + 3] * SrcMatrix[1 * 4 + 2])
				);

	if (Det == 0)
	{
		return Det;
	}

	VectorMatrixInverse(DstMatrix, SrcMatrix);

	return Det;
}

FORCEINLINE void MatrixTranspose(float* DstMatrix, const float* SrcMatrix, const int32 Row, const int32 Col)
{
	for (int32 i = 0; i < Row; ++i)
	{
		for (int32 j = 0; j < Col; ++j)
		{
			DstMatrix[j * Row + i] = SrcMatrix[i * Col + j];
		}
	}
}

FORCEINLINE void MatrixMultiply(float* DstMatrix, const float* SrcMatrix1, const int32 Row1, const int32 Col1, const float* SrcMatrix2, const int32 Row2, const int32 Col2)
{
	check(Col1 == Row2);

	for (int32 i = 0; i < Row1; ++i)
	{
		for (int32 j = 0; j < Col2; ++j)
		{
			float& Elem = DstMatrix[i * Col2 + j];
			Elem = 0;

			for (int32 k = 0; k < Col1; ++k)
			{
				Elem += SrcMatrix1[i * Col1 + k] * SrcMatrix2[k * Col2 + j];
			}
		}
	}
}

FORCEINLINE float MatrixDet(const float* SrcMatrix, const int32 N)
{
	float Det = 0;

	for (int32 i = 0; i < N; ++i)
	{
		float AddValue = 1;
		float SubValue = 1;

		for (int32 j = 0; j < N; ++j)
		{
			AddValue *= SrcMatrix[j * N + ((i + j) % N)];
			SubValue *= SrcMatrix[j * N + ((i + (N - j - 1)) % N)];
		}

		Det += AddValue;
		Det -= SubValue;
	}

	return Det;
}

FORCEINLINE float GetMappedRangeEaseInClamped(
	const float& InRangeMin,
	const float& InRangeMax,
	const float& OutRangeMin,
	const float& OutRangeMax,
	const float& Exp,
	const float& Value)
{
	float Pct = FMath::Clamp((Value - InRangeMin) / (InRangeMax - InRangeMin), 0.f, 1.f);
	return FMath::InterpEaseIn(OutRangeMin, OutRangeMax, Pct, Exp);
}
} // namespace

FAnimNode_FullbodyIKPractice::FAnimNode_FullbodyIKPractice()
	: Setting(nullptr)
{
}

void FAnimNode_FullbodyIKPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	Super::Initialize_AnyThread(Context);

	const USkeletalMeshComponent* Mesh = Context.AnimInstanceProxy->GetSkelMeshComponent();

	// UFullbodyIKSetting:SolversからSolverInternalsに情報をつめかえる
	for (FName IkEndBoneName : IkEndBoneNames)
	{
		FName BoneName = IkEndBoneName;

		while (true)
		{
			// エフェクタから親にさかのぼって行ってすべてSolverInternalとBoneIndicesに登録する
			int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
			if (BoneIndex == INDEX_NONE || SolverInternals.Contains(BoneIndex)) // 複数のエフェクタで途中から親を共有したら重複させない
			{
				break;
			}

			// ほげたつさんはワークデータにはInternalという名前をつける模様
			// ソルバー用のワークデータのマップに、BPから与えたデータを移し替える
			// BoneIndicesは素直にボーンの配列
			
			FSolverInternal& SolverInternal = SolverInternals.Add(BoneIndex, FSolverInternal());
			BoneIndices.Add(BoneIndex);

			FName ParentBoneName = Mesh->GetParentBone(BoneName);
			int32 ParentBoneIndex = Mesh->GetBoneIndex(ParentBoneName);
			// IkEndBoneNamesに入っているエフェクタのボーンだけでなく、その先祖のすべてのボーンに対してFFullbodyIKSolverの設定は存在しうる
			FFullbodyIKSolver Solver = GetSolver(BoneName);

			SolverInternal.BoneIndex = BoneIndex;
			SolverInternal.ParentBoneIndex = ParentBoneIndex;
			SolverInternal.BoneIndicesIndex = -1;
			SolverInternal.bTranslation = Solver.bTranslation;
			SolverInternal.X = Solver.X;
			SolverInternal.Y = Solver.Y;
			SolverInternal.Z = Solver.Z;

			if (ParentBoneIndex >= 0)
			{
				if (!SolverTree.Contains(ParentBoneIndex))
				{
					SolverTree.Add(ParentBoneIndex, TArray<int32>());
				}

				SolverTree[ParentBoneIndex].Add(BoneIndex);
			}
			
			BoneName = ParentBoneName;
		}
	}

	BoneIndices.Sort();
	BoneCount = BoneIndices.Num();
	BoneAxisCount = BoneCount * FAnimNode_FullbodyIKPractice::AXIS_COUNT;

	for (int32 i = 0; i < BoneCount; ++i)
	{
		int32 BoneIndex = BoneIndices[i];
		SolverInternals[BoneIndex].BoneIndicesIndex = i;
	}

	//
	// 行列のセットアップ
	//

	// 最大数で確保
	int32 DisplacementCount = BoneAxisCount;

	// Jはエフェクタまでのジョイント自由度*エフェクタの自由度なのでこうなっている
	// http://mukai-lab.org/content/JacobianInverseKinematics.pdf
	// 的に言うと、BoneAxisCountがNでAXIS_COUNTがM。Mが明らかに小さい
	// Jには、全エフェクタのデータを入れていると勘違いしないように
	// 各エフェクタのIK計算でそのエフェクタだけの情報を入れて使いまわす
	ElementsJ.SetNumZeroed(DisplacementCount * AXIS_COUNT);
	ElementsJt.SetNumZeroed(AXIS_COUNT * DisplacementCount);
	ElementsJtJ.SetNumZeroed(AXIS_COUNT * AXIS_COUNT);
	ElementsJtJi.SetNumZeroed(AXIS_COUNT * AXIS_COUNT);
	ElementsJp.SetNumZeroed(AXIS_COUNT * DisplacementCount);
	ElementsW0.SetNumZeroed(BoneAxisCount);
	ElementsWi.SetNumZeroed(DisplacementCount * DisplacementCount);
	ElementsJtWi.SetNumZeroed(AXIS_COUNT * DisplacementCount);
	ElementsJtWiJ.SetNumZeroed(AXIS_COUNT * AXIS_COUNT);
	ElementsJtWiJi.SetNumZeroed(AXIS_COUNT * AXIS_COUNT);
	ElementsJtWiJiJt.SetNumZeroed(AXIS_COUNT * DisplacementCount);
	ElementsJwp.SetNumZeroed(AXIS_COUNT * DisplacementCount);
	ElementsRt1.SetNumZeroed(BoneAxisCount);
	ElementsEta.SetNumZeroed(BoneAxisCount);
	ElementsEtaJ.SetNumZeroed(AXIS_COUNT);
	ElementsEtaJJp.SetNumZeroed(BoneAxisCount);
	ElementsRt2.SetNumZeroed(BoneAxisCount);

#if 0 // TODO:とりあえず加重行列はまだ考えない
	// 加重行列 W
	auto W0 = FBuffer(ElementsW0.GetData(), BoneAxisCount);
	for (int32 i = 0; i < BoneCount; ++i)
	{
		int32 BoneIndex = BoneIndices[i];
		W0.Ref(i * AXIS_COUNT + 0) = SolverInternals[BoneIndex].X.Weight;
		W0.Ref(i * AXIS_COUNT + 1) = SolverInternals[BoneIndex].Y.Weight;
		W0.Ref(i * AXIS_COUNT + 2) = SolverInternals[BoneIndex].Z.Weight;
	}
#endif
}

void FAnimNode_FullbodyIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Context, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	if (!Setting)
	{
		return;
	}

	UObject* AnimInstanceObject = Context.AnimInstanceProxy->GetAnimInstanceObject();
	// TODO:UAnimInstanceInterface_FullbodyIKについては後で調べて写経しよう　UInterfaceの動きは独特だ
	if (!AnimInstanceObject->GetClass()->ImplementsInterface(UAnimInstanceInterface_FullbodyIK::StaticClass()))
	{
		UE_LOG(LogTemp, Warning, TEXT("FAnimNode_FullbodyIK implements UAnimInstanceInterface_FullbodyIK."));
		return;
	}

	USkeletalMeshComponent* Mesh = Context.AnimInstanceProxy->GetSkelMeshComponent();
	if (Mesh == nullptr)
	{
		return;
	}

	if (Effectors.Effectors.Num() <= 0)
	{
		return;
	}

	CachedAnimInstanceObject = AnimInstanceObject;
	// TODO：これってなんなんだ？
	CachedComponentTransform = Context.AnimInstanceProxy->GetComponentTransform();

	// BPから渡されたFAnimNode_FullbodyIkEffectorPracticeデータをFEffectorInternalに移し替える
	// ほげたつさんのデモだとEffectorsは各キャラアクタのBPで作っている
	TArray<FEffectorInternal> EffectorInternals;
	for (const FAnimNode_FullbodyIkEffectorPractice& Effector : Effectors.Effectors)
	{
		if (Effector.EffectorBoneName == NAME_None || Effector.RootBoneName == NAME_None)
		{
			// 対象のボーン名を指定してないFAnimNode_FullbodyIkEffectorPracticeは無効
			continue;
		}

		FEffectorInternal EffectorInternal;
		EffectorInternal.EffectorType = Effector.EffectorType;
		EffectorInternal.EffectorBoneIndex = Mesh->GetBoneIndex(Effector.EffectorBoneName);
		EffectorInternal.RootBoneIndex = Mesh->GetBoneIndex(Effector.RootBoneName);

		if (EffectorInternal.EffectorBoneIndex == INDEX_NONE || EffectorInternal.RootBoneIndex == INDEX_NONE)
		{
			// 対象のボーン名を指定しててもスケルトンにそのボーンがなければFAnimNode_FullbodyIkEffectorPracticeは無効
			continue;
		}
		
		int32 BoneIndex = EffectorInternal.EffectorBoneIndex;

		if (!SolverInternals.Contains(BoneIndex))
		{
			// SolverInternalsにそのボーンが登録されてなければ無効。FAnimNode_FullbodyIkEffectorPracticeの設定がどのエフェクタにも影響しないということなので。
			continue;
		}

		EffectorInternal.ParentBoneIndex = SolverInternals[BoneIndex].ParentBoneIndex;
		EffectorInternal.Location = Effector.Location;
		EffectorInternal.Rotation = Effector.Rotation;

		// EffectorBoneNameで与えられたボーンを親にさかのぼっていってRootBoneNameで指定した親に一致しなければ無効
		bool bValidation = true;
		while (true)
		{
			int32 ParentBoneIndex = SolverInternals[BoneIndex].ParentBoneIndex;

			if (ParentBoneIndex == INDEX_NONE)
			{
				bValidation = false;
				break;
			}
			else if (ParentBoneIndex == EffectorInternal.RootBoneIndex)
			{
				break;
			}

			BoneIndex = ParentBoneIndex;
		}
		if (!bValidation)
		{
			continue;
		}

		EffectorInternals.Add(EffectorInternal);
	}

	int32 EffectorCount = EffectorInternals.Num();
	if (EffectorCount <= 0)
	{
		return;
	}

	for (int32 BoneIndex : BoneIndices)
	{
		IAnimInstanceInterface_FullbodyIK::Execute_InitializeBoneOffset(AnimInstanceObject, BoneIndex);

		// 初期Transformを保存
		const FCompactPoseBoneIndex& CompactPoseBoneIndex = FCompactPoseBoneIndex(BoneIndex);
		FSolverInternal& SolverInternal = SolverInternals[BoneIndex];
		SolverInternal.LocalTransform = Context.Pose.GetLocalSpaceTransform(CompactPoseBoneIndex);
		SolverInternal.ComponentTransform = Context.Pose.GetComponentSpaceTransform(CompactPoseBoneIndex);
		SolverInternal.InitLocalTransform = SolverInternal.LocalTransform;
		SolverInternal.InitComponentTransform = SolverInternal.ComponentTransform;
	}

	// Transform更新
	// LocationOffset, RotationOffsetの現在のポーズへの反映
#if 0
	// TODO:枝葉の部分なので実装は後回し
	SolveSolver(
		0,
		FTransform::Identity,
		[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
		{
			CurrentOffsetLocation += SavedOffsetLocation;
		},
		[&](int32 BoneIndex, FVector& SavedOffsetRotation, FVector& CurrentOffsetRotation)
		{
			CurrentOffsetRotation += SavedOffsetRotation;
		}
	);

	// 重心の更新
	UpdateCenterOfMass();
#endif

	int32 StepLoopCount = 0; // 反復計算をするので、ループの最大数の定義は必要
	// 反復のステップ数制限だけだとエフェクタが多ければステップ数上限を超えなくても計算回数は増えるので
	// 実際にポーズを計算した回数にも制限をもうける
	int32 EffectiveCount = 0;
	while (Setting->StepLoopCountMax > 0 && Setting->EffectiveCountMax && StepLoopCount < Setting->StepLoopCountMax)
	{
		for (int32 EffectorIndex = 0; EffectorIndex < EffectorCount; ++EffectorIndex)
		{
			// ここが計算の肝
			const FEffectorInternal& Effector = EffectorInternals[EffectorIndex];
			// TODO:なんだこれ？
			float EffectorStep[AXIS_COUNT];
			float EtaStep = 0.0f;

			FMemory::Memzero(EffectorStep, sizeof(float) * AXIS_COUNT);

			int32 DisplacementCount = 0;
			switch (Effector.EffectorType)
			{
			case EFullbodyIkEffectorTypePractice::KeepLocation:
				{
					FVector EndSolverLocation = GetWorldSpaceBoneLocation(Effector.EffectorBoneIndex);
					FVector DeltaLocation = Effector.Location - EndSolverLocation;
					float DeltaLocationSize = DeltaLocation.Size();

					if (DeltaLocationSize > Setting->ConvergenceDistance) // TODO:なんじゃこれ？
					{
						float Step = FMath::Min(Setting->StepSize, DeltaLocationSize);
						EtaStep += Step;
						FVector StepV = DeltaLocation / DeltaLocationSize * Step;
						EffectorStep[0] = StepV.X;
						EffectorStep[1] = StepV.Y;
						EffectorStep[2] = StepV.Z;
					}

					DisplacementCount = BoneAxisCount;
				}
				break;
			case EFullbodyIkEffectorTypePractice::KeepRotation:
				{
					const FSolverInternal& SolverInternal = SolverInternals[Effector.EffectorBoneIndex];

#if 0
					// Transform更新 // TODO:このSolveSolverが何をしてるかをまだわかってないな。。。
					SolveSolver(
						0,
						FTransform::Identity,
						[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
						{
						},
						[&](int32 BoneIndex, FRotator& SavedOffsetRotation, FRotator& CurrentOffsetRotation)
						{
							if (BoneIndex != Effector.EffectorBoneIndex)
							{
								return;
							}

							FTransform EffectorWorldTransform = FTransform(Effector.Rotation);
							FTransform EffectorComponentTransform = EffectorWorldTransform * CachedComponentTransform.Inverse();
							FTransform EffectorLocalTransform = EffectorComponentTransform * SolverInternals[SolverInternal.ParentBoneIndex].ComponentTransform.Inverse();

							FRotator EffectorLocalRotation = EffectorLocalTransform.Rotator();
							FRotator DeltaLocalRotation = EffectorLocalRotation - CurrentOffsetRotation;

							SavedOffsetRotation += DeltaLocalRotation;
							CurrentOffsetRotation += DeltaLocalRotation;
						}
					);
#endif
					DisplacementCount = BoneAxisCount;
				}
				break;
			case EFullbodyIkEffectorTypePractice::KeepLocationAndRotation:
				{
					const FSolverInternal& SolverInternal = SolverInternals[Effector.EffectorBoneIndex];

#if 0
					// KeepRotation側にあたる処理
					// Transform更新
					SolveSolver(0, FTransform::Identity,
						[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
						{
						},
						[&](int32 BoneIndex, FRotator& SavedOffsetRotation, FRotator& CurrentOffsetRotation)
						{
							if (BoneIndex != Effector.EffectorBoneIndex)
							{
								return;
							}

							FTransform EffectorWorldTransform = FTransform(Effector.Rotation);
							FTransform EffectorComponentTransform = EffectorWorldTransform * CachedComponentTransform.Inverse();
							FTransform EffectorLocalTransform = EffectorComponentTransform * SolverInternals[SolverInternal.ParentBoneIndex].ComponentTransform.Inverse();

							FRotator EffectorLocalRotation = EffectorLocalTransform.Rotator();
							FRotator DeltaLocalRotation = EffectorLocalRotation - CurrentOffsetRotation;

							SavedOffsetRotation += DeltaLocalRotation;
							CurrentOffsetRotation += DeltaLocalRotation;
						}
					);
#endif

					// KeepLocation側にあたる処理
					FVector EndSolverLocation = GetWorldSpaceBoneLocation(Effector.EffectorBoneIndex);
					FVector DeltaLocation = Effector.Location - EndSolverLocation;
					float DeltaLocationSize = DeltaLocation.Size();

					if (DeltaLocationSize > Setting->ConvergenceDistance) // TODO:なんじゃこれ？
					{
						float Step = FMath::Min(Setting->StepSize, DeltaLocationSize);
						EtaStep += Step;
						FVector StepV = DeltaLocation / DeltaLocationSize * Step;
						EffectorStep[0] = StepV.X;
						EffectorStep[1] = StepV.Y;
						EffectorStep[2] = StepV.Z;
					}

					DisplacementCount = BoneAxisCount;
				}
				break;
			case EFullbodyIkEffectorTypePractice::FollowOriginalLocation:
				{
					const FSolverInternal& SolverInternal = SolverInternals[Effector.EffectorBoneIndex];
					FTransform InitWorldTransform = SolverInternal.InitComponentTransform * CachedComponentTransform; //TODO:なぜこの計算でワールド行列が手に入るのか？

					FVector EndSolverLocation = GetWorldSpaceBoneLocation(Effector.EffectorBoneIndex);
					FVector DeltaLocation = InitWorldTransform.GetLocation() + CachedComponentTransform.TransformVector(Effector.Location) - EndSolverLocation; // TODO:なぜこれが目標位置への差分になるのか？
					float DeltaLocationSize = DeltaLocation.Size();

					if (DeltaLocationSize > Setting->ConvergenceDistance) // TODO:なんじゃこれ？
					{
						float Step = FMath::Min(Setting->StepSize, DeltaLocationSize);
						EtaStep += Step;
						FVector StepV = DeltaLocation / DeltaLocationSize * Step;
						EffectorStep[0] = StepV.X;
						EffectorStep[1] = StepV.Y;
						EffectorStep[2] = StepV.Z;
					}

					DisplacementCount = BoneAxisCount;
				}
				break;
			case EFullbodyIkEffectorTypePractice::FollowOriginalRotation:
				{
					const FSolverInternal& SolverInternal = SolverInternals[Effector.EffectorBoneIndex];
					FTransform InitWorldTransform = SolverInternal.InitComponentTransform * CachedComponentTransform; //TODO:なぜこの計算でワールド行列が手に入るのか？

#if 0
					// Transform更新
					SolveSolver(0, FTransform::Identity,
						[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
						{
						},
						[&](int32 BoneIndex, FRotator& SavedOffsetRotation, FRotator& CurrentOffsetRotation)
						{
							if (BoneIndex != Effector.EffectorBoneIndex)
							{
								return;
							}

							FTransform EffectorWorldTransform = FTransform(InitWorldTransform.Rotator());
							FTransform EffectorComponentTransform = EffectorWorldTransform * CachedComponentTransform.Inverse();
							FTransform EffectorLocalTransform = EffectorComponentTransform * SolverInternals[SolverInternal.ParentBoneIndex].ComponentTransform.Inverse();

							FRotator EffectorLocalRotation = EffectorLocalTransform.Rotator();
							FRotator DeltaLocalRotation = EffectorLocalRotation + Effector.Rotation - CurrentOffsetRotation;

							SavedOffsetRotation += DeltaLocalRotation;
							CurrentOffsetRotation += DeltaLocalRotation;
						}
					);
#endif

					DisplacementCount = BoneAxisCount;
				}
				break;
			case EFullbodyIkEffectorTypePractice::FollowOriginalLocationAndRotation:
				{
					const FSolverInternal& SolverInternal = SolverInternals[Effector.EffectorBoneIndex];
					FTransform InitWorldTransform = SolverInternal.InitComponentTransform * CachedComponentTransform; //TODO:なぜこの計算でワールド行列が手に入るのか？

#if 0
					// FollowOriginalRotationにあたる処理
					// Transform更新
					SolveSolver(0, FTransform::Identity,
						[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
						{
						},
						[&](int32 BoneIndex, FRotator& SavedOffsetRotation, FRotator& CurrentOffsetRotation)
						{
							if (BoneIndex != Effector.EffectorBoneIndex)
							{
								return;
							}

							FTransform EffectorWorldTransform = FTransform(InitWorldTransform.Rotator());
							FTransform EffectorComponentTransform = EffectorWorldTransform * CachedComponentTransform.Inverse();
							FTransform EffectorLocalTransform = EffectorComponentTransform * SolverInternals[SolverInternal.ParentBoneIndex].ComponentTransform.Inverse();

							FRotator EffectorLocalRotation = EffectorLocalTransform.Rotator();
							FRotator DeltaLocalRotation = EffectorLocalRotation + Effector.Rotation - CurrentOffsetRotation;

							SavedOffsetRotation += DeltaLocalRotation;
							CurrentOffsetRotation += DeltaLocalRotation;
						}
					);
#endif
					FVector EndSolverLocation = GetWorldSpaceBoneLocation(Effector.EffectorBoneIndex);
					FVector DeltaLocation = InitWorldTransform.GetLocation() + CachedComponentTransform.TransformVector(Effector.Location) - EndSolverLocation; // TODO:なぜこれが目標位置への差分になるのか？
					float DeltaLocationSize = DeltaLocation.Size();

					if (DeltaLocationSize > Setting->ConvergenceDistance) // TODO:なんじゃこれ？
					{
						float Step = FMath::Min(Setting->StepSize, DeltaLocationSize);
						EtaStep += Step;
						FVector StepV = DeltaLocation / DeltaLocationSize * Step;
						EffectorStep[0] = StepV.X;
						EffectorStep[1] = StepV.Y;
						EffectorStep[2] = StepV.Z;
					}

					DisplacementCount = BoneAxisCount;
				}
				break;
			}

			if (DisplacementCount <= 0)
			{
				continue;
			}

			if (EtaStep <= 0.0f)
			{
				continue;
			}

			EtaStep /= Setting->StepSize;

			FBuffer& J = FBuffer(ElementsJ.GetData(), DisplacementCount, AXIS_COUNT);
			FBuffer& Jt = FBuffer(ElementsJt.GetData(), AXIS_COUNT, DisplacementCount);
			FBuffer& JtJ = FBuffer(ElementsJtJ.GetData(), AXIS_COUNT, AXIS_COUNT);
			FBuffer& JtJi = FBuffer(ElementsJtJi.GetData(), AXIS_COUNT, AXIS_COUNT);
			FBuffer& Jp = FBuffer(ElementsJp.GetData(), AXIS_COUNT, DisplacementCount);
			FBuffer& W0 = FBuffer(ElementsW0.GetData(), BoneAxisCount);
			FBuffer& Wi = FBuffer(ElementsWi.GetData(), DisplacementCount, DisplacementCount);
			FBuffer& JtWi = FBuffer(ElementsJtWi.GetData(), AXIS_COUNT, DisplacementCount);
			FBuffer& JtWiJ = FBuffer(ElementsJtWiJ.GetData(), AXIS_COUNT, AXIS_COUNT);
			FBuffer& JtWiJi = FBuffer(ElementsJtWiJi.GetData(), AXIS_COUNT, AXIS_COUNT);
			FBuffer& JtWiJiJt = FBuffer(ElementsJtWiJiJt.GetData(), AXIS_COUNT, DisplacementCount);
			FBuffer& Jwp = FBuffer(ElementsJwp.GetData(), AXIS_COUNT, DisplacementCount);
			FBuffer& Rt1 = FBuffer(ElementsRt1.GetData(), BoneAxisCount);
			FBuffer& Eta = FBuffer(ElementsEta.GetData(), BoneAxisCount);
			FBuffer& EtaJ = FBuffer(ElementsEtaJ.GetData(), AXIS_COUNT);
			FBuffer& EtaJJp = FBuffer(ElementsEtaJJp.GetData(), BoneAxisCount);
			FBuffer& Rt2 = FBuffer(ElementsRt2.GetData(), BoneAxisCount);

			// ヤコビアン J
			// auto J = FBuffer(DisplacementCount, AXIS_COUNT);
			J.Reset();

			switch (Effector.EffectorType)
			{
				// TODO:なぜKeepRotationとFollowOriginalLocationはヤコビアンを計算しない？
			case EFullbodyIkEffectorTypePractice::KeepLocation:
			case EFullbodyIkEffectorTypePractice::KeepLocationAndRotation:
			case EFullbodyIkEffectorTypePractice::FollowOriginalLocation:
			case EFullbodyIkEffectorTypePractice::FollowOriginalLocationAndRotation:
				{
					CalcJacobian(Effector, J.Ptr());
				}
				break;
			}

			// J^T
			// auto Jt = FBuffer(AXIS_COUNT, DisplacementCount);
			Jt.Reset();
			MatrixTranspose(Jt.Ptr(), J.Ptr(), DisplacementCount, AXIS_COUNT);

			// J^T * J
			// auto JtJ = FBuffer(AXIS_COUNT, AXIS_COUNT);
			JtJ.Reset();
			MatrixMultiply(
				JtJ.Ptr(),
				Jt.Ptr(),
				AXIS_COUNT, DisplacementCount,
				J.Ptr(),
				DisplacementCount, AXIS_COUNT
			);

			// (J^T * J)^-1
			// auto JtJi = FBuffer(AXIS_COUNT, AXIS_COUNT);
			JtJi.Reset();
			float DetJtJi = MatrixInverse3(JtJi.Ptr(), JtJ.Ptr());
			if (DetJtJi == 0)
			{
				continue
					;
			}

			// ヤコビアン擬似逆行列 (J^T * J)^-1 * J^T
			// auto Jp = FBuffer(AXIS_COUNT, DisplacementCount);
			Jp.Reset();
			MatrixMultiply(
				Jp.Ptr(),
				JtJi.Ptr(),
				AXIS_COUNT, AXIS_COUNT,
				Jt.Ptr(),
				AXIS_COUNT, DisplacementCount
			);

#if 0 //TODO: 加重行列に関する部分はとりあえず省略
			// W^-1
			// auto Wi = FBuffer(DisplacementCount, DisplacementCount);
			Wi.Reset();
			for (int32 i = 0; i < DisplacementCount; ++i)
			{
				Wi.Ref(i, i) = 1.0f / W0.Ref(i % BoneAxisCount);
			}

			// J^T * W^-1
			// auto JtWi = FBuffer(AXIS_COUNT, DisplacementCount);
			JtWi.Reset();
			MatrixMultiply(
				JtWi.Ptr(),
				Jt.Ptr(), AXIS_COUNT, DisplacementCount,
				Wi.Ptr(), DisplacementCount, DisplacementCount
			);

			// J^T * W^-1 * J
			// auto JtWiJ = FBuffer(AXIS_COUNT, AXIS_COUNT);
			JtWiJ.Reset();
			MatrixMultiply(
				JtWiJ.Ptr(),
				JtWi.Ptr(), AXIS_COUNT, DisplacementCount,
				J.Ptr(), DisplacementCount, AXIS_COUNT
			);
			for (int32 i = 0; i < AXIS_COUNT; ++i)
			{
				JtWiJ.Ref(i, i) += Setting->JtJInverseBias;
			}

			// (J^T * W^-1 * J)^-1
			// auto JtWiJi = FBuffer(AXIS_COUNT, AXIS_COUNT);
			JtWiJi.Reset();
			float DetJtWiJ = MatrixInverse3(JtWiJi.Ptr(), JtWiJ.Ptr());
			if (DetJtWiJ == 0)
			{
				continue;
			}

			// (J^T * W^-1 * J)^-1 * J^T
			// auto JtWiJiJt = FBuffer(AXIS_COUNT, DisplacementCount);
			JtWiJiJt.Reset();
			MatrixMultiply(
				JtWiJiJt.Ptr(),
				JtWiJi.Ptr(), AXIS_COUNT, AXIS_COUNT,
				Jt.Ptr(), AXIS_COUNT, DisplacementCount
			);

			// ヤコビアン加重逆行列 (J^T * W^-1 * J)^-1 * J^T * W^-1
			// auto Jwp = FBuffer(AXIS_COUNT, DisplacementCount);
			Jwp.Reset();
			MatrixMultiply(
				Jwp.Ptr(),
				JtWiJiJt.Ptr(), AXIS_COUNT, DisplacementCount,
				Wi.Ptr(), DisplacementCount, DisplacementCount
			);
#endif
			// 関節角度ベクトル1 目標エフェクタ変位 * ヤコビアン加重逆行列
			// auto Rt1 = FBuffer(BoneAxisCount);




#if 0
			// Transform更新
			SolveSolver(0, FTransform::Identity,
				[&](int32 BoneIndex, FVector& SavedOffsetLocation, FVector& CurrentOffsetLocation)
				{
					int32 BoneIndicesIndex = SolverInternals[BoneIndex].BoneIndicesIndex;

					FVector DeltaLocation = FVector(
						Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 0) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 0),	// X
						Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 1) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 1),	// Y
						Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 2) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 2)		// Z
					);

					SavedOffsetLocation += DeltaLocation;
					CurrentOffsetLocation += DeltaLocation;
				},
				[&](int32 BoneIndex, FRotator& SavedOffsetRotation, FRotator& CurrentOffsetRotation)
				{
					int32 BoneIndicesIndex = SolverInternals[BoneIndex].BoneIndicesIndex;

					FRotator DeltaRotation = FRotator(
						FMath::RadiansToDegrees(Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 1) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 1)),	// Pitch
						FMath::RadiansToDegrees(Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 2) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 2)),	// Yaw
						FMath::RadiansToDegrees(Rt1.Ref(BoneIndicesIndex * AXIS_COUNT + 0) + Rt2.Ref(BoneIndicesIndex * AXIS_COUNT + 0))	// Roll
					);

					SavedOffsetRotation += DeltaRotation;
					CurrentOffsetRotation += DeltaRotation;
				}
			);
#endif

			++EffectiveCount;

			if (EffectiveCount >= Setting->EffectiveCountMax)
			{
				break;
			}
		}

		++StepLoopCount;
	}

	// 計算結果をノードの出力に反映。フルボディIKに関わるすべてのボーン（BoneIndicesに含まれるもの）に反映させる
	for (int32 BoneIndex : BoneIndices)
	{
		OutBoneTransforms.Add(FBoneTransform(FCompactPoseBoneIndex(BoneIndex), SolverInternals[BoneIndex].ComponentTransform));
	}
}

bool FAnimNode_FullbodyIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

FFullbodyIKSolver FAnimNode_FullbodyIKPractice::GetSolver(FName BoneName) const
{
	check(Setting != nullptr);

	for (const FFullbodyIKSolver& Solver : Setting->Solvers)
	{
		// アニメーションBPはロジックをボーン名に頼りがち
		if (Solver.BoneName == BoneName)
		{
			return Solver;
		}
	}

	// 見つからなければデフォルト
	FFullbodyIKSolver Solver;
	Solver.BoneName = BoneName;
	return Solver;
}

FTransform FAnimNode_FullbodyIKPractice::GetWorldSpaceBoneTransform(const int32& BoneIndex) const
{
	FTransform Transform = SolverInternals[BoneIndex].ComponentTransform;
	Transform *= CachedComponentTransform; //TODO:なぜこの計算でワールド行列が手に入るのか？
	return Transform;
}

FVector FAnimNode_FullbodyIKPractice::GetWorldSpaceBoneLocation(const int32& BoneIndex) const
{
	FTransform Transform = SolverInternals[BoneIndex].ComponentTransform;
	Transform *= CachedComponentTransform;
	return Transform.GetLocation();
}

FQuat FAnimNode_FullbodyIKPractice::GetWorldSpaceBoneRotation(const int32& BoneIndex) const
{
	FTransform Transform = SolverInternals[BoneIndex].ComponentTransform;
	Transform *= CachedComponentTransform;
	return Transform.GetRotation();
}

FTransform FAnimNode_FullbodyIKPractice::GetLocalSpaceBoneTransform(const int32& BoneIndex) const
{
	return SolverInternals[BoneIndex].LocalTransform;
}

FVector FAnimNode_FullbodyIKPractice::GetLocalSpaceBoneLocation(const int32& BoneIndex) const
{
	FTransform Transform = SolverInternals[BoneIndex].LocalTransform;
	return Transform.GetLocation();
}

FQuat FAnimNode_FullbodyIKPractice::GetLocalSpaceBoneRotation(const int32& BoneIndex) const
{
	FTransform Transform = SolverInternals[BoneIndex].LocalTransform;
	return Transform.GetRotation();
}

void FAnimNode_FullbodyIKPractice::CalcJacobian(const FEffectorInternal& EffectorInternal, float* Jacobian)
{
	// ここが呼び出し回数が多く一番計算負荷がかかるはず
	int32 BoneIndex = EffectorInternal.EffectorBoneIndex;
	const FVector& EndSolverLocation = GetWorldSpaceBoneLocation(BoneIndex);
	BoneIndex = SolverInternals[BoneIndex].ParentBoneIndex;

	while (true)
	{
		const FSolverInternal& SolverInternal = SolverInternals[BoneIndex];
		int32 ParentBoneIndex = SolverInternals[BoneIndex].ParentBoneIndex;

		FQuat ParentWorldRotation = FQuat::Identity;
		if (ParentBoneIndex != INDEX_NONE)
		{
			ParentWorldRotation = GetWorldSpaceBoneRotation(ParentBoneIndex);
		}

		if (SolverInternal.bTranslation) //TODO:これってなんだろう？ヤコビアンって変数としてジョイントのオイラー角しか考慮してないって思ってたけど、実はtranslationも考慮してるってこと？
		{
			FVector DVec[AXIS_COUNT];
			DVec[0] = FVector(1, 0, 0);
			DVec[1] = FVector(0, 1, 0);
			DVec[2] = FVector(0, 0, 1);
			// 多分、Tが、
			// T=|000X|
			//   |000Y|
			//   |000Z|
			//   |0001|
			// なので、それをX,Y,Zでそれぞれ微分したものは上記ベクトルにあたるのだろう
			// それとも、そもそも最初からオイラー角以外はIK計算には入れるつもりはなく、微分なんてしてないのか？
			// if (SolverInternal.bTranslation)のelse側を見ると、どうも微分なんてしてないように見えるが

			for (int32 Axis = 0; Axis < AXIS_COUNT; ++Axis)
			{
				// Jacobian[BoneCount * AXIS_COUNT][AXIS_COUNT]
				int32 JacobianIndex1 = SolverInternal.BoneIndicesIndex * AXIS_COUNT;
				int32 JacobianIndex2 = Axis;

				// TODO:ここが肝なのだがどうもよくわからないな。。一体何を計算しているんだ？
				// BoneIndexより子孫のやつのRotationを無視する、ということに見えるが
				FVector DVec3 = ParentWorldRotation.RotateVector(DVec[Axis]);

				// [行の番号*AXIS_COUNT + 列の番号] 列の数がAXIS_COUNTなので
				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 0] = DVec3.X;
				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 1] = DVec3.Y;
				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 2] = DVec3.Z;
			}
		}
		else
		{
			// こっちは普通のヤコビアンの計算方法なので理解できる
			FVector DeltaLocation = EndSolverLocation - GetWorldSpaceBoneLocation(BoneIndex);
			FVector UnrotateDeltaLocation = GetWorldSpaceBoneRotation(BoneIndex).UnrotateVector(DeltaLocation);
			FRotator BoneRotation = GetLocalSpaceBoneRotation(BoneIndex).Rotator();
			FMatrix DMat[AXIS_COUNT];

			DMat[0] = DiffRotX(BoneRotation.Roll) * RotY(BoneRotation.Pitch) * RotZ(BoneRotation.Yaw);
			DMat[1] = RotX(BoneRotation.Roll) * DiffRotY(BoneRotation.Pitch) * RotZ(BoneRotation.Yaw);
			DMat[2] = RotX(BoneRotation.Roll) * RotY(BoneRotation.Pitch) * DiffRotZ(BoneRotation.Yaw);

			for (int32 Axis = 0; Axis < AXIS_COUNT; ++Axis)
			{
				// Jacobian[BoneCount * AXIS_COUNT][AXIS_COUNT]
				int32 JacobianIndex1 = SolverInternal.BoneIndicesIndex * AXIS_COUNT;
				int32 JacobianIndex2 = Axis;

				FVector4 DVec4 = DMat[Axis].TransformVector(UnrotateDeltaLocation);
				FVector DVec3 = ParentWorldRotation.RotateVector(FVector(DVec4));

				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 0] = DVec3.X;
				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 1] = DVec3.Y;
				Jacobian[(JacobianIndex1 + JacobianIndex2) * AXIS_COUNT + 2] = DVec3.Z;
			}
		}

		if (BoneIndex = EffectorInternal.RootBoneIndex || ParentBoneIndex == INDEX_NONE)
		{
			break;
		}

		BoneIndex = ParentBoneIndex;
	}
}

#if 0
void FAnimNode_FullbodyIKPractice::SolveSolver(
	int32 BoneIndex,
	const FTransform& ParentComponentTransform,
	const TFunction<void(int32, FVector&, FVector&)>& LocationOffsetProcess,
	const TFunction<void(int32, FRotator&, FRotator&)>& RotationOffsetProcess)
{
	FSolverInternal& SolverInternal = SolverInternals[BoneIndex];

	if (SolverInternal.bTranslation)
	{
		FVector SavedOffsetLocation = IAnimInstanceInterface_FullbodyIK::Execute_GetBoneLocationOffset(CachedAnimInstanceObject, BoneIndex);
		FVector CurrentOffsetLocation = SolverInternal.LocalTransform.GetLocation();
	}
	else
	{
	}
}
#endif
