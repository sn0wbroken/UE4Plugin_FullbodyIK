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

		// IkEndBoneNamesに入っているエフェクタのボーンだけでなく、その先祖のすべてのボーンに対してFFullbodyIKSolverの設定は存在しうる
		while (true)
		{
			int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
			if (BoneIndex == INDEX_NONE || SolverInternals.Contains(BoneIndex))
			{
				break;
			}

			FSolverInternal& SolverInternal = SolverInternals.Add(BoneIndex, FSolverInternal());
			BoneIndices.Add(BoneIndex);

			FName ParentBoneName = Mesh->GetParentBone(BoneName);
			int32 ParentBoneIndex = Mesh->GetBoneIndex(ParentBoneName);
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

void FAnimNode_FullbodyIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	if (!Setting)
	{
		return;
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
