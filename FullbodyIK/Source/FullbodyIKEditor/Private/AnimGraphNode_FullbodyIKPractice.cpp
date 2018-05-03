#include "AnimGraphNode_FullbodyIKPractice.h"
#include "Animation/AnimInstance.h"

#define LOCTEXT_NAMESPACE "AnimGraphNode_FullbodyIKPractice"

/////////////////////////////////////////////////////
// UAnimGraphNode_FullbodyIK


UAnimGraphNode_FullbodyIKPractice::UAnimGraphNode_FullbodyIKPractice(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_FullbodyIKPractice::GetControllerDescription() const
{
	return LOCTEXT("FullbodyIKPractice", "Fullbody IK Practice");
}

FText UAnimGraphNode_FullbodyIKPractice::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_FullbodyIKPractice_Tooltip", "The Fullbody IK control applies an inverse kinematic (IK) solver to the full body.");
}

FText UAnimGraphNode_FullbodyIKPractice::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_FullbodyIKPractice::CopyNodeDataToPreviewNode(FAnimNode_Base* InPreviewNode)
{
	FAnimNode_FullbodyIKPractice* FullbodyIKPractice = static_cast<FAnimNode_FullbodyIKPractice*>(InPreviewNode);

	// copies Pin values from the internal node to get data which are not compiled yet
	// TODO:Œã‚Å•œŠˆ
	//FullbodyIK->Effectors = Node.Effectors;
}

void UAnimGraphNode_FullbodyIKPractice::CopyPinDefaultsToNodeData(UEdGraphPin* InPin)
{
}

void UAnimGraphNode_FullbodyIKPractice::CustomizeDetails(class IDetailLayoutBuilder& DetailBuilder)
{
}

FEditorModeID UAnimGraphNode_FullbodyIKPractice::GetEditorMode() const
{
	const static FEditorModeID FullbodyIKPracticeEditorMode;
	return FullbodyIKPracticeEditorMode;
}

void UAnimGraphNode_FullbodyIKPractice::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
}

void UAnimGraphNode_FullbodyIKPractice::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* SkelMeshComp) const
{
	// TODO:Œã‚Å•œŠˆ
	//if (bEnableDebugDraw && SkelMeshComp)
	//{
	//	if (FAnimNode_FullbodyIKPractice* ActiveNode = GetActiveInstanceNode<FAnimNode_FullbodyIKPractice>(SkelMeshComp->GetAnimInstance()))
	//	{
	//		ActiveNode->ConditionalDebugDraw(PDI, SkelMeshComp);
	//	}
	//}
}

#undef LOCTEXT_NAMESPACE
