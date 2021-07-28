#include "Enums.h"


// Standart UE4 classes
struct FString {};
struct FText {};
struct FName {};
struct FDelegate {};
struct FScriptMulticastDelegate {};

// TODO: TArray, TSoftClassPtr


// FortniteGame classes

struct FExternalAccountServiceConfig
{
	EExternalAccountType Type;
	struct FName ExternalServiceName;
};

struct FWebEnvUrl
{
	struct FString URL;
	struct FString RedirectUrl;
	struct FString Environment;
};

struct FGiftMessage
{
	struct FString GiftCode;
	struct FString SenderName;
};

struct FExchangeAccessParams
{
	struct FString EntitlementId;
	struct FString ReceiptId;
	struct FString VendorReceipt;
	struct FString AppStore;
};

struct FOnlineAccountTexts_FailedLoginConsole
{
	struct FText AgeRestriction;
	struct FText Generic;
	struct FText MissingAuthAssociation;
	struct FText NeedPremiumAccount;
	struct FText OnlinePlayRestriction;
	struct FText PatchAvailable;
	struct FText PleaseSignIn;
	struct FText SystemUpdateAvailable;
	struct FText UI;
	struct FText UnableToComplete;
	struct FText UnableToSignIn;
	struct FText UnableToStartPrivCheck;
	struct FText UnexpectedError;
};

struct FOnlineAccountTexts
{
	struct FText AllGiftCodesUsed;
	struct FText AssociateConsoleAuth;
	struct FText AutoLoginFailed;
	struct FText AutoLoginFailedMobile;
	struct FText BannedFromGame;
	struct FText CheckEntitledToPlay;
	struct FText CheckingRejoin;
	struct FText CheckServiceAvailability;
	struct FText ConsolePrivileges;
	struct FText CreateAccountCompleted;
	struct FText CreateAccountFailure;
	struct FText CreateHeadless;
	struct FText DoQosPingTests;
	struct FText DowntimeMinutesWarningText;
	struct FText DowntimeSecondsWarningText;
	struct FText DuplicateAuthAssociaton;
	struct FText EulaCheck;
	struct FText ExchangeConsoleGiftsForAccess;
	struct FText ExchangeConsolePurchaseForAccess;
	struct FText FailedAccountCreate;
	struct FText FailedEulaCheck_EulaAcceptanceFailed;
	struct FText FailedEulaCheck_MustAcceptEula;
	struct FText FailedLoginCredentialsMsg;
	struct FText FailedLoginAgeVerificationIncomplete;
	struct FText FailedLoginParentalLock;
	struct FText FailedLoginNoRealId;
	struct FText FailedLoginLockoutMsg;
	struct FText FailedLoginRequiresMFA;
	struct FText FailedLoginRequiresAuthAppMFA;
	struct FText FailedInvalidMFA;
	struct FText FailedLoginMsg;
	struct FText FailedLoginMsg_InvalidRefreshToken;
	struct FText FailedLoginTencent_UnableToSignIn;
	struct FText FailedLoginTencent_NotSignedInToWeGame;
	struct FText FailedLoginTencent_FailedToInitializeWeGame;
	struct FText FailedLoginTencent_WeGameSystemOffline;
	struct FText Tencent_ExitByAntiAddiction;
	struct FText FailedStartLogin;
	struct FText FounderChatExitedText;
	struct FText FounderChatJoinedText;
	struct FText GameDisplayName;
	struct FText GeneralLoginFailure;
	struct FText GlobalChatExitedText;
	struct FText GlobalChatJoinedText;
	struct FText HeadlessAccountFailed;
	struct FText InMatchShutdownTimeWarningText;
	struct FText InvalidUser;
	struct FText LoggedOutofMCP;
	struct FText DisconnectedFromMCP;
	struct FText LoggedOutReturnedToTitle;
	struct FText LoggedOutSwitchedProfile;
	struct FText LoggingIn;
	struct FText LoggingInConsoleAuth;
	struct FText LoggingOut;
	struct FText LoginConsole;
	struct FText LoginFailure;
	struct FText Logout_Unlink;
	struct FText LogoutCompleted;
	struct FText LostConnection;
	struct FText MCPTimeout;
	struct FText LightswitchCheckNetworkFailureMsg;
	struct FText NetworkConnectionUnavailable;
	struct FText NoPlayEntitlement;
	struct FText NoServerAccess;
	struct FText PlayAccessRevoked;
	struct FText PremiumAccountName_Default;
	struct FText PremiumAccountName_PS4;
	struct FText PremiumAccountName_Sony;
	struct FText PremiumAccountName_Switch;
	struct FText PremiumAccountName_XboxOne;
	struct FText RedeemOfflinePurchases;
	struct FText ServiceDowntime;
	struct FText SignInCompleting;
	struct FText SignIntoConsoleServices;
	struct FText TokenExpired;
	struct FText UnableToConnect;
	struct FText UnableToJoinWaitingRoomLoginQueue;
	struct FText UnexpectedConsoleAuthFailure;
	struct FText UnlinkConsoleFailed;
	struct FText UserLoginFailed;
	struct FText WaitingRoom;
	struct FText WaitingRoomError;
	struct FText WaitingRoomFailure;
	struct FText WaitingRoomWaiting;
	struct OnlineAccountTexts_FailedLoginConsole FailedLoginConsole;
	struct FText LoggingInExternalAuth;
	struct FText CreateDeviceAuth;
	struct FText ExtAuthCanceled;
	struct FText ExtAuthFailure;
	struct FText ExtAuthAssociationFailure;
	struct FText ExtAuthTimeout;
	struct FText ExtAuthMissingAuthAssociation;
	struct FText UnableToQueryReceipts;
};

struct FAIDataProviderValue
{
	class UClass* DataBinding;
	struct FName DataField;
};

struct FBlackboardKeySelector
{
	TArray<class UClass*> AllowedTypes;
	struct FName SelectedKeyName;
	class UClass* SelectedKeyType;
	byte SelectedKeyID;
	bool bNoneIsAllowedValue;
};

struct FAINoiseEvent
{
	struct Vector NoiseLocation;
	float Loudness;
	float MaxRange;
	class UClass* Instigator;
	struct FName Tag;
};

struct FAIMoveRequest
{
	class UClass* GoalActor;
};

struct FCrowdAvoidanceConfig
{
	float VelocityBias;
	float DesiredVelocityWeight;
	float CurrentVelocityWeight;
	float SideBiasWeight;
	float ImpactTimeWeight;
	float ImpactTimeRange;
	byte CustomPatternIdx;
	byte AdaptiveDivisions;
	byte AdaptiveRings;
	byte AdaptiveDepth;
};

struct FCrowdAvoidanceSamplingPattern
{
	TArray<float> Angles;
	TArray<float> Radii;
};

struct FEnvQueryInstanceCache
{
	class UClass* Template;
};

struct FEnvTraceData
{
	int VersionNum;
	class UClass* NavigationFilter;
	float ProjectDown;
	float ProjectUp;
	float ExtentX;
	float ExtentY;
	float ExtentZ;
	float PostProjectionVerticalOffset;
	ETraceTypeQuery TraceChannel;
	ECollisionChannel SerializedChannel;
	struct FName TraceProfileName;
	EEnvTraceShape TraceShape;
	EEnvQueryTrace TraceMode;
	bool bTraceComplex;
	bool bOnlyBlockingHits;
	bool bCanTraceOnNavMesh;
	bool bCanTraceOnGeometry;
	bool bCanDisableTrace;
	bool bCanProjectDown;
};

struct FEnvDirection
{
	class UClass* LineFrom;
	class UClass* LineTo;
	class UClass* Rotation;
	EEnvDirection DirMode;
};

struct FAIDamageEvent
{
	float Amount;
	struct Vector Location;
	struct Vector HitLocation;
	class UClass* DamagedActor;
	class UClass* Instigator;
	struct FName Tag;
};

struct FAIPredictionEvent
{
	class UClass* Requestor;
	class UClass* PredictedActor;
};

struct FAITeamStimulusEvent
{
	class UClass* Broadcaster;
	class UClass* Enemy;
};

struct FAITouchEvent
{
	class UClass* TouchReceiver;
	class UClass* OtherActor;
};

struct FAISenseAffiliationFilter
{
	bool bDetectEnemies;
	bool bDetectNeutrals;
	bool bDetectFriendlies;
};

struct FBTDecoratorLogic
{
	EBTDecoratorLogic Operation;
	uint16_t Number;
};

struct FBehaviorTreeTemplateInfo
{
	class UClass* Asset;
	class UClass* Template;
};

struct FBlackboardEntry
{
	struct FName EntryName;
	class UClass* KeyType;
	bool bInstanceSynced;
};

struct FBTCompositeChild
{
	class UClass* ChildComposite;
	class UClass* ChildTask;
	TArray<class UClass*> Decorators;
	TArray<struct BTDecoratorLogic> DecoratorOps;
};

struct FAIDynamicParam
{
	struct FName ParamName;
	EAIParamType ParamType;
	float Value;
	struct BlackboardKeySelector BBKey;
};

struct FEQSParametrizedQueryExecutionRequest
{
	class UClass* QueryTemplate;
	TArray<struct AIDynamicParam> QueryConfig;
	struct BlackboardKeySelector EQSQueryBlackboardKey;
	EEnvQueryRunMode RunMode;
	bool bUseBBKeyForQueryTemplate;
};

struct FIntervalCountdown
{
	float Interval;
};

struct FEnvNamedValue
{
	struct FName ParamName;
	EAIParamType ParamType;
	float Value;
};

struct FEnvOverlapData
{
	float ExtentX;
	float ExtentY;
	float ExtentZ;
	struct Vector ShapeOffset;
	ECollisionChannel OverlapChannel;
	EEnvOverlapShape OverlapShape;
	bool bOnlyBlockingHits;
	bool bOverlapComplex;
	bool bSkipOverlapQuerier;
};

struct FPawnActionStack
{
	class UClass* TopAction;
};

struct FPawnActionEvent
{
	class UClass* Action;
};

struct FAIRequestID
{
	uint32_t RequestID;
};

struct FAIStimulus
{
	float Age;
	float ExpirationAge;
	float Strength;
	struct Vector StimulusLocation;
	struct Vector ReceiverLocation;
	struct FName Tag;
	bool bSuccessfullySensed;
};

struct FActorPerceptionUpdateInfo
{
	int TargetId;
	Unknown Target;
	struct AIStimulus Stimulus;
};

struct FActorPerceptionBlueprintInfo
{
	class UClass* Target;
	TArray<struct AIStimulus> LastSensedStimuli;
	bool bIsHostile;
};

struct FAISightEvent
{
	class UClass* SeenActor;
	class UClass* Observer;
};

struct FEnvQueryRequest
{
	class UClass* QueryTemplate;
	class UClass* Owner;
	class UClass* World;
};

struct FEnvQueryManagerConfig
{
	float MaxAllowedTestingTime;
	bool bTestQueriesUsingBreadth;
	int QueryCountWarningThreshold;
	double QueryCountWarningInterval;
	double ExecutionTimeWarningSeconds;
	double HandlingResultTimeWarningSeconds;
	double GenerationTimeWarningSeconds;
};

struct FEnvQueryResult
{
	class UClass* ItemType;
	int OptionIndex;
	int QueryID;
};

struct FGenericTeamId
{
	byte TeamId;
};

struct FRecastGraphWrapper
{
	class UClass* RecastNavMeshActor;
};

struct FAmbientAudioBase
{
	struct TSoftClassPtr<UObject> Sound;
	struct GameplayTagQuery Requirements;
};

struct FAmbientProxy
{
};

struct FAnalyticsEventAttr
{
	struct FString Name;
	struct FString Value;
};

struct FAnimationBudgetAllocatorParameters
{
	float BudgetInMs;
	float MinQuality;
	int MaxTickRate;
	float WorkUnitSmoothingSpeed;
	float AlwaysTickFalloffAggression;
	float InterpolationFalloffAggression;
	int InterpolationMaxRate;
	int MaxInterpolatedComponents;
	float InterpolationTickMultiplier;
	float InitialEstimatedWorkUnitTimeMs;
	int MaxTickedOffsreenComponents;
	int StateChangeThrottleInFrames;
	float BudgetFactorBeforeReducedWork;
	float BudgetFactorBeforeReducedWorkEpsilon;
	float BudgetPressureSmoothingSpeed;
	int ReducedWorkThrottleMinInFrames;
	int ReducedWorkThrottleMaxInFrames;
	float BudgetFactorBeforeAggressiveReducedWork;
	int ReducedWorkThrottleMaxPerFrame;
	float BudgetPressureBeforeEmergencyReducedWork;
};

struct FNodeObject
{
	struct FName Name;
	struct FName ParentName;
};

struct FNodeHierarchyData
{
	TArray<struct NodeObject> Nodes;
	TArray<struct Transform> Transforms;
	Unknown NodeNameToIndexMapping;
};

struct FNodeHierarchyWithUserData
{
	struct NodeHierarchyData Hierarchy;
};

struct FFilterOptionPerAxis
{
	bool bX;
	bool bY;
	bool bZ;
};

struct FConstraintDescription
{
	bool bTranslation;
	bool bRotation;
	bool bScale;
	bool bParent;
	struct FilterOptionPerAxis TranslationAxes;
	struct FilterOptionPerAxis RotationAxes;
	struct FilterOptionPerAxis ScaleAxes;
};

struct FTransformConstraint
{
	struct ConstraintDescription Operator;
	struct FName SourceNode;
	struct FName TargetNode;
	float Weight;
	bool bMaintainOffset;
};

struct FConstraintOffset
{
	struct Vector Translation;
	struct Quat Rotation;
	struct Vector Scale;
	struct Transform Parent;
};

struct FConstraintDescriptor
{
	EConstraintType Type;
};

struct FConstraintData
{
	struct ConstraintDescriptor Constraint;
	float Weight;
	bool bMaintainOffset;
	struct Transform Offset;
	struct Transform CurrentTransform;
};

struct FTransformFilter
{
	struct FilterOptionPerAxis TranslationFilter;
	struct FilterOptionPerAxis RotationFilter;
	struct FilterOptionPerAxis ScaleFilter;
};

struct FCCDIKChainLink
{
};

struct FEulerTransform
{
	struct Vector Location;
	struct Rotator Rotation;
	struct Vector Scale;
};

struct FFABRIKChainLink
{
};

struct FAxis
{
	struct Vector Axis;
	bool bInLocalSpace;
};

struct FConstraintDescriptionEx
{
	struct FilterOptionPerAxis AxesFilterOption;
};

struct FNodeChain
{
	TArray<struct FName> Nodes;
};

struct FTransformNoScale
{
	struct Vector Location;
	struct Quat Rotation;
};

struct FAnimationSetup
{
	class UClass* AnimSequence;
	class UClass* AnimBlueprint;
	struct PerPlatformInt NumRandomizedInstances;
	struct PerPlatformBool Enabled;
};

struct FAnimationStateEntry
{
	byte State;
	TArray<struct AnimationSetup> AnimationSetups;
	bool bOnDemand;
	bool bAdditive;
	float BlendTime;
	bool bReturnToPreviousState;
	bool bSetNextState;
	byte NextState;
	struct PerPlatformInt MaximumNumberOfConcurrentInstances;
	float WiggleTimePercentage;
	bool bRequiresCurves;
};

struct FPerSkeletonAnimationSharingSetup
{
	class UClass* Skeleton;
	class UClass* SkeletalMesh;
	class UClass* BlendAnimBlueprint;
	class UClass* AdditiveAnimBlueprint;
	class UClass* StateProcessorClass;
	TArray<struct AnimationStateEntry> AnimationStates;
};

struct FAnimationSharingScalability
{
	struct PerPlatformBool UseBlendTransitions;
	struct PerPlatformFloat BlendSignificanceValue;
	struct PerPlatformInt MaximumNumberConcurrentBlends;
	struct PerPlatformFloat TickSignificanceValue;
};

struct FRandomPlayerSequenceEntry
{
	class UClass* Sequence;
	float ChanceToPlay;
	int MinLoopCount;
	int MaxLoopCount;
	float MinPlayrate;
	float MaxPlayrate;
	struct AlphaBlend BlendIn;
};

struct FSocketReference
{
	struct FName SocketName;
};

struct FBoneSocketTarget
{
	bool bUseSocket;
	struct BoneReference BoneReference;
	struct SocketReference SocketReference;
};

struct FAnimPhysConstraintSetup
{
	EAnimPhysLinearConstraintType LinearXLimitType;
	EAnimPhysLinearConstraintType LinearYLimitType;
	EAnimPhysLinearConstraintType LinearZLimitType;
	struct Vector LinearAxesMin;
	struct Vector LinearAxesMax;
	EAnimPhysAngularConstraintType AngularConstraintType;
	EAnimPhysTwistAxis TwistAxis;
	EAnimPhysTwistAxis AngularTargetAxis;
	float ConeAngle;
	struct Vector AngularLimitsMin;
	struct Vector AngularLimitsMax;
	struct Vector AngularTarget;
};

struct FAnimPhysSphericalLimit
{
	struct BoneReference DrivingBone;
	struct Vector SphereLocalOffset;
	float LimitRadius;
	ESphericalLimitType LimitType;
};

struct FAnimPhysPlanarLimit
{
	struct BoneReference DrivingBone;
	struct Transform PlaneTransform;
};

struct FRotationRetargetingInfo
{
	bool bEnabled;
	struct Transform Source;
	struct Transform Target;
	ERotationComponent RotationComponent;
	struct Vector TwistAxis;
	bool bUseAbsoluteAngle;
	float SourceMinimum;
	float SourceMaximum;
	float TargetMinimum;
	float TargetMaximum;
	EEasingFuncType EasingType;
	struct RuntimeFloatCurve CustomCurve;
	bool bFlipEasing;
	float EasingWeight;
	bool bClamp;
};

struct FAngularRangeLimit
{
	struct Vector LimitMin;
	struct Vector LimitMax;
	struct BoneReference Bone;
};

struct FBlendBoneByChannelEntry
{
	struct BoneReference SourceBone;
	struct BoneReference TargetBone;
	bool bBlendTranslation;
	bool bBlendRotation;
	bool bBlendScale;
};

struct FConstraint
{
	struct BoneReference TargetBone;
	EConstraintOffsetOption OffsetOption;
	ETransformConstraintType TransformType;
	struct FilterOptionPerAxis PerAxis;
};

struct FAnimLegIKDefinition
{
	struct BoneReference IKFootBone;
	struct BoneReference FKFootBone;
	int NumBonesInLimb;
	float MinRotationAngle;
	EAxis FootBoneForwardAxis;
	EAxis HingeRotationAxis;
	bool bEnableRotationLimit;
	bool bEnableKneeTwistCorrection;
};

struct FAnimLegIKData
{
};

struct FIKChain
{
};

struct FIKChainLink
{
};

struct FPoseDriverTransform
{
	struct Vector TargetTranslation;
	struct Rotator TargetRotation;
};

struct FPoseDriverTarget
{
	TArray<struct PoseDriverTransform> BoneTransforms;
	struct Rotator TargetRotation;
	float TargetScale;
	ERBFDistanceMethod DistanceMethod;
	ERBFFunctionType FunctionType;
	bool bApplyCustomCurve;
	struct RichCurve CustomCurve;
	struct FName DrivenName;
	bool bIsHidden;
};

struct FRBFParams
{
	int TargetDimensions;
	ERBFSolverType SolverType;
	float Radius;
	bool bAutomaticRadius;
	ERBFFunctionType Function;
	ERBFDistanceMethod DistanceMethod;
	EBoneAxis TwistAxis;
	float WeightThreshold;
	ERBFNormalizeMethod NormalizeMethod;
	struct Vector MedianReference;
	float MedianMin;
	float MedianMax;
};

struct FSimSpaceSettings
{
	float MasterAlpha;
	float VelocityScaleZ;
	float MaxLinearVelocity;
	float MaxAngularVelocity;
	float MaxLinearAcceleration;
	float MaxAngularAcceleration;
	float ExternalLinearDrag;
	struct Vector ExternalLinearDragV;
	struct Vector ExternalLinearVelocity;
	struct Vector ExternalAngularVelocity;
};

struct FSplineIKCachedBoneData
{
	struct BoneReference Bone;
	int RefSkeletonIndex;
};

struct FRotationLimit
{
	struct Vector LimitMin;
	struct Vector LimitMax;
};

struct FReferenceBoneFrame
{
	struct BoneReference Bone;
	struct Axis Axis;
};

struct FPositionHistory
{
	TArray<struct Vector> Positions;
	float Range;
};

struct FRBFEntry
{
	TArray<float> Values;
};

struct FTagAndValue
{
	struct FName Tag;
	struct FString Value;
};

struct FAssetRegistryDependencyOptions
{
	bool bIncludeSoftPackageReferences;
	bool bIncludeHardPackageReferences;
	bool bIncludeSearchableNames;
	bool bIncludeSoftManagementReferences;
	bool bIncludeHardManagementReferences;
};

struct FSubmixEffectDynamicProcessorFilterSettings
{
	bool bEnabled;
	float Cutoff;
	float GainDb;
};

struct FSubmixEffectDynamicsProcessorSettings
{
	ESubmixEffectDynamicsProcessorType DynamicsProcessorType;
	ESubmixEffectDynamicsPeakMode PeakMode;
	ESubmixEffectDynamicsChannelLinkMode LinkMode;
	float InputGainDb;
	float ThresholdDb;
	float Ratio;
	float KneeBandwidthDb;
	float LookAheadMsec;
	float AttackTimeMsec;
	float ReleaseTimeMsec;
	ESubmixEffectDynamicsKeySource KeySource;
	class UClass* ExternalAudioBus;
	class UClass* ExternalSubmix;
	bool bChannelLinked;
	bool bAnalogMode;
	bool bBypass;
	bool bKeyAudition;
	float KeyGainDb;
	float OutputGainDb;
	struct SubmixEffectDynamicProcessorFilterSettings KeyHighshelf;
	struct SubmixEffectDynamicProcessorFilterSettings KeyLowshelf;
};

struct FSubmixEffectEQBand
{
	float Frequency;
	float Bandwidth;
	float GainDb;
	bool bEnabled;
};

struct FSubmixEffectSubmixEQSettings
{
	TArray<struct SubmixEffectEQBand> EQBands;
};

struct FSubmixEffectReverbSettings
{
	bool bBypassEarlyReflections;
	float ReflectionsDelay;
	float GainHF;
	float ReflectionsGain;
	bool bBypassLateReflections;
	float LateDelay;
	float DecayTime;
	float Density;
	float Diffusion;
	float AirAbsorptionGainHF;
	float DecayHFRatio;
	float LateGain;
	float Gain;
	float WetLevel;
	float DryLevel;
	bool bBypass;
};

struct FAudioOutputDeviceInfo
{
	struct FString Name;
	struct FString DeviceID;
	int NumChannels;
	int SampleRate;
	EAudioMixerStreamDataFormatType Format;
	TArray<EAudioMixerChannelType> OutputChannelArray;
	bool bIsSystemDefault;
	bool bIsCurrentDevice;
};

struct FSwapAudioOutputResult
{
	struct FString CurrentDeviceId;
	struct FString RequestedDeviceId;
	ESwapAudioOutputDeviceResultState Result;
};

struct FPlatformRuntimeAudioCompressionOverrides
{
	bool bOverrideCompressionTimes;
	float DurationThreshold;
	int MaxNumRandomBranches;
	int SoundCueQualityIndex;
};

struct FARSessionPayload
{
	int ConfigFlags;
	class UClass* DefaultMeshMaterial;
	class UClass* DefaultWireframeMeshMaterial;
};

struct FARPlaneUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Transform WorldTransform;
	struct Vector Center;
	struct Vector Extents;
	TArray<struct Vector> BoundaryVertices;
	EARObjectClassification ObjectClassification;
};

struct FARPointUpdatePayload
{
};

struct FARFaceUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Vector LeftEyePosition;
	struct Vector RightEyePosition;
	struct Vector LookAtTarget;
};

struct FARImageUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Transform WorldTransform;
	class UClass* DetectedImage;
	struct Vector2D EstimatedSize;
};

struct FARQRCodeUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Transform WorldTransform;
	struct Vector Extents;
	struct FString QRCode;
};

struct FARPoseUpdatePayload
{
	struct Transform WorldTransform;
	TArray<struct Transform> JointTransforms;
};

struct FAREnvironmentProbeUpdatePayload
{
	struct Transform WorldTransform;
};

struct FARObjectUpdatePayload
{
	struct Transform WorldTransform;
};

struct FARMeshUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Transform WorldTransform;
	EARObjectClassification ObjectClassification;
};

struct FARGeoAnchorUpdatePayload
{
	struct ARSessionPayload SessionPayload;
	struct Transform WorldTransform;
	float Longitude;
	float Latitude;
	float AltitudeMeters;
	EARAltitudeSource AltitudeSource;
	struct FString AnchorName;
};

struct FARVideoFormat
{
	int FPS;
	int Width;
	int Height;
};

struct FARSkeletonDefinition
{
	int NumJoints;
	TArray<struct FName> JointNames;
	TArray<int> ParentIndices;
};

struct FARPose3D
{
	struct ARSkeletonDefinition SkeletonDefinition;
	TArray<struct Transform> JointTransforms;
	TArray<bool> IsJointTracked;
	EARJointTransformSpace JointTransformSpace;
};

struct FTrackedGeometryGroup
{
	class UClass* ARActor;
	class UClass* ARComponent;
	class UClass* TrackedGeometry;
};

struct FARSharedWorldReplicationState
{
	int PreviewImageOffset;
	int ARWorldOffset;
};

struct FARTraceResult
{
	float DistanceFromCamera;
	EARLineTraceChannels TraceChannel;
	struct Transform LocalTransform;
	class UClass* TrackedGeometry;
};

struct FARCameraIntrinsics
{
	struct IntPoint ImageResolution;
	struct Vector2D FocalLength;
	struct Vector2D PrincipalPoint;
};

struct FARPose2D
{
	struct ARSkeletonDefinition SkeletonDefinition;
	TArray<struct Vector2D> JointLocations;
	TArray<bool> IsJointTracked;
};

struct FARSessionStatus
{
	struct FString AdditionalInfo;
	EARSessionStatus Status;
};

struct FBattlePassLandingPageEntryPreviewInfo
{
	EBattlePassLandingPageSpecialEntryType SpecialEntryType;
	TArray<EFortItemType> SubscriptionItemTypesToDisplay;
	struct GameplayTag SpecialCharacterVariantChannelToModify;
	struct GameplayTag SpecialCharacterActiveVariantTag;
	TArray<struct TSoftClassPtr<UObject>> PreviewItems;
	float TransitionTime;
};

struct FBattlePassEntrySelectedParams
{
};

struct FSHAHashData
{
	byte Hash;
};

struct FChunkPartData
{
	struct Guid Guid;
	uint32_t Offset;
	uint32_t Size;
};

struct FFileManifestData
{
	struct FString Filename;
	struct SHAHashData FileHash;
	TArray<struct ChunkPartData> FileChunkParts;
	TArray<struct FString> InstallTags;
	bool bIsUnixExecutable;
	struct FString SymlinkTarget;
	bool bIsReadOnly;
	bool bIsCompressed;
};

struct FChunkInfoData
{
	struct Guid Guid;
	uint64_t Hash;
	struct SHAHashData ShaHash;
	int64_t FileSize;
	byte GroupNumber;
};

struct FCustomFieldData
{
	struct FString Key;
	struct FString Value;
};

struct FChaosClothWeightedValue
{
	float Low;
	float High;
};

struct FChaosDestructionEvent
{
	struct Vector Position;
	struct Vector Normal;
	struct Vector Velocity;
	struct Vector AngularVelocity;
	float ExtentMin;
	float ExtentMax;
	int ParticleID;
	float Time;
	int Type;
};

struct FChaosHandlerSet
{
	Unknown ChaosHandlers;
};

struct FBreakEventCallbackWrapper
{
};

struct FChaosDebugSubstepControl
{
	bool bPause;
	bool bSubstep;
	bool bStep;
};

struct FChaosPhysicsCollisionInfo
{
	class UClass* Component;
	class UClass* OtherComponent;
	struct Vector Location;
	struct Vector Normal;
	struct Vector AccumulatedImpulse;
	struct Vector Velocity;
	struct Vector OtherVelocity;
	struct Vector AngularVelocity;
	struct Vector OtherAngularVelocity;
	float Mass;
	float OtherMass;
};

struct FChaosBreakEvent
{
	class UClass* Component;
	struct Vector Location;
	struct Vector Velocity;
	struct Vector AngularVelocity;
	float Mass;
};

struct FSolverCollisionFilterSettings
{
	bool FilterEnabled;
	float MinMass;
	float MinSpeed;
	float MinImpulse;
};

struct FSolverBreakingFilterSettings
{
	bool FilterEnabled;
	float MinMass;
	float MinSpeed;
	float MinVolume;
};

struct FSolverTrailingFilterSettings
{
	bool FilterEnabled;
	float MinMass;
	float MinSpeed;
	float MinVolume;
};

struct FChaosSolverConfiguration
{
	int Iterations;
	int CollisionPairIterations;
	int PushOutIterations;
	int CollisionPushOutPairIterations;
	float CollisionMarginFraction;
	float CollisionMarginMax;
	float CollisionCullDistance;
	int JointPairIterations;
	int JointPushOutPairIterations;
	float ClusterConnectionFactor;
	EClusterUnionMethod ClusterUnionConnectionType;
	bool bGenerateCollisionData;
	struct SolverCollisionFilterSettings CollisionFilterSettings;
	bool bGenerateBreakData;
	struct SolverBreakingFilterSettings BreakingFilterSettings;
	bool bGenerateTrailingData;
	struct SolverTrailingFilterSettings TrailingFilterSettings;
	bool bGenerateContactGraph;
};

struct FSolverCollisionData
{
	struct Vector Location;
	struct Vector AccumulatedImpulse;
	struct Vector Normal;
	struct Vector Velocity1;
	struct Vector Velocity2;
	struct Vector AngularVelocity1;
	struct Vector AngularVelocity2;
	float Mass1;
	float Mass2;
	int ParticleIndex;
	int LevelsetIndex;
	int ParticleIndexMesh;
	int LevelsetIndexMesh;
};

struct FSolverBreakingData
{
	struct Vector Location;
	struct Vector Velocity;
	struct Vector AngularVelocity;
	float Mass;
	int ParticleIndex;
	int ParticleIndexMesh;
};

struct FSolverTrailingData
{
	struct Vector Location;
	struct Vector Velocity;
	struct Vector AngularVelocity;
	float Mass;
	int ParticleIndex;
	int ParticleIndexMesh;
};

struct FRecordedFrame
{
	TArray<struct Transform> Transforms;
	TArray<int> TransformIndices;
	TArray<int> PreviousTransformIndices;
	TArray<bool> DisabledFlags;
	TArray<struct SolverCollisionData> CollisionS;
	TArray<struct SolverBreakingData> Breakings;
	Unknown Trailings;
	float Timestamp;
};

struct FRecordedTransformTrack
{
	TArray<struct RecordedFrame> Records;
};

struct FCameraLookatTrackingSettings
{
	bool bEnableLookAtTracking;
	bool bDrawDebugLookAtTrackingPosition;
	float LookAtTrackingInterpSpeed;
	struct TSoftClassPtr<UObject> ActorToTrack;
	struct Vector RelativeOffset;
	bool bAllowRoll;
};

struct FCameraFilmbackSettings
{
	float SensorWidth;
	float SensorHeight;
	float SensorAspectRatio;
};

struct FCameraLensSettings
{
	float MinFocalLength;
	float MaxFocalLength;
	float MinFStop;
	float MaxFStop;
	float MinimumFocusDistance;
	int DiaphragmBladeCount;
};

struct FCameraTrackingFocusSettings
{
	struct TSoftClassPtr<UObject> ActorToTrack;
	struct Vector RelativeOffset;
	bool bDrawDebugTrackingFocusPoint;
};

struct FCameraFocusSettings
{
	ECameraFocusMethod FocusMethod;
	float ManualFocusDistance;
	struct CameraTrackingFocusSettings TrackingFocusSettings;
	bool bDrawDebugFocusPlane;
	struct Color DebugFocusPlaneColor;
	bool bSmoothFocusChanges;
	float FocusSmoothingInterpSpeed;
	float FocusOffset;
};

struct FNamedFilmbackPreset
{
	struct FString Name;
	struct CameraFilmbackSettings FilmbackSettings;
};

struct FNamedLensPreset
{
	struct FString Name;
	struct CameraLensSettings LensSettings;
};

struct FPointWeightMap
{
	TArray<float> Values;
};

struct FClothPhysicalMeshData
{
	TArray<struct Vector> Vertices;
	TArray<struct Vector> Normals;
	TArray<uint32_t> Indices;
	Unknown WeightMaps;
	TArray<float> InverseMasses;
	TArray<struct ClothVertBoneData> BoneData;
	int MaxBoneWeights;
	int NumFixedVerts;
	TArray<uint32_t> SelfCollisionIndices;
	TArray<float> MaxDistances;
	TArray<float> BackstopDistances;
	TArray<float> BackstopRadiuses;
	TArray<float> AnimDriveMultipliers;
};

struct FClothLODDataCommon
{
	struct ClothPhysicalMeshData PhysicalMeshData;
	struct ClothCollisionData CollisionData;
	bool bUseMultipleInfluences;
	float SkinningKernelRadius;
	bool bSmoothTransition;
};

struct FClothConstraintSetup_Legacy
{
	float Stiffness;
	float StiffnessMultiplier;
	float StretchLimit;
	float CompressionLimit;
};

struct FClothConfig_Legacy
{
	EClothingWindMethod_Legacy WindMethod;
	struct ClothConstraintSetup_Legacy VerticalConstraintConfig;
	struct ClothConstraintSetup_Legacy HorizontalConstraintConfig;
	struct ClothConstraintSetup_Legacy BendConstraintConfig;
	struct ClothConstraintSetup_Legacy ShearConstraintConfig;
	float SelfCollisionRadius;
	float SelfCollisionStiffness;
	float SelfCollisionCullScale;
	struct Vector Damping;
	float Friction;
	float WindDragCoefficient;
	float WindLiftCoefficient;
	struct Vector LinearDrag;
	struct Vector AngularDrag;
	struct Vector LinearInertiaScale;
	struct Vector AngularInertiaScale;
	struct Vector CentrifugalInertiaScale;
	float SolverFrequency;
	float StiffnessFrequency;
	float GravityScale;
	struct Vector GravityOverride;
	bool bUseGravityOverride;
	float TetherStiffness;
	float TetherLimit;
	float CollisionThickness;
	float AnimDriveSpringStiffness;
	float AnimDriveDamperStiffness;
};

struct FClothParameterMask_Legacy
{
	struct FName MaskName;
	EWeightMapTargetCommon CurrentTarget;
	float MaxValue;
	float MinValue;
	TArray<float> Values;
	bool bEnabled;
};

struct FClothVertBoneData
{
	int NumInfluences;
	uint16_t BoneIndices;
	float BoneWeights;
};

struct FClothCollisionPrim_Sphere
{
	int BoneIndex;
	float Radius;
	struct Vector LocalPosition;
};

struct FClothCollisionPrim_SphereConnection
{
	int SphereIndices;
};

struct FClothCollisionPrim_ConvexFace
{
	struct Plane Plane;
	TArray<int> Indices;
};

struct FClothCollisionPrim_Convex
{
	TArray<struct ClothCollisionPrim_ConvexFace> Faces;
	TArray<struct Vector> SurfacePoints;
	int BoneIndex;
};

struct FClothCollisionPrim_Box
{
	struct Vector LocalPosition;
	struct Quat LocalRotation;
	struct Vector HalfExtents;
	int BoneIndex;
};

struct FClothCollisionData
{
	TArray<struct ClothCollisionPrim_Sphere> Spheres;
	TArray<struct ClothCollisionPrim_SphereConnection> SphereConnections;
	TArray<struct ClothCollisionPrim_Convex> Convexes;
	TArray<struct ClothCollisionPrim_Box> Boxes;
};

struct FClothConstraintSetupNv
{
	float Stiffness;
	float StiffnessMultiplier;
	float StretchLimit;
	float CompressionLimit;
};

struct FConversationEntryList
{
	struct GameplayTag EntryTag;
	TArray<struct Guid> DestinationList;
};

struct FCommonDialogueBankParticipant
{
	struct FText FallbackName;
	struct GameplayTag ParticipantName;
	struct LinearColor NodeTint;
};

struct FConversationParticipantEntry
{
	class UClass* Actor;
	struct GameplayTag ParticipantID;
};

struct FConversationParticipants
{
	TArray<struct ConversationParticipantEntry> List;
};

struct FNetSerializeScriptStructCache_ConvVersion
{
	Unknown ScriptStructsToIndex;
	TArray<class UClass*> IndexToScriptStructs;
};

struct FConversationNodeHandle
{
	struct Guid NodeGUID;
};

struct FConversationContext
{
	class UClass* ConversationRegistry;
	class UClass* ActiveConversation;
	class UClass* ClientParticipant;
	class UClass* TaskBeingConsidered;
	TArray<struct ConversationNodeHandle> ReturnScopeStack;
	bool bServer;
};

struct FConversationNodeParameterPair
{
	struct FString Name;
	struct FString Value;
};

struct FConversationChoiceReference
{
	struct ConversationNodeHandle NodeReference;
	TArray<struct ConversationNodeParameterPair> NodeParameters;
};

struct FAdvanceConversationRequest
{
	struct ConversationChoiceReference Choice;
	TArray<struct ConversationNodeParameterPair> UserParameters;
};

struct FClientConversationMessage
{
	struct GameplayTag SpeakerID;
	struct FText ParticipantDisplayName;
	struct FText Text;
};

struct FConversationTaskResult
{
	EConversationTaskResultType Type;
	struct AdvanceConversationRequest AdvanceToChoice;
	struct ClientConversationMessage Message;
};

struct FClientConversationOptionEntry
{
	struct FText ChoiceText;
	struct GameplayTagContainer ChoiceTags;
	EConversationChoiceType ChoiceType;
	struct ConversationChoiceReference ChoiceReference;
	TArray<struct ConversationNodeParameterPair> ExtraData;
};

struct FClientConversationMessagePayload
{
	struct ClientConversationMessage Message;
	struct ConversationParticipants Participants;
	struct ConversationNodeHandle CurrentNode;
	TArray<struct ClientConversationOptionEntry> Options;
};

struct FConversationBranchPoint
{
	TArray<struct ConversationNodeHandle> ReturnScopeStack;
	struct ClientConversationOptionEntry ClientChoice;
};

struct FConversationChoiceDataHandle
{
};

struct FConversationChoiceData
{
};

struct FCommonInputKeyBrushConfiguration
{
	struct Key Key;
	struct SlateBrush KeyBrush;
};

struct FCommonInputKeySetBrushConfiguration
{
	TArray<struct Key> Keys;
	struct SlateBrush KeyBrush;
};

struct FCommonInputPlatformBaseData
{
	bool bSupported;
	ECommonInputType DefaultInputType;
	bool bSupportsMouseAndKeyboard;
	bool bSupportsGamepad;
	struct FName DefaultGamepadName;
	bool bCanChangeGamepadType;
	bool bSupportsTouch;
	TArray<struct TSoftClassPtr<UObject>> ControllerData;
	TArray<class UClass*> ControllerDataClasses;
};

struct FOperation
{
	EOperation Operation;
	class UClass* Panel;
	bool bIntroPanel;
	bool bActivatePanel;
	bool bOutroPanelBelow;
};

struct FCommonButtonStyleOptionalSlateSound
{
	bool bHasSound;
	struct SlateSound Sound;
};

struct FCommonNumberFormattingOptions
{
	ERoundingMode RoundingMode;
	bool UseGrouping;
	int MinimumIntegralDigits;
	int MaximumIntegralDigits;
	int MinimumFractionalDigits;
	int MaximumFractionalDigits;
};

struct FCommonRegisteredTabInfo
{
	int TabIndex;
	class UClass* TabButton;
	class UClass* ContentInstance;
};

struct FUIActionKeyMapping
{
	struct Key Key;
	float HoldTime;
};

struct FUIInputAction
{
	struct UIActionTag ActionTag;
	struct FText DefaultDisplayName;
	TArray<struct UIActionKeyMapping> KeyMappings;
};

struct FCommonAnalogCursorSettings
{
	int PreprocessorPriority;
	bool bEnableCursorAcceleration;
	float CursorAcceleration;
	float CursorMaxSpeed;
	float CursorDeadZone;
	float HoverSlowdownFactor;
	float ScrollDeadZone;
	float ScrollUpdatePeriod;
	float ScrollMultiplier;
};

struct FCommonInputActionHandlerData
{
	struct DataTableRowHandle InputActionRow;
	EInputActionState State;
};

struct FCommonInputTypeInfo
{
	struct Key Key;
	EInputActionState OverrrideState;
	bool bActionRequiresHold;
	float HoldTime;
	struct SlateBrush OverrideBrush;
};

struct FRigElement
{
	struct FName Name;
	int Index;
};

struct FRigBoneHierarchy
{
	TArray<struct RigBone> Bones;
	Unknown NameToIndexMapping;
	TArray<struct FName> Selection;
};

struct FRigSpaceHierarchy
{
	TArray<struct RigSpace> Spaces;
	Unknown NameToIndexMapping;
	TArray<struct FName> Selection;
};

struct FRigControlValueStorage
{
	float Float00;
	float Float01;
	float Float02;
	float Float03;
	float Float10;
	float Float11;
	float Float12;
	float Float13;
	float Float20;
	float Float21;
	float Float22;
	float Float23;
	float Float30;
	float Float31;
	float Float32;
	float Float33;
	bool bValid;
};

struct FRigControlValue
{
	struct RigControlValueStorage FloatStorage;
	struct Transform Storage;
};

struct FRigControlHierarchy
{
	TArray<struct RigControl> Controls;
	Unknown NameToIndexMapping;
	TArray<struct FName> Selection;
};

struct FRigCurveContainer
{
	TArray<struct RigCurve> Curves;
	Unknown NameToIndexMapping;
	TArray<struct FName> Selection;
};

struct FRigHierarchyContainer
{
	struct RigBoneHierarchy BoneHierarchy;
	struct RigSpaceHierarchy SpaceHierarchy;
	struct RigControlHierarchy ControlHierarchy;
	struct RigCurveContainer CurveContainer;
	int Version;
};

struct FControlRigDrawInstruction
{
	struct FName Name;
	EControlRigDrawSettings PrimitiveType;
	TArray<struct Vector> Positions;
	struct LinearColor Color;
	float Thickness;
	struct Transform Transform;
};

struct FControlRigDrawContainer
{
	TArray<struct ControlRigDrawInstruction> Instructions;
};

struct FRigElementKey
{
	ERigElementType Type;
	struct FName Name;
};

struct FRigInfluenceEntry
{
	struct RigElementKey Source;
	TArray<struct RigElementKey> AffectedList;
};

struct FRigInfluenceMap
{
	struct FName EventName;
	TArray<struct RigInfluenceEntry> Entries;
	Unknown KeyToIndex;
};

struct FRigInfluenceMapPerEvent
{
	TArray<struct RigInfluenceMap> Maps;
	Unknown EventToIndex;
};

struct FControlRigComponentMappedElement
{
	struct ComponentReference ComponentReference;
	int TransformIndex;
	struct FName TransformName;
	ERigElementType ElementType;
	struct FName ElementName;
	EControlRigComponentMapDirection Direction;
	struct Transform Offset;
	float Weight;
	EControlRigComponentSpace Space;
	class UClass* SceneComponent;
	int ElementIndex;
	int SubIndex;
};

struct FControlRigGizmoDefinition
{
	struct FName GizmoName;
	struct TSoftClassPtr<UObject> StaticMesh;
	struct Transform Transform;
};

struct FCachedRigElement
{
	struct RigElementKey Key;
	uint16_t Index;
	int ContainerVersion;
};

struct FRigPoseElement
{
	struct CachedRigElement Index;
	struct Transform GlobalTransform;
	struct Transform LocalTransform;
	float CurveValue;
};

struct FRigPose
{
	TArray<struct RigPoseElement> Elements;
};

struct FChannelMapInfo
{
	int ControlIndex;
	int TotalChannelIndex;
	int ChannelIndex;
	int ParentControlIndex;
	struct FName ChannelTypeName;
};

struct FEnumParameterNameAndCurve
{
	struct FName ParameterName;
	struct MovieSceneByteChannel ParameterCurve;
};

struct FIntegerParameterNameAndCurve
{
	struct FName ParameterName;
	struct MovieSceneIntegerChannel ParameterCurve;
};

struct FConstraintNodeData
{
	struct Transform RelativeParent;
	struct ConstraintOffset ConstraintOffset;
	struct FName LinkedNode;
	TArray<struct TransformConstraint> Constraints;
};

struct FControlRigIOSettings
{
	bool bUpdatePose;
	bool bUpdateCurves;
};

struct FControlRigComponentMappedCurve
{
	struct FName Source;
	struct FName Target;
};

struct FControlRigComponentMappedBone
{
	struct FName Source;
	struct FName Target;
};

struct FControlRigComponentMappedComponent
{
	class UClass* Component;
	struct FName ElementName;
	ERigElementType ElementType;
	EControlRigComponentMapDirection Direction;
};

struct FGizmoActorCreationParam
{
};

struct FCRFourPointBezier
{
	struct Vector A;
	struct Vector B;
	struct Vector C;
	struct Vector D;
};

struct FControlRigSequenceObjectReference
{
	class UClass* ControlRigClass;
};

struct FControlRigSequenceObjectReferences
{
	TArray<struct ControlRigSequenceObjectReference> Array;
};

struct FControlRigSequenceObjectReferenceMap
{
	TArray<struct Guid> BindingIds;
	TArray<struct ControlRigSequenceObjectReferences> References;
};

struct FControlRigSettingsPerPinBool
{
	Unknown Values;
};

struct FControlRigValidationContext
{
};

struct FCRSimContainer
{
	float TimeStep;
	float AccumulatedTime;
	float TimeLeftForStep;
};

struct FCRSimLinearSpring
{
	int SubjectA;
	int SubjectB;
	float Coefficient;
	float Equilibrium;
};

struct FCRSimPoint
{
	float Mass;
	float Size;
	float LinearDamping;
	float InheritMotion;
	struct Vector Position;
	struct Vector LinearVelocity;
};

struct FCRSimPointConstraint
{
	ECRSimConstraintType Type;
	int SubjectA;
	int SubjectB;
	struct Vector DataA;
	struct Vector DataB;
};

struct FCRSimPointForce
{
	ECRSimPointForceType ForceType;
	struct Vector Vector;
	float Coefficient;
	bool bNormalize;
};

struct FCRSimSoftCollision
{
	struct Transform Transform;
	ECRSimSoftCollisionType ShapeType;
	float MinimumDistance;
	float MaximumDistance;
	EControlRigAnimEasingType FalloffType;
	float Coefficient;
	bool bInverted;
};

struct FRigHierarchyRef
{
};

struct FRigMirrorSettings
{
	EAxis MirrorAxis;
	EAxis AxisToFlip;
	struct FString OldName;
	struct FString NewName;
};

struct FRigHierarchyCopyPasteContent
{
	TArray<ERigElementType> Types;
	TArray<struct FString> Contents;
	TArray<struct Transform> LocalTransforms;
	TArray<struct Transform> GlobalTransforms;
};

struct FRigEventContext
{
};

struct FRigElementKeyCollection
{
};

struct FRigControlModifiedContext
{
};

struct FRigInfluenceEntryModifier
{
	TArray<struct RigElementKey> AffectedList;
};

struct FRigUnit_AimItem_Target
{
	float Weight;
	struct Vector Axis;
	struct Vector Target;
	EControlRigVectorKind Kind;
	struct RigElementKey Space;
};

struct FRigUnit_AimBone_DebugSettings
{
	bool bEnabled;
	float Scale;
	struct Transform WorldOffset;
};

struct FRigUnit_AimBone_Target
{
	float Weight;
	struct Vector Axis;
	struct Vector Target;
	EControlRigVectorKind Kind;
	struct FName Space;
};

struct FAimTarget
{
	float Weight;
	struct Transform Transform;
	struct Vector AlignVector;
};

struct FRigUnit_AimConstraint_WorkData
{
	TArray<struct ConstraintData> ConstraintData;
};

struct FBlendTarget
{
	struct Transform Transform;
	float Weight;
};

struct FRigUnit_Harmonics_TargetItem
{
	struct RigElementKey Item;
	float Ratio;
};

struct FRigUnit_BoneHarmonics_WorkData
{
	TArray<struct CachedRigElement> CachedItems;
	struct Vector WaveTime;
};

struct FRigUnit_BoneHarmonics_BoneTarget
{
	struct FName Bone;
	float Ratio;
};

struct FRigUnit_CCDIK_RotationLimitPerItem
{
	struct RigElementKey Item;
	float Limit;
};

struct FRigUnit_CCDIK_WorkData
{
	TArray<struct CCDIKChainLink> Chain;
	TArray<struct CachedRigElement> CachedItems;
	TArray<int> RotationLimitIndex;
	TArray<float> RotationLimitsPerItem;
	struct CachedRigElement CachedEffector;
};

struct FRigUnit_CCDIK_RotationLimit
{
	struct FName Bone;
	float Limit;
};

struct FRigUnit_ChainHarmonics_Reach
{
	bool bEnabled;
	struct Vector ReachTarget;
	struct Vector ReachAxis;
	float ReachMinimum;
	float ReachMaximum;
	EControlRigAnimEasingType ReachEase;
};

struct FRigUnit_ChainHarmonics_Wave
{
	bool bEnabled;
	struct Vector WaveFrequency;
	struct Vector WaveAmplitude;
	struct Vector WaveOffset;
	struct Vector WaveNoise;
	float WaveMinimum;
	float WaveMaximum;
	EControlRigAnimEasingType WaveEase;
};

struct FRigUnit_ChainHarmonics_Pendulum
{
	bool bEnabled;
	float PendulumStiffness;
	struct Vector PendulumGravity;
	float PendulumBlend;
	float PendulumDrag;
	float PendulumMinimum;
	float PendulumMaximum;
	EControlRigAnimEasingType PendulumEase;
	struct Vector UnwindAxis;
	float UnwindMinimum;
	float UnwindMaximum;
};

struct FRigUnit_ChainHarmonics_WorkData
{
	struct Vector Time;
	TArray<struct CachedRigElement> Items;
	TArray<float> Ratio;
	TArray<struct Vector> LocalTip;
	TArray<struct Vector> PendulumTip;
	TArray<struct Vector> PendulumPosition;
	TArray<struct Vector> PendulumVelocity;
	TArray<struct Vector> HierarchyLine;
	TArray<struct Vector> VelocityLines;
};

struct FRigUnit_DebugTransformArrayMutable_WorkData
{
	TArray<struct Transform> DrawTransforms;
};

struct FRigUnit_DistributeRotation_Rotation
{
	struct Quat Rotation;
	float Ratio;
};

struct FRigUnit_DistributeRotation_WorkData
{
	TArray<struct CachedRigElement> CachedItems;
	TArray<int> ItemRotationA;
	TArray<int> ItemRotationB;
	TArray<float> ItemRotationT;
	TArray<struct Transform> ItemLocalTransforms;
};

struct FRigUnit_FABRIK_WorkData
{
	TArray<struct FABRIKChainLink> Chain;
	TArray<struct CachedRigElement> CachedItems;
	struct CachedRigElement CachedEffector;
};

struct FRigUnit_FitChainToCurve_Rotation
{
	struct Quat Rotation;
	float Ratio;
};

struct FRigUnit_FitChainToCurve_DebugSettings
{
	bool bEnabled;
	float Scale;
	struct LinearColor CurveColor;
	struct LinearColor SegmentsColor;
	struct Transform WorldOffset;
};

struct FRigUnit_FitChainToCurve_WorkData
{
	float ChainLength;
	TArray<struct Vector> ItemPositions;
	TArray<float> ItemSegments;
	TArray<struct Vector> CurvePositions;
	TArray<float> CurveSegments;
	TArray<struct CachedRigElement> CachedItems;
	TArray<int> ItemRotationA;
	TArray<int> ItemRotationB;
	TArray<float> ItemRotationT;
	TArray<struct Transform> ItemLocalTransforms;
};

struct FRigUnit_MathRBFInterpolateVectorWorkData
{
};

struct FMathRBFInterpolateVectorXform_Target
{
	struct Vector Target;
	struct Transform Value;
};

struct FMathRBFInterpolateVectorQuat_Target
{
	struct Vector Target;
	struct Quat Value;
};

struct FMathRBFInterpolateVectorColor_Target
{
	struct Vector Target;
	struct LinearColor Value;
};

struct FMathRBFInterpolateVectorVector_Target
{
	struct Vector Target;
	struct Vector Value;
};

struct FMathRBFInterpolateVectorFloat_Target
{
	struct Vector Target;
	float Value;
};

struct FRigUnit_MathRBFInterpolateQuatWorkData
{
};

struct FMathRBFInterpolateQuatXform_Target
{
	struct Quat Target;
	struct Transform Value;
};

struct FMathRBFInterpolateQuatQuat_Target
{
	struct Quat Target;
	struct Quat Value;
};

struct FMathRBFInterpolateQuatColor_Target
{
	struct Quat Target;
	struct LinearColor Value;
};

struct FMathRBFInterpolateQuatVector_Target
{
	struct Quat Target;
	struct Vector Value;
};

struct FMathRBFInterpolateQuatFloat_Target
{
	struct Quat Target;
	float Value;
};

struct FRigUnit_ModifyBoneTransforms_PerBone
{
	struct FName Bone;
	struct Transform Transform;
};

struct FRigUnit_ModifyTransforms_WorkData
{
	TArray<struct CachedRigElement> CachedItems;
};

struct FRigUnit_ModifyTransforms_PerItem
{
	struct RigElementKey Item;
	struct Transform Transform;
};

struct FRigUnit_MultiFABRIK_EndEffector
{
	struct FName Bone;
	struct Vector Location;
};

struct FRigUnit_MultiFABRIK_WorkData
{
};

struct FRigUnit_PointSimulation_BoneTarget
{
	struct FName Bone;
	int TranslationPoint;
	int PrimaryAimPoint;
	int SecondaryAimPoint;
};

struct FRigUnit_PointSimulation_DebugSettings
{
	bool bEnabled;
	float Scale;
	float CollisionScale;
	bool bDrawPointsAsSpheres;
	struct LinearColor Color;
	struct Transform WorldOffset;
};

struct FRigUnit_PointSimulation_WorkData
{
	struct CRSimPointContainer Simulation;
	TArray<struct CachedRigElement> BoneIndices;
};

struct FRigUnit_SetMultiControlRotator_Entry
{
	struct FName Control;
	struct Rotator Rotator;
	EBoneGetterSetterMode Space;
};

struct FRigUnit_SetMultiControlVector2D_Entry
{
	struct FName Control;
	struct Vector2D Vector;
};

struct FRigUnit_SetMultiControlInteger_Entry
{
	struct FName Control;
	int IntegerValue;
};

struct FRigUnit_SetMultiControlFloat_Entry
{
	struct FName Control;
	float FloatValue;
};

struct FRigUnit_SetMultiControlBool_Entry
{
	struct FName Control;
	bool BoolValue;
};

struct FRigUnit_SlideChain_WorkData
{
	float ChainLength;
	TArray<float> ItemSegments;
	TArray<struct CachedRigElement> CachedItems;
	TArray<struct Transform> Transforms;
	TArray<struct Transform> BlendedTransforms;
};

struct FRigUnit_SpringIK_DebugSettings
{
	bool bEnabled;
	float Scale;
	struct LinearColor Color;
	struct Transform WorldOffset;
};

struct FRigUnit_SpringIK_WorkData
{
	TArray<struct CachedRigElement> CachedBones;
	struct CachedRigElement CachedPoleVector;
	TArray<struct Transform> Transforms;
	struct CRSimPointContainer Simulation;
};

struct FConstraintTarget
{
	struct Transform Transform;
	float Weight;
	bool bMaintainOffset;
	struct TransformFilter Filter;
};

struct FRigUnit_TransformConstraint_WorkData
{
	TArray<struct ConstraintData> ConstraintData;
	Unknown ConstraintDataToTargets;
};

struct FRigUnit_TwistBones_WorkData
{
	TArray<struct CachedRigElement> CachedItems;
	TArray<float> ItemRatios;
	TArray<struct Transform> ItemTransforms;
};

struct FRigUnit_TwoBoneIKSimple_DebugSettings
{
	bool bEnabled;
	float Scale;
	struct Transform WorldOffset;
};

struct FStructReference
{
};

struct FJoinabilitySettings
{
	struct FName SessionName;
	bool bPublicSearchable;
	bool bAllowInvites;
	bool bJoinViaPresence;
	bool bJoinViaPresenceFriendsOnly;
	int MaxPlayers;
	int MaxPartySize;
};

struct FUniqueNetIdWrapper
{
};

struct FGuid
{
	int A;
	int B;
	int C;
	int D;
};

struct FVector
{
	float X;
	float Y;
	float Z;
};

struct FVector4
{
	float X;
	float Y;
	float Z;
	float W;
};

struct FVector2D
{
	float X;
	float Y;
};

struct FTwoVectors
{
	struct Vector v1;
	struct Vector v2;
};

struct FRotator
{
	float Pitch;
	float Yaw;
	float Roll;
};

struct FQuat
{
	float X;
	float Y;
	float Z;
	float W;
};

struct FPackedNormal
{
	byte X;
	byte Y;
	byte Z;
	byte W;
};

struct FPackedRGB10A2N
{
	int Packed;
};

struct FPackedRGBA16N
{
	int XY;
	int ZW;
};

struct FIntPoint
{
	int X;
	int Y;
};

struct FIntVector
{
	int X;
	int Y;
	int Z;
};

struct FColor
{
	byte B;
	byte G;
	byte R;
	byte A;
};

struct FLinearColor
{
	float R;
	float G;
	float B;
	float A;
};

struct FBox
{
	struct Vector Min;
	struct Vector Max;
	byte IsValid;
};

struct FBox2D
{
	struct Vector2D Min;
	struct Vector2D Max;
	byte bIsValid;
};

struct FBoxSphereBounds
{
	struct Vector Origin;
	struct Vector BoxExtent;
	float SphereRadius;
};

struct FOrientedBox
{
	struct Vector Center;
	struct Vector AxisX;
	struct Vector AxisY;
	struct Vector AxisZ;
	float ExtentX;
	float ExtentY;
	float ExtentZ;
};

struct FMatrix
{
	struct Plane XPlane;
	struct Plane YPlane;
	struct Plane ZPlane;
	struct Plane WPlane;
};

struct FInterpCurvePointFloat
{
	float InVal;
	float OutVal;
	float ArriveTangent;
	float LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveFloat
{
	TArray<struct InterpCurvePointFloat> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FInterpCurvePointVector2D
{
	float InVal;
	struct Vector2D OutVal;
	struct Vector2D ArriveTangent;
	struct Vector2D LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveVector2D
{
	TArray<struct InterpCurvePointVector2D> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FInterpCurvePointVector
{
	float InVal;
	struct Vector OutVal;
	struct Vector ArriveTangent;
	struct Vector LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveVector
{
	TArray<struct InterpCurvePointVector> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FInterpCurvePointQuat
{
	float InVal;
	struct Quat OutVal;
	struct Quat ArriveTangent;
	struct Quat LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveQuat
{
	TArray<struct InterpCurvePointQuat> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FInterpCurvePointTwoVectors
{
	float InVal;
	struct TwoVectors OutVal;
	struct TwoVectors ArriveTangent;
	struct TwoVectors LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveTwoVectors
{
	TArray<struct InterpCurvePointTwoVectors> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FInterpCurvePointLinearColor
{
	float InVal;
	struct LinearColor OutVal;
	struct LinearColor ArriveTangent;
	struct LinearColor LeaveTangent;
	EInterpCurveMode InterpMode;
};

struct FInterpCurveLinearColor
{
	TArray<struct InterpCurvePointLinearColor> Points;
	bool bIsLooped;
	float LoopKeyOffset;
};

struct FTransform
{
	struct Quat Rotation;
	struct Vector Translation;
	struct Vector Scale3D;
};

struct FRandomStream
{
	int InitialSeed;
	int Seed;
};

struct FDateTime
{
};

struct FFrameNumber
{
	int Value;
};

struct FFrameRate
{
	int Numerator;
	int Denominator;
};

struct FFrameTime
{
	struct FrameNumber FrameNumber;
	float SubFrame;
};

struct FQualifiedFrameTime
{
	struct FrameTime Time;
	struct FrameRate Rate;
};

struct FTimecode
{
	int Hours;
	int Minutes;
	int Seconds;
	int Frames;
	bool bDropFrameFormat;
};

struct FTimespan
{
};

struct FSoftObjectPath
{
	struct FName AssetPathName;
	struct FString SubPathString;
};

struct FPrimaryAssetType
{
	struct FName Name;
};

struct FPrimaryAssetId
{
	struct PrimaryAssetType PrimaryAssetType;
	struct FName PrimaryAssetName;
};

struct FFallbackStruct
{
};

struct FFloatRangeBound
{
	ERangeBoundTypes Type;
	float Value;
};

struct FFloatRange
{
	struct FloatRangeBound LowerBound;
	struct FloatRangeBound UpperBound;
};

struct FInt32RangeBound
{
	ERangeBoundTypes Type;
	int Value;
};

struct FInt32Range
{
	struct Int32RangeBound LowerBound;
	struct Int32RangeBound UpperBound;
};

struct FFrameNumberRangeBound
{
	ERangeBoundTypes Type;
	struct FrameNumber Value;
};

struct FFrameNumberRange
{
	struct FrameNumberRangeBound LowerBound;
	struct FrameNumberRangeBound UpperBound;
};

struct FFloatInterval
{
	float Min;
	float Max;
};

struct FInt32Interval
{
	int Min;
	int Max;
};

struct FPolyglotTextData
{
	ELocalizedTextSourceCategory Category;
	struct FString NativeCulture;
	struct FString Namespace;
	struct FString Key;
	struct FString NativeString;
	Unknown LocalizedStrings;
	bool bIsMinimalPatch;
	struct FText CachedText;
};

struct FAutomationEvent
{
	EAutomationEventType Type;
	struct FString Message;
	struct FString Context;
	struct Guid Artifact;
};

struct FAutomationExecutionEntry
{
	struct AutomationEvent Event;
	struct FString Filename;
	int LineNumber;
	struct DateTime Timestamp;
};

struct FARFilter
{
	TArray<struct FName> PackageNames;
	TArray<struct FName> PackagePaths;
	TArray<struct FName> ObjectPaths;
	TArray<struct FName> ClassNames;
	Unknown RecursiveClassesExclusionSet;
	bool bRecursivePaths;
	bool bRecursiveClasses;
	bool bIncludeOnlyOnDiskAssets;
};

struct FAssetBundleEntry
{
	struct FName BundleName;
	TArray<struct SoftObjectPath> BundleAssets;
};

struct FAssetBundleData
{
	TArray<struct AssetBundleEntry> Bundles;
};

struct FAssetData
{
	struct FName ObjectPath;
	struct FName PackageName;
	struct FName PackagePath;
	struct FName AssetName;
	struct FName AssetClass;
};

struct FTestUninitializedScriptStructMembersTest
{
	class UClass* UninitializedObjectReference;
	class UClass* InitializedObjectReference;
	float UnusedValue;
};

struct FObject
{
};

struct FCraftingObjectRepStateData
{
	ECraftingObjectState CraftingObjectState;
	float StateChangeServerTime;
};

struct FCraftingResult
{
	struct FName ResultLootTierKey;
	TArray<struct ItemAndCount> Results;
};

struct FCraftingIngredientRequirement
{
	struct GameplayTagContainer IngredientTags;
	int Count;
};

struct FCraftingIngredientQueryState
{
	struct CraftingIngredientRequirement Requirement;
	int Owned;
	int Missing;
};

struct FCraftingUpgradeRule
{
	struct GameplayTagRequirements SourceItemTags;
	struct GameplayTagRequirements TargetItemTags;
	byte UpgradeFlags;
};

struct FCurieEffectContainer
{
	struct GameplayTagQuery TargetFilter;
	class UClass* GameplayEffect;
};

struct FCurieElementAttachHandlersContainer
{
	TArray<class UClass*> Handlers;
};

struct FCurieElementAttachConditionHandlersContainer
{
	TArray<class UClass*> Handlers;
};

struct FCurieElementPairKey
{
};

struct FCurieElementInteractWithElementHandlersContainer
{
	TArray<class UClass*> Handlers;
};

struct FCurieElementInteractWithMaterialHandlersContainer
{
	TArray<class UClass*> Handlers;
};

struct FCurieElementInteractWithContainerHandlersContainer
{
	TArray<class UClass*> Handlers;
};

struct FCurieContainerHandle
{
};

struct FCurieInteractParamsHandle
{
};

struct FCurieInteractHandle
{
};

struct FCurieElementHandle
{
};

struct FCurieStateHandle
{
};

struct FDataAssetDirectoryTestPODStruct
{
	EDataAssetDirectoryTestEnum EnumProperty;
	int IntProperty;
	float FloatProperty;
	bool BoolProperty;
	struct FString StringProperty;
	struct FName NameProperty;
	struct FText TextProperty;
};

struct FDataAssetDirectoryTestSimpleStruct
{
	int IntProperty;
};

struct FDataRegistryIdFormat
{
	struct GameplayTag BaseGameplayTag;
};

struct FDataRegistryCachePolicy
{
	bool bCacheIsAlwaysVolatile;
	bool bUseCurveTableCacheVersion;
	int MinNumberKept;
	int MaxNumberKept;
	float ForceKeepSeconds;
	float ForceReleaseSeconds;
};

struct FDataRegistrySource_DataTableRules
{
	bool bPrecacheTable;
	float CachedTableKeepSeconds;
};

struct FDataRegistryLookup
{
};

struct FDataRegistryType
{
	struct FName Name;
};

struct FDataRegistryId
{
	struct DataRegistryType RegistryType;
	struct FName ItemName;
};

struct FDataRegistrySourceItemId
{
};

struct FContextData
{
	TArray<Unknown> Contexts;
};

struct FDirectorData
{
	struct TSoftClassPtr<UObject> DirectorClass;
	class UClass* Instance;
};

struct FDynamicHUDAllowed
{
	struct TSoftClassPtr<UObject> Widget;
	EDynamicHUDZOrder ZOrder;
	int CustomZOrder;
	bool bIsUnique;
	struct FName UniqueId;
	bool bUseSafeZone;
	TArray<class UClass*> LayoutConstraints;
};

struct FDynamicHUDUnallowed
{
	struct TSoftClassPtr<UObject> Widget;
	bool bUseUniqueID;
	struct FName UniqueId;
	bool bIncludeAll;
};

struct FAdaptorPolygon2Group
{
	uint32_t RenderingSectionIndex;
	int MaterialIndex;
	int MaxTriangles;
};

struct FAdaptorPolygon
{
	struct PolygonGroupID PolygonGroupID;
	TArray<struct AdaptorTriangleID> TriangulatedPolygonTriangleIndices;
};

struct FPolygonGroupForPolygon
{
	struct PolygonID PolygonID;
	struct PolygonGroupID PolygonGroupID;
};

struct FMeshElementAttributeValue
{
};

struct FMeshElementAttributeData
{
	struct FName AttributeName;
	int AttributeIndex;
	struct MeshElementAttributeValue AttributeValue;
};

struct FMeshElementAttributeList
{
	TArray<struct MeshElementAttributeData> Attributes;
};

struct FPolygonGroupToCreate
{
	struct MeshElementAttributeList PolygonGroupAttributes;
	struct PolygonGroupID OriginalPolygonGroupID;
};

struct FVertexToMove
{
	struct VertexID VertexID;
	struct Vector NewVertexPosition;
};

struct FVertexIndexAndInstanceID
{
	int ContourIndex;
	struct VertexInstanceID VertexInstanceID;
};

struct FVertexInstancesForPolygonHole
{
	TArray<struct VertexIndexAndInstanceID> VertexIndicesAndInstanceIDs;
};

struct FChangeVertexInstancesForPolygon
{
	struct PolygonID PolygonID;
	TArray<struct VertexIndexAndInstanceID> PerimeterVertexIndicesAndInstanceIDs;
	TArray<struct VertexInstancesForPolygonHole> VertexIndicesAndInstanceIDsForEachHole;
};

struct FVertexAttributesForPolygonHole
{
	TArray<struct MeshElementAttributeList> VertexAttributeList;
};

struct FVertexAttributesForPolygon
{
	struct PolygonID PolygonID;
	TArray<struct MeshElementAttributeList> PerimeterVertexAttributeLists;
	TArray<struct VertexAttributesForPolygonHole> VertexAttributeListsForEachHole;
};

struct FAttributesForEdge
{
	struct EdgeID EdgeID;
	struct MeshElementAttributeList EdgeAttributes;
};

struct FAttributesForVertexInstance
{
	struct VertexInstanceID VertexInstanceID;
	struct MeshElementAttributeList VertexInstanceAttributes;
};

struct FAttributesForVertex
{
	struct VertexID VertexID;
	struct MeshElementAttributeList VertexAttributes;
};

struct FVertexPair
{
	struct VertexID VertexID0;
	struct VertexID VertexID1;
};

struct FPolygonToSplit
{
	struct PolygonID PolygonID;
	TArray<struct VertexPair> VertexPairsToSplitAt;
};

struct FVertexAndAttributes
{
	struct VertexInstanceID VertexInstanceID;
	struct VertexID VertexID;
	struct MeshElementAttributeList PolygonVertexAttributes;
};

struct FPolygonToCreate
{
	struct PolygonGroupID PolygonGroupID;
	TArray<struct VertexAndAttributes> PerimeterVertices;
	struct PolygonID OriginalPolygonID;
	EPolygonEdgeHardness PolygonEdgeHardness;
};

struct FEdgeToCreate
{
	struct VertexID VertexID0;
	struct VertexID VertexID1;
	struct MeshElementAttributeList EdgeAttributes;
	struct EdgeID OriginalEdgeID;
};

struct FVertexInstanceToCreate
{
	struct VertexID VertexID;
	struct MeshElementAttributeList VertexInstanceAttributes;
	struct VertexInstanceID OriginalVertexInstanceID;
};

struct FVertexToCreate
{
	struct MeshElementAttributeList VertexAttributes;
	struct VertexID OriginalVertexID;
};

struct FSubdividedQuadVertex
{
	int VertexPositionIndex;
	struct Vector2D TextureCoordinate0;
	struct Vector2D TextureCoordinate1;
	struct Color VertexColor;
	struct Vector VertexNormal;
	struct Vector VertexTangent;
	float VertexBinormalSign;
};

struct FSubdividedQuad
{
	struct SubdividedQuadVertex QuadVertex0;
	struct SubdividedQuadVertex QuadVertex1;
	struct SubdividedQuadVertex QuadVertex2;
	struct SubdividedQuadVertex QuadVertex3;
};

struct FSubdivisionLimitSection
{
	TArray<struct SubdividedQuad> SubdividedQuads;
};

struct FSubdividedWireEdge
{
	int EdgeVertex0PositionIndex;
	int EdgeVertex1PositionIndex;
};

struct FSubdivisionLimitData
{
	TArray<struct Vector> VertexPositions;
	TArray<struct SubdivisionLimitSection> Sections;
	TArray<struct SubdividedWireEdge> SubdividedWireEdges;
};

struct FRenderingPolygonGroup
{
	uint32_t RenderingSectionIndex;
	int MaterialIndex;
	int MaxTriangles;
};

struct FRenderingPolygon
{
	struct PolygonGroupID PolygonGroupID;
	TArray<struct TriangleID> TriangulatedPolygonTriangleIndices;
};

struct FEngineServiceNotification
{
	struct FString Text;
	double TimeSeconds;
};

struct FEngineServiceTerminate
{
	struct FString UserName;
};

struct FEngineServiceExecuteCommand
{
	struct FString Command;
	struct FString UserName;
};

struct FEngineServiceAuthGrant
{
	struct FString UserName;
	struct FString UserToGrant;
};

struct FEngineServiceAuthDeny
{
	struct FString UserName;
	struct FString UserToDeny;
};

struct FEngineServicePong
{
	struct FString CurrentLevel;
	int EngineVersion;
	bool HasBegunPlay;
	struct Guid InstanceID;
	struct FString InstanceType;
	struct Guid SessionId;
	float WorldTimeSeconds;
};

struct FEngineServicePing
{
};

struct FAutoCompleteCommand
{
	struct FString Command;
	struct FString Desc;
};

struct FGameModeName
{
	struct FString Name;
	struct SoftClassPath GameMode;
};

struct FTickFunction
{
	ETickingGroup TickGroup;
	ETickingGroup EndTickGroup;
	bool bTickEvenWhenPaused;
	bool bCanEverTick;
	bool bStartWithTickEnabled;
	bool bAllowTickOnDedicatedServer;
	float TickInterval;
};

struct FSimpleMemberReference
{
	class UClass* MemberParent;
	struct FName MemberName;
	struct Guid MemberGuid;
};

struct FRepMovement
{
	struct Vector LinearVelocity;
	struct Vector AngularVelocity;
	struct Vector Location;
	struct Rotator Rotation;
	bool bSimulatedPhysicSleep;
	bool bRepPhysics;
	EVectorQuantization LocationQuantizationLevel;
	EVectorQuantization VelocityQuantizationLevel;
	ERotatorQuantization RotationQuantizationLevel;
};

struct FRepAttachment
{
	class UClass* AttachParent;
	struct Vector_NetQuantize100 LocationOffset;
	struct Vector_NetQuantize100 RelativeScale3D;
	struct Rotator RotationOffset;
	struct FName AttachSocket;
	class UClass* AttachComponent;
};

struct FDebugTextInfo
{
	class UClass* SrcActor;
	struct Vector SrcActorOffset;
	struct Vector SrcActorDesiredOffset;
	struct FString DebugText;
	float TimeRemaining;
	float Duration;
	struct Color TextColor;
	bool bAbsoluteLocation;
	bool bKeepAttachedToActor;
	bool bDrawShadow;
	struct Vector OrigActorLocation;
	class UClass* Font;
	float FontScale;
};

struct FBPComponentClassOverride
{
	struct FName ComponentName;
	class UClass* ComponentClass;
};

struct FStatColorMapEntry
{
	float In;
	struct Color Out;
};

struct FStatColorMapping
{
	struct FString StatName;
	TArray<struct StatColorMapEntry> ColorMap;
	bool DisableBlend;
};

struct FGameNameRedirect
{
	struct FName OldGameName;
	struct FName NewGameName;
};

struct FClassRedirect
{
	struct FName ObjectName;
	struct FName OldClassName;
	struct FName NewClassName;
	struct FName OldSubobjName;
	struct FName NewSubobjName;
	struct FName NewClassClass;
	struct FName NewClassPackage;
	bool InstanceOnly;
};

struct FPluginRedirect
{
	struct FString OldPluginName;
	struct FString NewPluginName;
};

struct FStructRedirect
{
	struct FName OldStructName;
	struct FName NewStructName;
};

struct FDirectoryPath
{
	struct FString Path;
};

struct FDropNoteInfo
{
	struct Vector Location;
	struct Rotator Rotation;
	struct FString Comment;
};

struct FNetDriverDefinition
{
	struct FName DefName;
	struct FName DriverClassName;
	struct FName DriverClassNameFallback;
};

struct FDebugDisplayProperty
{
	class UClass* Obj;
	class UClass* WithinClass;
};

struct FChannelDefinition
{
	struct FName ChannelName;
	struct FName ClassName;
	class UClass* ChannelClass;
	int StaticChannelIndex;
	bool bTickOnCreate;
	bool bServerOpen;
	bool bClientOpen;
	bool bInitialServer;
	bool bInitialClient;
};

struct FLightingChannels
{
	bool bChannel0;
	bool bChannel1;
	bool bChannel2;
};

struct FCustomPrimitiveData
{
	TArray<float> Data;
};

struct FCollisionResponseContainer
{
	ECollisionResponse WorldStatic;
	ECollisionResponse WorldDynamic;
	ECollisionResponse Pawn;
	ECollisionResponse Visibility;
	ECollisionResponse Camera;
	ECollisionResponse PhysicsBody;
	ECollisionResponse Vehicle;
	ECollisionResponse Destructible;
	ECollisionResponse EngineTraceChannel1;
	ECollisionResponse EngineTraceChannel2;
	ECollisionResponse EngineTraceChannel3;
	ECollisionResponse EngineTraceChannel4;
	ECollisionResponse EngineTraceChannel5;
	ECollisionResponse EngineTraceChannel6;
	ECollisionResponse GameTraceChannel1;
	ECollisionResponse GameTraceChannel2;
	ECollisionResponse GameTraceChannel3;
	ECollisionResponse GameTraceChannel4;
	ECollisionResponse GameTraceChannel5;
	ECollisionResponse GameTraceChannel6;
	ECollisionResponse GameTraceChannel7;
	ECollisionResponse GameTraceChannel8;
	ECollisionResponse GameTraceChannel9;
	ECollisionResponse GameTraceChannel10;
	ECollisionResponse GameTraceChannel11;
	ECollisionResponse GameTraceChannel12;
	ECollisionResponse GameTraceChannel13;
	ECollisionResponse GameTraceChannel14;
	ECollisionResponse GameTraceChannel15;
	ECollisionResponse GameTraceChannel16;
	ECollisionResponse GameTraceChannel17;
	ECollisionResponse GameTraceChannel18;
};

struct FResponseChannel
{
	struct FName Channel;
	ECollisionResponse Response;
};

struct FCollisionResponse
{
	struct CollisionResponseContainer ResponseToChannels;
	TArray<struct ResponseChannel> ResponseArray;
};

struct FWalkableSlopeOverride
{
	EWalkableSlopeBehavior WalkableSlopeBehavior;
	float WalkableSlopeAngle;
};

struct FVertexOffsetUsage
{
	int Usage;
};

struct FSkelMeshComponentLODInfo
{
	TArray<bool> HiddenMaterials;
};

struct FSingleAnimationPlayData
{
	class UClass* AnimToPlay;
	bool bSavedLooping;
	bool bSavedPlaying;
	float SavedPosition;
	float SavedPlayRate;
};

struct FAnimNotifyEventReference
{
	class UClass* NotifySource;
};

struct FAnimNotifyArray
{
	TArray<struct AnimNotifyEventReference> Notifies;
};

struct FAnimNotifyQueue
{
	TArray<struct AnimNotifyEventReference> AnimNotifies;
	Unknown UnfilteredMontageAnimNotifies;
};

struct FAnimLinkableElement
{
	class UClass* LinkedMontage;
	int SlotIndex;
	int SegmentIndex;
	EAnimLinkMethod LinkMethod;
	EAnimLinkMethod CachedLinkMethod;
	float SegmentBeginTime;
	float SegmentLength;
	float LinkValue;
	class UClass* LinkedSequence;
};

struct FActiveForceFeedbackEffect
{
	class UClass* ForceFeedbackEffect;
};

struct FGeomSelection
{
	int Type;
	int Index;
	int SelectionIndex;
};

struct FSplineCurves
{
	struct InterpCurveVector Position;
	struct InterpCurveQuat Rotation;
	struct InterpCurveVector Scale;
	struct InterpCurveFloat ReparamTable;
	class UClass* MetaData;
	uint32_t Version;
};

struct FExpressionInput
{
	int OutputIndex;
	struct FName ExpressionName;
};

struct FBasedMovementInfo
{
	class UClass* MovementBase;
	struct FName BoneName;
	struct Vector_NetQuantize100 Location;
	struct Rotator Rotation;
	bool bServerHasBaseComponent;
	bool bRelativeRotation;
	bool bServerHasVelocity;
};

struct FRootMotionSourceSettings
{
	byte Flags;
};

struct FRootMotionSourceGroup
{
	bool bHasAdditiveSources;
	bool bHasOverrideSources;
	bool bHasOverrideSourcesWithIgnoreZAccumulate;
	bool bIsAdditiveVelocityApplied;
	struct RootMotionSourceSettings LastAccumulatedSettings;
	struct Vector_NetQuantize10 LastPreAdditiveVelocity;
};

struct FRootMotionMovementParams
{
	bool bHasRootMotion;
	float BlendWeight;
	struct Transform RootMotionTransform;
};

struct FRepRootMotionMontage
{
	bool bIsActive;
	class UClass* AnimMontage;
	float Position;
	struct Vector_NetQuantize100 Location;
	struct Rotator Rotation;
	class UClass* MovementBase;
	struct FName MovementBaseBoneName;
	bool bRelativePosition;
	bool bRelativeRotation;
	struct RootMotionSourceGroup AuthoritativeRootMotion;
	struct Vector_NetQuantize10 Acceleration;
	struct Vector_NetQuantize10 LinearVelocity;
};

struct FSimulatedRootMotionReplicatedMove
{
	float Time;
	struct RepRootMotionMontage RootMotion;
};

struct FStaticMeshComponentLODInfo
{
};

struct FStreamingTextureBuildInfo
{
	uint32_t PackedRelativeBox;
	int TextureLevelIndex;
	float TexelFactor;
};

struct FLightmassPrimitiveSettings
{
	bool bUseTwoSidedLighting;
	bool bShadowIndirectOnly;
	bool bUseEmissiveForStaticLighting;
	bool bUseVertexNormalForHemisphereGather;
	float EmissiveLightFalloffExponent;
	float EmissiveLightExplicitInfluenceRadius;
	float EmissiveBoost;
	float DiffuseBoost;
	float FullyOccludedSamplesFraction;
};

struct FBlueprintComponentChangedPropertyInfo
{
	struct FName PropertyName;
	int ArrayIndex;
	class UClass* PropertyScope;
};

struct FBlueprintCookedComponentInstancingData
{
	TArray<struct BlueprintComponentChangedPropertyInfo> ChangedPropertyList;
	bool bHasValidCookedData;
};

struct FMovementProperties
{
	bool bCanCrouch;
	bool bCanJump;
	bool bCanWalk;
	bool bCanSwim;
	bool bCanFly;
};

struct FSoundConcurrencySettings
{
	int MaxCount;
	bool bLimitToOwner;
	EMaxConcurrentResolutionRule ResolutionRule;
	float RetriggerTime;
	float VolumeScale;
	EConcurrencyVolumeScaleMode VolumeScaleMode;
	float VolumeScaleAttackTime;
	bool bVolumeScaleCanRelease;
	float VolumeScaleReleaseTime;
	float VoiceStealReleaseTime;
};

struct FKeyHandleMap
{
};

struct FIndexedCurve
{
	struct KeyHandleMap KeyHandlesToIndices;
};

struct FRichCurveKey
{
	ERichCurveInterpMode InterpMode;
	ERichCurveTangentMode TangentMode;
	ERichCurveTangentWeightMode TangentWeightMode;
	float Time;
	float Value;
	float ArriveTangent;
	float ArriveTangentWeight;
	float LeaveTangent;
	float LeaveTangentWeight;
};

struct FRuntimeFloatCurve
{
	struct RichCurve EditorCurveData;
	class UClass* ExternalCurve;
};

struct FSoundSubmixSendInfo
{
	ESendLevelControlMethod SendLevelControlMethod;
	ESubmixSendStage SendStage;
	class UClass* SoundSubmix;
	float SendLevel;
	float MinSendLevel;
	float MaxSendLevel;
	float MinSendDistance;
	float MaxSendDistance;
	struct RuntimeFloatCurve CustomSendLevelCurve;
};

struct FSoundSourceBusSendInfo
{
	ESourceBusSendLevelControlMethod SourceBusSendLevelControlMethod;
	class UClass* SoundSourceBus;
	class UClass* AudioBus;
	float SendLevel;
	float MinSendLevel;
	float MaxSendLevel;
	float MinSendDistance;
	float MaxSendDistance;
	struct RuntimeFloatCurve CustomSendLevelCurve;
};

struct FBaseAttenuationSettings
{
	EAttenuationDistanceModel DistanceAlgorithm;
	EAttenuationShape AttenuationShape;
	float dBAttenuationAtMax;
	ENaturalSoundFalloffMode FalloffMode;
	struct Vector AttenuationShapeExtents;
	float ConeOffset;
	float FalloffDistance;
	float ConeSphereRadius;
	float ConeSphereFalloffDistance;
	struct RuntimeFloatCurve CustomAttenuationCurve;
};

struct FAttenuationSubmixSendSettings
{
	class UClass* Submix;
	ESubmixSendMethod SubmixSendMethod;
	float SubmixSendLevelMin;
	float SubmixSendLevelMax;
	float SubmixSendDistanceMin;
	float SubmixSendDistanceMax;
	float ManualSubmixSendLevel;
	struct RuntimeFloatCurve CustomSubmixSendCurve;
};

struct FSoundAttenuationPluginSettings
{
	TArray<class UClass*> SpatializationPluginSettingsArray;
	TArray<class UClass*> OcclusionPluginSettingsArray;
	TArray<class UClass*> ReverbPluginSettingsArray;
};

struct FPerPlatformFloat
{
	float Default;
};

struct FEngineShowFlagsSetting
{
	struct FString ShowFlagName;
	bool Enabled;
};

struct FWeightedBlendable
{
	float Weight;
	class UClass* Object;
};

struct FWeightedBlendables
{
	TArray<struct WeightedBlendable> Array;
};

struct FPostProcessSettings
{
	bool bOverride_WhiteTemp;
	bool bOverride_WhiteTint;
	bool bOverride_ColorSaturation;
	bool bOverride_ColorContrast;
	bool bOverride_ColorGamma;
	bool bOverride_ColorGain;
	bool bOverride_ColorOffset;
	bool bOverride_ColorSaturationShadows;
	bool bOverride_ColorContrastShadows;
	bool bOverride_ColorGammaShadows;
	bool bOverride_ColorGainShadows;
	bool bOverride_ColorOffsetShadows;
	bool bOverride_ColorSaturationMidtones;
	bool bOverride_ColorContrastMidtones;
	bool bOverride_ColorGammaMidtones;
	bool bOverride_ColorGainMidtones;
	bool bOverride_ColorOffsetMidtones;
	bool bOverride_ColorSaturationHighlights;
	bool bOverride_ColorContrastHighlights;
	bool bOverride_ColorGammaHighlights;
	bool bOverride_ColorGainHighlights;
	bool bOverride_ColorOffsetHighlights;
	bool bOverride_ColorCorrectionShadowsMax;
	bool bOverride_ColorCorrectionHighlightsMin;
	bool bOverride_BlueCorrection;
	bool bOverride_ExpandGamut;
	bool bOverride_ToneCurveAmount;
	bool bOverride_FilmWhitePoint;
	bool bOverride_FilmSaturation;
	bool bOverride_FilmChannelMixerRed;
	bool bOverride_FilmChannelMixerGreen;
	bool bOverride_FilmChannelMixerBlue;
	bool bOverride_FilmContrast;
	bool bOverride_FilmDynamicRange;
	bool bOverride_FilmHealAmount;
	bool bOverride_FilmToeAmount;
	bool bOverride_FilmShadowTint;
	bool bOverride_FilmShadowTintBlend;
	bool bOverride_FilmShadowTintAmount;
	bool bOverride_FilmSlope;
	bool bOverride_FilmToe;
	bool bOverride_FilmShoulder;
	bool bOverride_FilmBlackClip;
	bool bOverride_FilmWhiteClip;
	bool bOverride_SceneColorTint;
	bool bOverride_SceneFringeIntensity;
	bool bOverride_ChromaticAberrationStartOffset;
	bool bOverride_AmbientCubemapTint;
	bool bOverride_AmbientCubemapIntensity;
	bool bOverride_BloomMethod;
	bool bOverride_BloomIntensity;
	bool bOverride_BloomThreshold;
	bool bOverride_Bloom1Tint;
	bool bOverride_Bloom1Size;
	bool bOverride_Bloom2Size;
	bool bOverride_Bloom2Tint;
	bool bOverride_Bloom3Tint;
	bool bOverride_Bloom3Size;
	bool bOverride_Bloom4Tint;
	bool bOverride_Bloom4Size;
	bool bOverride_Bloom5Tint;
	bool bOverride_Bloom5Size;
	bool bOverride_Bloom6Tint;
	bool bOverride_Bloom6Size;
	bool bOverride_BloomSizeScale;
	bool bOverride_BloomConvolutionTexture;
	bool bOverride_BloomConvolutionSize;
	bool bOverride_BloomConvolutionCenterUV;
	bool bOverride_BloomConvolutionPreFilter;
	bool bOverride_BloomConvolutionPreFilterMin;
	bool bOverride_BloomConvolutionPreFilterMax;
	bool bOverride_BloomConvolutionPreFilterMult;
	bool bOverride_BloomConvolutionBufferScale;
	bool bOverride_BloomDirtMaskIntensity;
	bool bOverride_BloomDirtMaskTint;
	bool bOverride_BloomDirtMask;
	bool bOverride_CameraShutterSpeed;
	bool bOverride_CameraISO;
	bool bOverride_AutoExposureMethod;
	bool bOverride_AutoExposureLowPercent;
	bool bOverride_AutoExposureHighPercent;
	bool bOverride_AutoExposureMinBrightness;
	bool bOverride_AutoExposureMaxBrightness;
	bool bOverride_AutoExposureCalibrationConstant;
	bool bOverride_AutoExposureSpeedUp;
	bool bOverride_AutoExposureSpeedDown;
	bool bOverride_AutoExposureBias;
	bool bOverride_AutoExposureBiasCurve;
	bool bOverride_AutoExposureMeterMask;
	bool bOverride_AutoExposureApplyPhysicalCameraExposure;
	bool bOverride_HistogramLogMin;
	bool bOverride_HistogramLogMax;
	bool bOverride_LensFlareIntensity;
	bool bOverride_LensFlareTint;
	bool bOverride_LensFlareTints;
	bool bOverride_LensFlareBokehSize;
	bool bOverride_LensFlareBokehShape;
	bool bOverride_LensFlareThreshold;
	bool bOverride_VignetteIntensity;
	bool bOverride_GrainIntensity;
	bool bOverride_GrainJitter;
	bool bOverride_AmbientOcclusionIntensity;
	bool bOverride_AmbientOcclusionStaticFraction;
	bool bOverride_AmbientOcclusionRadius;
	bool bOverride_AmbientOcclusionFadeDistance;
	bool bOverride_AmbientOcclusionFadeRadius;
	bool bOverride_AmbientOcclusionDistance;
	bool bOverride_AmbientOcclusionRadiusInWS;
	bool bOverride_AmbientOcclusionPower;
	bool bOverride_AmbientOcclusionBias;
	bool bOverride_AmbientOcclusionQuality;
	bool bOverride_AmbientOcclusionMipBlend;
	bool bOverride_AmbientOcclusionMipScale;
	bool bOverride_AmbientOcclusionMipThreshold;
	bool bOverride_AmbientOcclusionTemporalBlendWeight;
	bool bOverride_RayTracingAO;
	bool bOverride_RayTracingAOSamplesPerPixel;
	bool bOverride_RayTracingAOIntensity;
	bool bOverride_RayTracingAORadius;
	bool bOverride_LPVIntensity;
	bool bOverride_LPVDirectionalOcclusionIntensity;
	bool bOverride_LPVDirectionalOcclusionRadius;
	bool bOverride_LPVDiffuseOcclusionExponent;
	bool bOverride_LPVSpecularOcclusionExponent;
	bool bOverride_LPVDiffuseOcclusionIntensity;
	bool bOverride_LPVSpecularOcclusionIntensity;
	bool bOverride_LPVSize;
	bool bOverride_LPVSecondaryOcclusionIntensity;
	bool bOverride_LPVSecondaryBounceIntensity;
	bool bOverride_LPVGeometryVolumeBias;
	bool bOverride_LPVVplInjectionBias;
	bool bOverride_LPVEmissiveInjectionIntensity;
	bool bOverride_LPVFadeRange;
	bool bOverride_LPVDirectionalOcclusionFadeRange;
	bool bOverride_IndirectLightingColor;
	bool bOverride_IndirectLightingIntensity;
	bool bOverride_ColorGradingIntensity;
	bool bOverride_ColorGradingLUT;
	bool bOverride_DepthOfFieldFocalDistance;
	bool bOverride_DepthOfFieldFstop;
	bool bOverride_DepthOfFieldMinFstop;
	bool bOverride_DepthOfFieldBladeCount;
	bool bOverride_DepthOfFieldSensorWidth;
	bool bOverride_DepthOfFieldDepthBlurRadius;
	bool bOverride_DepthOfFieldDepthBlurAmount;
	bool bOverride_DepthOfFieldFocalRegion;
	bool bOverride_DepthOfFieldNearTransitionRegion;
	bool bOverride_DepthOfFieldFarTransitionRegion;
	bool bOverride_DepthOfFieldScale;
	bool bOverride_DepthOfFieldNearBlurSize;
	bool bOverride_DepthOfFieldFarBlurSize;
	bool bOverride_MobileHQGaussian;
	bool bOverride_DepthOfFieldOcclusion;
	bool bOverride_DepthOfFieldSkyFocusDistance;
	bool bOverride_DepthOfFieldVignetteSize;
	bool bOverride_MotionBlurAmount;
	bool bOverride_MotionBlurMax;
	bool bOverride_MotionBlurTargetFPS;
	bool bOverride_MotionBlurPerObjectSize;
	bool bOverride_ScreenPercentage;
	bool bOverride_ScreenSpaceReflectionIntensity;
	bool bOverride_ScreenSpaceReflectionQuality;
	bool bOverride_ScreenSpaceReflectionMaxRoughness;
	bool bOverride_ScreenSpaceReflectionRoughnessScale;
	bool bOverride_ReflectionsType;
	bool bOverride_RayTracingReflectionsMaxRoughness;
	bool bOverride_RayTracingReflectionsMaxBounces;
	bool bOverride_RayTracingReflectionsSamplesPerPixel;
	bool bOverride_RayTracingReflectionsShadows;
	bool bOverride_RayTracingReflectionsTranslucency;
	bool bOverride_TranslucencyType;
	bool bOverride_RayTracingTranslucencyMaxRoughness;
	bool bOverride_RayTracingTranslucencyRefractionRays;
	bool bOverride_RayTracingTranslucencySamplesPerPixel;
	bool bOverride_RayTracingTranslucencyShadows;
	bool bOverride_RayTracingTranslucencyRefraction;
	bool bOverride_RayTracingGI;
	bool bOverride_RayTracingGIMaxBounces;
	bool bOverride_RayTracingGISamplesPerPixel;
	bool bOverride_PathTracingMaxBounces;
	bool bOverride_PathTracingSamplesPerPixel;
	bool bMobileHQGaussian;
	EBloomMethod BloomMethod;
	EAutoExposureMethod AutoExposureMethod;
	float WhiteTemp;
	float WhiteTint;
	struct Vector4 ColorSaturation;
	struct Vector4 ColorContrast;
	struct Vector4 ColorGamma;
	struct Vector4 ColorGain;
	struct Vector4 ColorOffset;
	struct Vector4 ColorSaturationShadows;
	struct Vector4 ColorContrastShadows;
	struct Vector4 ColorGammaShadows;
	struct Vector4 ColorGainShadows;
	struct Vector4 ColorOffsetShadows;
	struct Vector4 ColorSaturationMidtones;
	struct Vector4 ColorContrastMidtones;
	struct Vector4 ColorGammaMidtones;
	struct Vector4 ColorGainMidtones;
	struct Vector4 ColorOffsetMidtones;
	struct Vector4 ColorSaturationHighlights;
	struct Vector4 ColorContrastHighlights;
	struct Vector4 ColorGammaHighlights;
	struct Vector4 ColorGainHighlights;
	struct Vector4 ColorOffsetHighlights;
	float ColorCorrectionHighlightsMin;
	float ColorCorrectionShadowsMax;
	float BlueCorrection;
	float ExpandGamut;
	float ToneCurveAmount;
	float FilmSlope;
	float FilmToe;
	float FilmShoulder;
	float FilmBlackClip;
	float FilmWhiteClip;
	struct LinearColor FilmWhitePoint;
	struct LinearColor FilmShadowTint;
	float FilmShadowTintBlend;
	float FilmShadowTintAmount;
	float FilmSaturation;
	struct LinearColor FilmChannelMixerRed;
	struct LinearColor FilmChannelMixerGreen;
	struct LinearColor FilmChannelMixerBlue;
	float FilmContrast;
	float FilmToeAmount;
	float FilmHealAmount;
	float FilmDynamicRange;
	struct LinearColor SceneColorTint;
	float SceneFringeIntensity;
	float ChromaticAberrationStartOffset;
	float BloomIntensity;
	float BloomThreshold;
	float BloomSizeScale;
	float Bloom1Size;
	float Bloom2Size;
	float Bloom3Size;
	float Bloom4Size;
	float Bloom5Size;
	float Bloom6Size;
	struct LinearColor Bloom1Tint;
	struct LinearColor Bloom2Tint;
	struct LinearColor Bloom3Tint;
	struct LinearColor Bloom4Tint;
	struct LinearColor Bloom5Tint;
	struct LinearColor Bloom6Tint;
	float BloomConvolutionSize;
	class UClass* BloomConvolutionTexture;
	struct Vector2D BloomConvolutionCenterUV;
	float BloomConvolutionPreFilterMin;
	float BloomConvolutionPreFilterMax;
	float BloomConvolutionPreFilterMult;
	float BloomConvolutionBufferScale;
	class UClass* BloomDirtMask;
	float BloomDirtMaskIntensity;
	struct LinearColor BloomDirtMaskTint;
	struct LinearColor AmbientCubemapTint;
	float AmbientCubemapIntensity;
	class UClass* AmbientCubemap;
	float CameraShutterSpeed;
	float CameraISO;
	float DepthOfFieldFstop;
	float DepthOfFieldMinFstop;
	int DepthOfFieldBladeCount;
	float AutoExposureBias;
	float AutoExposureBiasBackup;
	bool bOverride_AutoExposureBiasBackup;
	bool AutoExposureApplyPhysicalCameraExposure;
	class UClass* AutoExposureBiasCurve;
	class UClass* AutoExposureMeterMask;
	float AutoExposureLowPercent;
	float AutoExposureHighPercent;
	float AutoExposureMinBrightness;
	float AutoExposureMaxBrightness;
	float AutoExposureSpeedUp;
	float AutoExposureSpeedDown;
	float HistogramLogMin;
	float HistogramLogMax;
	float AutoExposureCalibrationConstant;
	float LensFlareIntensity;
	struct LinearColor LensFlareTint;
	float LensFlareBokehSize;
	float LensFlareThreshold;
	class UClass* LensFlareBokehShape;
	struct LinearColor LensFlareTints;
	float VignetteIntensity;
	float GrainJitter;
	float GrainIntensity;
	float AmbientOcclusionIntensity;
	float AmbientOcclusionStaticFraction;
	float AmbientOcclusionRadius;
	bool AmbientOcclusionRadiusInWS;
	float AmbientOcclusionFadeDistance;
	float AmbientOcclusionFadeRadius;
	float AmbientOcclusionDistance;
	float AmbientOcclusionPower;
	float AmbientOcclusionBias;
	float AmbientOcclusionQuality;
	float AmbientOcclusionMipBlend;
	float AmbientOcclusionMipScale;
	float AmbientOcclusionMipThreshold;
	float AmbientOcclusionTemporalBlendWeight;
	bool RayTracingAO;
	int RayTracingAOSamplesPerPixel;
	float RayTracingAOIntensity;
	float RayTracingAORadius;
	struct LinearColor IndirectLightingColor;
	float IndirectLightingIntensity;
	ERayTracingGlobalIlluminationType RayTracingGIType;
	int RayTracingGIMaxBounces;
	int RayTracingGISamplesPerPixel;
	float ColorGradingIntensity;
	class UClass* ColorGradingLUT;
	float DepthOfFieldSensorWidth;
	float DepthOfFieldFocalDistance;
	float DepthOfFieldDepthBlurAmount;
	float DepthOfFieldDepthBlurRadius;
	float DepthOfFieldFocalRegion;
	float DepthOfFieldNearTransitionRegion;
	float DepthOfFieldFarTransitionRegion;
	float DepthOfFieldScale;
	float DepthOfFieldNearBlurSize;
	float DepthOfFieldFarBlurSize;
	float DepthOfFieldOcclusion;
	float DepthOfFieldSkyFocusDistance;
	float DepthOfFieldVignetteSize;
	float MotionBlurAmount;
	float MotionBlurMax;
	int MotionBlurTargetFPS;
	float MotionBlurPerObjectSize;
	float LPVIntensity;
	float LPVVplInjectionBias;
	float LPVSize;
	float LPVSecondaryOcclusionIntensity;
	float LPVSecondaryBounceIntensity;
	float LPVGeometryVolumeBias;
	float LPVEmissiveInjectionIntensity;
	float LPVDirectionalOcclusionIntensity;
	float LPVDirectionalOcclusionRadius;
	float LPVDiffuseOcclusionExponent;
	float LPVSpecularOcclusionExponent;
	float LPVDiffuseOcclusionIntensity;
	float LPVSpecularOcclusionIntensity;
	EReflectionsType ReflectionsType;
	float ScreenSpaceReflectionIntensity;
	float ScreenSpaceReflectionQuality;
	float ScreenSpaceReflectionMaxRoughness;
	float RayTracingReflectionsMaxRoughness;
	int RayTracingReflectionsMaxBounces;
	int RayTracingReflectionsSamplesPerPixel;
	EReflectedAndRefractedRayTracedShadows RayTracingReflectionsShadows;
	bool RayTracingReflectionsTranslucency;
	ETranslucencyType TranslucencyType;
	float RayTracingTranslucencyMaxRoughness;
	int RayTracingTranslucencyRefractionRays;
	int RayTracingTranslucencySamplesPerPixel;
	EReflectedAndRefractedRayTracedShadows RayTracingTranslucencyShadows;
	bool RayTracingTranslucencyRefraction;
	int PathTracingMaxBounces;
	int PathTracingSamplesPerPixel;
	float LPVFadeRange;
	float LPVDirectionalOcclusionFadeRange;
	float ScreenPercentage;
	struct WeightedBlendables WeightedBlendables;
};

struct FStreamingLevelsToConsider
{
	TArray<class UClass*> StreamingLevels;
};

struct FLevelCollection
{
	class UClass* GameState;
	class UClass* NetDriver;
	class UClass* DemoNetDriver;
	class UClass* PersistentLevel;
	Unknown Levels;
};

struct FPSCPoolElem
{
	class UClass* PSC;
};

struct FPSCPool
{
	TArray<struct PSCPoolElem> FreeElements;
};

struct FWorldPSCPool
{
	Unknown WorldParticleSystemPools;
};

struct FNavAgentSelector
{
	bool bSupportsAgent0;
	bool bSupportsAgent1;
	bool bSupportsAgent2;
	bool bSupportsAgent3;
	bool bSupportsAgent4;
	bool bSupportsAgent5;
	bool bSupportsAgent6;
	bool bSupportsAgent7;
	bool bSupportsAgent8;
	bool bSupportsAgent9;
	bool bSupportsAgent10;
	bool bSupportsAgent11;
	bool bSupportsAgent12;
	bool bSupportsAgent13;
	bool bSupportsAgent14;
	bool bSupportsAgent15;
};

struct FParticleSystemLOD
{
};

struct FLODSoloTrack
{
	TArray<byte> SoloEnableSetting;
};

struct FNamedEmitterMaterial
{
	struct FName Name;
	class UClass* Material;
};

struct FHitResult
{
	int FaceIndex;
	float Time;
	float Distance;
	struct Vector_NetQuantize Location;
	struct Vector_NetQuantize ImpactPoint;
	struct Vector_NetQuantizeNormal Normal;
	struct Vector_NetQuantizeNormal ImpactNormal;
	struct Vector_NetQuantize TraceStart;
	struct Vector_NetQuantize TraceEnd;
	float PenetrationDepth;
	int Item;
	byte ElementIndex;
	bool bBlockingHit;
	bool bStartPenetrating;
	Unknown PhysMaterial;
	Unknown Actor;
	Unknown Component;
	struct FName BoneName;
	struct FName MyBoneName;
};

struct FAudioComponentParam
{
	struct FName ParamName;
	float FloatParam;
	bool BoolParam;
	int IntParam;
	class UClass* SoundWaveParam;
};

struct FSoundModulationDestinationSettings
{
	float Value;
	class UClass* Modulator;
};

struct FSoundModulationDefaultSettings
{
	struct SoundModulationDestinationSettings VolumeModulationDestination;
	struct SoundModulationDestinationSettings PitchModulationDestination;
	struct SoundModulationDestinationSettings HighpassModulationDestination;
	struct SoundModulationDestinationSettings LowpassModulationDestination;
};

struct FLightmassLightSettings
{
	float IndirectLightingSaturation;
	float ShadowExponent;
	bool bUseAreaShadowsForStationaryLight;
};

struct FCachedKeyToActionInfo
{
	class UClass* PlayerInput;
};

struct FFindFloorResult
{
	bool bBlockingHit;
	bool bWalkableFloor;
	bool bLineTrace;
	float FloorDist;
	float LineDist;
	struct HitResult HitResult;
};

struct FNavAvoidanceMask
{
	bool bGroup0;
	bool bGroup1;
	bool bGroup2;
	bool bGroup3;
	bool bGroup4;
	bool bGroup5;
	bool bGroup6;
	bool bGroup7;
	bool bGroup8;
	bool bGroup9;
	bool bGroup10;
	bool bGroup11;
	bool bGroup12;
	bool bGroup13;
	bool bGroup14;
	bool bGroup15;
	bool bGroup16;
	bool bGroup17;
	bool bGroup18;
	bool bGroup19;
	bool bGroup20;
	bool bGroup21;
	bool bGroup22;
	bool bGroup23;
	bool bGroup24;
	bool bGroup25;
	bool bGroup26;
	bool bGroup27;
	bool bGroup28;
	bool bGroup29;
	bool bGroup30;
	bool bGroup31;
};

struct FNavigationLinkBase
{
	float LeftProjectHeight;
	float MaxFallDownLength;
	float SnapRadius;
	float SnapHeight;
	struct NavAgentSelector SupportedAgents;
	bool bSupportsAgent0;
	bool bSupportsAgent1;
	bool bSupportsAgent2;
	bool bSupportsAgent3;
	bool bSupportsAgent4;
	bool bSupportsAgent5;
	bool bSupportsAgent6;
	bool bSupportsAgent7;
	bool bSupportsAgent8;
	bool bSupportsAgent9;
	bool bSupportsAgent10;
	bool bSupportsAgent11;
	bool bSupportsAgent12;
	bool bSupportsAgent13;
	bool bSupportsAgent14;
	bool bSupportsAgent15;
	ENavLinkDirection Direction;
	bool bUseSnapHeight;
	bool bSnapToCheapestArea;
	bool bCustomFlag0;
	bool bCustomFlag1;
	bool bCustomFlag2;
	bool bCustomFlag3;
	bool bCustomFlag4;
	bool bCustomFlag5;
	bool bCustomFlag6;
	bool bCustomFlag7;
	class UClass* AreaClass;
};

struct FMinimalViewInfo
{
	struct Vector Location;
	struct Rotator Rotation;
	float FOV;
	float DesiredFOV;
	float OrthoWidth;
	float OrthoNearClipPlane;
	float OrthoFarClipPlane;
	float AspectRatio;
	bool bConstrainAspectRatio;
	bool bUseFieldOfViewForLOD;
	ECameraProjectionMode ProjectionMode;
	float PostProcessBlendWeight;
	struct PostProcessSettings PostProcessSettings;
	struct Vector2D OffCenterProjectionOffset;
};

struct FCameraCacheEntry
{
	float Timestamp;
	struct MinimalViewInfo POV;
};

struct FTViewTarget
{
	class UClass* Target;
	struct MinimalViewInfo POV;
	class UClass* PlayerState;
};

struct FKeyBind
{
	struct Key Key;
	struct FString Command;
	bool Control;
	bool Shift;
	bool Alt;
	bool Cmd;
	bool bIgnoreCtrl;
	bool bIgnoreShift;
	bool bIgnoreAlt;
	bool bIgnoreCmd;
	bool bDisabled;
};

struct FReverbSettings
{
	bool bApplyReverb;
	class UClass* ReverbEffect;
	class UClass* ReverbPluginEffect;
	float Volume;
	float FadeTime;
};

struct FInteriorSettings
{
	bool bIsWorldSettings;
	float ExteriorVolume;
	float ExteriorTime;
	float ExteriorLPF;
	float ExteriorLPFTime;
	float InteriorVolume;
	float InteriorTime;
	float InteriorLPF;
	float InteriorLPFTime;
};

struct FBroadphaseSettings
{
	bool bUseMBPOnClient;
	bool bUseMBPOnServer;
	bool bUseMBPOuterBounds;
	struct Box MBPBounds;
	struct Box MBPOuterBounds;
	uint32_t MBPNumSubdivs;
};

struct FNetViewer
{
	class UClass* Connection;
	class UClass* InViewer;
	class UClass* ViewTarget;
	struct Vector ViewLocation;
	struct Vector ViewDir;
};

struct FInstancedStaticMeshInstanceData
{
	struct Matrix Transform;
};

struct FInstancedStaticMeshMappingInfo
{
};

struct FLightmassMaterialInterfaceSettings
{
	float EmissiveBoost;
	float DiffuseBoost;
	float ExportResolutionScale;
	bool bCastShadowAsMasked;
	bool bOverrideCastShadowAsMasked;
	bool bOverrideEmissiveBoost;
	bool bOverrideDiffuseBoost;
	bool bOverrideExportResolutionScale;
};

struct FMaterialTextureInfo
{
	float SamplingScale;
	int UVChannelIndex;
	struct FName TextureName;
};

struct FMaterialParameterInfo
{
	struct FName Name;
	EMaterialParameterAssociation Association;
	int Index;
};

struct FScalarParameterValue
{
	struct MaterialParameterInfo ParameterInfo;
	float ParameterValue;
	struct Guid ExpressionGUID;
};

struct FVectorParameterValue
{
	struct MaterialParameterInfo ParameterInfo;
	struct LinearColor ParameterValue;
	struct Guid ExpressionGUID;
};

struct FTextureParameterValue
{
	struct MaterialParameterInfo ParameterInfo;
	class UClass* ParameterValue;
	struct Guid ExpressionGUID;
};

struct FRuntimeVirtualTextureParameterValue
{
	struct MaterialParameterInfo ParameterInfo;
	class UClass* ParameterValue;
	struct Guid ExpressionGUID;
};

struct FFontParameterValue
{
	struct MaterialParameterInfo ParameterInfo;
	class UClass* FontValue;
	int FontPage;
	struct Guid ExpressionGUID;
};

struct FMaterialInstanceBasePropertyOverrides
{
	bool bOverride_OpacityMaskClipValue;
	bool bOverride_BlendMode;
	bool bOverride_ShadingModel;
	bool bOverride_DitheredLODTransition;
	bool bOverride_CastDynamicShadowAsMasked;
	bool bOverride_TwoSided;
	bool TwoSided;
	bool DitheredLODTransition;
	bool bCastDynamicShadowAsMasked;
	EBlendMode BlendMode;
	EMaterialShadingModel ShadingModel;
	float OpacityMaskClipValue;
};

struct FStaticParameterBase
{
	struct MaterialParameterInfo ParameterInfo;
	bool bOverride;
	struct Guid ExpressionGUID;
};

struct FMaterialLayersFunctions
{
	TArray<class UClass*> Layers;
	TArray<class UClass*> Blends;
	TArray<bool> LayerStates;
	struct FString KeyString;
};

struct FStaticParameterSet
{
	TArray<struct StaticSwitchParameter> StaticSwitchParameters;
	TArray<struct StaticComponentMaskParameter> StaticComponentMaskParameters;
	TArray<struct StaticTerrainLayerWeightParameter> TerrainLayerWeightParameters;
	TArray<struct StaticMaterialLayersParameter> MaterialLayersParameters;
};

struct FMaterialCachedParameterEntry
{
	TArray<uint64_t> NameHashes;
	TArray<struct MaterialParameterInfo> ParameterInfos;
	TArray<struct Guid> ExpressionGuids;
	TArray<bool> Overrides;
};

struct FMaterialCachedParameters
{
	struct MaterialCachedParameterEntry RuntimeEntries;
	TArray<float> ScalarValues;
	TArray<struct LinearColor> VectorValues;
	TArray<class UClass*> TextureValues;
	TArray<class UClass*> FontValues;
	TArray<int> FontPageValues;
	TArray<class UClass*> RuntimeVirtualTextureValues;
};

struct FSoundWaveSpectralDataEntry
{
	float Magnitude;
	float NormalizedMagnitude;
};

struct FSoundWaveSpectralTimeData
{
	TArray<struct SoundWaveSpectralDataEntry> Data;
	float TimeSec;
};

struct FSoundWaveEnvelopeTimeData
{
	float Amplitude;
	float TimeSec;
};

struct FSubtitleCue
{
	struct FText Text;
	float Time;
};

struct FInterpolationParameter
{
	float InterpolationTime;
	EFilterInterpolationType InterpolationType;
};

struct FBoneReference
{
	struct FName BoneName;
};

struct FPerBoneInterpolation
{
	struct BoneReference BoneReference;
	float InterpolationSpeedPerSec;
};

struct FBlendSample
{
	class UClass* Animation;
	struct Vector SampleValue;
	float RateScale;
};

struct FEditorElement
{
	int Indices;
	float Weights;
};

struct FBlendParameter
{
	struct FString DisplayName;
	float Min;
	float Max;
	int GridNum;
};

struct FCustomAttributeSetting
{
	struct FString Name;
	struct FString Meaning;
};

struct FAnimGroupInfo
{
	struct FName Name;
	struct LinearColor Color;
};

struct FBakedStateExitTransition
{
	int CanTakeDelegateIndex;
	int CustomResultNodeIndex;
	int TransitionIndex;
	bool bDesiredTransitionReturnValue;
	bool bAutomaticRemainingTimeRule;
	TArray<int> PoseEvaluatorLinks;
};

struct FBakedAnimationState
{
	struct FName StateName;
	TArray<struct BakedStateExitTransition> Transitions;
	int StateRootNodeIndex;
	int StartNotify;
	int EndNotify;
	int FullyBlendedNotify;
	bool bIsAConduit;
	int EntryRuleNodeIndex;
	TArray<int> PlayerNodeIndices;
	TArray<int> LayerNodeIndices;
	bool bAlwaysResetOnEntry;
};

struct FAnimationStateBase
{
	struct FName StateName;
};

struct FBakedAnimationStateMachine
{
	struct FName MachineName;
	int InitialState;
	TArray<struct BakedAnimationState> States;
	TArray<struct AnimationTransitionBetweenStates> Transitions;
};

struct FCachedPoseIndices
{
	TArray<int> OrderedSavedPoseNodeIndices;
};

struct FExposedValueCopyRecord
{
	int CopyIndex;
	EPostCopyOperation PostCopyOperation;
};

struct FExposedValueHandler
{
	struct FName BoundFunction;
	TArray<struct ExposedValueCopyRecord> CopyRecords;
	class UClass* Function;
	Unknown ValueHandlerNodeProperty;
};

struct FGraphAssetPlayerInformation
{
	TArray<int> PlayerNodeIndices;
};

struct FAnimGraphBlendOptions
{
	float BlendInTime;
	float BlendOutTime;
};

struct FAnimBlueprintFunction
{
	struct FName Name;
	struct FName Group;
	int OutputPoseNodeIndex;
	TArray<struct FName> InputPoseNames;
	TArray<int> InputPoseNodeIndices;
	bool bImplemented;
};

struct FAnimBlueprintFunctionData
{
	Unknown OutputPoseNodeProperty;
	TArray<Unknown> InputPoseNodeProperties;
	TArray<Unknown> InputProperties;
};

struct FSmartName
{
	struct FName DisplayName;
};

struct FAnimCurveBase
{
	struct FName LastObservedName;
	struct SmartName Name;
	int CurveTypeFlags;
};

struct FRawCurveTracks
{
	TArray<struct FloatCurve> FloatCurves;
};

struct FAnimSegment
{
	class UClass* AnimReference;
	float StartPos;
	float AnimStartTime;
	float AnimEndTime;
	float AnimPlayRate;
	int LoopingCount;
};

struct FAnimTrack
{
	TArray<struct AnimSegment> AnimSegments;
};

struct FAlphaBlend
{
	class UClass* CustomCurve;
	float BlendTime;
	EAlphaBlendOption BlendOption;
};

struct FAnimSyncMarker
{
	struct FName MarkerName;
	float Time;
};

struct FMarkerSyncData
{
	TArray<struct AnimSyncMarker> AuthoredSyncMarkers;
};

struct FSlotAnimationTrack
{
	struct FName SlotName;
	struct AnimTrack AnimTrack;
};

struct FBranchingPointMarker
{
	int NotifyIndex;
	float TriggerTime;
	EAnimNotifyEventType NotifyEventType;
};

struct FTimeStretchCurveMarker
{
	float Time;
	float Alpha;
};

struct FTimeStretchCurve
{
	float SamplingRate;
	float CurveValueMinPrecision;
	TArray<struct TimeStretchCurveMarker> Markers;
	float Sum_dT_i_by_C_i;
};

struct FTrackToSkeletonMap
{
	int BoneTreeIndex;
};

struct FStringCurveKey
{
	float Time;
	struct FString Value;
};

struct FBakedStringCustomAttribute
{
	struct FName AttributeName;
	struct StringCurve StringCurve;
};

struct FIntegralKey
{
	float Time;
	int Value;
};

struct FBakedIntegerCustomAttribute
{
	struct FName AttributeName;
	struct IntegralCurve IntCurve;
};

struct FSimpleCurveKey
{
	float Time;
	float Value;
};

struct FBakedFloatCustomAttribute
{
	struct FName AttributeName;
	struct SimpleCurve FloatCurve;
};

struct FBakedCustomAttributePerBoneData
{
	int BoneTreeIndex;
	TArray<struct BakedStringCustomAttribute> StringAttributes;
	TArray<struct BakedIntegerCustomAttribute> IntAttributes;
	TArray<struct BakedFloatCustomAttribute> FloatAttributes;
};

struct FAnimSetMeshLinkup
{
	TArray<int> BoneToTrackTable;
};

struct FPrimaryAssetRules
{
	int Priority;
	int ChunkId;
	bool bApplyRecursively;
	EPrimaryAssetCookRule CookRule;
};

struct FPrimaryAssetTypeInfo
{
	struct FName PrimaryAssetType;
	struct TSoftClassPtr<UObject> AssetBaseClass;
	class UClass* AssetBaseClassLoaded;
	bool bHasBlueprintClasses;
	bool bIsEditorOnly;
	TArray<struct DirectoryPath> Directories;
	TArray<struct SoftObjectPath> SpecificAssets;
	struct PrimaryAssetRules Rules;
	TArray<struct FString> AssetScanPaths;
	bool bIsDynamicAsset;
	int NumberOfAssets;
};

struct FPrimaryAssetRulesOverride
{
	struct PrimaryAssetId PrimaryAssetId;
	struct PrimaryAssetRules Rules;
};

struct FPrimaryAssetRulesCustomOverride
{
	struct PrimaryAssetType PrimaryAssetType;
	struct DirectoryPath FilterDirectory;
	struct FString FilterString;
	struct PrimaryAssetRules Rules;
};

struct FAssetManagerRedirect
{
	struct FString Old;
	struct FString New;
};

struct FAssetMapping
{
	class UClass* SourceAsset;
	class UClass* TargetAsset;
};

struct FAtmospherePrecomputeParameters
{
	float DensityHeight;
	float DecayHeight;
	int MaxScatteringOrder;
	int TransmittanceTexWidth;
	int TransmittanceTexHeight;
	int IrradianceTexWidth;
	int IrradianceTexHeight;
	int InscatterAltitudeSampleNum;
	int InscatterMuNum;
	int InscatterMuSNum;
	int InscatterNuNum;
};

struct FAudioQualitySettings
{
	struct FText DisplayName;
	int MaxChannels;
};

struct FSoundDebugEntry
{
	struct FName DebugName;
	struct SoftObjectPath Sound;
};

struct FDefaultAudioBusSettings
{
	struct SoftObjectPath AudioBus;
};

struct FAudioVolumeSubmixSendSettings
{
	EAudioVolumeLocationState ListenerLocationState;
	EAudioVolumeLocationState SourceLocationState;
	TArray<struct SoundSubmixSendInfo> SubmixSends;
};

struct FAudioVolumeSubmixOverrideSettings
{
	class UClass* Submix;
	TArray<class UClass*> SubmixEffectChain;
	float CrossfadeTime;
};

struct FEditorMapPerformanceTestDefinition
{
	struct SoftObjectPath PerformanceTestmap;
	int TestTimer;
};

struct FFilePath
{
	struct FString FilePath;
};

struct FImportFactorySettingValues
{
	struct FString SettingName;
	struct FString Value;
};

struct FEditorImportWorkflowDefinition
{
	struct FilePath ImportFilePath;
	TArray<struct ImportFactorySettingValues> FactorySettings;
};

struct FBuildPromotionImportWorkflowSettings
{
	struct EditorImportWorkflowDefinition Diffuse;
	struct EditorImportWorkflowDefinition Normal;
	struct EditorImportWorkflowDefinition StaticMesh;
	struct EditorImportWorkflowDefinition ReimportStaticMesh;
	struct EditorImportWorkflowDefinition BlendShapeMesh;
	struct EditorImportWorkflowDefinition MorphMesh;
	struct EditorImportWorkflowDefinition SkeletalMesh;
	struct EditorImportWorkflowDefinition Animation;
	struct EditorImportWorkflowDefinition Sound;
	struct EditorImportWorkflowDefinition SurroundSound;
	TArray<struct EditorImportWorkflowDefinition> OtherAssetsToImport;
};

struct FBuildPromotionOpenAssetSettings
{
	struct FilePath BlueprintAsset;
	struct FilePath MaterialAsset;
	struct FilePath ParticleSystemAsset;
	struct FilePath SkeletalMeshAsset;
	struct FilePath StaticMeshAsset;
	struct FilePath TextureAsset;
};

struct FBuildPromotionNewProjectSettings
{
	struct DirectoryPath NewProjectFolderOverride;
	struct FString NewProjectNameOverride;
};

struct FBuildPromotionTestSettings
{
	struct FilePath DefaultStaticMeshAsset;
	struct BuildPromotionImportWorkflowSettings ImportWorkflow;
	struct BuildPromotionOpenAssetSettings OpenAssets;
	struct BuildPromotionNewProjectSettings NewProjectSettings;
	struct FilePath SourceControlMaterial;
};

struct FMaterialEditorPromotionSettings
{
	struct FilePath DefaultMaterialAsset;
	struct FilePath DefaultDiffuseTexture;
	struct FilePath DefaultNormalTexture;
};

struct FParticleEditorPromotionSettings
{
	struct FilePath DefaultParticleAsset;
};

struct FBlueprintEditorPromotionSettings
{
	struct FilePath FirstMeshPath;
	struct FilePath SecondMeshPath;
	struct FilePath DefaultParticleAsset;
};

struct FExternalToolDefinition
{
	struct FString ToolName;
	struct FilePath ExecutablePath;
	struct FString CommandLineOptions;
	struct DirectoryPath WorkingDirectory;
	struct FString ScriptExtension;
	struct DirectoryPath ScriptDirectory;
};

struct FEditorImportExportTestDefinition
{
	struct FilePath ImportFilePath;
	struct FString ExportFileExtension;
	bool bSkipExport;
	TArray<struct ImportFactorySettingValues> FactorySettings;
};

struct FLaunchOnTestSettings
{
	struct FilePath LaunchOnTestmap;
	struct FString DeviceID;
};

struct FBandwidthTestItem
{
	TArray<byte> Kilobyte;
};

struct FBandwidthTestGenerator
{
	TArray<struct BandwidthTestItem> ReplicatedBuffers;
};

struct FBoneNode
{
	struct FName Name;
	int ParentIndex;
	EBoneTranslationRetargetingMode TranslationRetargetingMode;
};

struct FVirtualBone
{
	struct FName SourceBoneName;
	struct FName TargetBoneName;
	struct FName VirtualBoneName;
};

struct FSmartNameContainer
{
};

struct FAnimSlotGroup
{
	struct FName GroupName;
	TArray<struct FName> SlotNames;
};

struct FBlendProfileBoneEntry
{
	struct BoneReference BoneReference;
	float BlendScale;
};

struct FKShapeElem
{
	float RestOffset;
	struct FName Name;
	bool bContributeToMass;
	ECollisionEnabled CollisionEnabled;
};

struct FKAggregateGeom
{
	TArray<struct KSphereElem> SphereElems;
	TArray<struct KBoxElem> BoxElems;
	TArray<struct KSphylElem> SphylElems;
	TArray<struct KConvexElem> ConvexElems;
	TArray<struct KTaperedCapsuleElem> TaperedCapsuleElems;
};

struct FBranchFilter
{
	struct FName BoneName;
	int BlendDepth;
};

struct FInputBlendPose
{
	TArray<struct BranchFilter> BranchFilters;
};

struct FBuilderPoly
{
	TArray<int> VertexIndices;
	int Direction;
	struct FName ItemName;
	int PolyFlags;
};

struct FActiveCameraShakeInfo
{
	class UClass* ShakeInstance;
	Unknown ShakeSource;
	bool bIsCustomInitialized;
};

struct FPooledCameraShakes
{
	TArray<class UClass*> PooledShakes;
};

struct FDelegateArray
{
	TArray<struct FDelegate> Delegates;
};

struct FCollisionResponseTemplate
{
	struct FName Name;
	ECollisionEnabled CollisionEnabled;
	bool bCanModify;
	struct FName ObjectTypeName;
	TArray<struct ResponseChannel> CustomResponses;
};

struct FCustomChannelSetup
{
	ECollisionChannel Channel;
	ECollisionResponse DefaultResponse;
	bool bTraceType;
	bool bStaticObject;
	struct FName Name;
};

struct FCustomProfile
{
	struct FName Name;
	TArray<struct ResponseChannel> CustomResponses;
};

struct FRedirector
{
	struct FName OldName;
	struct FName NewName;
};

struct FBlueprintComponentDelegateBinding
{
	struct FName ComponentPropertyName;
	struct FName DelegatePropertyName;
	struct FName FunctionNameToBind;
};

struct FCullDistanceSizePair
{
	float Size;
	float CullDistance;
};

struct FDataDrivenConsoleVariable
{
	EFDataDrivenCVarType Type;
	struct FString Name;
	struct FString Tooltip;
	float DefaultValueFloat;
	int DefaultValueInt;
	bool DefaultValueBool;
};

struct FDebugCameraControllerSettingsViewModeIndex
{
	EViewModeIndex ViewModeIndex;
};

struct FRollbackNetStartupActorInfo
{
	class UClass* Archetype;
	class UClass* Level;
	TArray<class UClass*> ObjReferences;
};

struct FMulticastRecordOptions
{
	struct FString FuncPathName;
	bool bServerSkip;
	bool bClientSkip;
};

struct FTextureLODGroup
{
	ETextureGroup Group;
	int LODBias;
	int LODBias_Smaller;
	int LODBias_Smallest;
	int NumStreamedMips;
	ETextureMipGenSettings MipGenSettings;
	int MinLODSize;
	int MaxLODSize;
	int MaxLODSize_Smaller;
	int MaxLODSize_Smallest;
	int OptionalLODBias;
	int OptionalMaxLODSize;
	struct FName MinMagFilter;
	struct FName MipFilter;
	ETextureMipLoadOptions MipLoadOptions;
	bool HighPriorityLoad;
	bool DuplicateNonOptionalMips;
	float Downscale;
	ETextureDownscaleOptions DownscaleOptions;
	int VirtualTextureTileCountBias;
	int VirtualTextureTileSizeBias;
};

struct FDialogueContext
{
	class UClass* Speaker;
	TArray<class UClass*> Targets;
};

struct FDialogueContextMapping
{
	struct DialogueContext Context;
	class UClass* SoundWave;
	struct FString LocalizationKeyFormat;
	class UClass* Proxy;
};

struct FEdGraphTerminalType
{
	struct FName TerminalCategory;
	struct FName TerminalSubCategory;
	Unknown TerminalSubCategoryObject;
	bool bTerminalIsConst;
	bool bTerminalIsWeakPointer;
	bool bTerminalIsUObjectWrapper;
};

struct FEdGraphPinType
{
	struct FName PinCategory;
	struct FName PinSubCategory;
	Unknown PinSubCategoryObject;
	struct SimpleMemberReference PinSubCategoryMemberReference;
	struct EdGraphTerminalType PinValueType;
	EPinContainerType ContainerType;
	bool bIsArray;
	bool bIsReference;
	bool bIsConst;
	bool bIsWeakPointer;
	bool bIsUObjectWrapper;
};

struct FExponentialHeightFogData
{
	float FogDensity;
	float FogHeightFalloff;
	float FogHeightOffset;
};

struct FFontCharacter
{
	int StartU;
	int StartV;
	int USize;
	int VSize;
	byte TextureIndex;
	int VerticalOffset;
};

struct FFontImportOptionsData
{
	struct FString FontName;
	float Height;
	bool bEnableAntialiasing;
	bool bEnableBold;
	bool bEnableItalic;
	bool bEnableUnderline;
	bool bAlphaOnly;
	EFontImportCharacterSet CharacterSet;
	struct FString Chars;
	struct FString UnicodeRange;
	struct FString CharsFilePath;
	struct FString CharsFileWildcard;
	bool bCreatePrintableOnly;
	bool bIncludeASCIIRange;
	struct LinearColor ForegroundColor;
	bool bEnableDropShadow;
	int TexturePageWidth;
	int TexturePageMaxHeight;
	int XPadding;
	int YPadding;
	int ExtendBoxTop;
	int ExtendBoxBottom;
	int ExtendBoxRight;
	int ExtendBoxLeft;
	bool bEnableLegacyMode;
	int Kerning;
	bool bUseDistanceFieldAlpha;
	int DistanceFieldScaleFactor;
	float DistanceFieldScanRadiusScale;
};

struct FForceFeedbackChannelDetails
{
	bool bAffectsLeftLarge;
	bool bAffectsLeftSmall;
	bool bAffectsRightLarge;
	bool bAffectsRightSmall;
	struct RuntimeFloatCurve Curve;
};

struct FHapticFeedbackDetails_Curve
{
	struct RuntimeFloatCurve Frequency;
	struct RuntimeFloatCurve Amplitude;
};

struct FHLODProxyMesh
{
	Unknown LODActor;
	class UClass* StaticMesh;
	struct FName Key;
};

struct FComponentKey
{
	class UClass* OwnerClass;
	struct FName SCSVariableName;
	struct Guid AssociatedGuid;
};

struct FComponentOverrideRecord
{
	class UClass* ComponentClass;
	class UClass* ComponentTemplate;
	struct ComponentKey ComponentKey;
	struct BlueprintCookedComponentInstancingData CookedComponentInstancingData;
};

struct FBlueprintInputDelegateBinding
{
	bool bConsumeInput;
	bool bExecuteWhenPaused;
	bool bOverrideParentBinding;
};

struct FInputAxisProperties
{
	float DeadZone;
	float Sensitivity;
	float Exponent;
	bool bInvert;
};

struct FInputAxisConfigEntry
{
	struct FName AxisKeyName;
	struct InputAxisProperties AxisProperties;
};

struct FInputActionKeyMapping
{
	struct FName ActionName;
	bool bShift;
	bool bCtrl;
	bool bAlt;
	bool bCmd;
	struct Key Key;
};

struct FInputAxisKeyMapping
{
	struct FName AxisName;
	float Scale;
	struct Key Key;
};

struct FInputActionSpeechMapping
{
	struct FName ActionName;
	struct FName SpeechKeyword;
};

struct FCurveEdEntry
{
	class UClass* CurveObject;
	struct Color CurveColor;
	struct FString CurveName;
	int bHideCurve;
	int bColorCurve;
	int bFloatingPointColorCurve;
	int bClamp;
	float ClampLow;
	float ClampHigh;
};

struct FCurveEdTab
{
	struct FString TabName;
	TArray<struct CurveEdEntry> Curves;
	float ViewStartInput;
	float ViewEndInput;
	float ViewStartOutput;
	float ViewEndOutput;
};

struct FInterpControlPoint
{
	struct Vector PositionControlPoint;
	bool bPositionIsRelative;
};

struct FAnimControlTrackKey
{
	float StartTime;
	class UClass* AnimSeq;
	float AnimStartOffset;
	float AnimEndOffset;
	float AnimPlayRate;
	bool bLooping;
	bool bReverse;
};

struct FBoolTrackKey
{
	float Time;
	bool Value;
};

struct FDirectorTrackCut
{
	float Time;
	float TransitionTime;
	struct FName TargetCamGroup;
	int ShotNumber;
};

struct FEventTrackKey
{
	float Time;
	struct FName EventName;
};

struct FPrimitiveMaterialRef
{
	class UClass* Primitive;
	class UClass* Decal;
	int ElementIndex;
};

struct FInterpLookupPoint
{
	struct FName GroupName;
	float Time;
};

struct FInterpLookupTrack
{
	TArray<struct InterpLookupPoint> Points;
};

struct FParticleReplayTrackKey
{
	float Time;
	float Duration;
	int ClipIDNumber;
};

struct FSoundTrackKey
{
	float Time;
	float Volume;
	float Pitch;
	class UClass* Sound;
};

struct FToggleTrackKey
{
	float Time;
	ETrackToggleAction ToggleAction;
};

struct FVisibilityTrackKey
{
	float Time;
	EVisibilityTrackAction Action;
	EVisibilityTrackCondition ActiveCondition;
};

struct FLayerActorStats
{
	class UClass* Type;
	int Total;
};

struct FReplicatedStaticActorDestructionInfo
{
	class UClass* ObjClass;
};

struct FHLODInstancingKey
{
	class UClass* StaticMesh;
	class UClass* Material;
};

struct FComponentSync
{
	struct FName Name;
	ESyncOption SyncOption;
};

struct FLODMappingData
{
	TArray<int> Mapping;
	TArray<int> InverseMapping;
};

struct FMaterialInput
{
	int OutputIndex;
	struct FName ExpressionName;
};

struct FMaterialShadingModelField
{
	uint16_t ShadingModelField;
};

struct FMaterialFunctionInfo
{
	struct Guid StateId;
	class UClass* Function;
};

struct FMaterialParameterCollectionInfo
{
	struct Guid StateId;
	class UClass* ParameterCollection;
};

struct FMaterialCachedExpressionData
{
	struct MaterialCachedParameters Parameters;
	TArray<class UClass*> ReferencedTextures;
	TArray<struct MaterialFunctionInfo> FunctionInfos;
	TArray<struct MaterialParameterCollectionInfo> ParameterCollectionInfos;
	TArray<class UClass*> DefaultLayers;
	TArray<class UClass*> DefaultLayerBlends;
	TArray<class UClass*> GrassTypes;
	TArray<struct FName> DynamicParameterNames;
	TArray<bool> QualityLevelsUsed;
	bool bHasRuntimeVirtualTextureOutput;
	bool bHasSceneColor;
	uint32_t MaterialAttributesPropertyConnectedBitmask;
};

struct FMaterialSpriteElement
{
	class UClass* Material;
	class UClass* DistanceToOpacityCurve;
	bool bSizeIsInScreenSpace;
	float BaseSizeX;
	float BaseSizeY;
	class UClass* DistanceToSizeCurve;
};

struct FCustomInput
{
	struct FName InputName;
	struct ExpressionInput Input;
};

struct FCustomOutput
{
	struct FName OutputName;
	ECustomMaterialOutputType OutputType;
};

struct FCustomDefine
{
	struct FString DefineName;
	struct FString DefineValue;
};

struct FCollectionParameterBase
{
	struct FName ParameterName;
	struct Guid ID;
};

struct FInterpGroupActorInfo
{
	struct FName ObjectName;
	TArray<class UClass*> Actors;
};

struct FCameraCutInfo
{
	struct Vector Location;
	float Timestamp;
};

struct FPurchaseInfo
{
	struct FString Identifier;
	struct FString DisplayName;
	struct FString DisplayDescription;
	struct FString DisplayPrice;
};

struct FNetworkEmulationProfileDescription
{
	struct FString ProfileName;
	struct FString Tooltip;
};

struct FNodeItem
{
	struct FName ParentName;
	struct Transform Transform;
};

struct FDistributionLookupTable
{
	float TimeScale;
	float TimeBias;
	TArray<float> Values;
	byte Op;
	byte EntryCount;
	byte EntryStride;
	byte SubEntryStride;
	byte LockFlag;
};

struct FRawDistribution
{
	struct DistributionLookupTable Table;
};

struct FBeamModifierOptions
{
	bool bModify;
	bool bScale;
	bool bLock;
};

struct FParticleRandomSeedInfo
{
	struct FName ParameterName;
	bool bGetSeedFromInstance;
	bool bInstanceSeedIsIndex;
	bool bResetSeedOnEmitterLooping;
	bool bRandomlySelectSeedArray;
	TArray<int> RandomSeeds;
};

struct FParticleEvent_GenerateInfo
{
	EParticleEventType Type;
	int Frequency;
	int ParticleFrequency;
	bool FirstTimeOnly;
	bool LastTimeOnly;
	bool UseReflectedImpactVector;
	bool bUseOrbitOffset;
	struct FName CustomName;
	TArray<class UClass*> ParticleModuleEventsToSendToGame;
};

struct FLocationBoneSocketInfo
{
	struct FName BoneSocketName;
	struct Vector Offset;
};

struct FOrbitOptions
{
	bool bProcessDuringSpawn;
	bool bProcessDuringUpdate;
	bool bUseEmitterTime;
};

struct FEmitterDynamicParameter
{
	struct FName ParamName;
	bool bUseEmitterTime;
	bool bSpawnTimeOnly;
	EEmitterDynamicParameterValue ValueMethod;
	bool bScaleVelocityByParamValue;
	struct RawDistributionFloat ParamValue;
};

struct FParticleBurst
{
	int Count;
	int CountLow;
	float Time;
};

struct FGPUSpriteLocalVectorFieldInfo
{
	class UClass* Field;
	struct Transform Transform;
	struct Rotator MinInitialRotation;
	struct Rotator MaxInitialRotation;
	struct Rotator RotationRate;
	float Intensity;
	float Tightness;
	bool bIgnoreComponentTransform;
	bool bTileX;
	bool bTileY;
	bool bTileZ;
	bool bUseFixDT;
};

struct FFloatDistribution
{
	struct DistributionLookupTable Table;
};

struct FGPUSpriteEmitterInfo
{
	class UClass* RequiredModule;
	class UClass* SpawnModule;
	class UClass* SpawnPerUnitModule;
	TArray<class UClass*> SpawnModules;
	struct GPUSpriteLocalVectorFieldInfo LocalVectorField;
	struct FloatDistribution VectorFieldScale;
	struct FloatDistribution DragCoefficient;
	struct FloatDistribution PointAttractorStrength;
	struct FloatDistribution Resilience;
	struct Vector ConstantAcceleration;
	struct Vector PointAttractorPosition;
	float PointAttractorRadiusSq;
	struct Vector OrbitOffsetBase;
	struct Vector OrbitOffsetRange;
	struct Vector2D InvMaxSize;
	float InvRotationRateScale;
	float MaxLifetime;
	int MaxParticleCount;
	EParticleScreenAlignment ScreenAlignment;
	EParticleAxisLock LockAxisFlag;
	bool bEnableCollision;
	EParticleCollisionMode CollisionMode;
	bool bRemoveHMDRoll;
	float MinFacingCameraBlendDistance;
	float MaxFacingCameraBlendDistance;
	struct RawDistributionVector DynamicColor;
	struct RawDistributionFloat DynamicAlpha;
	struct RawDistributionVector DynamicColorScale;
	struct RawDistributionFloat DynamicAlphaScale;
};

struct FGPUSpriteResourceData
{
	TArray<struct Color> QuantizedColorSamples;
	TArray<struct Color> QuantizedMiscSamples;
	TArray<struct Color> QuantizedSimulationAttrSamples;
	struct Vector4 ColorScale;
	struct Vector4 ColorBias;
	struct Vector4 MiscScale;
	struct Vector4 MiscBias;
	struct Vector4 SimulationAttrCurveScale;
	struct Vector4 SimulationAttrCurveBias;
	struct Vector4 SubImageSize;
	struct Vector4 SizeBySpeed;
	struct Vector ConstantAcceleration;
	struct Vector OrbitOffsetBase;
	struct Vector OrbitOffsetRange;
	struct Vector OrbitFrequencyBase;
	struct Vector OrbitFrequencyRange;
	struct Vector OrbitPhaseBase;
	struct Vector OrbitPhaseRange;
	float GlobalVectorFieldScale;
	float GlobalVectorFieldTightness;
	float PerParticleVectorFieldScale;
	float PerParticleVectorFieldBias;
	float DragCoefficientScale;
	float DragCoefficientBias;
	float ResilienceScale;
	float ResilienceBias;
	float CollisionRadiusScale;
	float CollisionRadiusBias;
	float CollisionTimeBias;
	float CollisionRandomSpread;
	float CollisionRandomDistribution;
	float OneMinusFriction;
	float RotationRateScale;
	float CameraMotionBlurAmount;
	EParticleScreenAlignment ScreenAlignment;
	EParticleAxisLock LockAxisFlag;
	struct Vector2D PivotOffset;
	bool bRemoveHMDRoll;
	float MinFacingCameraBlendDistance;
	float MaxFacingCameraBlendDistance;
};

struct FParticleSysParam
{
	struct FName Name;
	EParticleSysParamType ParamType;
	float Scalar;
	float Scalar_Low;
	struct Vector Vector;
	struct Vector Vector_Low;
	struct Color Color;
	class UClass* Actor;
	class UClass* Material;
};

struct FSolverIterations
{
	float FixedTimeStep;
	int SolverIterations;
	int JointIterations;
	int CollisionIterations;
	int SolverPushOutIterations;
	int JointPushOutIterations;
	int CollisionPushOutIterations;
};

struct FPhysicalAnimationData
{
	struct FName BodyName;
	bool bIsLocalSimulation;
	float OrientationStrength;
	float AngularVelocityStrength;
	float PositionStrength;
	float VelocityStrength;
	float MaxLinearForce;
	float MaxAngularForce;
};

struct FPhysicalAnimationProfile
{
	struct FName ProfileName;
	struct PhysicalAnimationData PhysicalAnimationData;
};

struct FConstrainComponentPropName
{
	struct FName ComponentName;
};

struct FConstraintInstanceBase
{
};

struct FConstraintBaseParams
{
	float Stiffness;
	float Damping;
	float Restitution;
	float ContactDistance;
	bool bSoftConstraint;
};

struct FConstraintDrive
{
	float Stiffness;
	float Damping;
	float MaxForce;
	bool bEnablePositionDrive;
	bool bEnableVelocityDrive;
};

struct FLinearDriveConstraint
{
	struct Vector PositionTarget;
	struct Vector VelocityTarget;
	struct ConstraintDrive XDrive;
	struct ConstraintDrive YDrive;
	struct ConstraintDrive ZDrive;
	bool bEnablePositionDrive;
};

struct FAngularDriveConstraint
{
	struct ConstraintDrive TwistDrive;
	struct ConstraintDrive SwingDrive;
	struct ConstraintDrive SlerpDrive;
	struct Rotator OrientationTarget;
	struct Vector AngularVelocityTarget;
	EAngularDriveMode AngularDriveMode;
};

struct FConstraintProfileProperties
{
	float ProjectionLinearTolerance;
	float ProjectionAngularTolerance;
	float ProjectionLinearAlpha;
	float ProjectionAngularAlpha;
	float LinearBreakThreshold;
	float AngularBreakThreshold;
	struct LinearConstraint LinearLimit;
	struct ConeConstraint ConeLimit;
	struct TwistConstraint TwistLimit;
	struct LinearDriveConstraint LinearDrive;
	struct AngularDriveConstraint AngularDrive;
	bool bDisableCollision;
	bool bParentDominates;
	bool bEnableProjection;
	bool bEnableSoftProjection;
	bool bAngularBreakable;
	bool bLinearBreakable;
};

struct FPhysicsConstraintProfileHandle
{
	struct ConstraintProfileProperties ProfileProperties;
	struct FName ProfileName;
};

struct FRigidBodyErrorCorrection
{
	float PingExtrapolation;
	float PingLimit;
	float ErrorPerLinearDifference;
	float ErrorPerAngularDifference;
	float MaxRestoredStateError;
	float MaxLinearHardSnapDistance;
	float PositionLerp;
	float AngleLerp;
	float LinearVelocityCoefficient;
	float AngularVelocityCoefficient;
	float ErrorAccumulationSeconds;
	float ErrorAccumulationDistanceSq;
	float ErrorAccumulationSimilarity;
};

struct FPhysicalSurfaceName
{
	EPhysicalSurface Type;
	struct FName Name;
};

struct FChaosPhysicsSettings
{
	EChaosThreadingMode DefaultThreadingModel;
	EChaosSolverTickMode DedicatedThreadTickMode;
	EChaosBufferMode DedicatedThreadBufferMode;
};

struct FPoseData
{
	TArray<struct Transform> LocalSpacePose;
	Unknown TrackToBufferIndex;
	TArray<float> CurveData;
};

struct FPoseDataContainer
{
	TArray<struct SmartName> PoseNames;
	TArray<struct FName> Tracks;
	Unknown TrackMap;
	TArray<struct PoseData> Poses;
	TArray<struct AnimCurveBase> Curves;
};

struct FPreviewMeshCollectionEntry
{
	struct TSoftClassPtr<UObject> SkeletalMesh;
};

struct FCollectionReference
{
	struct FName CollectionName;
};

struct FPerPlatformBool
{
	bool Default;
};

struct FRigTransformConstraint
{
	EConstraintTransform TranformType;
	struct FName ParentSpace;
	float Weight;
};

struct FTransformBaseConstraint
{
	TArray<struct RigTransformConstraint> TransformConstraints;
};

struct FTransformBase
{
	struct FName Node;
	struct TransformBaseConstraint Constraints;
};

struct FNode
{
	struct FName Name;
	struct FName ParentName;
	struct Transform Transform;
	struct FString DisplayName;
	bool bAdvanced;
};

struct FRPCAnalyticsThreshold
{
	struct FName RPC;
	int CountPerSec;
	double TimePerSec;
};

struct FBPVariableMetaDataEntry
{
	struct FName DataKey;
	struct FString DataValue;
};

struct FMeshUVChannelInfo
{
	bool bInitialized;
	bool bOverrideDensities;
	float LocalUVDensities;
};

struct FSkeletalMaterial
{
	class UClass* MaterialInterface;
	struct FName MaterialSlotName;
	struct MeshUVChannelInfo UVChannelData;
};

struct FBoneMirrorInfo
{
	int SourceIndex;
	EAxis BoneFlipAxis;
};

struct FSkeletalMeshBuildSettings
{
	bool bRecomputeNormals;
	bool bRecomputeTangents;
	bool bUseMikkTSpace;
	bool bComputeWeightedNormals;
	bool bRemoveDegenerates;
	bool bUseHighPrecisionTangentBasis;
	bool bUseFullPrecisionUVs;
	bool bBuildAdjacencyBuffer;
	float ThresholdPosition;
	float ThresholdTangentNormal;
	float ThresholdUV;
	float MorphThresholdPosition;
};

struct FSkeletalMeshOptimizationSettings
{
	ESkeletalMeshTerminationCriterion TerminationCriterion;
	float NumOfTrianglesPercentage;
	float NumOfVertPercentage;
	uint32_t MaxNumOfTriangles;
	uint32_t MaxNumOfVerts;
	float MaxDeviationPercentage;
	ESkeletalMeshOptimizationType ReductionMethod;
	ESkeletalMeshOptimizationImportance SilhouetteImportance;
	ESkeletalMeshOptimizationImportance TextureImportance;
	ESkeletalMeshOptimizationImportance ShadingImportance;
	ESkeletalMeshOptimizationImportance SkinningImportance;
	bool bRemapMorphTargets;
	bool bRecalcNormals;
	float WeldingThreshold;
	float NormalsThreshold;
	int MaxBonesPerVertex;
	bool bEnforceBoneBoundaries;
	float VolumeImportance;
	bool bLockEdges;
	bool bLockColorBounaries;
	int BaseLOD;
};

struct FSkeletalMeshLODInfo
{
	struct PerPlatformFloat ScreenSize;
	float LODHysteresis;
	TArray<int> LODMaterialMap;
	struct SkeletalMeshBuildSettings BuildSettings;
	struct SkeletalMeshOptimizationSettings ReductionSettings;
	TArray<struct BoneReference> BonesToRemove;
	TArray<struct BoneReference> BonesToPrioritize;
	float WeightOfPrioritization;
	class UClass* BakePose;
	class UClass* BakePoseOverride;
	struct FString SourceImportFilename;
	ESkinCacheUsage SkinCacheUsage;
	bool bHasBeenSimplified;
	bool bHasPerLODVertexColors;
	bool bAllowCPUAccess;
	bool bSupportUniformlyDistributedSampling;
};

struct FPerPlatformInt
{
	int Default;
};

struct FSkeletalMeshSamplingRegionMaterialFilter
{
	struct FName MaterialName;
};

struct FSkeletalMeshSamplingRegionBoneFilter
{
	struct FName BoneName;
	bool bIncludeOrExclude;
	bool bApplyToChildren;
};

struct FSkeletalMeshSamplingRegion
{
	struct FName Name;
	int LODIndex;
	bool bSupportUniformlyDistributedSampling;
	TArray<struct SkeletalMeshSamplingRegionMaterialFilter> MaterialFilters;
	TArray<struct SkeletalMeshSamplingRegionBoneFilter> BoneFilters;
};

struct FSkeletalMeshSamplingLODBuiltData
{
};

struct FSkeletalMeshSamplingRegionBuiltData
{
};

struct FSkeletalMeshSamplingBuiltData
{
	TArray<struct SkeletalMeshSamplingLODBuiltData> WholeMeshBuiltData;
	TArray<struct SkeletalMeshSamplingRegionBuiltData> RegionBuiltData;
};

struct FSkeletalMeshSamplingInfo
{
	TArray<struct SkeletalMeshSamplingRegion> Regions;
	struct SkeletalMeshSamplingBuiltData BuiltData;
};

struct FSkinWeightProfileInfo
{
	struct FName Name;
	struct PerPlatformBool DefaultProfile;
	struct PerPlatformInt DefaultProfileFromLODIndex;
};

struct FBoneFilter
{
	bool bExcludeSelf;
	struct FName BoneName;
};

struct FSkeletalMeshLODGroupSettings
{
	struct PerPlatformFloat ScreenSize;
	float LODHysteresis;
	EBoneFilterActionOption BoneFilterActionOption;
	TArray<struct BoneFilter> BoneList;
	TArray<struct FName> BonesToPrioritize;
	float WeightOfPrioritization;
	class UClass* BakePose;
	struct SkeletalMeshOptimizationSettings ReductionSettings;
};

struct FTentDistribution
{
	float TipAltitude;
	float TipValue;
	float Width;
};

struct FSoundClassProperties
{
	float Volume;
	float Pitch;
	float LowPassFilterFrequency;
	float AttenuationDistanceScale;
	float StereoBleed;
	float LFEBleed;
	float VoiceCenterChannelVolume;
	float RadioFilterVolume;
	float RadioFilterVolumeThreshold;
	bool bApplyEffects;
	bool bAlwaysPlay;
	bool bIsUISound;
	bool bIsMusic;
	bool bCenterChannelOnly;
	bool bApplyAmbientVolumes;
	bool bReverb;
	float Default2DReverbSendAmount;
	struct SoundModulationDefaultSettings ModulationSettings;
	EAudioOutputTarget OutputTarget;
	ESoundWaveLoadingBehavior LoadingBehavior;
	class UClass* DefaultSubmix;
};

struct FPassiveSoundMixModifier
{
	class UClass* SoundMix;
	float MinVolumeThreshold;
	float MaxVolumeThreshold;
};

struct FSourceEffectChainEntry
{
	class UClass* Preset;
	bool bBypass;
};

struct FSoundGroup
{
	ESoundGroup SoundGroup;
	struct FString DisplayName;
	bool bAlwaysDecompressOnLoad;
	float DecompressedDuration;
};

struct FAudioEffectParameters
{
};

struct FSoundClassAdjuster
{
	class UClass* SoundClassObject;
	float VolumeAdjuster;
	float PitchAdjuster;
	float LowPassFilterFrequency;
	bool bApplyToChildren;
	float VoiceCenterChannelVolumeAdjuster;
};

struct FDialogueWaveParameter
{
	class UClass* DialogueWave;
	struct DialogueContext Context;
};

struct FDistanceDatum
{
	float FadeInDistanceStart;
	float FadeInDistanceEnd;
	float FadeOutDistanceStart;
	float FadeOutDistanceEnd;
	float Volume;
};

struct FModulatorContinuousParams
{
	struct FName ParameterName;
	float Default;
	float MinInput;
	float MaxInput;
	float MinOutput;
	float MaxOutput;
	EModulationParamMode ParamMode;
};

struct FSplineMeshParams
{
	struct Vector StartPos;
	struct Vector StartTangent;
	struct Vector2D StartScale;
	float StartRoll;
	struct Vector2D StartOffset;
	struct Vector EndPos;
	struct Vector2D EndScale;
	struct Vector EndTangent;
	float EndRoll;
	struct Vector2D EndOffset;
};

struct FPerQualityLevelInt
{
	int Default;
	Unknown PerQuality;
};

struct FStaticMaterial
{
	class UClass* MaterialInterface;
	struct FName MaterialSlotName;
	struct FName ImportedMaterialSlotName;
	struct MeshUVChannelInfo UVChannelData;
};

struct FEquirectProps
{
	struct Box2D LeftUVRect;
	struct Box2D RightUVRect;
	struct Vector2D LeftScale;
	struct Vector2D RightScale;
	struct Vector2D LeftBias;
	struct Vector2D RightBias;
};

struct FSubsurfaceProfileStruct
{
	struct LinearColor SurfaceAlbedo;
	struct LinearColor MeanFreePathColor;
	float MeanFreePathDistance;
	float WorldUnitScale;
	bool bEnableBurley;
	float ScatterRadius;
	struct LinearColor SubsurfaceColor;
	struct LinearColor FalloffColor;
	struct LinearColor BoundaryColorBleed;
	float ExtinctionScale;
	float NormalScale;
	float ScatteringDistribution;
	float IOR;
	float Roughness0;
	float Roughness1;
	float LobeMix;
	struct LinearColor TransmissionTintColor;
};

struct FTimelineEventEntry
{
	float Time;
	struct FDelegate EventFunc;
};

struct FTimelineVectorTrack
{
	class UClass* VectorCurve;
	struct FDelegate InterpFunc;
	struct FName TrackName;
	struct FName VectorPropertyName;
};

struct FTimelineFloatTrack
{
	class UClass* FloatCurve;
	struct FDelegate InterpFunc;
	struct FName TrackName;
	struct FName FloatPropertyName;
};

struct FTimelineLinearColorTrack
{
	class UClass* LinearColorCurve;
	struct FDelegate InterpFunc;
	struct FName TrackName;
	struct FName LinearColorPropertyName;
};

struct FTimeline
{
	ETimelineLengthMode LengthMode;
	bool bLooping;
	bool bReversePlayback;
	bool bPlaying;
	float Length;
	float PlayRate;
	float Position;
	TArray<struct TimelineEventEntry> Events;
	TArray<struct TimelineVectorTrack> InterpVectors;
	TArray<struct TimelineFloatTrack> InterpFloats;
	TArray<struct TimelineLinearColorTrack> InterpLinearColors;
	struct FDelegate TimelinePostUpdateFunc;
	struct FDelegate TimelineFinishedFunc;
	Unknown PropertySetObject;
	struct FName DirectionPropertyName;
};

struct FTTTrackBase
{
	struct FName TrackName;
	bool bIsExternalCurve;
};

struct FTouchInputControl
{
	class UClass* Image1;
	class UClass* Image2;
	struct Vector2D Center;
	struct Vector2D VisualSize;
	struct Vector2D ThumbSize;
	struct Vector2D InteractionSize;
	struct Vector2D InputScale;
	struct Key MainInputKey;
	struct Key AltInputKey;
};

struct FHardwareCursorReference
{
	struct FName CursorPath;
	struct Vector2D HotSpot;
};

struct FVirtualTextureBuildSettings
{
	int TileSize;
	int TileBorderSize;
	bool bEnableCompressCrunch;
	bool bEnableCompressZlib;
};

struct FVirtualTextureSpacePoolConfig
{
	int MinTileSize;
	int MaxTileSize;
	TArray<EPixelFormat> Formats;
	int SizeInMegabyte;
	bool bAllowSizeScale;
	uint32_t ScalabilityGroup;
};

struct FVoiceSettings
{
	class UClass* ComponentToAttachTo;
	class UClass* AttenuationSettings;
	class UClass* SourceEffectChain;
};

struct FMaterialProxySettings
{
	struct IntPoint TextureSize;
	float GutterSpace;
	float MetallicConstant;
	float RoughnessConstant;
	float AnisotropyConstant;
	float SpecularConstant;
	float OpacityConstant;
	float OpacityMaskConstant;
	float AmbientOcclusionConstant;
	ETextureSizingType TextureSizingType;
	EMaterialMergeType MaterialMergeType;
	EBlendMode BlendMode;
	bool bAllowTwoSidedMaterial;
	bool bNormalMap;
	bool bTangentMap;
	bool bMetallicMap;
	bool bRoughnessMap;
	bool bAnisotropyMap;
	bool bSpecularMap;
	bool bEmissiveMap;
	bool bOpacityMap;
	bool bOpacityMaskMap;
	bool bAmbientOcclusionMap;
	struct IntPoint DiffuseTextureSize;
	struct IntPoint NormalTextureSize;
	struct IntPoint TangentTextureSize;
	struct IntPoint MetallicTextureSize;
	struct IntPoint RoughnessTextureSize;
	struct IntPoint AnisotropyTextureSize;
	struct IntPoint SpecularTextureSize;
	struct IntPoint EmissiveTextureSize;
	struct IntPoint OpacityTextureSize;
	struct IntPoint OpacityMaskTextureSize;
	struct IntPoint AmbientOcclusionTextureSize;
};

struct FMeshProxySettings
{
	int ScreenSize;
	float VoxelSize;
	struct MaterialProxySettings MaterialSettings;
	float MergeDistance;
	struct Color UnresolvedGeometryColor;
	float MaxRayCastDist;
	float HardAngleThreshold;
	int LightMapResolution;
	EProxyNormalComputationMethod NormalCalculationMethod;
	ELandscapeCullingPrecision LandscapeCullingPrecision;
	bool bCalculateCorrectLODModel;
	bool bOverrideVoxelSize;
	bool bOverrideTransferDistance;
	bool bUseHardAngleThreshold;
	bool bComputeLightMapResolution;
	bool bRecalculateNormals;
	bool bUseLandscapeCulling;
	bool bAllowAdjacency;
	bool bAllowDistanceField;
	bool bReuseMeshLightmapUVs;
	bool bCreateCollision;
	bool bAllowVertexColors;
	bool bGenerateLightmapUVs;
};

struct FMeshMergingSettings
{
	int TargetLightMapResolution;
	EUVOutput OutputUVs;
	struct MaterialProxySettings MaterialSettings;
	int GutterSize;
	int SpecificLOD;
	EMeshLODSelectionType LODSelectionType;
	bool bGenerateLightMapUV;
	bool bComputedLightMapResolution;
	bool bPivotPointAtZero;
	bool bMergePhysicsData;
	bool bMergeMaterials;
	bool bCreateMergedMaterial;
	bool bBakeVertexDataToMesh;
	bool bUseVertexDataForBakingMaterial;
	bool bUseTextureBinning;
	bool bReuseMeshLightmapUVs;
	bool bMergeEquivalentMaterials;
	bool bUseLandscapeCulling;
	bool bIncludeImposters;
	bool bAllowDistanceField;
};

struct FHierarchicalSimplification
{
	float TransitionScreenSize;
	float OverrideDrawDistance;
	bool bUseOverrideDrawDistance;
	bool bAllowSpecificExclusion;
	bool bSimplifyMesh;
	bool bOnlyGenerateClustersForVolumes;
	bool bReusePreviousLevelClusters;
	struct MeshProxySettings ProxySetting;
	struct MeshMergingSettings MergeSetting;
	float DesiredBoundRadius;
	float DesiredFillingPercentage;
	int MinNumberOfActorsToBuild;
};

struct FVectorDistribution
{
	struct DistributionLookupTable Table;
};

struct FVector4Distribution
{
	struct DistributionLookupTable Table;
};

struct FFloatRK4SpringInterpolator
{
	float StiffnessConstant;
	float DampeningRatio;
};

struct FVectorRK4SpringInterpolator
{
	float StiffnessConstant;
	float DampeningRatio;
};

struct FFormatArgumentData
{
	struct FString ArgumentName;
	EFormatArgumentType ArgumentValueType;
	struct FText ArgumentValue;
	int ArgumentValueInt;
	float ArgumentValueFloat;
	ETextGender ArgumentValueGender;
};

struct FExpressionOutput
{
	struct FName OutputName;
};

struct FBranchingPointNotifyPayload
{
};

struct FPlatformInterfaceData
{
	struct FName DataName;
	EPlatformInterfaceDataType Type;
	int IntValue;
	float FloatValue;
	struct FString StringValue;
	class UClass* ObjectValue;
};

struct FPlatformInterfaceDelegateResult
{
	bool bSuccessful;
	struct PlatformInterfaceData Data;
};

struct FDebugFloatHistory
{
	TArray<float> Samples;
	float MaxSamples;
	float MinValue;
	float MaxValue;
	bool bAutoAdjustMinMax;
};

struct FLatentActionInfo
{
	int Linkage;
	int UUID;
	struct FName ExecutionFunction;
	class UClass* CallbackTarget;
};

struct FTimerHandle
{
	uint64_t Handle;
};

struct FCollisionProfileName
{
	struct FName Name;
};

struct FGenericStruct
{
	int Data;
};

struct FUserActivity
{
	struct FString ActionName;
};

struct FCurveTableRowHandle
{
	class UClass* CurveTable;
	struct FName RowName;
};

struct FDataTableRowHandle
{
	class UClass* DataTable;
	struct FName RowName;
};

struct FForceFeedbackParameters
{
	struct FName Tag;
	bool bLooping;
	bool bIgnoreTimeDilation;
	bool bPlayWhilePaused;
};

struct FViewTargetTransitionParams
{
	float BlendTime;
	EViewTargetBlendFunction BlendFunction;
	float BlendExp;
	bool bLockOutgoing;
};

struct FUpdateLevelStreamingLevelStatus
{
	struct FName PackageName;
	int LODIndex;
	bool bNewShouldBeLoaded;
	bool bNewShouldBeVisible;
	bool bNewShouldBlockOnLoad;
};

struct FUpdateLevelVisibilityLevelInfo
{
	struct FName PackageName;
	struct FName Filename;
	bool bIsVisible;
};

struct FTableRowBase
{
};

struct FRootMotionSourceStatus
{
	byte Flags;
};

struct FRootMotionFinishVelocitySettings
{
	ERootMotionFinishVelocityMode Mode;
	struct Vector SetVelocity;
	float ClampVelocity;
};

struct FRootMotionSource
{
	uint16_t Priority;
	uint16_t LocalID;
	ERootMotionAccumulateMode AccumulateMode;
	struct FName InstanceName;
	float StartTime;
	float CurrentTime;
	float PreviousTime;
	float Duration;
	struct RootMotionSourceStatus Status;
	struct RootMotionSourceSettings Settings;
	bool bInLocalSpace;
	struct RootMotionMovementParams RootMotionParams;
	struct RootMotionFinishVelocitySettings FinishVelocityParams;
};

struct FKeyHandleLookupTable
{
};

struct FAnimNode_Base
{
};

struct FPoseLinkBase
{
	int LinkID;
};

struct FAnimInstanceProxy
{
};

struct FInputRange
{
	float Min;
	float Max;
};

struct FInputScaleBiasClamp
{
	bool bMapRange;
	bool bClampResult;
	bool bInterpResult;
	struct InputRange InRange;
	struct InputRange OutRange;
	float Scale;
	float Bias;
	float ClampMin;
	float ClampMax;
	float InterpSpeedIncreasing;
	float InterpSpeedDecreasing;
};

struct FInputAlphaBoolBlend
{
	float BlendInTime;
	float BlendOutTime;
	EAlphaBlendOption BlendOption;
	bool bInitialized;
	class UClass* CustomCurve;
	struct AlphaBlend AlphaBlend;
};

struct FInputScaleBias
{
	float Scale;
	float Bias;
};

struct FComponentReference
{
	class UClass* OtherActor;
	struct FName ComponentProperty;
	struct FString PathToComponent;
};

struct FRuntimeCurveLinearColor
{
	struct RichCurve ColorCurves;
	class UClass* ExternalCurve;
};

struct FFloatSpringState
{
};

struct FCachedAnimStateData
{
	struct FName StateMachineName;
	struct FName StateName;
};

struct FCachedAnimStateArray
{
	TArray<struct CachedAnimStateData> States;
};

struct FSoundSubmixSpectralAnalysisBandSettings
{
	float BandFrequency;
	int AttackTimeMsec;
	int ReleaseTimeMsec;
	float QFactor;
};

struct FPerBoneBlendWeight
{
	int SourceIndex;
	float BlendWeight;
};

struct FPoseSnapshot
{
	TArray<struct Transform> LocalTransforms;
	TArray<struct FName> BoneNames;
	struct FName SkeletalMeshName;
	struct FName SnapshotName;
	bool bIsValid;
};

struct FAnimCurveParam
{
	struct FName Name;
};

struct FActorComponentDuplicatedObjectData
{
};

struct FActorComponentInstanceData
{
	class UClass* SourceComponentTemplate;
	EComponentCreationMethod SourceComponentCreationMethod;
	int SourceComponentTypeSerializedIndex;
	TArray<byte> SavedProperties;
	struct ActorComponentDuplicatedObjectData UniqueTransientPackage;
	TArray<struct ActorComponentDuplicatedObjectData> DuplicatedObjects;
	TArray<class UClass*> ReferencedObjects;
	TArray<struct FName> ReferencedNames;
};

struct FAnimationGroupReference
{
	struct FName GroupName;
	EAnimGroupRole GroupRole;
	EAnimSyncGroupScope GroupScope;
};

struct FAnimGroupInstance
{
};

struct FAnimTickRecord
{
	class UClass* SourceAsset;
};

struct FMarkerSyncAnimPosition
{
	struct FName PreviousMarkerName;
	struct FName NextMarkerName;
	float PositionBetweenMarkers;
};

struct FBlendFilter
{
};

struct FBlendSampleData
{
	int SampleDataIndex;
	class UClass* Animation;
	float TotalWeight;
	float Time;
	float PreviousTime;
	float SamplePlayRate;
};

struct FAnimationRecordingSettings
{
	bool bRecordInWorldSpace;
	bool bRemoveRootAnimation;
	bool bAutoSaveAsset;
	float SampleRate;
	float Length;
	ERichCurveInterpMode InterpMode;
	ERichCurveTangentMode TangentMode;
	bool bRecordTransforms;
	bool bRecordCurves;
};

struct FComponentSpacePose
{
	TArray<struct Transform> Transforms;
	TArray<struct FName> Names;
};

struct FLocalSpacePose
{
	TArray<struct Transform> Transforms;
	TArray<struct FName> Names;
};

struct FNamedTransform
{
	struct Transform Value;
	struct FName Name;
};

struct FNamedColor
{
	struct Color Value;
	struct FName Name;
};

struct FNamedVector
{
	struct Vector Value;
	struct FName Name;
};

struct FNamedFloat
{
	float Value;
	struct FName Name;
};

struct FAnimParentNodeAssetOverride
{
	class UClass* NewAsset;
	struct Guid ParentNodeGuid;
};

struct FAnimBlueprintDebugData
{
};

struct FAnimationFrameSnapshot
{
};

struct FStateMachineDebugData
{
};

struct FStateMachineStateDebugData
{
};

struct FRootMotionExtractionStep
{
	class UClass* AnimSequence;
	float StartPosition;
	float EndPosition;
};

struct FAnimationErrorStats
{
};

struct FSlotEvaluationPose
{
	EAdditiveAnimationType AdditiveType;
	float Weight;
};

struct FA2Pose
{
	TArray<struct Transform> Bones;
};

struct FQueuedDrawDebugItem
{
	EDrawDebugItemType ItemType;
	struct Vector StartLoc;
	struct Vector EndLoc;
	struct Vector Center;
	struct Rotator Rotation;
	float Radius;
	float Size;
	int Segments;
	struct Color Color;
	bool bPersistentLines;
	float LifeTime;
	float Thickness;
	struct FString Message;
	struct Vector2D TextScale;
};

struct FAnimInstanceSubsystemData
{
};

struct FAnimMontageInstance
{
	class UClass* Montage;
	bool bPlaying;
	float DefaultBlendTimeMultiplier;
	TArray<int> NextSections;
	TArray<int> PrevSections;
	TArray<struct AnimNotifyEvent> ActiveStateBranchingPoints;
	float Position;
	float PlayRate;
	struct AlphaBlend Blend;
	int DisableRootMotionCount;
};

struct FInertializationPoseDiff
{
};

struct FInertializationCurveDiff
{
};

struct FInertializationBoneDiff
{
};

struct FInertializationPose
{
};

struct FAnimationPotentialTransition
{
};

struct FAnimationActiveTransitionEntry
{
	class UClass* BlendProfile;
};

struct FCompressedTrack
{
	TArray<byte> ByteStream;
	TArray<float> Times;
	float Mins;
	float Ranges;
};

struct FCurveTrack
{
	struct FName CurveName;
	TArray<float> CurveWeights;
};

struct FScaleTrack
{
	TArray<struct Vector> ScaleKeys;
	TArray<float> Times;
};

struct FRotationTrack
{
	TArray<struct Quat> RotKeys;
	TArray<float> Times;
};

struct FTranslationTrack
{
	TArray<struct Vector> PosKeys;
	TArray<float> Times;
};

struct FRawAnimSequenceTrack
{
	TArray<struct Vector> PosKeys;
	TArray<struct Quat> RotKeys;
	TArray<struct Vector> ScaleKeys;
};

struct FAnimSequenceTrackContainer
{
	TArray<struct RawAnimSequenceTrack> AnimationTracks;
	TArray<struct FName> TrackNames;
};

struct FAnimationTransitionRule
{
	struct FName RuleToExecute;
	bool TransitionReturnVal;
	int TransitionIndex;
};

struct FAnimNotifyTrack
{
	struct FName TrackName;
	struct LinearColor TrackColor;
};

struct FPerBoneBlendWeights
{
	TArray<struct PerBoneBlendWeight> BoneBlendWeights;
};

struct FAssetImportInfo
{
};

struct FAssetManagerSearchRules
{
	TArray<struct FString> AssetScanPaths;
	TArray<struct FString> IncludePatterns;
	TArray<struct FString> ExcludePatterns;
	class UClass* AssetBaseClass;
	bool bHasBlueprintClasses;
	bool bForceSynchronousScan;
	bool bSkipVirtualPathExpansion;
	bool bSkipManagerIncludeCheck;
};

struct FNavAvoidanceData
{
};

struct FGridBlendSample
{
	struct EditorElement GridElement;
	float BlendWeight;
};

struct FBPEditorBookmarkNode
{
	struct Guid NodeGUID;
	struct Guid ParentGuid;
	struct FText DisplayName;
};

struct FEditedDocumentInfo
{
	struct SoftObjectPath EditedObjectPath;
	struct Vector2D SavedViewOffset;
	float SavedZoomAmount;
	class UClass* EditedObject;
};

struct FBPInterfaceDescription
{
	class UClass* Interface;
	TArray<class UClass*> Graphs;
};

struct FBPVariableDescription
{
	struct FName VarName;
	struct Guid VarGuid;
	struct EdGraphPinType VarType;
	struct FString FriendlyName;
	struct FText Category;
	uint64_t PropertyFlags;
	struct FName RepNotifyFunc;
	ELifetimeCondition ReplicationCondition;
	TArray<struct BPVariableMetaDataEntry> MetaDataArray;
	struct FString DefaultValue;
};

struct FBlueprintMacroCosmeticInfo
{
};

struct FCompilerNativizationOptions
{
	struct FName PlatformName;
	bool ServerOnlyPlatform;
	bool ClientOnlyPlatform;
	bool bExcludeMonolithicHeaders;
	TArray<struct FName> ExcludedModules;
	Unknown ExcludedAssets;
	TArray<struct FString> ExcludedFolderPaths;
};

struct FEventGraphFastCallPair
{
	class UClass* FunctionToPatch;
	int EventGraphCallOffset;
};

struct FBlueprintDebugData
{
};

struct FPointerToUberGraphFrame
{
};

struct FDebuggingInfoForSingleFunction
{
};

struct FNodeToCodeAssociation
{
};

struct FAnimCurveType
{
};

struct FBookmarkBaseJumpToSettings
{
};

struct FBookmark2DJumpToSettings
{
};

struct FCachedAnimTransitionData
{
	struct FName StateMachineName;
	struct FName FromStateName;
	struct FName ToStateName;
};

struct FCachedAnimRelevancyData
{
	struct FName StateMachineName;
	struct FName StateName;
};

struct FCachedAnimAssetPlayerData
{
	struct FName StateMachineName;
	struct FName StateName;
};

struct FCameraLensInterfaceClassSupport
{
	class UClass* Class;
};

struct FCameraShakeDuration
{
	float Duration;
	ECameraShakeDurationType Type;
};

struct FCameraShakeInfo
{
	struct CameraShakeDuration Duration;
	float BlendIn;
	float BlendOut;
};

struct FCameraShakeStopParams
{
	bool bImmediately;
};

struct FCameraShakeUpdateResult
{
};

struct FCameraShakeScrubParams
{
	float AbsoluteTime;
	float ShakeScale;
	float DynamicScale;
	float BlendingWeight;
	struct MinimalViewInfo POV;
};

struct FCameraShakeUpdateParams
{
	float DeltaTime;
	float ShakeScale;
	float DynamicScale;
	float BlendingWeight;
	struct MinimalViewInfo POV;
};

struct FCameraShakeStartParams
{
	bool bIsRestarting;
};

struct FDummySpacerCameraTypes
{
};

struct FCanvasIcon
{
	class UClass* Texture;
	float U;
	float V;
	float UL;
	float vL;
};

struct FWrappedStringElement
{
	struct FString Value;
	struct Vector2D LineExtent;
};

struct FTextSizingParameters
{
	float DrawX;
	float DrawY;
	float DrawXL;
	float DrawYL;
	struct Vector2D Scaling;
	class UClass* DrawFont;
	struct Vector2D SpacingAdjust;
};

struct FCharacterNetworkSerializationPackedBits
{
};

struct FChildActorAttachedActorInfo
{
	Unknown Actor;
	struct FName SocketName;
	struct Transform RelativeTransform;
};

struct FAutoCompleteNode
{
	int IndexChar;
	TArray<int> AutoCompleteListIndices;
};

struct FCurveAtlasColorAdjustments
{
	bool bChromaKeyTexture;
	float AdjustBrightness;
	float AdjustBrightnessCurve;
	float AdjustVibrance;
	float AdjustSaturation;
	float AdjustRGBCurve;
	float AdjustHue;
	float AdjustMinAlpha;
	float AdjustMaxAlpha;
};

struct FNamedCurveValue
{
	struct FName Name;
	float Value;
};

struct FCustomAttribute
{
	struct FName Name;
	int VariantType;
	TArray<float> Times;
};

struct FCustomAttributePerBoneData
{
	int BoneTreeIndex;
	TArray<struct CustomAttribute> Attributes;
};

struct FDataTableCategoryHandle
{
	class UClass* DataTable;
	struct FName ColumnName;
	struct FName RowContents;
};

struct FGraphReference
{
	class UClass* MacroGraph;
	class UClass* GraphBlueprint;
	struct Guid GraphGuid;
};

struct FEdGraphPinReference
{
	Unknown OwningNode;
	struct Guid PinId;
};

struct FEdGraphSchemaAction
{
	struct FText MenuDescription;
	struct FText TooltipDescription;
	struct FText Category;
	struct FText Keywords;
	int Grouping;
	int SectionId;
	TArray<struct FString> MenuDescriptionArray;
	TArray<struct FString> FullSearchTitlesArray;
	TArray<struct FString> FullSearchKeywordsArray;
	TArray<struct FString> FullSearchCategoryArray;
	TArray<struct FString> LocalizedMenuDescriptionArray;
	TArray<struct FString> LocalizedFullSearchTitlesArray;
	TArray<struct FString> LocalizedFullSearchKeywordsArray;
	TArray<struct FString> LocalizedFullSearchCategoryArray;
	struct FString SearchText;
};

struct FScreenMessageString
{
	uint64_t Key;
	struct FString ScreenMessage;
	struct Color DisplayColor;
	float TimeToDisplay;
	float CurrentTimeDisplayed;
	struct Vector2D TextScale;
};

struct FURL
{
	struct FString Protocol;
	struct FString Host;
	int Port;
	int Valid;
	struct FString Map;
	struct FString RedirectUrl;
	TArray<struct FString> Op;
	struct FString Portal;
};

struct FFullyLoadedPackagesInfo
{
	EFullyLoadPackageType FullyLoadType;
	struct FString Tag;
	TArray<struct FName> PackagesToLoad;
	TArray<class UClass*> LoadedObjects;
};

struct FLevelStreamingStatus
{
	struct FName PackageName;
	bool bShouldBeLoaded;
	bool bShouldBeVisible;
	uint32_t LODIndex;
};

struct FNamedNetDriver
{
	class UClass* NetDriver;
};

struct FWorldContext
{
	struct URL LastURL;
	struct URL LastRemoteURL;
	class UClass* PendingNetGame;
	TArray<struct FullyLoadedPackagesInfo> PackagesToFullyLoad;
	TArray<class UClass*> LoadedLevelsForPendingMapChange;
	TArray<class UClass*> ObjectReferencers;
	TArray<struct LevelStreamingStatus> PendingLevelStreamingStatusUpdates;
	class UClass* GameViewport;
	class UClass* OwningGameInstance;
	TArray<struct NamedNetDriver> ActiveNetDrivers;
};

struct FExposureSettings
{
	float FixedEV100;
	bool bFixed;
};

struct FTickPrerequisite
{
};

struct FCanvasUVTri
{
	struct Vector2D V0_Pos;
	struct Vector2D V0_UV;
	struct LinearColor V0_Color;
	struct Vector2D V1_Pos;
	struct Vector2D V1_UV;
	struct LinearColor V1_Color;
	struct Vector2D V2_Pos;
	struct Vector2D V2_UV;
	struct LinearColor V2_Color;
};

struct FDepthFieldGlowInfo
{
	bool bEnableGlow;
	struct LinearColor GlowColor;
	struct Vector2D GlowOuterRadius;
	struct Vector2D GlowInnerRadius;
};

struct FFontRenderInfo
{
	bool bClipText;
	bool bEnableShadow;
	struct DepthFieldGlowInfo GlowInfo;
};

struct FDamageEvent
{
	class UClass* DamageTypeClass;
};

struct FRadialDamageParams
{
	float BaseDamage;
	float MinimumDamage;
	float InnerRadius;
	float OuterRadius;
	float DamageFalloff;
};

struct FMeshBuildSettings
{
	bool bUseMikkTSpace;
	bool bRecomputeNormals;
	bool bRecomputeTangents;
	bool bComputeWeightedNormals;
	bool bRemoveDegenerates;
	bool bBuildAdjacencyBuffer;
	bool bBuildReversedIndexBuffer;
	bool bUseHighPrecisionTangentBasis;
	bool bUseFullPrecisionUVs;
	bool bGenerateLightmapUVs;
	bool bGenerateDistanceFieldAsIfTwoSided;
	bool bSupportFaceRemap;
	int MinLightmapResolution;
	int SrcLightmapIndex;
	int DstLightmapIndex;
	float BuildScale;
	struct Vector BuildScale3D;
	float DistanceFieldResolutionScale;
	class UClass* DistanceFieldReplacementMesh;
};

struct FPOV
{
	struct Vector Location;
	struct Rotator Rotation;
	float FOV;
};

struct FAnimUpdateRateParameters
{
	EUpdateRateShiftBucket ShiftBucket;
	bool bInterpolateSkippedFrames;
	bool bShouldUseLodMap;
	bool bShouldUseMinLod;
	bool bSkipUpdate;
	bool bSkipEvaluation;
	int UpdateRate;
	int EvaluationRate;
	float TickedPoseOffestTime;
	float AdditionalTime;
	int BaseNonRenderedUpdateRate;
	int MaxEvalRateForInterpolation;
	TArray<float> BaseVisibleDistanceFactorThesholds;
	Unknown LODToFrameSkipMap;
	int SkippedUpdateFrames;
	int SkippedEvalFrames;
};

struct FAnimSlotDesc
{
	struct FName SlotName;
	int NumChannels;
};

struct FAnimSlotInfo
{
	struct FName SlotName;
	TArray<float> ChannelWeights;
};

struct FMTDResult
{
	struct Vector Direction;
	float Distance;
};

struct FOverlapResult
{
	Unknown Actor;
	Unknown Component;
	bool bBlockingHit;
};

struct FSwarmDebugOptions
{
	bool bDistributionEnabled;
	bool bForceContentExport;
	bool bInitialized;
};

struct FLightmassDebugOptions
{
	bool bDebugMode;
	bool bStatsEnabled;
	bool bGatherBSPSurfacesAcrossComponents;
	float CoplanarTolerance;
	bool bUseImmediateImport;
	bool bImmediateProcessMappings;
	bool bSortMappings;
	bool bDumpBinaryFiles;
	bool bDebugMaterials;
	bool bPadMappings;
	bool bDebugPaddings;
	bool bOnlyCalcDebugTexelMappings;
	bool bUseRandomColors;
	bool bColorBordersGreen;
	bool bColorByExecutionTime;
	float ExecutionTimeDivisor;
};

struct FBasedPosition
{
	class UClass* Base;
	struct Vector Position;
	struct Vector CachedBaseLocation;
	struct Rotator CachedBaseRotation;
	struct Vector CachedTransPosition;
};

struct FFractureEffect
{
	class UClass* ParticleSystem;
	class UClass* Sound;
};

struct FRigidBodyContactInfo
{
	struct Vector ContactPosition;
	struct Vector ContactNormal;
	float ContactPenetration;
	class UClass* PhysMaterial;
};

struct FCollisionImpactData
{
	TArray<struct RigidBodyContactInfo> ContactInfos;
	struct Vector TotalNormalImpulse;
	struct Vector TotalFrictionImpulse;
	bool bIsVelocityDeltaUnderThreshold;
};

struct FRigidBodyState
{
	struct Vector_NetQuantize100 Position;
	struct Quat Quaternion;
	struct Vector_NetQuantize100 LinVel;
	struct Vector_NetQuantize100 AngVel;
	byte Flags;
};

struct FPredictProjectilePathPointData
{
	struct Vector Location;
	struct Vector Velocity;
	float Time;
};

struct FPredictProjectilePathResult
{
	TArray<struct PredictProjectilePathPointData> PathData;
	struct PredictProjectilePathPointData LastTraceDestination;
	struct HitResult HitResult;
};

struct FPredictProjectilePathParams
{
	struct Vector StartLocation;
	struct Vector LaunchVelocity;
	bool bTraceWithCollision;
	float ProjectileRadius;
	float MaxSimTime;
	bool bTraceWithChannel;
	ECollisionChannel TraceChannel;
	TArray<EObjectTypeQuery> ObjectTypes;
	TArray<class UClass*> ActorsToIgnore;
	float SimFrequency;
	float OverrideGravityZ;
	EDrawDebugTrace DrawDebugType;
	float DrawDebugTime;
	bool bTraceComplex;
};

struct FActiveHapticFeedbackEffect
{
	class UClass* HapticEffect;
};

struct FClusterNode
{
	struct Vector BoundMin;
	int FirstChild;
	struct Vector BoundMax;
	int LastChild;
	int FirstInstance;
	int LastInstance;
	struct Vector MinInstanceScale;
	struct Vector MaxInstanceScale;
};

struct FClusterNode_DEPRECATED
{
	struct Vector BoundMin;
	int FirstChild;
	struct Vector BoundMax;
	int LastChild;
	int FirstInstance;
	int LastInstance;
};

struct FHLODISMComponentDesc
{
	class UClass* StaticMesh;
	class UClass* Material;
	TArray<struct Transform> Instances;
};

struct FImportanceTexture
{
	struct IntPoint Size;
	int NumMips;
	TArray<float> MarginalCDF;
	TArray<float> ConditionalCDF;
	TArray<struct Color> TextureData;
	Unknown Texture;
	EImportanceWeight Weighting;
};

struct FInstancedStaticMeshLightMapInstanceData
{
	struct Transform Transform;
	TArray<struct Guid> MapBuildDataIds;
};

struct FInterpEdSelKey
{
	class UClass* Group;
	class UClass* Track;
	int KeyIndex;
	float UnsnappedPosition;
};

struct FCameraPreviewInfo
{
	class UClass* PawnClass;
	class UClass* AnimSeq;
	struct Vector Location;
	struct Rotator Rotation;
	class UClass* PawnInst;
};

struct FSubTrackGroup
{
	struct FString GroupName;
	TArray<int> TrackIndices;
	bool bIsCollapsed;
	bool bIsSelected;
};

struct FSupportedSubTrackInfo
{
	class UClass* SupportedClass;
	struct FString SubTrackName;
	int GroupIndex;
};

struct FVectorSpringState
{
};

struct FDrawToRenderTargetContext
{
	class UClass* RenderTarget;
};

struct FLatentActionManager
{
};

struct FLevelSimplificationDetails
{
	bool bCreatePackagePerAsset;
	float DetailsPercentage;
	struct MaterialProxySettings StaticMeshMaterialSettings;
	bool bOverrideLandscapeExportLOD;
	int LandscapeExportLOD;
	struct MaterialProxySettings LandscapeMaterialSettings;
	bool bBakeFoliageToLandscape;
	bool bBakeGrassToLandscape;
	bool bGenerateMeshNormalMap;
	bool bGenerateMeshMetallicMap;
	bool bGenerateMeshRoughnessMap;
	bool bGenerateMeshSpecularMap;
	bool bGenerateLandscapeNormalMap;
	bool bGenerateLandscapeMetallicMap;
	bool bGenerateLandscapeRoughnessMap;
	bool bGenerateLandscapeSpecularMap;
};

struct FStreamableTextureInstance
{
};

struct FBatchedPoint
{
	struct Vector Position;
	struct LinearColor Color;
	float PointSize;
	float RemainingLifeTime;
	byte DepthPriority;
};

struct FBatchedLine
{
	struct Vector Start;
	struct Vector End;
	struct LinearColor Color;
	float Thickness;
	float RemainingLifeTime;
	byte DepthPriority;
};

struct FClientReceiveData
{
	class UClass* LocalPC;
	struct FName MessageType;
	int MessageIndex;
	struct FString MessageString;
	class UClass* RelatedPlayerState_1;
	class UClass* RelatedPlayerState_2;
	class UClass* OptionalObject;
};

struct FParameterGroupData
{
	struct FString GroupName;
	int GroupSortPriority;
};

struct FStaticComponentMaskValue
{
	bool R;
	bool G;
	bool B;
	bool A;
};

struct FParameterChannelNames
{
	struct FText R;
	struct FText G;
	struct FText B;
	struct FText A;
};

struct FFunctionExpressionOutput
{
	class UClass* ExpressionOutput;
	struct Guid ExpressionOutputId;
	struct ExpressionOutput Output;
};

struct FFunctionExpressionInput
{
	class UClass* ExpressionInput;
	struct Guid ExpressionInputId;
	struct ExpressionInput Input;
};

struct FScalarParameterAtlasInstanceData
{
	bool bIsUsedAsAtlasPosition;
	struct TSoftClassPtr<UObject> Curve;
	struct TSoftClassPtr<UObject> Atlas;
};

struct FMemberReference
{
	class UClass* MemberParent;
	struct FString MemberScope;
	struct FName MemberName;
	struct Guid MemberGuid;
	bool bSelfContext;
	bool bWasDeprecated;
};

struct FMeshInstancingSettings
{
	class UClass* ActorClassToUse;
	int InstanceReplacementThreshold;
	EMeshInstancingReplacementMethod MeshReplacementMethod;
	bool bSkipMeshesWithVertexColors;
	bool bUseHLODVolumes;
	class UClass* ISMComponentToUse;
};

struct FMeshReductionSettings
{
	float PercentTriangles;
	float PercentVertices;
	float MaxDeviation;
	float PixelError;
	float WeldingThreshold;
	float HardAngleThreshold;
	int BaseLODModel;
	EMeshFeatureImportance SilhouetteImportance;
	EMeshFeatureImportance TextureImportance;
	EMeshFeatureImportance ShadingImportance;
	bool bRecalculateNormals;
	bool bGenerateUniqueLightmapUVs;
	bool bKeepSymmetry;
	bool bVisibilityAided;
	bool bCullOccluded;
	EStaticMeshReductionTerimationCriterion TerminationCriterion;
	EMeshFeatureImportance VisibilityAggressiveness;
	EMeshFeatureImportance VertexColorImportance;
};

struct FNameCurveKey
{
	float Time;
	struct FName Value;
};

struct FPacketSimulationSettings
{
	int PktLoss;
	int PktLossMaxSize;
	int PktLossMinSize;
	int PktOrder;
	int PktDup;
	int PktLag;
	int PktLagVariance;
	int PktLagMin;
	int PktLagMax;
	int PktIncomingLagMin;
	int PktIncomingLagMax;
	int PktIncomingLoss;
	int PktJitter;
};

struct FParticleCurvePair
{
	struct FString CurveName;
	class UClass* CurveObject;
};

struct FBeamTargetData
{
	struct FName TargetName;
	float TargetPercentage;
};

struct FParticleSystemReplayFrame
{
};

struct FParticleEmitterReplayFrame
{
};

struct FFreezablePerPlatformInt
{
};

struct FPlayerMuteList
{
	bool bHasVoiceHandshakeCompleted;
	int VoiceChannelIdx;
};

struct FPreviewAttachedObjectPair
{
	struct TSoftClassPtr<UObject> AttachedObject;
	class UClass* Object;
	struct FName AttachedTo;
};

struct FPreviewAssetAttachContainer
{
	TArray<struct PreviewAttachedObjectPair> AttachedObjects;
};

struct FSpriteCategoryInfo
{
	struct FName Category;
	struct FText DisplayName;
	struct FText Description;
};

struct FQuartzPulseOverrideStep
{
	int NumberOfPulses;
	EQuartzCommandQuantization PulseDuration;
};

struct FQuartzTimeSignature
{
	int NumBeats;
	EQuartzTimeSignatureQuantization BeatType;
	TArray<struct QuartzPulseOverrideStep> OptionalPulseOverride;
};

struct FQuartzClockSettings
{
	struct QuartzTimeSignature TimeSignature;
	bool bIgnoreLevelChange;
};

struct FQuartzQuantizationBoundary
{
	EQuartzCommandQuantization Quantization;
	float Multiplier;
	EQuarztQuantizationReference CountingReferencePoint;
	bool bFireOnClockStart;
};

struct FQuartzTransportTimeStamp
{
};

struct FLevelNameAndTime
{
	struct FString LevelName;
	uint32_t LevelChangeTimeInMS;
};

struct FCompressedRichCurve
{
};

struct FRPCDoSState
{
	bool bLogEscalate;
	bool bSendEscalateAnalytics;
	bool bKickPlayer;
	bool bTrackRecentRPCs;
	int16_t EscalateQuotaRPCsPerFrame;
	int16_t EscalateTimeQuotaMSPerFrame;
	int16_t EscalateQuotaRPCsPerPeriod;
	int16_t EscalateTimeQuotaMSPerPeriod;
	int8_t EscalateQuotaTimePeriod;
	int8_t EscalationCountTolerance;
	int16_t EscalationTimeToleranceMS;
	int16_t RPCRepeatLimitPerPeriod;
	int16_t RPCRepeatLimitMSPerPeriod;
	int8_t RPCRepeatLimitTimePeriod;
	int16_t CooloffTime;
	int16_t AutoEscalateTime;
};

struct FCameraExposureSettings
{
	EAutoExposureMethod Method;
	float LowPercent;
	float HighPercent;
	float MinBrightness;
	float MaxBrightness;
	float SpeedUp;
	float SpeedDown;
	float Bias;
	class UClass* BiasCurve;
	class UClass* MeterMask;
	float HistogramLogMin;
	float HistogramLogMax;
	float CalibrationConstant;
	bool ApplyPhysicalCameraExposure;
};

struct FGaussianSumBloomSettings
{
	float Intensity;
	float Threshold;
	float SizeScale;
	float Filter1Size;
	float Filter2Size;
	float Filter3Size;
	float Filter4Size;
	float Filter5Size;
	float Filter6Size;
	struct LinearColor Filter1Tint;
	struct LinearColor Filter2Tint;
	struct LinearColor Filter3Tint;
	struct LinearColor Filter4Tint;
	struct LinearColor Filter5Tint;
	struct LinearColor Filter6Tint;
};

struct FConvolutionBloomSettings
{
	class UClass* Texture;
	float Size;
	struct Vector2D CenterUV;
	float PreFilterMin;
	float PreFilterMax;
	float PreFilterMult;
	float BufferScale;
};

struct FLensBloomSettings
{
	struct GaussianSumBloomSettings GaussianSum;
	struct ConvolutionBloomSettings Convolution;
	EBloomMethod Method;
};

struct FLensImperfectionSettings
{
	class UClass* DirtMask;
	float DirtMaskIntensity;
	struct LinearColor DirtMaskTint;
};

struct FLensSettings
{
	struct LensBloomSettings Bloom;
	struct LensImperfectionSettings Imperfections;
	float ChromaticAberration;
};

struct FFilmStockSettings
{
	float Slope;
	float Toe;
	float Shoulder;
	float BlackClip;
	float WhiteClip;
};

struct FColorGradePerRangeSettings
{
	struct Vector4 Saturation;
	struct Vector4 Contrast;
	struct Vector4 Gamma;
	struct Vector4 Gain;
	struct Vector4 Offset;
};

struct FColorGradingSettings
{
	struct ColorGradePerRangeSettings Global;
	struct ColorGradePerRangeSettings Shadows;
	struct ColorGradePerRangeSettings Midtones;
	struct ColorGradePerRangeSettings Highlights;
	float ShadowsMax;
	float HighlightsMin;
};

struct FSceneViewExtensionIsActiveFunctor
{
};

struct FClothPhysicsProperties_Legacy
{
	float VerticalResistance;
	float HorizontalResistance;
	float BendResistance;
	float ShearResistance;
	float Friction;
	float Damping;
	float TetherStiffness;
	float TetherLimit;
	float Drag;
	float StiffnessFrequency;
	float GravityScale;
	float MassScale;
	float InertiaBlend;
	float SelfCollisionThickness;
	float SelfCollisionSquashScale;
	float SelfCollisionStiffness;
	float SolverFrequency;
	float FiberCompression;
	float FiberExpansion;
	float FiberResistance;
};

struct FClothingAssetData_Legacy
{
	struct FName AssetName;
	struct FString ApexFileName;
	bool bClothPropertiesChanged;
	struct ClothPhysicsProperties_Legacy PhysicsProperties;
};

struct FSkeletalMeshClothBuildParams
{
	Unknown TargetAsset;
	int TargetLod;
	bool bRemapParameters;
	struct FString AssetName;
	int LODIndex;
	int SourceSection;
	bool bRemoveFromMesh;
	struct TSoftClassPtr<UObject> PhysicsAsset;
};

struct FBoneMirrorExport
{
	struct FName BoneName;
	struct FName SourceBoneName;
	EAxis BoneFlipAxis;
};

struct FNameMapping
{
	struct FName NodeName;
	struct FName BoneName;
};

struct FRigConfiguration
{
	class UClass* Rig;
	TArray<struct NameMapping> BoneMappingTable;
};

struct FBoneReductionSetting
{
	TArray<struct FName> BonesToRemove;
};

struct FReferencePose
{
	struct FName PoseName;
	TArray<struct Transform> ReferencePose;
};

struct FSkeletonToMeshLinkup
{
	TArray<int> SkeletonToMeshTable;
	TArray<int> MeshToSkeletonTable;
};

struct FSkelMeshSkinWeightInfo
{
	int Bones;
	byte Weights;
};

struct FSmartNameMapping
{
};

struct FCurveMetaData
{
};

struct FSoundClassEditorData
{
};

struct FSoundNodeEditorData
{
};

struct FSoundWaveEnvelopeDataPerSound
{
	float Envelope;
	float PlaybackTime;
	class UClass* SoundWave;
};

struct FSoundWaveSpectralData
{
	float FrequencyHz;
	float Magnitude;
	float NormalizedMagnitude;
};

struct FSoundWaveSpectralDataPerSound
{
	TArray<struct SoundWaveSpectralData> SpectralData;
	float PlaybackTime;
	class UClass* SoundWave;
};

struct FStreamedAudioPlatformData
{
};

struct FSplinePoint
{
	float InputKey;
	struct Vector Position;
	struct Vector ArriveTangent;
	struct Vector LeaveTangent;
	struct Rotator Rotation;
	struct Vector Scale;
	ESplinePointType Type;
};

struct FMaterialRemapIndex
{
	uint32_t ImportVersionKey;
	TArray<int> MaterialRemap;
};

struct FAssetEditorOrbitCameraPosition
{
	bool bIsSet;
	struct Vector CamOrbitPoint;
	struct Vector CamOrbitZoom;
	struct Rotator CamOrbitRotation;
};

struct FMeshSectionInfo
{
	int MaterialIndex;
	bool bEnableCollision;
	bool bCastShadow;
	bool bVisibleInRayTracing;
	bool bForceOpaque;
};

struct FMeshSectionInfoMap
{
	Unknown Map;
};

struct FStaticMeshSourceModel
{
	struct MeshBuildSettings BuildSettings;
	struct MeshReductionSettings ReductionSettings;
	float LODDistance;
	struct PerPlatformFloat ScreenSize;
	struct FString SourceImportFilename;
};

struct FStaticMeshOptimizationSettings
{
	EOptimizationType ReductionMethod;
	float NumOfTrianglesPercentage;
	float MaxDeviationPercentage;
	float WeldingThreshold;
	bool bRecalcNormals;
	float NormalsThreshold;
	byte SilhouetteImportance;
	byte TextureImportance;
	byte ShadingImportance;
};

struct FPaintedVertex
{
	struct Vector Position;
	struct Color Color;
	struct Vector4 Normal;
};

struct FStaticMeshVertexColorLODData
{
	TArray<struct PaintedVertex> PaintedVertices;
	TArray<struct Color> VertexBufferColors;
	uint32_t LODIndex;
};

struct FTextureFormatSettings
{
	ETextureCompressionSettings CompressionSettings;
	bool CompressionNoAlpha;
	bool CompressionNone;
	bool CompressionYCoCg;
	bool SRGB;
};

struct FTexturePlatformData
{
};

struct FTextureSource
{
};

struct FTextureSourceBlock
{
	int BlockX;
	int BlockY;
	int SizeX;
	int SizeY;
	int NumSlices;
	int NumMips;
};

struct FStreamingRenderAssetPrimitiveInfo
{
	class UClass* RenderAsset;
	struct BoxSphereBounds Bounds;
	float TexelFactor;
	uint32_t PackedRelativeBox;
	bool bAllowInvalidTexelFactorWhenUnregistered;
};

struct FTTTrackId
{
	int TrackType;
	int TrackIndex;
};

struct FTimeStretchCurveInstance
{
	bool bHasValidData;
};

struct FLevelViewportInfo
{
	struct Vector CamPosition;
	struct Rotator CamRotation;
	float CamOrthoZoom;
	bool CamUpdated;
};

struct FLightmassWorldInfoSettings
{
	float StaticLightingLevelScale;
	int NumIndirectLightingBounces;
	int NumSkyLightingBounces;
	float IndirectLightingQuality;
	float IndirectLightingSmoothness;
	struct Color EnvironmentColor;
	float EnvironmentIntensity;
	float EmissiveBoost;
	float DiffuseBoost;
	EVolumeLightingMethod VolumeLightingMethod;
	bool bUseAmbientOcclusion;
	bool bGenerateAmbientOcclusionMaterialMask;
	bool bVisualizeMaterialDiffuse;
	bool bVisualizeAmbientOcclusion;
	bool bCompressLightmaps;
	float VolumetricLightmapDetailCellSize;
	float VolumetricLightmapMaximumBrickMemoryMb;
	float VolumetricLightmapSphericalHarmonicSmoothing;
	float VolumeLightSamplePlacementScale;
	float DirectIlluminationOcclusionFraction;
	float IndirectIlluminationOcclusionFraction;
	float OcclusionExponent;
	float FullyOccludedSamplesFraction;
	float MaxOcclusionDistance;
};

struct FEntityComponentContainer
{
	TArray<class UClass*> Array;
};

struct FComponentData
{
};

struct FScriptDiagnosticSourceLocation
{
};

struct FScriptDiagnosticMessage
{
	EScriptDiagnosticMessageType MessageType;
	struct DateTime Timestamp;
	struct FString Channel;
	struct FText MessageStr;
	struct ScriptDiagnosticSourceLocation SourceLocation;
};

struct FSlotDescription
{
	struct FName SlotName;
	int ColumnCount;
	int RowCount;
	bool bUseFeaturedTextStyle;
	bool bEnableAutoScroll;
};

struct FEpicCMSPage
{
};

struct FTileDefinition
{
	struct FString TypeString;
	struct FString Title;
	struct FString Subtitle;
	struct FString Eyebrow;
	struct FString Link;
	struct FString GroupId;
	struct DateTime Countdown;
	EDateType CountdownType;
	struct FString MediaUrl;
	bool IsVisible;
};

struct FDownloadCacheEntry
{
	struct FString FilePath;
	struct FString SourceUrl;
	double LastAccessTime;
};

struct FDownloadCache
{
	int Version;
	Unknown Cache;
};

struct FManagedGameplayTagDataTableItem
{
	struct GameplayTag RootTag;
	class UClass* DataTable;
};

struct FTagTableManagerHelper
{
};

struct FEventModeFocusActor
{
	struct TSoftClassPtr<UObject> Target;
	float Distance;
	struct Vector Offset;
	float FOV;
};

struct FEventModeWidgetCachedData
{
	EUIExtensionSlot Slot;
	Unknown Widget;
};

struct FEyeTrackerStereoGazeData
{
	struct Vector LeftEyeOrigin;
	struct Vector LeftEyeDirection;
	struct Vector RightEyeOrigin;
	struct Vector RightEyeDirection;
	struct Vector FixationPoint;
	float ConfidenceValue;
};

struct FEyeTrackerGazeData
{
	struct Vector GazeOrigin;
	struct Vector GazeDirection;
	struct Vector FixationPoint;
	float ConfidenceValue;
};

struct FFieldObjectCommands
{
	TArray<struct FName> TargetNames;
	TArray<class UClass*> RootNodes;
	TArray<class UClass*> MetaDatas;
};

struct FFoliageVertexColorChannelMask
{
	bool UseMask;
	float MaskThreshold;
	bool InvertMask;
};

struct FFoliageTypeObject
{
	class UClass* FoliageTypeObject;
	class UClass* TypeInstance;
	bool bIsAsset;
	class UClass* Type;
};

struct FProceduralFoliageInstance
{
	struct Quat Rotation;
	struct Vector Location;
	float Age;
	struct Vector Normal;
	float Scale;
	class UClass* Type;
};

struct FFortConversationConditionalMessage
{
	class UClass* Condition;
	struct FText Message;
};

struct FFortConversationNodeConditionalMessages
{
	TArray<struct FortConversationConditionalMessage> Messages;
};

struct FFortTargetFilter
{
	EFortTargetSelectionFilter ActorTypeFilter;
	class UClass* ActorClassFilter;
	bool bExcludeInstigator;
	bool bUseTrapsOwningPawnAsInstigator;
	bool bExcludeRequester;
	bool bExcludeAllAttachedToInstigator;
	bool bExcludeAthenaVehicleOfInstigator;
	bool bExcludeAllAttachedToRequester;
	bool bExcludePawnFriends;
	bool bExcludeFriendlyAI;
	bool bExcludeAllAI;
	bool bExcludePawnNeutrals;
	bool bExcludePawnEnemies;
	bool bExcludeNonPawnFriends;
	bool bExcludeNonPawnEnemies;
	bool bConsiderFriendlyFireWhenExcludingFriends;
	bool bExcludeDBNOPawns;
	bool bExcludeWaterVolumes;
	bool bExcludeWaterBodies;
	bool bExcludeAthenaVehicleOccupiedBySource;
	bool bExcludeAthenaVehicleUnoccupied;
	bool bExcludeAthenaVehicleOccupied;
	bool bExcludeAthenaVehicleFromObstructionChecks;
	bool bExcludeWithoutNavigationCorridor;
	bool bExcludeNonPlayerBuiltPieces;
	bool bExcludePlayerBuiltPieces;
	bool bExcludeNonBGABuildings;
	bool bExcludeNonBlockingHits;
	bool bExcludeProjectiles;
	bool bTraceComplexCollision;
	bool bExcludeLandscape;
	bool bConsiderPhysicsPawnsAsNonPlayerPawns;
	struct GameplayTagContainer ExclusionGameplayTags;
};

struct FFortAbilityTargetSelection
{
	EFortTargetSelectionShape Shape;
	struct FString CustomShapeComponentName;
	EFortTargetSelectionTestType TestType;
	EFortAbilityTargetingSource PrimarySource;
	EFortAbilityTargetingSource SecondarySource;
	struct ScalableFloat Range;
	struct Vector HalfExtents;
	bool bAlignShapeEdgeToSourceLocation;
	struct ScalableFloat ConeYawAngle;
	struct ScalableFloat ConePitchAngle;
	struct ScalableFloat ConeMinRadius;
	struct FortTargetFilter TargetFilter;
	bool bExcludeObstructedByWorld;
	bool bShouldAttachedActorsObstructTarget;
	bool bCreateHitResultWhenNoTargetsFound;
	bool bUseProjectileRotationForDamageZones;
	EFortAbilityTargetSelectionUsage TargetSelectionUsage;
	int MaxTargets;
};

struct FFortAbilityTargetSelectionList
{
	TArray<struct FortAbilityTargetSelection> List;
	bool bStopAtFirstSuccess;
	bool bKeepCheckingListOnIndestructibleHit;
	struct GameplayTagContainer AbilityTargetBlockedGameplayTags;
	bool bUseWeaponRanges;
	bool bUseMaxYawAngleToTarget;
	float MaxYawAngleToTarget;
};

struct FFortGameplayEffectContainer
{
	struct GameplayTag ApplicationTag;
	struct FortAbilityTargetSelectionList TargetSelection;
	TArray<class UClass*> TargetGameplayEffectClasses;
	TArray<class UClass*> OwnerGameplayEffectClasses;
	struct GameplayTagContainer ActivationCues;
	struct GameplayTagContainer ImpactCues;
	bool bUseCalculationInTooltips;
	bool bOverrideChargeMagnitude;
	float ChargeMagnitudeOverrideValue;
};

struct FFortAbilityCost
{
	EFortAbilityCostSource CostSource;
	struct ScalableFloat CostValue;
	class UClass* ItemDefinition;
	bool bOnlyApplyCostOnHit;
};

struct FFortGameplayAbilityBehaviorDistanceData
{
	struct GameplayTagContainer DistanceDataTag;
	float Distance;
};

struct FFortCharacterPartMontageInfo
{
	EFortCustomPartType CharacterPart;
	class UClass* AnimMontage;
};

struct FFortGameplayAbilityMontageInfo
{
	class UClass* MontageToPlay;
	float AnimPlayRate;
	float AnimRootMotionTranslationScale;
	EFortGameplayAbilityMontageSectionToPlay MontageSectionToPlay;
	struct FName OverrideSection;
	bool bPlayRandomSection;
	TArray<struct FortCharacterPartMontageInfo> CharacterPartMontages;
};

struct FAbilityTrackedActorSettings
{
	struct ScalableFloat MaximumTrackedActors;
};

struct FBoneSet
{
	TArray<struct BoneReference> Bones;
	TArray<struct FName> Sockets;
};

struct FAnimTagProperty
{
	struct GameplayTag BackingGameplayTag;
	Unknown PropertyToEdit;
	struct FName PropertyName;
	bool bUseExactTag;
};

struct FBuildingWeakSpotData
{
	Unknown ParentObject;
	struct Vector_NetQuantizeNormal Normal;
	struct Vector_NetQuantize10 Position;
	int HitCount;
};

struct FAIDirectorDebugInfo
{
	float Timestamp;
	TArray<float> DebugGraphData;
};

struct FLastBuildableState
{
	class UClass* LastBuildableMetaData;
	bool LastBuildableMirrored;
	int LastBuildableRotationIterations;
};

struct FFortDamageNumberInfo
{
	struct Vector WorldLocation;
	struct Vector HitNormal;
	bool bIsCriticalDamage;
	int Damage;
	EFortDamageNumberType DamageNumberType;
	float VisualDamageScale;
	EFortElementalDamageType ElementalDamageType;
	EStatCategory ScoreType;
	bool bAttachScoreNumberToPlayer;
	class UClass* StaticMeshComponent;
	TArray<class UClass*> MeshMIDs;
	TArray<int> DamageNumberArray;
	struct GameplayTagContainer DamagedActorTags;
	Unknown DamagedActor;
	Unknown DamageCauser;
};

struct FRecentlyRemovedQuickbarInfo
{
	struct Guid ItemGuid;
	int RemovedFromSlot;
	class UClass* ItemDefinition;
};

struct FQueuedItemsToDropViaPickup
{
	class UClass* DestructionPawn;
	int TotalNumItemsToDrop;
	TArray<class UClass*> ItemsToDrop;
};

struct FCosmeticVariantInfo
{
	struct GameplayTag VariantChannelTag;
	struct GameplayTag ActiveVariantTag;
};

struct FFortAthenaLoadout
{
	struct FString BannerIconId;
	struct FString BannerColorId;
	class UClass* SkyDiveContrail;
	class UClass* Glider;
	class UClass* Pickaxe;
	bool bIsDefaultCharacter;
	class UClass* Character;
	TArray<struct McpVariantChannelInfo> CharacterVariantChannels;
	bool bForceUpdateVariants;
	class UClass* Hat;
	class UClass* Backpack;
	class UClass* LoadingScreen;
	class UClass* BattleBus;
	class UClass* VehicleDecoration;
	class UClass* CallingCard;
	class UClass* MapMarker;
	TArray<class UClass*> Dances;
	class UClass* VictoryPose;
	class UClass* MusicPack;
	class UClass* ItemWrapOverride;
	TArray<class UClass*> ItemWraps;
	class UClass* CharmOverride;
	TArray<class UClass*> Charms;
	class UClass* PetSkin;
};

struct FFortSavedWeaponModSlot
{
	struct FString WeaponModTemplateID;
	bool bIsDynamic;
};

struct FFortGiftingInfo
{
	struct FString PlayerName;
	class UClass* HeroType;
};

struct FFortItemEntryStateValue
{
	int IntValue;
	struct FName NameValue;
	EFortItemEntryState StateType;
};

struct FFortWeaponModSlot
{
	class UClass* WeaponMod;
	bool bIsDynamic;
};

struct FFortRewardActivity
{
	EFortRewardActivityType ActivityType;
	struct Guid ActivityGuid;
	struct FText TitleText;
	struct FText DescriptionText;
	float RewardDisplayTime;
	TArray<struct FortItemEntry> RewardItems;
	TArray<struct FortItemEntry> MissedRewardItems;
	EFortCompletionResult ActivityCompletionResult;
	int AdditionalCompletionMissionPoints;
};

struct FFortRewardReport
{
	struct FText MissionName;
	struct FText TheaterName;
	struct FText Difficulty;
	float DifficultyValue;
	TArray<struct FortRewardActivity> RewardActivities;
	bool bIsFinalized;
};

struct FFortUpdatedObjectiveStat
{
	class UClass* Quest;
	struct FName BackendName;
	int StatValue;
	int ShadowStatValue;
	int StatDelta;
	int CurrentStage;
	bool bSharedQuestUpdate;
	bool bShowToast;
};

struct FFortPersistentGameplayStatValue
{
	struct FString StatName;
	int StatValue;
};

struct FFortPersistentGameplayStatContainer
{
	TArray<struct FortPersistentGameplayStatValue> GameplayStats;
};

struct FLockOnInfo
{
	ELockOnState State;
	Unknown Weapon;
	Unknown LockOnTarget;
	struct Rotator CamRotAtTargetAcquisiton;
	struct Vector2D LockOnCoords;
	float TargetAcquisitionTime;
	float TargetLockOnTime;
	float TargetOutOfSightTime;
	float CooldownStartTime;
};

struct FFortAttributeInitializationKey
{
	struct FName AttributeInitCategory;
	struct FName AttributeInitSubCategory;
};

struct FFortDeliveryInfoRequirementsFilter
{
	struct GameplayTagRequirements SourceTagRequirements;
	struct GameplayTagRequirements TargetTagRequirements;
	EFortTeamAffiliation ApplicableTeamAffiliation;
	bool bConsiderTeamAffiliationToInstigator;
	EFortTeam ApplicableTeam;
	bool bConsiderTeam;
	bool bApplyToPlayerPawns;
	bool bApplyToAIPawns;
	bool bApplyToBuildingActors;
	EFortDeliveryInfoBuildingActorSpecification BuildingActorSpecification;
	bool bApplyToGlobalEnvironmentAbilityActor;
};

struct FProximityBasedGEDeliveryInfoBase
{
	struct FortDeliveryInfoRequirementsFilter DeliveryRequirements;
	EFortProximityBasedGEApplicationType ProximityApplicationType;
};

struct FGameplayEffectApplicationInfoHard
{
	class UClass* GameplayEffect;
	float Level;
};

struct FFortAbilitySetDeliveryInfo
{
	struct FortDeliveryInfoRequirementsFilter DeliveryRequirements;
	TArray<struct TSoftClassPtr<UObject>> AbilitySets;
};

struct FFortAbilitySetHandle
{
	Unknown TargetAbilitySystemComponent;
	TArray<struct GameplayAbilitySpecHandle> GrantedAbilityHandles;
	TArray<struct ActiveGameplayEffectHandle> AppliedEffectHandles;
	TArray<struct Guid> ItemGuidsForAdditionalItems;
};

struct FBuildingGameplayActorAbilityDeliveryBucket
{
	struct GameplayTag Tag;
	TArray<struct ProximityBasedGEDeliveryInfoHard> ProximityEffectBuckets;
	TArray<struct FortAbilitySetDeliveryInfo> PawnPersistentAbilitySetBuckets;
	TArray<struct FortAbilitySetHandle> PersistentlyAppliedAbilitySets;
	bool bEnabled;
	bool bEnabledByDefault;
	bool bHasGEsToApplyOnTouch;
	bool bHasGEsToApplyOnExit;
	bool bHasGEsToApplyOnPulseTimer;
	bool bHasPersistentEffects;
};

struct FBuildingGameplayActorAbilityDeliveryInfo
{
	TArray<struct BuildingGameplayActorAbilityDeliveryBucket> DeliveryBuckets;
	struct ScalableFloat ProximityPulseInterval;
	struct ScalableFloat ProximityPrePulseTime;
	bool bHasGEsToApplyOnTouch;
	bool bHasGEsToApplyOnExit;
	bool bHasGEsToApplyOnPulseTimer;
	bool bHasPersistentEffects;
	class UClass* OwningActor;
	TArray<class UClass*> DeferredTouchActorsToProcess;
};

struct FVehicleSafeTeleportInfo
{
	struct Vector Location;
	struct Rotator Rotation;
};

struct FPredictionReplicationProxy_AP
{
	int ClientFrameNumber;
	int ServerFrameNumber;
	TArray<byte> Data;
};

struct FPredictionReplicationProxy_SP
{
	int ServerFrameNumber;
	TArray<byte> Data;
};

struct FCachedWaterData
{
	struct Vector PlaneLocation;
	struct Vector PlaneNormal;
	struct Vector SurfacePosition;
	struct Vector WaterVelocity;
	float Depth;
	int WaterBodyIndex;
};

struct FIgnoredPawn
{
	class UClass* Pawn;
	float Time;
};

struct FPredictedDestroyedBuilding
{
	class UClass* Building;
	float Time;
};

struct FFortAthenaVehicleInputState
{
	float ForwardAlpha;
	float RightAlpha;
	float PitchAlpha;
	float LookUpDelta;
	float TurnDelta;
	float SteerAlpha;
	float GravityOffset;
	struct Vector MovementDir;
	bool bIsSprinting;
	bool bIsJumping;
	bool bIsBraking;
	bool bIsHonking;
	bool bIgnoreForwardInAir;
	bool bMovementModifier0;
	bool bMovementModifier1;
	bool bMovementModifier2;
};

struct FFortRechargingActionTimer
{
	float ChargeRate;
	float ActiveExpenseRate;
	float PassiveExpenseRate;
	float MinActiveDuration;
	float MinActivationCharge;
	float ActiveCooldownTime;
	float ChargeThreshold;
	float Charge;
	bool bIsActive;
	bool bIsCharging;
	bool bIsPassive;
};

struct FVehicleSpringInfo
{
	struct FName SpringStart;
	struct Vector SpringStartLocalOffset;
	struct FName ForceApplicationPoint;
	float SpringLength;
	float SpringStiff;
	float SpringDamp;
	float SpringRadius;
	float MaxAccelChange;
	int8_t SeatSocketIndex;
	bool bIsLookAhead;
	bool bNormalToGroundTriangle;
	bool bForceAlongSpringGroundNormal;
	float LookAheadMinSpeed;
	float LookAheadMaxSpeed;
	float LookAheadMinStiff;
	float LookAheadMaxStiff;
	struct Transform LocalStartTM;
	struct Transform LocalApplyTM;
	struct HitResult Hit;
	struct Plane GroundPlane;
	float SprungMass;
	bool bEnabled;
	struct Vector Start;
	struct Vector End;
	struct Vector RayDir;
	struct Vector ForcePt;
	struct Vector ForceDir;
};

struct FSpringGroundTriangle
{
	struct FName Socket0;
	struct FName Socket1;
	struct FName Socket2;
};

struct FSMVehicleGear
{
	float TopSpeed;
	float MinSpeed;
	float PushForce;
	float RampTime;
	float SteeringAngleMultiplier;
	bool bAutoBrake;
	bool bIgnoreGravity;
};

struct FSeatTransitionMontage
{
	class UClass* Montage;
	int FromSeatIndex;
	int ToSeatIndex;
	bool bUseFromSeatIndex;
	bool bUseToSeatIndex;
};

struct FVehicleRuntimeModifiers
{
	byte DataVersion;
	bool bHasInfiniteFuel;
	bool bSupportCosmeticWrap;
};

struct FAthenaVehicleShootingCone
{
	float YawConstraint;
	float PitchConstraint;
};

struct FActionDefForUI
{
	struct FName InputAction;
	struct FName GamepadInputAction;
	struct FText ActionLabel;
};

struct FAthenaCarPlayerSlot
{
	struct FName SeatSocket;
	struct FName SeatChoiceSocket;
	struct FName SeatIndicatorSocket;
	struct FText SeatChoiceDisplayText;
	struct FName SeatCollision;
	TArray<struct FName> ExitSockets;
	struct AthenaVehicleShootingCone ShootingCone;
	class UClass* SoundOnEnter;
	class UClass* SoundOnExit;
	class UClass* AnimInstanceOverride;
	class UClass* AnimLayerOverride;
	bool bUsePerSeatAnimInstanceOverride;
	bool bIsSelectable;
	bool bUseGroundMotion;
	bool bUseVehicleIsOnGround;
	bool bCanEmote;
	bool bCanCarryDBNOPlayer;
	bool bForceCrouch;
	bool bPlayEnterSoundForTransition;
	bool bPlayExitSoundForTransition;
	bool bIsPushDriver;
	bool bCanOnlyFireWhenTargeting;
	struct Vector ActorSpaceCameraOffset;
	struct Vector VehicleSpaceCameraOffset;
	float SlopeCompensationCameraOffset;
	struct Vector StandingFiringOffset;
	struct Vector CrouchingFiringOffset;
	struct Vector EmoteOffset;
	class UClass* Player;
	Unknown ControllerUser;
	float PlayerEntryTime;
	float EnterSeatTime;
	bool bConstrainPawnToSeatTransform;
	bool bConstrainPawnToSeatDuringTransitionMontage;
	bool bOffsetPlayerRelativeAttachLocation;
	bool bUseExitTimer;
	class UClass* WeaponComponent;
	bool bIsMountedWeaponOnlySeat;
	float CameraPitchConstraint;
	float CameraYawConstraint;
	bool bReserved;
	TArray<struct ActionDefForUI> ActionDefForUI;
	ESlotEnvironmentExposure EnvironmentExposure;
};

struct FAthenaCarPlayerSlotUnreplicated
{
	class UClass* Input;
};

struct FReplicatedAthenaVehicleAttributes
{
	float FrontLateralFrictionScale;
	float RearLateralFrictionScale;
	float BrakeForceTractionScale;
	float ForwardForceTractionScale;
	float SlopeAntigravityScale;
	float TopSpeedScale;
	float VehicleGravityScale;
};

struct FVehicleDamageablePart
{
	int ConfigIndex;
	int BoneIndex;
	int ShapeIndex;
	float Health;
};

struct FVehicleToggleablePart
{
	int ShapeIndex;
	bool bEnabled;
};

struct FVehicleTargetOrientation
{
	struct Vector UpVector;
	struct Vector ForwardVector;
	struct Vector Location;
};

struct FVehicleCosmeticInfo
{
	class UClass* MostRecentCosmeticSourcePawn;
	class UClass* ActiveCosmeticItem;
	class UClass* PawnAssociatedWithWrap;
	class UClass* ActiveCosmeticWrap;
	TArray<class UClass*> SpawnedCosmeticComponents;
	class UClass* ItemWrapModifier;
};

struct FSphericalDriveParams
{
	float Radius;
	float MaxSpeedKmh;
	float LowSpeedAccelerationForce;
	float HighSpeedAccelerationForce;
	float MaxInclineDeg;
	float MaxAirControlForce;
	float MaxAirControlSpeedKmh;
	float ShellAngularDrag;
	float DragCoefficient;
	float DragCoefficientAutoBrake;
	float MaxAutoBrakeSpeedKmh;
	float ContactRepulsionForce;
	float ContactThreshold;
	float MassDirectionMaxAngleDeg;
	float MassDirectionStiffness;
	bool bMassDirectionInvert;
	float TractionMultiplier;
	float BounceForce;
};

struct FTowhookParams
{
	float MaxCableLength;
	float MinCableLength;
	bool bUseSpring;
	bool bUseRope;
	bool bUseAsTractorBeam;
	float SpringStiffness;
	float SpringDamping;
	float SpringMaxStiffnessForce;
	float SpringMaxStiffnessVelocity;
	float SpringDeformationRate;
	float RopeGive;
	float RopeBreakForce;
	float RopeYankForce;
	float ExtendSpeedKmh;
	float ContractSpeedKmh;
	bool bApplySpringToSelf;
	bool bApplySpringToOther;
	bool bTakeUpSlack;
	bool bEnableParentDominates;
	bool bAttachToOwnersVehicle;
	float TractorBeamRestLength;
	float TractorBeamUpSpringStiffness;
	float TractorBeamSideSpringStiffness;
	float TractorBeamUpSpringDamping;
	float TractorBeamSideSpringDamping;
	float TractorBeamMaxStiffnessTime;
	struct ScalableFloat TractorBeamHeightModifierLength;
	struct ScalableFloat TractorBeamHeightModifierStartPositionExtraDistance;
	struct ScalableFloat TractorBeamHeightScalingForceMagnitude;
};

struct FNetTowhookAttachState
{
	class UClass* Component;
	struct Vector_NetQuantize100 LocalLocation;
	struct Vector_NetQuantize100 LocalNormal;
};

struct FFortTowhookModel
{
};

struct FSphericalDriveContact
{
	struct Vector Location;
	struct Vector Normal;
	struct Vector Impulse;
	float Traction;
	Unknown Component;
	bool bIsBouncy;
};

struct FFortGameplayCueSpawnCondition
{
	EFortGameplayCueSourceCondition SourceCondition;
	TArray<EPhysicalSurface> AllowedSurfaces;
	TArray<EPhysicalSurface> DisallowedSurfaces;
	float ChanceToPlay;
	EParticleSignificanceLevel Significance;
	int RequiredDetailMode;
	bool bRequireVisible;
};

struct FFortGameplayCueAttachInfo
{
	struct FName SocketName;
	EFortGameplayCueAttachType AttachType;
	bool bAttachToWeapon;
	bool bAttachToHitResult;
	bool bUseUnsmoothedNetworkPosition;
	bool bIgnoreScale;
	bool bIgnoreRotation;
	struct Vector OverrideScale;
	struct Rotator OverrideRotation;
};

struct FFortGameplayCueParticleInfo
{
	struct FortGameplayCueSpawnCondition Condition;
	struct FortGameplayCueAttachInfo Attachment;
	EFXType FXType;
	class UClass* NiagaraSystem;
	class UClass* ParticleSystem;
	bool bCastShadow;
	bool bOverrideCondition;
	bool bOverrideAttachment;
};

struct FFortGameplayCueAudioInfo
{
	struct FortGameplayCueSpawnCondition Condition;
	struct FortGameplayCueAttachInfo Attachment;
	class UClass* SoundCue;
	float DelayBeforePlayInSeconds;
	bool bOverrideCondition;
	bool bOverrideAttachment;
};

struct FFortGameplayCueAOEInfo
{
	float InnerRadius;
	float OuterRadius;
};

struct FFortGameplayCueCameraShakeInfo
{
	class UClass* Shake;
	float Scale;
	ECameraShakePlaySpace Playspace;
	struct Rotator UserPlaySpaceRotation;
	bool bAlwaysPlayOnTarget;
	bool bCalculateUserPlaySpaceRotationFromLocation;
	bool bCancelOnRemove;
	struct FortGameplayCueAOEInfo Falloff;
};

struct FFortGameplayCueCameraLensEffectInfo
{
	class UClass* CameraLensEffect;
	struct FortGameplayCueAOEInfo Falloff;
	bool bAlwaysPlayOnTarget;
	bool bCancelOnRemove;
};

struct FFortGameplayCueForceFeedbackInfo
{
	class UClass* ForceFeedbackEffect;
	float EffectRadius;
	class UClass* FarForceFeedbackEffect;
	float FarEffectRadius;
	struct FName EffectTag;
	bool bAlwaysPlayOnTarget;
};

struct FFortGameplayCueDecalInfo
{
	struct FortGameplayCueSpawnCondition Condition;
	struct FortGameplayCueAttachInfo Attachment;
	bool bOverrideCondition;
	bool bOverrideAttachment;
	bool bOverrideFadeOut;
	class UClass* Decal;
	float FadeOutStartDelay;
	float FadeOutDuration;
};

struct FFortBurstEffectData
{
	TArray<struct FortGameplayCueParticleInfo> BurstParticles;
	TArray<struct FortGameplayCueAudioInfo> BurstSounds;
	struct FortGameplayCueCameraShakeInfo BurstCameraShake;
	struct FortGameplayCueCameraLensEffectInfo BurstCameraLensEffect;
	struct FortGameplayCueForceFeedbackInfo BurstForceFeedbackEffect;
	struct FortGameplayCueDecalInfo BurstDecal;
};

struct FFortGameplayCueSpawnResult
{
	TArray<class UClass*> ParticleComponents;
	TArray<class UClass*> AudioComponents;
	class UClass* CameraShake;
	Unknown CameraLensEffect;
	class UClass* DecalActor;
};

struct FInterpOffsetData
{
	struct Vector ViewOffset;
	struct Vector LargeBodyTypeAddtnlOffset;
	float PitchAngle;
};

struct FInterpOffset
{
	TArray<struct InterpOffsetData> SamplePoints;
};

struct FPenetrationAvoidanceFeeler
{
	struct Rotator AdjustmentRot;
	float WorldWeight;
	float PawnWeight;
	float Extent;
	int TraceInterval;
	int FramesUntilNextTrace;
};

struct FFortAnimInput_VelocityImpact
{
	struct Vector LastVelocity;
	struct Vector DeltaVelocityThreshold;
	struct Vector ImpactScale;
	struct InputRange ImpactLimitX;
	struct InputRange ImpactLimitY;
	struct InputRange ImpactLimitZ;
	struct FloatRK4SpringInterpolator SpringInterpolatorX;
	struct FloatRK4SpringInterpolator SpringInterpolatorY;
	struct FloatRK4SpringInterpolator SpringInterpolatorZ;
	bool bTestVelocity;
	struct Vector TestVelocity;
	bool bIsForwardImpact;
	bool bIsBackwardImpact;
	bool bIsLeftImpact;
	bool bIsRightImpact;
	bool bIsUpImpact;
	bool bIsDownImpact;
};

struct FFortAnimInput_SocketBasedIKTarget
{
	struct FName SocketName;
	struct FName AlphaCurveName;
	struct Rotator TargetRotation;
	struct Vector TargetLocation;
	float Alpha;
};

struct FFortAnimInput_VehicleDriverAnimAsset
{
	class UClass* DriveNPose;
	class UClass* DriveEPose;
	class UClass* DriveWPose;
	class UClass* DriveNAdditivePose;
	class UClass* DriveFastAdditivePose;
	class UClass* OverrideDriverPose;
	class UClass* DriveIdle;
	class UClass* DriveIdleFastAdditive;
	class UClass* DriverHeadAimOffset;
	class UClass* DriveNStart;
	class UClass* Braking;
	class UClass* BoostStart;
	class UClass* BoostLoop;
	class UClass* ReverseStart;
	class UClass* ReverseLoop;
	class UClass* ReverseEnd;
	class UClass* ReturnToIdleTransition;
	class UClass* PoseCorrectionAdditive;
	class UClass* CollisionN;
	class UClass* CollisionS;
	class UClass* CollisionE;
	class UClass* CollisionW;
};

struct FVehicleDamageablePartConfig
{
	struct FName ShapeName;
	struct FName BoneName;
	float MaxHealth;
	bool bStartsDisabled;
};

struct FFortPhysicsVehicleDamageOverrideConfigs
{
	struct GameplayTag Tag;
	struct ScalableFloat MaxDamageOverride;
	struct ScalableFloat MinDamageOverride;
	struct ScalableFloat MinimumSpeed;
	struct ScalableFloat SpeedForMaximumDamage;
	struct ScalableFloat SpeedForMinimumDamage;
};

struct FUIExtension
{
	EUIExtensionSlot Slot;
	struct TSoftClassPtr<UObject> WidgetClass;
};

struct FFortGameFeatureDataBaseCurveTableOverrides
{
	struct TSoftClassPtr<UObject> BaseTable;
	Unknown PlaylistToOverrideTableMap;
};

struct FFortGameFeatureDataBaseDataTableOverrides
{
	struct TSoftClassPtr<UObject> BaseTable;
	Unknown PlaylistToOverrideTableMap;
};

struct FFortGameFeatureLootTableData
{
	struct TSoftClassPtr<UObject> LootTierData;
	struct TSoftClassPtr<UObject> LootPackageData;
};

struct FGoalSelectionCriteria
{
	class UClass* GoalSelectionQuery;
};

struct FPawnGoalSelectionCriteria
{
	struct GameplayTagContainer IncludeEnemiesWithTags;
	struct GameplayTagContainer ExcludeEnemiesWithTags;
	TArray<struct GoalSelectionCriteria> GoalSelectionCriteria;
};

struct FPawnGoalSelectionTableEntry
{
	struct GameplayTagContainer RequiredGameplayTags;
	struct PawnGoalSelectionCriteria PawnGoalSelectionCriteria;
};

struct FFortGameFeatureItemsToFullyLoadData
{
	struct GameplayTagContainer PlaylistTags;
	TArray<class UClass*> ItemsToFullyLoad;
};

struct FFortAnimInput_SpeedWarping
{
	class UClass* PlayRateAdjustmentCurve;
	struct Vector2D SpeedWarpingLimits;
	struct Vector2D SpeedWarpingLimitsAddlRateScale;
	float SpeedWarpingAmount;
	float PlayRate;
};

struct FFortAnimInput_ShoppingCart
{
	bool bIsUsingShoppingCart;
	bool bIsUsingVehicle;
	ECoastState CoastState;
	bool bIsCoastStatePedaling;
	bool bIsCoastStateCoasting;
	bool bIsCoastStateDismount;
	bool bIsCoastStateIdle;
	bool bIsInAir;
	bool bIsCoasting;
	bool bIsPedaling;
	bool bIsReadyToPedal;
	float IsReadyToPedal;
	bool bWantsToCoast;
	bool bIsCoastIdling;
	bool bIsStartCoasting;
	bool bIsEndCoasting;
	bool bIsDismountingFromCoast;
	bool bIsCoastingOrDismountingFromCoast;
	bool bIsStandingInPlace;
	bool bIsSprinting;
	bool bIsSprintingAndMovingForward;
	bool bIsMovingForwardNotSprinting;
	bool bIsBraking;
	bool bIsReversing;
	bool bIsMoving;
	bool bIsMovingForward;
	bool bIsMovingBackwards;
	bool bIsMovingOrTurningInPlace;
	bool bIsInAirSteady;
	bool bIsOnSlope;
	bool bAimFWD;
	bool bAimBWD;
	bool bAimLFT;
	bool bAimRGT;
	float ForwardVelocity;
	float ForwardSpeedKmH;
	float CurrentBrakeForce;
	float RunForwardAlpha;
	bool bIsAcceleratingForward;
	bool bIsAccelBreakingOrReversing;
	float SteerAngle;
	float SteerAngleInterpSpeed;
	float CoastSteerAngleInterpSpeed;
	float IsReadyToPedalInterpSpeed;
	float StandingInPlaceSteerAngle;
	float SlopePitchDegreeAngle;
	float SlopeRollDegreeAngle;
	float PawnToVehicleDeltaYawAngleDegrees;
	float AimCardDirDeadZoneAngleDegrees;
	float AimCardDirAngleOffsetDegrees;
	float AimFWDDeltaAngleDegrees;
	float AimBWDDeltaAngleDegrees;
	float AimLFTDeltaAngleDegrees;
	float AimRGTDeltaAngleDegrees;
	int LastCardDirIndex;
};

struct FFortAnimInput_GolfCart
{
	bool bIsUsingGolfCart;
	bool bIsDriver;
	bool bIsFrontPassenger;
	bool bIsBackLeftPassenger;
	bool bIsBackRightPassenger;
	bool bIsFrontPassengerAndLeaning;
	bool bIsBackPassengerAndLeaning;
	bool bIsDrifting;
	bool bIsBoosting;
	bool bIsEBraking;
	bool bIsReversing;
	bool bIsBraking;
	bool bIsMoving;
	bool bIsMovingForward;
	bool bIsPowerSliding;
	bool bIsLeaning;
	bool bIsLeaningOrBouncing;
	bool bIsBounceCrouching;
	bool bIsBounceCrouched;
	bool bIsBounceJumping;
	bool bIsBounceRecoiling;
	bool bIsSteeringRight;
	bool bIsSteeringLeft;
	float RunForwardAlpha;
	float BounceCompression;
	struct Vector LeanPosition;
	float LeanPositionX;
	float LeanPositionY;
	float LeanPositionZ;
	bool bAimFWD;
	bool bAimBWD;
	bool bAimLFT;
	bool bAimRGT;
	float PawnToVehicleDeltaYawAngleDegrees;
	float AimCardDirDeadZoneAngleDegrees;
	float AimCardDirAngleOffsetDegrees;
	int LastCardDirIndex;
	float AimFWDDeltaAngleDegrees;
	float AimBWDDeltaAngleDegrees;
	float AimLFTDeltaAngleDegrees;
	float AimRGTDeltaAngleDegrees;
	float SlopePitchDegreeAngle;
	float SlopeRollDegreeAngle;
	float SteerAngle;
};

struct FFortAnimInput_Quad
{
	bool bIsUsingQuad;
	bool bIsDriver;
	bool bIsFrontPassenger;
	bool bIsBackPassenger;
	bool bIsBackPassengerAndLeaning;
	bool bIsDrifting;
	bool bIsBoosting;
	bool bIsReversing;
	bool bIsBraking;
	bool bIsMoving;
	bool bIsMovingForward;
	bool bIsLeaning;
	bool bIsLeaningOrBouncing;
	bool bIsBounceCrouching;
	bool bIsBounceCrouched;
	bool bIsBounceJumping;
	bool bIsBounceRecoiling;
	bool bIsSteeringRight;
	bool bIsSteeringLeft;
	float RunForwardAlpha;
	float BounceCompression;
	struct Vector LeanPosition;
	float LeanPositionX;
	float LeanPositionY;
	float LeanPositionZ;
	float VerticalVelocity;
	float VerticalAcceleration;
	bool bAimFWD;
	bool bAimBWD;
	bool bAimLFT;
	bool bAimRGT;
	float PawnToVehicleDeltaYawAngleDegrees;
	float AimCardDirDeadZoneAngleDegrees;
	float AimCardDirAngleOffsetDegrees;
	int LastCardDirIndex;
	float AimFWDDeltaAngleDegrees;
	float AimBWDDeltaAngleDegrees;
	float AimLFTDeltaAngleDegrees;
	float AimRGTDeltaAngleDegrees;
	float SlopePitchDegreeAngle;
	float SlopeRollDegreeAngle;
	float SteerAngle;
	float SteerAlpha;
	float SteerAngleDeadZoneDegrees;
	float SteeringRotation;
	float VehiclePitch;
	float VehicleRoll;
};

struct FFortAnimInput_MountedTurret
{
	bool bIsUsingMountedTurret;
	float AimingYaw;
	float AimingPitch;
	float PedalScaler;
};

struct FFortAnimInput_FerretVehicle
{
	bool bIsUsingFerretVehicle;
	bool bIsDriver;
	bool bIsFrontPassenger;
	bool bIsBackLeftPassenger;
	bool bIsBackRightPassenger;
	bool bIsFrontPassengerAndLeaning;
	bool bIsBackPassengerAndLeaning;
	bool bIsDrifting;
	bool bIsBoosting;
	bool bIsReversing;
	bool bIsBraking;
	bool bIsMoving;
	bool bIsMovingForward;
	bool bIsLeaning;
	bool bIsLeaningOrBouncing;
	bool bIsBounceCrouching;
	bool bIsBounceCrouched;
	bool bIsBounceJumping;
	bool bIsBounceRecoiling;
	byte bIsSteeringRight;
	byte bIsSteeringLeft;
	byte bIsShooting;
	byte bIsFerretPassengerRotating;
	float RunForwardAlpha;
	float BounceCompression;
	struct Vector LeanPosition;
	float LeanPositionX;
	float LeanPositionY;
	float LeanPositionZ;
	bool bAimFWD;
	bool bAimBWD;
	bool bAimLFT;
	bool bAimRGT;
	float PawnToVehicleDeltaYawAngleDegrees;
	float AimCardDirDeadZoneAngleDegrees;
	float AimCardDirAngleOffsetDegrees;
	int LastCardDirIndex;
	float AimFWDDeltaAngleDegrees;
	float AimBWDDeltaAngleDegrees;
	float AimLFTDeltaAngleDegrees;
	float AimRGTDeltaAngleDegrees;
	float SlopePitchDegreeAngle;
	float SlopeRollDegreeAngle;
	float SteerAngle;
	struct Vector SeatSwitchDirection;
};

struct FFortAnimInput_Zipline
{
	bool bIsZiplining;
	bool bShouldPlayPivotTransition;
	float LeanYaw;
	float PivotBlendDelayRemaining;
	float PivotBlendDelay;
	EFortCardinalDirection PivotCardinalDirection;
	struct Vector WorldVelocityLastTick;
};

struct FFortAnimInput_Ragdoll
{
	bool bIsRagdolling;
	bool bSliding;
	bool bTumbling;
	bool bStopped;
	struct Vector2D Impact;
	struct Vector Facing;
};

struct FFortAnimInput_JackalVehicle
{
	bool bIsUsingJackalVehicle;
	bool bIsSteeringLeft;
	bool bIsSteeringRight;
	float SteerAngleDeadZoneDegrees;
	float SteerAngle;
	float LeanYaw;
	float QuantizedSteerAngle;
	float SteerAlpha;
	float RunForwardAlpha;
	float SlopePitchDegreeAngle;
	float SlopeRollDegreeAngle;
	float DistanceFromGround;
	float PivotPlayRate;
	bool bIsReversing;
	bool bIsBraking;
	bool bIsMoving;
	bool bIsMovingForward;
	bool bIsSprinting;
	bool bInAir;
	bool bIsFalling;
	bool bIsJumping;
	bool bIsRelaxed;
	bool bIsBoosting;
	bool bHasReachedJumpChargeStartThreshold;
	bool bHasReachedJumpChargeMidThreshold;
	bool bHasReachedJumpChargeFullThreshold;
	bool bAdjustRootForFemaleRider;
	bool bIsPlayingEmoteOnHoverboard;
	bool bShouldApplyAdditive;
	bool bPlayPivotOnGroundAndNotBoosting;
	bool bIsOnLowerHill;
	bool bLowerBodyIdleToLoopTransition;
	bool bInterruptHoverboardFullybody;
	bool bEnterFullBodyHoverboardState;
	bool bDefaultToJumpStartTransition;
	bool bLocomotionPoseToJumpTransition;
	bool bBoostingToBoostingJumpTransition;
	bool bJumpToLocomotionPoseTransition;
	bool bJumpApexToJumpFallTransition;
	bool bIdleToMovementStartTransition;
	bool bIdleToMovementLoopTransition;
	bool bMovementLoopToMovementStopTransition;
	bool bMovementLoopToPivotTransition;
	bool bMovementLoopToIdleTransition;
	bool bIdleAdditiveToCollisionNTransition;
	bool bSplitBodyToHoverboardBRTransition;
	bool bHoverboardBRToSplitBodyTransition;
	bool bHoverboardBRMovementToJumpChargeTransition;
	bool bIdlesToJackalVehicleTransition;
	bool bPlayAdditiveLeans;
	bool bPlayBalloonLeans;
	bool bPlayJumpTrickVertical;
	bool bPlayJumpTrick;
	bool bPlayMovingFast;
	bool bPlayHipAdjustmentAdditive;
	bool bPlayDriveSouth;
	bool bPlayHeadAimOffset;
	bool bPlaySlopeAimOffset;
	float JumpCombatAdditiveWeight;
	float MeleeTwistIdle;
	float MeleeTwistLocomotionLoop;
	struct FortAnimInput_VelocityImpact VelocityImpact;
	struct Vector ImpactDisplacement;
	float SteerYaw;
	struct Vector EmoteHoverboardPosition;
	struct Rotator EmoteHoverboardRotation;
	bool bShouldAttachFeetToHoverboard;
	struct Vector FootLeftLocationOffset;
	struct Rotator FootLeftRotationOffset;
	struct Vector FootRightLocationOffset;
	struct Rotator FootRightRotationOffset;
	float StoppedThreshold;
	float MovingForwardThreshold;
	float MovingFowardFastThreshold;
	float DefaultToJumpStartTransitionThreshold;
	float JumpTrickAngularVelocityThreshold;
	float JumpDistanceFromGroundThreshold;
	float VelocityStartThreshold;
	float MovingFastThreshold;
	float RelaxedSpeedThreshold;
	float JumpChargeStartThreshold;
	float JumpChargeMidThreshold;
	float JumpChargeFullThreshold;
	float RotatingAngularVelocityThreshold;
	float IdleToLoopTransitionSpeedThreshold;
	float IdleToMovementLoopTransitionThreshold;
	float MeleeTwistIdleMultiplier;
	float MeleeTwistLocomotionLoopMultiplier;
	float LeanYawForMaxPivotPlayRate;
	float SteerAlphaForMaxPivotPlayRate;
};

struct FFortAnimInput_OctopusVehicle
{
	bool bIsUsingOctopusVehicle;
};

struct FFortAnimInput_CommonVehicle
{
	bool bIsUsingVehicle;
	bool bIsJumpingVehicle;
	bool bCanChargeJump;
	bool bIsChargingJump;
	bool bIsJumping;
	bool bIsOnGround;
	bool bCanDriverAimWeapon;
};

struct FFortAnimInput_CommonWeapon
{
	bool bIsWeaponEquipped;
	bool bForceUpperBodyTargeting;
};

struct FFortAnimInput_BowWeapon
{
	class UClass* BowChargeSpeedModifierCurve;
	struct Transform RightHandIKExtraOffset;
	struct Vector RightHandIKOffsetLocation;
	struct Rotator RightHandIKOffsetRotation;
	struct FName WeaponAttachmentSocketName;
	struct FName ArrowNockSocketName;
	float BowAimYaw;
	float BowAimPitch;
	float ChargeBlendSpaceCrouchSpeed;
	float ChargeBlendSpaceCrouchSpeedInterpRate;
	float ChargeBlendSpaceCrouchSpeedTarget;
	float ChargePlayRate;
	float ChargeBlendInTime;
	float FastChargeThreshold;
	float ChargeBlendInTimeDefault;
	float ChargeBlendInTimeFastCharge;
	float WeaponRaisedAdditiveLeanMultiplier;
	bool bIsBowEquipped;
	bool bIsBowCharging;
	bool bIsBowAtMaxCharge;
	bool bEnableRightHandIK;
};

struct FFortAnimInput_CreativeMoveTool
{
	bool bIsFlying;
};

struct FFortAnimInput_DBNOCarried
{
	struct CachedAnimStateData DropStateData;
	struct FloatSpringState CarrierYawSpringState;
	struct FloatSpringState CarrierSpinePitchSpringState;
	class UClass* CarrierDropMontage;
	class UClass* CarrierPickupMontage;
	class UClass* CarrierPawn;
	class UClass* CarrierAnimBP;
	class UClass* CarrierInterrogationMontage;
	struct Vector LHandIKLocation;
	struct Rotator LHandIKRotation;
	struct Vector TempAttachLocation;
	struct Rotator TempAttachRotation;
	float SubAnimPhysicsWeight;
	float DropMontagePosition;
	float PickupMontagePosition;
	float InterrogationMontagePosition;
	float PickupToIdleTransitionPosition;
	float CarrierPawnVelocityZ;
	float CarrierYawDeltaSmoothed;
	float CarriedJogNAnimPosition;
	float CarriedJogSAnimPosition;
	float CarriedCrouchNAnimPosition;
	float CarriedCrouchSAnimPosition;
	float CarriedSprintAnimPosition;
	float CarriedCrouchSprintAnimPosition;
	float CarrierSpinePitch;
	float DropStateWeight;
	bool bBeingCarried;
	bool bIsBeingPickedUp;
	bool bIsBeingDropped;
	bool bCarrierIsCrouching;
	bool bCarrierIsMovingBackward;
	bool bCarrierHasSyncMarkers;
	bool bTransitionFromPickupToIdle;
	bool bTransitionFromIdleToJogging;
	bool bTransitionFromJoggingToSprinting;
	bool bTransitionFromInAirToLanding;
	bool bJackalPlayJumpTrickVertical;
};

struct FFortAnimInput_RandomizeMontageSection
{
	class UClass* CurrentAnimation;
	struct FName MontageSectionName;
	float TimeUntilNextSectionChange;
	int CurrentSectionParamIndex;
};

struct FFortAnimInput_SlopeInfo
{
	struct Vector RootSlopeTranslation;
	struct Vector LeftFootSlopeTranslation;
	struct Vector RightFootSlopeTranslation;
	struct Rotator SlopeRotation;
};

struct FFortAnimInput_TransitionProperties
{
	bool bTransition_DoubleJump_Fast;
	bool bTransition_DoubleJump;
};

struct FFortAnimInput_TurnInPlace
{
	struct CachedAnimStateArray TurnInitiatingStates;
	struct CachedAnimStateArray TurnTransitionStates;
	struct FName TurnRotationAmountCurveName;
	struct FName AllowTurnInPlaceCurveName;
	struct Vector2D EmoteYawOffsetSofteningInputRange;
	struct Vector2D EmoteYawOffsetSofteningOutputRange;
	float TurnThreshold90;
	float InitialTurnCurveValue;
	float MaxAllowedRootYawOffset;
	float RootYawOffset;
	float RootYawOffsetAlpha;
	bool bWantsToTurn;
	bool bWantsToTurnInVehicle;
	bool bWantsToTurnAgain;
	bool bTurningLeft;
	float LastTurnRotationAmount;
};

struct FFortAnimInput_AdjustedAimOffset
{
	float YawOffset;
	float PitchOffset;
	float TargetingYawOffset;
	float TargetingPitchOffset;
};

struct FFortAnimInput_AdjustedAim
{
	struct FortAnimInput_AdjustedAimOffset WeaponOffsets;
	float YawOffset;
	float PitchOffset;
	float YawScale;
	float PitchScale;
	float ResultingYaw;
	float ResultingPitch;
	struct FName ZeroOutPitchWeightCurveName;
};

struct FFortAnimInput_SpringGlider
{
	struct FloatSpringState GliderXRotationRightSpringLeg;
	struct FloatSpringState GliderYRotationRightSpringLeg;
	struct FloatSpringState GliderZLocationSpring;
	struct FloatSpringState GliderXLocationSpring;
	struct FloatSpringState GliderYLocationSpring;
	struct Rotator GliderOffsetRotator;
	struct Rotator GliderOffsetRotationRightLeg;
	struct Rotator GliderOffsetRotationLeftLeg;
	struct Vector GliderOffsetLocation;
	float GliderYOffsetRotationRightLeg;
	float GliderYOffsetRotationLeftLeg;
	float GliderXOffsetRotationRightLeg;
	float GliderXOffsetRotationLeftLeg;
	float GliderZOffset;
	float GliderXOffset;
	float GliderYOffset;
};

struct FFortAnimInput_HandIK
{
	float IKAlphaOverrideInterpSpeed;
	float IKSpaceSwitchOverrideInterpSpeed;
	float HandIKRetargetingWeight;
	float HandsInRootSpaceAlpha;
	float LeftHandIKAlpha;
	float RightHandIKAlpha;
	EFortHandIKOverrideType LeftHandIKOverrideType;
	EFortHandIKOverrideType RightHandIKOverrideType;
};

struct FFortAnimInput_PelvisAdjustment
{
	class UClass* PawnMesh;
	float DistanceToFeet;
	float LegLength;
	float DesiredLegLengthTreshold;
	float DotProductBetweenUpVectorsTreshold;
	float PelvisInterpSpeed;
	float EmotePelvisOffsetInterpSpeed;
	float EmotePelvisOffsetAlpha;
	struct Vector CurrentEmotePelvisOffset;
	int PelvisBoneIndex;
	int FootLeftBoneIndex;
	int FootRightBoneIndex;
};

struct FFortAnimInput_Skydiving
{
	struct Vector ParaGilderRootOffset;
	struct Rotator ParaGliderRootRotationOverride;
	bool bUseParaGlideRootModifier;
	bool bIsSkydivingFromLaunchPad;
	bool bIsSkydivingFromBus;
	bool bIsInVortex;
	bool bIsUsingUmbrella;
	bool bIsActivelyStrafingInAir;
	bool bIsDiving;
	bool bIsDivingUpInVortex;
	bool bIsParachuteOpen;
	bool bIsSkydiving;
	bool bIsParachuteLeaning;
	bool bIsSkydiveLeaning;
	bool bIsLeaning;
	bool bIsSkydiveDiveMode;
	bool bParachuteLeanTransition;
	bool bPlayedParachuteLeanTransition;
	bool bPlaySkydiveDrift;
	bool bSkydiveDriftDelayActive;
	bool bSkydiveDriftAnimAllowed;
	bool bIsGliderRight;
	bool bIsGliderCenter;
	bool bIsGliderLeft;
	bool bIsGliderForward;
	bool bIsGliderBack;
	float LocalAccelForward;
	float LocalAccelRight;
	float LocalVelocityRight;
	float SkydiveAimPitch;
	float SkydiveAimPitchInterpSpeed;
	float SkydiveAimYaw;
	float DeployChuteAnimRate;
	float SkydiveDriftAnimRate;
	float SkydiveDriftAnimRateCurrent;
	float SkydiveFidgetAnimRate;
	float SkydiveFidgetAnimRateCurrent;
	float SkydiveAdditiveAlpha;
	float SkydiveDriftDelay;
	int SkydiveDriftAnim;
	int SkydiveDriftAnimMax;
	int LaunchpadAnim;
	ESkydivingDirection LocalAccelDir;
	ESkydivingDirection DirectionLast;
};

struct FFortAnimInput_StandingPawnAnimAsset
{
	class UClass* UpperBodyAdditivePoseOffset;
	class UClass* IdleNoise;
	class UClass* TargetingPose;
	class UClass* NonTargetingPose;
};

struct FFortAnimPlayrateRange
{
	float MinPlayrate;
	float MaxPlayrate;
};

struct FPelvisMod_VerticalInput_Spring
{
	float VerticalLeanForwardA;
	float VerticalLeanForwardB;
	float VerticalLeanForwardStiffness;
	float VerticalLeanForwardDamping;
	float VerticalLeanForwardMass;
};

struct FPelvisMod_LateralInput_Spring
{
	float LateralLeanStrafeA;
	float LateralLeanStrafeB;
	float LateralTurnA;
	float LateralTurnB;
	float LateralStiffness;
	float LateralDamping;
	float LateralMass;
};

struct FPelvisMod_BankAngleInput_Spring
{
	float BankLeanStrafeA;
	float BankLeanStrafeB;
	float BankTurnA;
	float BankTurnB;
	float BankStiffness;
	float BankDampening;
	float BankMass;
	float BankClampMin;
	float BankClampMax;
};

struct FThighMod_LegAngleInput
{
	float LegBankPitchForwardA;
	float LegBankPitchForwardB;
	float LegBankPitchLeanStrafeA;
	float LegBankPitchLeanStrafeB;
	float LegBankPitchTurnA;
	float LegBankPitchTurnB;
};

struct FThighMod_LegPitchSpring
{
	float LegPitchStiffness;
	float LegPitchDampening;
	float LegPitchMass;
	float LegPitchClampMin;
	float LegPitchClampMax;
};

struct FThighMod_LegBankSpring
{
	float LegBankStiffness;
	float LegBankDampening;
	float LegBankMass;
	float LegBankClampMin;
	float LegBankClampMax;
};

struct FThighMod_LeftLegDrag
{
	float LeftLegBankSpeed;
	float LeftLegPitchSpeed;
};

struct FFortAnimInput_PlayerGliderAnimAsset
{
	class UClass* Default_Base_BS;
	class UClass* Default_BodyAdditive_MaleMedium_BS;
	class UClass* Default_BodyAdditive_MaleLarge_BS;
	class UClass* Default_BodyAdditive_FemaleSmall_BS;
	class UClass* Default_BodyAdditive_FemaleMedium_BS;
	class UClass* Default_BodyAdditive_FemaleLarge_BS;
	class UClass* Default_TurnAdditive_MaleMedium_BS;
	class UClass* Default_TurnAdditive_MaleLarge_BS;
	class UClass* Default_TurnAdditive_FemaleSmall_BS;
	class UClass* Default_TurnAdditive_FemaleMedium_BS;
	class UClass* Default_TurnAdditive_FemaleLarge_BS;
	class UClass* Into_Base_BS;
	class UClass* Into_BodyAdditive_MaleMedium_BS;
	class UClass* Into_BodyAdditive_MaleLarge_BS;
	class UClass* Into_BodyAdditive_FemaleSmall_BS;
	class UClass* Into_BodyAdditive_FemaleMedium_BS;
	class UClass* Into_BodyAdditive_FemaleLarge_BS;
	class UClass* Into_TurnAdditive_MaleMedium_BS;
	class UClass* Into_TurnAdditive_MaleLarge_BS;
	class UClass* Into_TurnAdditive_FemaleSmall_BS;
	class UClass* Into_TurnAdditive_FemaleMedium_BS;
	class UClass* Into_TurnAdditive_FemaleLarge_BS;
	class UClass* Lean_MaleMedium_BS;
	class UClass* Lean_MaleLarge_BS;
	class UClass* Lean_FemaleSmall_BS;
	class UClass* Lean_FemaleMedium_BS;
	class UClass* Lean_FemaleLarge_BS;
	class UClass* LeanAdditive_Center_MaleMedium_Pose;
	class UClass* LeanAdditive_Center_MaleLarge_Pose;
	class UClass* LeanAdditive_Center_FemaleSmall_Pose;
	class UClass* LeanAdditive_Center_FemaleMedium_Pose;
	class UClass* LeanAdditive_Center_FemaleLarge_Pose;
	class UClass* LeanAdditive_Into_BS;
	class UClass* LeanAdditive_ForwardInto_Anim;
	class UClass* LeanAdditive_ForwardInto_FromDeploy_Anim;
	class UClass* LeanAdditive_BackInto_Anim;
	class UClass* LeanAdditive_LeftInto_Anim;
	class UClass* LeanAdditive_RightInto_Anim;
	class UClass* LeanAdditive_ForwardOut_Anim;
	class UClass* LeanAdditive_BackOut_Anim;
	class UClass* LeanAdditive_LeftOut_Anim;
	class UClass* LeanAdditive_RightOut_Anim;
	class UClass* ToGlide_BS;
	class UClass* ToGlide_Lean_BS;
	class UClass* ToDive_BS;
	class UClass* ToDive_Lean_BS;
	class UClass* Dive_WeaponR_Additive_BS;
	class UClass* Glide_WeaponR_Additive_Anim;
	class UClass* GenericAdditive_Male_BS;
	class UClass* GenericAdditive_Female_BS;
	float RootModPitchMin;
	float RootModPitchMax;
	float RootModYOffsetMin;
	float RootModYOffsetMax;
	struct PelvisMod_VerticalInput_Spring PelvisModVertical;
	struct PelvisMod_LateralInput_Spring PelvisModLateral;
	struct PelvisMod_BankAngleInput_Spring PelvisModBankAngle;
	struct ThighMod_LegAngleInput ThighModLegAngle;
	struct ThighMod_LegPitchSpring ThighModLegPitch;
	struct ThighMod_LegBankSpring ThighModLegBank;
	struct ThighMod_LeftLegDrag ThighModLeftLegDrag;
	EGliderType PlayerGliderType;
	bool bEnableSpringMods;
	bool bAllowPlayerDeployRootMod;
	bool bUseSurfStyle;
};

struct FFortAnimInput_WeaponAdditiveAnimAsset
{
	class UClass* TargetingAdditivePoseOffset;
	class UClass* NonTargetedAdditivePoseOffset;
	class UClass* RelaxedAdditivePoseOffset;
	class UClass* RelaxedWhileSprintingAdditiveAnim;
	class UClass* RelaxedWhileCrouchSprintingAdditiveAnim;
};

struct FGameplayTagAnimationData
{
	struct GameplayTagContainer GameplayTags;
	EFortCustomGender ValidGenders;
	struct TSoftClassPtr<UObject> AnimMontage;
};

struct FGameplayTagAnimations
{
	TArray<struct GameplayTagAnimationData> GameplayTagAnimData;
};

struct FMaterialInterfaceArray
{
	TArray<class UClass*> Materials;
};

struct FFortEffectDistanceQuality
{
	float MinDistanceCinematic;
	float MinDistanceEpic;
	float MinDistanceHigh;
	float MinDistanceMedium;
	float MinDistanceLow;
	bool bAllowCinematic;
	bool bAllowEpic;
	bool bAllowHigh;
	bool bAllowMedium;
	bool bAllowLow;
};

struct FFortEquippedWeaponModSlot
{
	struct FortAbilitySetHandle EquippedAbilitySetHandle;
	class UClass* EquippedWeaponMod;
};

struct FCosmeticVariantCache
{
	class UClass* ItemDefFor;
};

struct FWeaponHudData
{
	struct FName KeyActionName;
	struct FName GamepadActionName;
	struct FText ActionDescription;
	bool bVisible;
	struct FString KeyActionId;
};

struct FTaggedParticleSubstitution
{
	struct FName Tag;
	class UClass* Substitute;
};

struct FTaggedSoundSubstitution
{
	struct FName Tag;
	class UClass* Substitute;
};

struct FTaggedStaticMeshSubstitution
{
	struct FName Tag;
	class UClass* Substitute;
};

struct FLightProperty_Color
{
	bool bEnabled;
	struct LinearColor Color;
	bool bUsingSRGB;
};

struct FTaggedInLightProperties
{
	struct FName Tag;
	struct LightProperty_Color ColorSubstitute;
};

struct FAddOrRemoveGameplayTags
{
	struct GameplayTagContainer Added;
	struct GameplayTagContainer Removed;
};

struct FMeshSet
{
	float Weight;
	EFortResourceType ResourceType;
	bool bDoNotBlockBuildings;
	bool bDestroyOnPlayerBuildingPlacement;
	bool bNeedsDamageOverlay;
	class UClass* BaseMesh;
	class UClass* BreakEffect;
	class UClass* DeathParticles;
	struct FName DeathParticleSocketName;
	class UClass* DeathSound;
	class UClass* ConstructedEffect;
	TArray<struct TaggedParticleSubstitution> SwapInParticles;
	TArray<struct TaggedSoundSubstitution> SwapInSounds;
	TArray<struct TaggedStaticMeshSubstitution> SwapInMeshes;
	TArray<struct TaggedInLightProperties> SwapInLightProperties;
	struct AddOrRemoveGameplayTags BuildingOwnedTagDelta;
	class UClass* SearchedMesh;
	struct CurveTableRowHandle SearchSpeed;
	float LootNoiseRange;
	struct Vector LootSpawnLocation;
};

struct FTierMeshSets
{
	int Tier;
	TArray<struct MeshSet> MeshSets;
};

struct FFortBounceData
{
	float StartTime;
	float BounceValue;
	float Radius;
	struct LinearColor DeformationVector;
	struct LinearColor DeformationCenter;
	EFortBounceType BounceType;
	bool bLocalInstigator;
	bool bIsPlaying;
};

struct FBuildingNavObstacle
{
	struct Box LocalBounds;
	EBuildingNavObstacleType ObstacleType;
};

struct FEditorOnlyBuildingInstanceMaterialParameters
{
	TArray<struct ScalarParameterValue> ScalarParams;
	TArray<struct VectorParameterValue> VectorParams;
	TArray<struct TextureParameterValue> TextureParams;
};

struct FBuildingActorMinimalReplicationProxy
{
	int16_t Health;
	int16_t MaxHealth;
};

struct FChosenQuotaInfo
{
	int LootTier;
	struct FName LootTierKey;
};

struct FRandomDayphaseFX
{
	class UClass* ParticleSystem;
	TArray<class UClass*> AltParticleSystems;
	TArray<EFortDayPhase> RequiredDayphases;
	float ChanceToSpawnFX;
	EDetailMode DetailMode;
	float MaxDrawDistance;
	bool bRandomSelectionAlreadyHappened;
	class UClass* SpawnedComponent;
};

struct FAnimatingMaterialPair
{
	class UClass* Original;
	class UClass* Override;
};

struct FProxyGameplayCueDamage
{
	struct GameplayEffectContextHandle EffectContext;
	uint32_t ProxyGameplayCueDamageMagnitude;
};

struct FLootTierGroupTagOverride
{
	struct ScalableFloat IsEnabled;
	struct FName OverrideLootTierGroup;
	struct GameplayTagQuery PlayerTagQuery;
};

struct FRandomUpgradeCalendarData
{
	ECalendarDrivenState ReactionWhenEventIsPresent;
	struct FString EventName;
};

struct FRandomUpgradeData
{
	struct FName LootTierGroupIfApplied;
	struct ScalableFloat ChanceToApplyPerContainer;
	struct ScalableFloat Enabled;
	TArray<struct RandomUpgradeCalendarData> CalendarDrivenEnableState;
	TArray<struct MarkedActorDisplayInfo> MarkerDisplayInfoOverride;
	class UClass* SoundIndicatorIconOverride;
	struct LinearColor SoundIndicatorTintOverride;
};

struct FFortSearchBounceData
{
	struct Vector BounceNormal;
	uint32_t SearchAnimationCount;
	class UClass* SearchingPawn;
};

struct FFortPlayspaceMatchmakingSettings
{
	EFortPlayspaceMatchmakingRules MatchmakingRule;
	TArray<struct FName> SpecificPlaylists;
	TArray<struct FName> ExcludedPlaylists;
};

struct FAthenaGameMessageData
{
	EAthenaGameMsgType MsgType;
	struct FText MsgText;
	class UClass* MsgSound;
	float MsgDelay;
	bool bIsTeamBased;
	int TeamIndex;
	float DisplayTime;
	class UClass* TargetPlayerController;
};

struct FFortMutatorAudioStinger
{
	class UClass* SoundCue;
	float LoopTimeBeforeFade;
	float FadeTime;
};

struct FTeamMapInfo
{
	TArray<byte> ReplicatedSeedPack;
	byte TeamId;
};

struct FMapLocationRenderData
{
	struct SlateFontInfo Font;
};

struct FFortBotTargetInfo
{
	class UClass* SourceActor;
	class UClass* SupportingActor;
	class UClass* AlternateTargetingActor;
};

struct FFortBotTargetHandler
{
	TArray<struct FortBotTargetInfo> Targets;
};

struct FFortBotInventoryInfo
{
	class UClass* ItemDefinition;
	class UClass* FortItem;
};

struct FBotDelayedStimulus
{
	class UClass* SourceActor;
};

struct FFortBotThreatActorInfo
{
	class UClass* ThreatActor;
};

struct FAlertLevelInfo
{
	TArray<class UClass*> SensesConfig;
};

struct FDebugMinimapData
{
	bool bIsOverridden;
	struct SlateBrush DebugMinimapIconBrush;
	struct Vector2D DebugMinimapIconScale;
	struct SlateBrush DebugCompassIconBrush;
	struct Vector2D DebugCompassIconScale;
};

struct FAIHotSpotSlotConfig
{
	struct Vector Offset;
	struct Vector Direction;
	EFortHotSpotSlot SlotType;
};

struct FDataIntegrityPair
{
	class UClass* BotMutator;
	class UClass* BotPolicyData;
	class UClass* MutatorPawn;
	class UClass* AISpawnerData;
	class UClass* AISpawnerPawn;
};

struct FObjectCostVersion
{
	int MajorVersion;
	int64_t Timestamp;
	uint32_t MinorVersionStringHash;
};

struct FObjectIdentifier
{
	Unknown ContainedIdentifierHashes;
	uint32_t CachedHash;
	struct TSoftClassPtr<UObject> SoftClassPtr;
};

struct FObjectCostData
{
	Unknown PerMetricCosts;
};

struct FObjectCostCollection
{
	Unknown HashToIdentifier;
	Unknown ObjectIdentifiersToCostData;
};

struct FObjectCostContainer
{
	Unknown VersionedObjectCostCollections;
};

struct FAIHotSpotSlotInfo
{
	class UClass* HotSpot;
	int SlotIndex;
};

struct FIntensityContribution
{
	EFortCombatFactors CombatFactor;
	EFortAIDirectorFactor ContributingAIDirectorFactor;
	float MaxContribution;
	bool bModifyContributionByCompletionPercentage;
	struct CurveTableRowHandle CompletionPercentageInitialMultiplier;
	struct CurveTableRowHandle CompletionPercentageToStartReducingMultiplier;
	struct CurveTableRowHandle CompletionPercentageToStopReducingMultiplier;
	bool bModifyByNumberOfCriticalEncounterGoals;
};

struct FIntensityData
{
	TArray<struct IntensityContribution> ContributingFactors;
	float ContributionsTotal;
	TArray<class UClass*> ExceptionEditModes;
	float ExceptionEditModeWeight;
};

struct FFortAIEncounterPIDController
{
	float ProportionalGain;
	float IntegralGain;
	float DerivativeGain;
};

struct FFortAIEncounterPIDControllerSettings
{
	struct CurveTableRowHandle ProportionalGain;
	struct CurveTableRowHandle IntegralGain;
	struct CurveTableRowHandle DerivativeGain;
};

struct FUtilityContribution
{
	float MaxContribution;
	EFortCombatFactors ContributingFactor;
	EFortAIDirectorFactor ContributingAIDirectorFactor;
	EFortFactorContributionType ContributionType;
};

struct FUtilityData
{
	TArray<struct UtilityContribution> ContributingFactors;
	float ContributionsTotal;
	bool bApplyRecentSelectionPenalty;
	float RecentlySelectedPenaltyPercentage;
	float PenaltyFallOffRate;
	struct FString DebugGraphName;
	struct LinearColor DebugGraphColor;
};

struct FFortPlayerPerformanceEstimateSettings
{
	struct CurveTableRowHandle PlayerPerformanceEstimateTransformMin;
	struct CurveTableRowHandle PlayerPerformanceEstimateTransformOrigin;
	struct CurveTableRowHandle PlayerPerformanceEstimateTransformMax;
	float EncounterPlayerPerformanceWeight;
	float PreviousWavePlayerPerformanceWeight;
	float CampaignPlayerPerformanceWeight;
};

struct FFortAIEncounterSpawnGroupCap
{
	struct CurveTableRowHandle MinSpawnGroupNumberCap;
	struct CurveTableRowHandle MaxSpawnGroupNumberCap;
};

struct FFortAIEncounterSpawnGroupCapsCategory
{
	struct GameplayTagQuery TagQuery;
	bool bApplyGroupPopulationCurveToCategoryMax;
	struct CurveTableRowHandle InitialSpawnGroupAvailabilityDelaySeconds;
	struct CurveTableRowHandle SpawnGroupAvailabilityDelaySeconds;
	struct GameplayTagQuery UnlockingTagQuery;
	TArray<struct FortAIEncounterSpawnGroupCap> SpawnGroupCapsPerPlayerCount;
	float InitialSpawnGroupAvailabilityTime;
	float NumActiveCategorySpawnGroups;
	TArray<float> SpawnGroupAvailabilityTimes;
	int NumSpawnGroupAvailable;
	class UClass* CategorySource;
};

struct FFortAIEncounterSpawnGroupCapsProfile
{
	struct GameplayTagContainer EncounterTypeTags;
	TArray<struct FortAIEncounterSpawnGroupCapsCategory> PopulationCategories;
};

struct FFortAIEncounterSpawnPointsProfile
{
	struct GameplayTagContainer EncounterTypeTags;
	TArray<struct CurveTableRowHandle> MaxSpawnPointsPerPlayerCount;
	TArray<struct CurveTableRowHandle> MinSpawnPointsPerPlayerCount;
};

struct FFortAIEncounterPawnDifficultyLevelModifier
{
	struct GameplayTagQuery EncounterTagRequirementsQuery;
	struct CurveTableRowHandle DifficultyLevelModifierCurve;
};

struct FFortAISpawnGroupUpgradeData
{
	class UClass* SpawnGroupUpgrade;
	class UClass* UpgradeProbabilities;
	class UClass* SpawnGroupCapsCategories;
	struct GameplayTagQuery EncounterTagRequirementsQuery;
};

struct FEncounterEnvironmentQueryInfo
{
	class UClass* EnvironmentQuery;
	TArray<struct EnvNamedValue> QueryParams;
	bool bIsDirectional;
};

struct FFortAIEncounterRequirements
{
	EFortMissionType AssociatedMissionType;
	struct GameplayTagQuery TagQuery;
};

struct FFortAILootDropModifiers
{
	struct FortAIEncounterRequirements Requirements;
	TArray<struct DataTableRowHandle> LootDropModifierRows;
};

struct FFortEncounterPawnNumberCaps
{
	bool bApplyPawnNumberCaps;
	TArray<struct CurveTableRowHandle> PawnCapsPerPlayerCount;
};

struct FFortAISpawnGroupUpgradeUIData
{
	bool bAlwaysDisplayHealthBar;
	bool bOverrideHealthBarColor;
	struct TSoftClassPtr<UObject> UpgradeIconImage;
	struct SlateColor UpgradeIconTintColor;
	struct LinearColor HealthBarColorOverride;
	struct FText UpgradeName;
};

struct FFortAIPawnLootDropData
{
	float LootDropChance;
	struct FName WorldItemTierGroup;
	struct FName WorldItemInstancedTierGroup;
	struct FName AccountItemTierGroup;
};

struct FPendingSpawnInfo
{
	class UClass* PawnClassToSpawn;
	class UClass* SpawnPoint;
	struct Vector SpawnLocation;
	struct Rotator SpawnRotation;
	class UClass* SpawnSource;
	bool bSpawnedFromExternalSpawner;
	int SpawnSetIndex;
	EFortressAIType AIType;
	class UClass* TargetPlayer;
	class UClass* EncounterInfo;
	float DifficultyLevel;
	class UClass* SpawnGroup;
	struct Guid SpawnGroupGuid;
	int EnemyIndexInSpawnGroup;
	float TimeToSpawn;
	struct Guid PendingSpawnInfoGuid;
	bool bIgnoreCollision;
	bool bKillBuildingActorsAtSpawnLocation;
	float EncounterAILifespan;
	float ScoreMultiplier;
	bool bDebugSpawnedAI;
	TArray<class UClass*> AbilitySetsToGrantOnSpawn;
	TArray<class UClass*> ModifiersToApplyOnSpawn;
	struct FortAISpawnGroupUpgradeUIData UpgradeUIData;
	struct FortAIPawnLootDropData LootDropData;
};

struct FFortPendingStoppedEncounterData
{
	class UClass* Encounter;
	EFortObjectiveStatus ObjectiveStatus;
	bool bForceDestroyAI;
	bool bEncounterCompletedSuccessfully;
};

struct FUtilityTypeFloatPair
{
	EFortAIUtility Utility;
	float Value;
};

struct FDroppingAgentData
{
	class UClass* AIController;
	class UClass* MovementBase;
};

struct FFortServerBotInfo
{
	class UClass* BotController;
	class UClass* SelectedPoiVolume;
};

struct FMovingLootInfo
{
	struct Vector LastLocationInOctree;
};

struct FCachedSupplyDrop
{
	class UClass* supplydrop;
	bool bInOctree;
	struct Vector LastLocation;
};

struct FBattleBusPOI
{
	struct ScalableFloat IsEnabled;
	struct GameplayTagQuery POIFilterQuery;
	TArray<class UClass*> ValidPOIVolumeList;
};

struct FNavigationPOI
{
	struct ScalableFloat IsEnabled;
	struct GameplayTagQuery POIFilterQuery;
	TArray<class UClass*> ValidPOIVolumeList;
};

struct FConstructionBuildingInfo
{
	class UClass* BuildingActorClass;
};

struct FCachedPOIVolumeLocations
{
	class UClass* POIVolume;
};

struct FPlayerLODViewConeConfig
{
	struct ScalableFloat ObserverVisionAngleDeg;
	struct ScalableFloat ViewConeMaxRadius;
	struct ScalableFloat AlwaysVisibleRadius;
	EFortAILODLevel FortAILODLevel;
};

struct FPlayerLODViewConeHysteresisConfig
{
	struct ScalableFloat AdditionalObserverVisionAngleDeg;
	struct ScalableFloat AdditionalRadius;
};

struct FFortAIDirectorPerLODConfig
{
	struct ScalableFloat MaxNPCCosts;
	EFortAILODLevel FortAILODLevel;
};

struct FMcpVariantReader
{
	struct FString Channel;
	struct FString Active;
	TArray<struct FString> Owned;
};

struct FAthenaCosmeticMaterialOverride
{
	struct FName ComponentName;
	int MaterialOverrideIndex;
	struct TSoftClassPtr<UObject> OverrideMaterial;
};

struct FFortCosmeticVariantPreviewElement
{
	TArray<struct McpVariantChannelInfo> VariantOptions;
	class UClass* Item;
};

struct FFortCosmeticVariantPreview
{
	struct FText UnlockCondition;
	float PreviewTime;
	TArray<struct McpVariantChannelInfo> VariantOptions;
	TArray<struct FortCosmeticVariantPreviewElement> AdditionalItems;
};

struct FFortCosmeticAdaptiveStatPair
{
	struct FortStatManagerTag StatTag;
	int StatValue;
};

struct FFortCosmeticAdaptiveStatPreview
{
	struct FText UnlockCondition;
	TArray<struct FortCosmeticAdaptiveStatPair> StatValues;
};

struct FBarrierFlagDisplayData
{
	class UClass* HeadMesh;
	struct SlateBrush MiniMap_EnabledBrush;
	struct SlateBrush MiniMap_DisabledBrush;
	struct SlateBrush Compass_EnabledBrush;
	struct SlateBrush Compass_DisabledBrush;
	struct Vector2D MapSize;
	struct Vector2D CompassSize;
	struct Vector MeshScale;
};

struct FBarrierObjectiveDisplayData
{
	class UClass* HeadMesh;
	struct Vector MeshScale;
	struct Vector MeshRelativeOffset;
	TArray<class UClass*> MaterialsToSwap;
};

struct FBuildingFoundationStreamingData
{
	struct FName FoundationName;
	struct Vector FoundationLocation;
	struct Box BoundingBox;
	struct GameplayTagContainer GameplayTags;
	struct IntPoint GridCoordinates;
	TArray<Unknown> ProxyInfo;
	TArray<int> ChildStreamingDataIndices;
	byte PersistentHLODLevelIndex;
};

struct FDynamicBuildingFoundationRepData
{
	struct Rotator Rotation;
	struct Vector Translation;
	EDynamicFoundationEnabledState EnabledState;
};

struct FFortItemQuantityPair
{
	struct PrimaryAssetId ItemPrimaryAssetId;
	int Quantity;
};

struct FFortHiddenRewardQuantityPair
{
	struct FName TemplateId;
	int Quantity;
};

struct FFortMcpQuestRewardInfo
{
	TArray<struct FortItemQuantityPair> Rewards;
};

struct FInlineObjectiveStatTagCheckEntry
{
	struct GameplayTag Tag;
	EInlineObjectiveStatTagCheckEntryType Type;
	bool Require;
};

struct FFortQuestObjectiveStat
{
	TArray<struct InlineObjectiveStatTagCheckEntry> TagConditions;
	struct FString Condition;
	TArray<struct FString> TemplateIds;
	EFortQuestObjectiveStatEvent Type;
	bool bIsCached;
	bool bHasInclusiveTargetTags;
	bool bHasInclusiveSourceTags;
	bool bHasInclusiveContextTags;
};

struct FFortMcpQuestObjectiveInfo
{
	struct FName BackendName;
	TArray<struct FortQuestObjectiveStat> InlineObjectiveStats;
	struct DataTableRowHandle ObjectiveStatHandle;
	TArray<struct DataTableRowHandle> AlternativeStatHandles;
	EFortQuestObjectiveItemEvent ItemEvent;
	bool bHidden;
	bool bRequirePrimaryMissionCompletion;
	bool bCanProgressInZone;
	bool bDisplayDynamicAnnouncementUpdate;
	EObjectiveStatusUpdateType DynamicStatusUpdateType;
	EFortInventoryFilter LinkVaultTab;
	EFortFrontendInventoryFilter LinkToItemManagement;
	struct TSoftClassPtr<UObject> ItemReference;
	struct FString ItemTemplateIdOverride;
	struct FName LinkSquadID;
	int LinkSquadIndex;
	struct FText Description;
	struct FText HudShortDescription;
	struct TSoftClassPtr<UObject> HudIcon;
	int Count;
	int Stage;
	int DynamicStatusUpdatePercentInterval;
	float DynamicUpdateCompletionDelay;
	struct TSoftClassPtr<UObject> ScriptedAction;
	struct TSoftClassPtr<UObject> FrontendScriptedAction;
};

struct FFortQuestMissionCreationContext
{
	struct TSoftClassPtr<UObject> MissionInfo;
	TArray<struct GameplayTagContainer> MissionCreationContextTags;
	bool bSetQuestOwnerAsMissionOwner;
	int MaxNumberToSpawnInWorld;
};

struct FFortMissionConfigDataBucket
{
	struct GameplayTag Tag;
	struct TSoftClassPtr<UObject> ConfigDataClass;
};

struct FFortMissionConfigDataParams
{
	TArray<struct FortMissionConfigDataBucket> ConfigParams;
};

struct FAthenaCharacterTaggedPartsList
{
	TArray<struct TSoftClassPtr<UObject>> Parts;
};

struct FCharmSoundAssetEntry
{
	struct TSoftClassPtr<UObject> Sound;
	struct FName Desc;
};

struct FStormShieldRadiusGrowthData
{
	float TargetRadius;
	float StartingRadius;
	float GrowthRate;
	float SafeAreaStartRadiusChangeTime;
	float SafeAreaFinishRadiusChangeTime;
	EMissionStormShieldState State;
};

struct FStormShieldMoveData
{
	float MoveRate;
	struct Vector TargetLocation;
	struct Vector StartingLocation;
	float SafeAreaStartLocationChangeTime;
	float SafeAreaFinishLocationChangeTime;
};

struct FFortEmoteMapping
{
	EFortCustomBodyType BodyType;
	EFortCustomGender Gender;
	struct TSoftClassPtr<UObject> EmoteMontage;
};

struct FSectionNameAndWeight
{
	struct FName SectionName;
	float SectionWeight;
};

struct FMontageItemAccessData
{
	struct GameplayTag AccessTag;
	class UClass* AccessToken;
};

struct FMontageVisibilityData
{
	EMontageVisibilityRule Rule;
	class UClass* Item;
};

struct FFortSpawnSlotData
{
	struct Vector SpawnSlotLocation;
	class UClass* OccupyingAI;
	EFortRiftSlotStatus SlotStatus;
};

struct FFortRiftReservationHandle
{
	int RiftReservationID;
};

struct FFortRiftReservation
{
	bool bDesiredVisible;
	bool bDesiredActive;
	struct FortRiftReservationHandle ReservationHandle;
};

struct FFortEncounterSettings
{
	TArray<EFortEncounterDirection> ForbiddenSpawnDirections;
	bool bRiftsDestroyPlayerBuiltBuildings;
	bool bValidateIfPlayerIsAtSpawnLocation;
	bool bMustFindSpawnPoints;
	bool bStopIfCantFindSpawnPoint;
	bool bIgnoreCollisionWhenSpawningAI;
	bool bTrackCombatParticipation;
	bool bDisplayThreatVisuals;
	float BurstSpawnThreatVisualsEndDelayOverride;
	int NumRiftsToUseOverride;
	bool bUseEQSQueryToFindAISpawnLocations;
	bool bRelevantForTotalAICap;
	bool bEnableRecreateRift;
	bool bRespawnRiftWhenRiftDead;
	bool bRandomiseQueryRiftLocations;
	bool bOverrideEqsFallback;
	struct EncounterEnvironmentQueryInfo EqsFallbackOverride;
	float PreSpawnRequeryTime;
	float SpawnAIIntervalTime;
	float SpawnRiftIntervalTime;
	bool bSpawnFirstRiftNoDelay;
	class UClass* RiftSelectionQuery;
	class UClass* RiftSlotsEQSQueryOverride;
	TArray<class UClass*> ScriptedSpawnPoints;
	class UClass* RiftClassOverride;
	TArray<class UClass*> WorkingScriptedSpawnPoints;
	int EncounterGroupID;
	int ZoneIndex;
	int DifficultyIndex;
	float AIDespawnDistanceOverride;
	TArray<class UClass*> InjectedOverrideCategories;
};

struct FVariantSwapMontagePartRequirement
{
	EFortCustomPartType PartType;
	struct TSoftClassPtr<UObject> Part;
};

struct FVariantSwapMontageData
{
	struct FName MontageSectionName;
	struct GameplayTag VariantMetaTagRequired;
	bool bRequireCharacterPart;
	TArray<struct VariantSwapMontagePartRequirement> PartRequirements;
};

struct FFillFloorPositionData
{
	struct ScalableFloat MoveTime;
	struct ScalableFloat Height;
	struct ScalableFloat PostMoveDelay;
};

struct FFortPickupTagTestContainer
{
	struct GameplayTagContainer Tags;
	struct FText FailReason;
};

struct FFortPickupRestrictionLists
{
	struct FortPickupTagTestContainer WhiteList;
	struct FortPickupTagTestContainer Blacklist;
};

struct FSpecialActorSingleStatData
{
	ESpecialActorStatType StatType;
	float Value;
	float StatLogicValue;
};

struct FCharacterPartsExtraSpecial
{
	TArray<class UClass*> CharacterPartsForExtraSpecial;
	struct GameplayTagContainer SkinMetaTagsForExtraSpecial;
};

struct FFortCreativeTagsHelper
{
	TArray<struct FName> CreativeTags;
};

struct FFortCosmeticSwapRequirement
{
	EFortCosmeticSwapRequirementPart ItemCategory;
	struct GameplayTagContainer RequiredMetaTags;
};

struct FFortSwapItemAndVariantData
{
	class UClass* Item;
	TArray<struct McpVariantChannelInfo> ChannelInfoList;
};

struct FFortCosmeticDependentSwapData
{
	TArray<struct FortCosmeticSwapRequirement> SwapRequirements;
	TArray<struct FortSwapItemAndVariantData> SwapData;
	EFortAppliedSwapItemAndVariantState ForcedSwapState;
	struct Guid SwapId;
};

struct FFortGliderLayeredAudioOneshotGate
{
	struct TSoftClassPtr<UObject> SoundRef;
	float GateValue;
	ELayeredAudioTriggerDir Direction;
	bool FadeWhenOutsideGate;
	float MinTimeSinceTrigger;
	float InterruptFadeTime;
};

struct FFortGliderLayeredAudioFloatParam
{
	ELayeredAudioInterpolationType InterpType;
	class UClass* Curve;
	float AttackSpeed;
	float ReleaseSpeed;
	TArray<struct FortGliderLayeredAudioOneshotGate> Oneshots;
};

struct FMarshalledVFXData
{
	struct GameplayTagContainer ParameterGroups;
	EFXType Type;
	struct TSoftClassPtr<UObject> Asset;
	struct FName AttachAtBone;
	struct Transform RelativeOffset;
	struct GameplayTag EffectIdTag;
	bool bAutoActivate;
};

struct FParameterNameMapping
{
	struct FName CascadeName;
	struct FName NiagaraName;
};

struct FMarshalledVFXAuthoredData
{
	TArray<struct MarshalledVFXData> NiagaraVFX;
	TArray<struct MarshalledVFXData> CascadeVFX;
	Unknown NameReplacements;
};

struct FMarkerID
{
	int PlayerID;
	int InstanceID;
};

struct FTotalResKBIncAssetCostPair
{
	struct FString AssetName;
	uint32_t CostKBInc;
};

struct FMetaNavCachedEntry
{
};

struct FNavDataSetVariantSettings
{
	struct TSoftClassPtr<UObject> Level;
	uint32_t OceanFloodLevel;
};

struct FBoxNavInvoker
{
	class UClass* Invoker;
};

struct FFortUICameraFrameTargetBounds
{
	struct Vector Origin;
	float CylinderHalfHeight;
	float CylinderRadius;
};

struct FAthenaWeaponStats
{
	struct FString WeaponId;
	int Stats;
};

struct FAthenaXPStats
{
	struct FName Stat;
	int Count;
	int XP;
	EFortAccoladeSubtype Subtype;
};

struct FAthenaMatchStats
{
	struct FString StatBucket;
	struct FString MatchID;
	struct FString MatchEndTime;
	struct FString MatchPlatform;
	int Stats;
	TArray<struct AthenaWeaponStats> WeaponStats;
	TArray<struct AthenaXPStats> XPStats;
	bool bIsValid;
	struct FString FactionTag;
};

struct FAthenaLevelInfo
{
	int AccountLevel;
	int Level;
	int MaxLevel;
	int LevelXp;
	int LevelXpForLevel;
	int BookLevel;
	int BookMaxLevel;
	int BookLevelXp;
	int BookLevelXpForLevel;
};

struct FAthenaMatchTeamStats
{
	int Place;
	int TotalPlayers;
};

struct FAthenaMatchXpMultiplierGroup
{
	EAthenaMatchXpMultiplierSource Source;
	int Amount;
};

struct FAthenaAwardGroup
{
	ERewardSource RewardSource;
	TArray<struct McpLootEntry> Items;
	int Score;
	float SeasonXp;
	int BookXp;
};

struct FAthenaRewardResult
{
	int LevelsGained;
	int BookLevelsGained;
	int TotalSeasonXpGained;
	int TotalBookXpGained;
	int PrePenaltySeasonXpGained;
	TArray<struct AthenaMatchXpMultiplierGroup> XpMultipliers;
	TArray<struct AthenaAwardGroup> Rewards;
	float AntiAddictionMultiplier;
};

struct FAthenaTravelLogEntry
{
	float Time;
	struct Vector Position;
	struct Rotator Rotation;
	EAthenaTravelEventType Type;
	struct FName InstigatorName;
	struct UniqueNetIdRepl InstigatorId;
	EAthenaTravelLogPlayerType InstigatorPlayerType;
	struct FName ReceiverName;
	struct UniqueNetIdRepl ReceiverId;
	EAthenaTravelLogPlayerType ReceiverPlayerType;
	float Value;
	struct JsonObjectWrapper Meta;
	struct GameplayTagContainer LocationPOITags;
};

struct FAthenaTravelRecord
{
	TArray<struct AthenaTravelLogEntry> Log;
};

struct FAthenaQuickChatLeafEntry
{
	struct FText Label;
	struct FText FullChatMessage;
	struct SlateBrush Brush;
	bool bPopulateBrushFromContextObject;
	EAthenaQuickChatFilteringType FilterType;
	class UClass* EmojiItemDefinition;
	ETeamMemberState TeamCommType;
	struct GameplayTag OptionGameplayTag;
};

struct FCloneMachineRepData
{
	class UClass* CloneMachine;
	struct Vector Location;
};

struct FRewardKeyData
{
	struct TSoftClassPtr<UObject> Key;
	struct GameplayTag NodeTagMatchReq;
	int RewardKeyMaxCount;
	int RewardKeyInitialCount;
	struct TSoftClassPtr<UObject> UnlockingItemDef;
	bool bUseUnlockingItemDisplayName;
};

struct FFortGiftBoxFortmatData
{
	struct FString StringAssetType;
	struct FString StringData;
};

struct FChallengeGiftBoxData
{
	struct TSoftClassPtr<UObject> GiftBoxToUse;
	TArray<struct FortGiftBoxFortmatData> GiftBoxFormatData;
};

struct FAthenaRewardItemReference
{
	struct TSoftClassPtr<UObject> ItemDefinition;
	struct FString TemplateId;
	int Quantity;
	struct ChallengeGiftBoxData RewardGiftBox;
	bool IsChaseReward;
	EAthenaRewardItemType RewardType;
	EAthenaRewardVisualImportanceType RewardVisualImportanceType;
};

struct FRewardNode
{
	struct TSoftClassPtr<UObject> RequiredKey;
	int KeyCount;
	int MinKeyCountToUnlock;
	int DaysFromEventStartToUnlock;
	struct GameplayTagContainer ChildNodes;
	struct GameplayTagContainer ParentNodes;
	struct GameplayTag NodeTag;
	bool bGrantedAtGraphDestruction;
	bool bRequiredOwnership;
	TArray<struct AthenaRewardItemReference> Rewards;
	struct FString RewardOperation;
	struct TSoftClassPtr<UObject> RewardContextItem;
	TArray<struct CosmeticVariantInfo> HardDefinedVisuals;
};

struct FRewardKeyState
{
	struct FString static_key_template_id;
	int unlock_keys_used;
};

struct FAthenaVehicleOverride
{
	struct FString RequiredCalendarEvent;
	struct TSoftClassPtr<UObject> DefaultVehicleClass;
	struct TSoftClassPtr<UObject> OverrideVehicleClass;
};

struct FBattlePassLevelReward
{
	struct FString OfferId;
	bool bHideFromGiftBox;
	int GrantedOnLevel;
	struct AthenaRewardItemReference Reward;
};

struct FAthenaSeasonPageGrid
{
	int LevelsNeededForUnlock;
	int RewardsNeededForUnlock;
	struct IntPoint GridSize;
	TArray<class UClass*> RewardEntryList;
};

struct FAthenaSeasonItemCustomSkinCategoryData
{
	struct FText Name;
	TArray<class UClass*> Entries;
	int RequiredRewardsToUnlock;
	float ZoomLevel;
	float Rotation;
};

struct FTrackCategory
{
	struct TSoftClassPtr<UObject> CategoryIcon;
	struct FText CategoryName;
	int CategoryStartingLevel;
};

struct FAthenaRewardScheduleLevel
{
	TArray<struct AthenaRewardItemReference> Rewards;
};

struct FAthenaRewardSchedule
{
	TArray<struct AthenaRewardScheduleLevel> Levels;
};

struct FTrackDynamicBackground
{
	struct TSoftClassPtr<UObject> BackgroundSubstance;
	struct LinearColor PrimaryColor;
	struct LinearColor SecondaryColor;
	struct LinearColor TertiaryColor;
	bool bIsSpecial;
	bool bIsFoil;
	int MinimalDiscoveryLevel;
};

struct FAthenaSeasonBannerLevel
{
	struct TSoftClassPtr<UObject> SurroundImage;
	struct TSoftClassPtr<UObject> BannerMaterial;
};

struct FAthenaSeasonBannerLevelSchedule
{
	TArray<struct AthenaSeasonBannerLevel> Levels;
};

struct FXpDisplayConversion
{
	struct TSoftClassPtr<UObject> XpItemDef;
	int ValueToReplaceAt;
};

struct FAthenaMidSeasonUpdateItemReq
{
	struct TSoftClassPtr<UObject> Item;
	int Count;
};

struct FAthenaMidSeasonUpdateQuestReq
{
	struct TSoftClassPtr<UObject> Quest;
	bool bCompletionRequired;
};

struct FAthenaMidSeasonUpdate
{
	int SeasonLevelRequirement;
	int BookLevelRequirement;
	bool SeasonPurchasedRequirement;
	TArray<struct AthenaMidSeasonUpdateItemReq> ItemRequirements;
	TArray<struct AthenaMidSeasonUpdateQuestReq> QuestRequirements;
	struct AthenaRewardScheduleLevel Grants;
	TArray<struct TSoftClassPtr<UObject>> Removals;
};

struct FAthenaBattlePassOffer
{
	struct FString OfferId;
	struct AthenaRewardItemReference RewardItem;
	TArray<struct AthenaRewardItemReference> ChainedRewardItemList;
	struct DataTableRowHandle OfferPriceRowHandle;
};

struct FVectorParticleParameter
{
	struct Vector Value;
	struct FName ParameterName;
};

struct FFloatParticleParameter
{
	float Value;
	struct FName ParameterName;
};

struct FTrapDetectionState
{
};

struct FCharacterDisplaySettings
{
	Unknown PrimaryItem;
};

struct FXPDisplayData
{
	struct FName XPType;
	struct FText TitleText;
	struct FText DisplayText;
	class UClass* IconMaterial;
};

struct FBuildingActorHotSpotDirection
{
	class UClass* HotSpotConfig;
	struct Vector Offset;
	bool bMirrorX;
	bool bMirrorY;
	EFortHotSpotDirection Direction;
	EHotspotTypeConfigMode TypeConfigUsage;
};

struct FConnectivityCube
{
};

struct FAuxiliaryEditTileMeshData
{
	class UClass* TileMesh;
	class UClass* TileTexture;
	struct Rotator RelativeRot;
};

struct FEditModeState
{
	class UClass* EditClass;
	int RotationIterations;
	bool bMirrored;
	bool bCurrentlyValid;
};

struct FTileCompInterpData
{
	struct Vector InitialTranslation;
	struct Vector DesiredTranslation;
};

struct FQueuedFlushNetDormancyInfo
{
	class UClass* Actor;
};

struct FChaserPatrolTurnData
{
	bool bPatrolTurning;
	struct Vector PatrolTurnStartDirection;
	struct Vector PatrolTurnEndDirection;
	float PatrolMoveSpeed;
};

struct FSplatterCellIndex
{
	int X;
	int Y;
	int Z;
};

struct FRebootCardReplicatedState
{
	float ChipExpirationServerStartTime;
	class UClass* PlayerState;
};

struct FFortMapData
{
	struct TSoftClassPtr<UObject> BuildingWorld;
	struct GameplayTagContainer BuildingLevelTags;
	float SelectionWeight;
};

struct FGameplayEffectApplicationInfo
{
	struct TSoftClassPtr<UObject> GameplayEffect;
	float Level;
};

struct FFortGameplayEffectDeliveryInfo
{
	struct FortDeliveryInfoRequirementsFilter DeliveryRequirements;
	TArray<struct GameplayEffectApplicationInfo> GameplayEffects;
};

struct FCollectorUnitInfo
{
	class UClass* InputItem;
	struct ScalableFloat InputCount;
	class UClass* OverrideInputItemTexture;
	bool bUseDefinedOutputItem;
	class UClass* OutputItem;
	TArray<struct FortItemEntry> OutputItemEntry;
	struct FName OverrideOutputItemLootTierGroupName;
	struct FortGameplayEffectDeliveryInfo OutputGameplayEffect;
	class UClass* OverrideOutputItemTexture;
};

struct FCollectorTrackedData
{
	byte Team;
	class UClass* Player;
};

struct FPlayerWeaponUpgradeHoldData
{
	class UClass* InteractingPC;
	Unknown UpgradeTargetWeapon;
};

struct FCaptureAreaTeamInfo
{
	TArray<class UClass*> Players;
	byte InsideTeamIndex;
};

struct FCaptureHandle
{
};

struct FSpawnGroupVisuals
{
	struct TSoftClassPtr<UObject> SpawnGroup;
	struct TSoftClassPtr<UObject> AIPawn;
	struct TSoftClassPtr<UObject> PhysicsAsset;
};

struct FMusicPlayerData
{
	int SongIndex;
	float ServerTimeSongStarted;
};

struct FBuildingActorNavArea
{
	int AreaBits;
};

struct FWaypointIndex
{
	int WaypointGroup;
	int WaypointIndex;
};

struct FMOBATurretPrioritySetting
{
	int AIPriority;
	int PlayerPriority;
	int BuildingPriority;
};

struct FClimbLinkData
{
	uint32_t UniqueLinkId;
};

struct FBulletWhipTrackerData
{
	bool bAttachSoundToOwner;
	float PassByRadiusMax;
	float PassByRadiusMin;
	class UClass* PassByFarSound;
	class UClass* PassByCloseSound;
	float MinimumTriggerDistance;
	float TriggerAheadDistance;
	class UClass* CurrentAudioComp;
	float PreviousPlaneDotProd;
	float CachedPassDistance;
	struct Vector CachedPassLocation;
	float PassByClosenessIntensity;
	bool bActive;
};

struct FVersionedBudget
{
	ELevelSaveRecordVersion Version;
	int Value;
};

struct FCombatEventData
{
	float Heat;
	float MaxHeatContribution;
	float CoolDownRate;
	struct FString EventName;
	struct LinearColor DebugGraphColor;
	EFortCombatEventContribution ContributionType;
};

struct FCombatEventMultiplier
{
	EFortCombatEvents CombatEvent;
	float MaxContribution;
};

struct FCombatFactorData
{
	TArray<struct CombatEventMultiplier> ContributingCombatEvents;
	float MaxValue;
	struct FString DebugFactorName;
	struct LinearColor DebugGraphColor;
};

struct FCombatThresholdData
{
	struct ScalableFloat HeatLevel;
	struct ScalableFloat ExitHeatLevel;
	struct FString ThresholdName;
	struct LinearColor DebugGraphColor;
};

struct FReplicatedVkResolvedModule
{
	struct VkResolvedModule ResolvedModule;
	bool bShouldBeInstalled;
};

struct FDeferredCreativeTask
{
	class UClass* ActorPtr;
};

struct FCreativeIslandResource
{
	float WorldRadius;
	struct FName Tag;
	int MaxCount;
};

struct FFortMiniMapIconMaterialParameterData
{
	EMiniMapIconParameterDataType DataType;
	struct FName Name;
	float Scalar;
	struct LinearColor Vector;
	class UClass* Texture;
};

struct FFortMiniMapData
{
	class UClass* MiniMapIcon;
	struct Vector2D IconScale;
	bool bUseIconSize;
	struct Vector2D IconMaterialSize;
	bool bIsVisible;
	bool bIsVisibleOnMiniMap;
	bool bIsVisibleOnMap;
	bool bIsVisibilityBasedOnTeam;
	bool bShowVerticalOffset;
	bool bShowFarOffIndicator;
	bool bDisplayIconEvenOnFogOfWar;
	bool bAllowLocalOverrides;
	bool bUseTeamAffiliationColors;
	struct LinearColor Color;
	struct LinearColor FriendColor;
	struct LinearColor EnemyColor;
	struct LinearColor NeutralColor;
	struct LinearColor PulseColor;
	float ColorPulsesPerSecond;
	float SizePulsesPerSecond;
	float ViewableDistance;
	struct Vector LocationOffset;
	int Priority;
	byte Team;
	TArray<struct FortMiniMapIconMaterialParameterData> MiniMapIconMaterialParameterDataList;
};

struct FFortMiniMapIndicatorTextProperties
{
	struct SlateFontInfo DisplayTextFont;
	struct LinearColor DisplayTextColor;
	struct Vector2D DisplayTextOffset;
};

struct FCreativeMiniMapComponentIconData
{
	int IconIndex;
	ECreativeMinimapComponentIconColorType IconColor;
};

struct FCreativeQuestData
{
	class UClass* PlayerState;
	int Progress;
	bool bActive;
};

struct FVehicleTrickInfo
{
	float LastOnGroundTime;
	float mCreditDisabledTime;
	int TrickScore;
	int TrickAxisCount;
	float AirControlsAlpha;
	float AirDistance;
	float AirDistanceSqrd;
	float AirTime;
	float AirHeight;
	float TimeAtLaunch;
	struct Vector LocationAtLaunch;
	struct Vector ForwardVectorAtLaunch;
	struct Vector UpVectorAtLaunch;
	struct Vector FlatForwardVectorAtLaunch;
	struct Vector PrevForwardVec;
	struct Vector PrevRightVec;
	struct Vector PrevUpVec;
	int PeterPanCount;
	int StoopingSquirrelCount;
	bool bDidPeterPan;
	bool bDidStoopingSquirrel;
	bool bInAirTrick;
	bool bCreditTrick;
	bool bTrickDeactivated;
	bool bStuckLanding;
	bool bDoingRotationTrick;
};

struct FGhostModeRepData
{
	bool bInGhostMode;
	class UClass* GhostModeItemDef;
	int PreviousFocusedSlot;
	float TimeExitedGhostMode;
};

struct FCreativeOptionVariableBase
{
	struct GameplayTag VariableTag;
	int Value;
};

struct FFortPlayerDeathReport
{
	float ServerTimeForRespawn;
	float ServerTimeForResurrect;
	float LethalDamage;
	class UClass* KillerPlayerState;
	class UClass* KillerPawn;
	float KillerHealthPercent;
	float KillerShieldPercent;
	class UClass* KillerWeapon;
	class UClass* DamageCauser;
	bool bDroppedBackpack;
	bool bNotifyUI;
	struct GameplayTagContainer Tags;
	struct Vector ViewLocationAtTimeOfDeath;
	struct Rotator ViewRotationAtTimeOfDeath;
};

struct FLevelStreamRequestHandshakeState
{
	bool bLevelStreamingCompleted;
};

struct FCreativeIslandData
{
	struct FString McpId;
	struct FText IslandName;
	struct FString PublishedIslandCode;
	int PublishedIslandVersion;
	struct DateTime LastLoadedDate;
	struct DateTime DeletedAt;
	bool bIsDeleted;
};

struct FQuickBarSlotData
{
	TArray<EFortItemType> AcceptedItemTypes;
	bool bStaticSlot;
	struct TSoftClassPtr<UObject> DefaultItem;
};

struct FQuickBarData
{
	TArray<struct QuickBarSlotData> QuickbarSlots;
};

struct FBuildingStats
{
	int BuildingsPlaced;
	int WallsPlaced;
	int StairsPlaced;
	int FloorsPlaced;
	int RoofsPlaced;
	int WoodBuildingsPlaced;
	int StoneBuildingsPlaced;
	int MetalBuildingsPlaced;
};

struct FBuildingEditAnalyticEvent
{
	EFortBuildingType BuildingType;
	EFortResourceType ResourceType;
	struct Vector Location;
};

struct FCreativePlotSessionData
{
	int TimesInventoryOpened;
	int TimesIslandMenuOpened;
	int TimesGameStarted;
};

struct FItemAndCount
{
	int Count;
	class UClass* Item;
};

struct FCompositeBool
{
	bool bDefaultValue;
	TArray<struct TSoftClassPtr<UObject>> ModifyingObjects;
};

struct FQuickBarEquippedItemGuids
{
	struct Guid EquippedItemGuids;
	int NumEnabledSlots;
};

struct FCreativeThumbnailCacheData
{
};

struct FColorSwatchPair
{
	struct FName ColorName;
	struct LinearColor ColorValue;
	struct FText ColorDisplayName;
};

struct FCustomPartTextureParameter
{
	int MaterialIndexForTextureParameter;
	struct FName TextureParameterNameForMaterial;
	struct TSoftClassPtr<UObject> TextureOverride;
};

struct FCustomAccessoryMorphs
{
};

struct FCustomPartMaterialOverrideData
{
	int MaterialOverrideIndex;
	struct TSoftClassPtr<UObject> OverrideMaterial;
};

struct FCustomPartScalarParameter
{
	int MaterialIndexForScalarParameter;
	struct FName ScalarParameterNameForMaterial;
	float ScalarOverride;
};

struct FCustomPartVectorParameter
{
	int MaterialIndexForVectorParameter;
	struct FName VectorParameterNameForMaterial;
	struct LinearColor VectorOverride;
};

struct FAttachToComponentParams
{
	EAttachmentRule LocationRule;
	EAttachmentRule RotationRule;
	EAttachmentRule ScaleRule;
	bool bWeldSimulatedBodies;
};

struct FCharacterPartAttachmentParams
{
	struct FName SocketName;
	ECharacterPartAttachmentTargetType AttachmentTarget;
	struct AttachToComponentParams AttachmentRules;
};

struct FFortAnimInput_SkydivingExternalForce
{
	bool bUseSkydivingVectorForce;
	struct Vector SkydivingVectorForce;
	struct Vector HeadToPelvisDirection;
	struct Vector FloatingMultiplier;
	struct Vector FloatingAdditive;
	struct Vector DivingMultiplier;
	struct Vector DivingAdditive;
	struct Vector ParachutingMultiplier;
	struct Vector ParachutingAdditive;
	bool bUseNoisyClothGravity;
	bool bApplyNoiseInActorSpace;
	float PerlinRangedOutMinX;
	float PerlinRangedOutMaxX;
	float PerlinRangedOutMinY;
	float PerlinRangedOutMaxY;
	float PerlinRangedOutMinZ;
	float PerlinRangedOutMaxZ;
};

struct FFortAnimInput_Facial
{
	struct LiveLinkSubjectName SubjectName;
	EFortFacialAnimTypes CurrentAnimType;
	bool bCurvesOnly;
};

struct FBlackWidowLegSinAnimationScalar
{
	bool bUseConstantValue;
	float ConstantValue;
	float TimeOffset;
	float FrequencyOffset;
	float SinOffset;
	float ResultMultiplier;
	struct Vector2D MappedRangeOutput;
};

struct FBlackWidowLegSinAnimationRotator
{
	struct BlackWidowLegSinAnimationScalar RollAnimation;
	struct BlackWidowLegSinAnimationScalar PitchAnimation;
	struct BlackWidowLegSinAnimationScalar YawAnimation;
};

struct FMarshalledVFXRuntimeData
{
	TArray<class UClass*> DynamicSystems;
	class UClass* BasedOn;
};

struct FSlurpLegendSwapToVariantData
{
	float DelayBeforeSwitching;
	struct McpVariantChannelInfo VariantData;
};

struct FDAD_CosmeticItemUserOption
{
	struct FString Name;
	struct FText DisplayName;
	struct TSoftClassPtr<UObject> CosmeticItemAsset;
	struct GameplayTag GameplayTag;
	struct GameplayTagContainer DefaultVariants;
	struct LinearColor VFXColor;
	struct FString StoreLink;
};

struct FPlaylistOptionValue
{
	struct FText DisplayName;
	struct FString OptionValueName;
};

struct FPlaylistOverrideData
{
	struct FName PlaylistName;
	bool bEnabled;
	TArray<struct FString> RegionsDisabled;
};

struct FAudioDynamicSoundData
{
	struct FName Name;
	EDynamicSoundOverride SoundOverrideType;
	float Volume;
};

struct FPlaylistAccess
{
	bool bForcePlaylistOff;
	bool bEnabled;
	bool bVisibleWhenDisabled;
	bool bInvisibleWhenEnabled;
	bool bIsDefaultPlaylist;
	EPlaylistAdvertisementType AdvertiseType;
	bool bDisplayAsLimitedTime;
	int DisplayPriority;
	int CategoryIndex;
};

struct FPlaylistAccessOverride
{
	TArray<struct FString> Regions;
	TArray<struct FString> Platforms;
	struct PlaylistAccess OverrideAccess;
};

struct FPlaylistFrontEndSettings
{
	struct FName PlaylistName;
	struct PlaylistAccess PlaylistAccess;
	TArray<struct PlaylistAccessOverride> AccessOverrides;
};

struct FAthenaDataTableSet
{
	class UClass* LootTierData;
	class UClass* LootPackages;
	class UClass* RangedWeapons;
	class UClass* GameData;
	class UClass* ResourceRates;
	class UClass* VehicleData;
	class UClass* AILootOnDeathData;
};

struct FDAD_Island
{
	struct FString IslandName;
	struct FString IslandCode;
	struct FString MetricName;
	struct FString MetricInterval;
	bool bLowestToHighest;
	int Nth;
};

struct FDataDrivenServiceBriefConfig
{
	struct FText TitleText;
	struct FText DescriptionText;
	struct FText TransactionText;
	struct FText StockText;
	Unknown CannotUseReasonParameterToDisplayText;
	struct FString CannotUseReasonParameterKey;
	struct FString CostParameterKey;
	struct FString StockCountParameterKey;
};

struct FFortCloudSaveRecordInfo
{
	int RecordIndex;
	int ArchiveNumber;
	struct FString RecordFilename;
};

struct FFortCloudSaveInfo
{
	int SaveCount;
	TArray<struct FortCloudSaveRecordInfo> SavedRecords;
};

struct FDeferredActorData
{
	class UClass* BuildingActor;
	int ActorRecordIndex;
	struct Transform BuildingTransform;
};

struct FViewOffsetData
{
	struct Vector OffsetHigh;
	struct Vector OffsetMid;
	struct Vector OffsetLow;
};

struct FAbilityToolSpawnParameters
{
	class UClass* SpawnClass;
	struct Vector Location;
	struct Rotator Rotation;
	class UClass* AttachedToActor;
};

struct FAbilityActivatedByInputData
{
	class UClass* Ability;
	struct GameplayTagQuery ActivationTagQuery;
};

struct FAbilityKitItem
{
	class UClass* Item;
	int Quantity;
	EFortReplenishmentType Replenishment;
};

struct FReplicatedMontagePair
{
	class UClass* Montage1;
	class UClass* Montage2;
	struct FName Section1;
	struct FName Section2;
	int8_t RepIndex;
};

struct FGameplayAbilityRepSharedAnim_Base
{
	EFortSharedAnimationState AnimState;
	byte MontageSectionToPlay;
};

struct FTokenAttributePair
{
	struct GameplayTag Token;
	struct GameplayAttribute Attribute;
};

struct FFortAbilityTagRelationship
{
	struct GameplayTag AbilityTag;
	struct GameplayTagContainer AbilityTagsToBlock;
	struct GameplayTagContainer AbilityTagsToCancel;
};

struct FFortMoveConfig
{
	class UClass* FocusTarget;
	class UClass* PushPawnClassOnBump;
};

struct FFortGameplayEffectContainerSpec
{
	struct FortAbilityTargetSelectionList TargetSelection;
	TArray<struct GameplayEffectSpecHandle> TargetGameplayEffectSpecs;
	TArray<struct GameplayEffectSpecHandle> OwnerGameplayEffectSpecs;
	struct GameplayTagContainer ActivationCues;
	struct GameplayTagContainer ImpactCues;
	float ImpactNoiseRange;
	float FlyByNoiseRange;
	bool bOverrideChargeMagnitude;
	float ChargeMagnitudeOverrideValue;
};

struct FAccoladeSecondaryXpType
{
	struct GameplayTag Type;
	struct ScalableFloat XpAmount;
};

struct FPlaylistToActivityMapping
{
	struct FName PlaylistName;
	struct FString ActivityId;
};

struct FFortAffiliationActorIdentifierList
{
	TArray<uint32_t> AffiliationComponentUIDs;
};

struct FFortAffiliationComponentSpecificRelations
{
	struct FortAffiliationActorIdentifierList Identifiers;
	TArray<class UClass*> Components;
};

struct FPropertyOverrideMk2
{
	struct FString PropertyScope;
	struct FString PropertyName;
	struct FString PropertyData;
	struct FString DefaultPropertyData;
};

struct FPropertyOverrideId
{
	uint64_t PropertyHashes;
};

struct FPropertyOverrideData
{
	TArray<struct PropertyOverrideMk2> PropertyOverrides;
	TArray<struct PropertyOverrideId> SharedPropertyIds;
	TArray<struct PropertyOverrideId> PendingPropertyIds;
	EPropertyOverrideTargetType OverrideMode;
	class UClass* BaseObject;
	class UClass* MutableObject;
};

struct FPendingRequestManager
{
};

struct FFortAIAssignmentIdentifier
{
	EAssignmentType AssignmentType;
	struct GameplayTagContainer AssignmentGameplayTags;
	EFortTeam AssignmentTeam;
};

struct FFortAIGoalInfo
{
	Unknown Actor;
	struct Vector Location;
	bool bActorAlwaysPerceived;
};

struct FGoalSelectionQueryInfo
{
	class UClass* GoalSelectionQuery;
	struct GameplayTagContainer RequiredGameplayTags;
};

struct FSoundIndicatorTypePicker
{
	struct GameplayTagContainer Tags;
	EFortSoundIndicatorTypes SoundIndicatorType;
};

struct FFortPickupEntryData
{
	struct InterpCurveFloat FloatCurve;
	struct Guid PickupGuid;
	float StartTime;
};

struct FPawnDamageZones
{
	bool bActive;
	TArray<struct FName> Bones;
};

struct FFortSpokenLine
{
	class UClass* Audio;
	class UClass* AnimMontage;
	class UClass* AnimSequence;
	class UClass* Addressee;
	EFortFeedbackBroadcastFilter BroadcastFilter;
	float Delay;
	bool bInterruptCurrentLine;
	bool bCanBeInterrupted;
	bool bCanQue;
};

struct FFortPawnVocalChord
{
	class UClass* FeedbackAudioComponent;
	struct FortSpokenLine ReplicatedSpokenLine;
	struct FortSpokenLine PendingSpokenLine;
	struct FortSpokenLine QueuedSpokenLine;
	struct FortSpokenLine CurrentSpokenLine;
};

struct FFortActiveMontageDecisionWindow
{
	class UClass* DecisionWindow;
	class UClass* DecisionAnimation;
	bool bReceivedPrimaryInput;
	bool bReceivedSecondaryInput;
	bool bAlreadyProcessedInput;
};

struct FDamagerInfo
{
	class UClass* DamageCauser;
	int DamageAmount;
	struct GameplayTagContainer SourceTags;
};

struct FDamageDoneInfo
{
	struct GameplayTagContainer TrackedDamageTagGrouping;
	float DamageAmount;
};

struct FDamageDoneSourceInfo
{
	struct GameplayTagContainer TrackedSourceTags;
	float DamageAmount;
};

struct FDamageDoneTargetInfo
{
	struct GameplayTagContainer TrackedTargetTags;
	TArray<struct DamageDoneSourceInfo> TrackedSourceData;
};

struct FCalloutEntry
{
	struct GameplayTag CalloutTag;
	struct SlateBrush CalloutIcon;
};

struct FFortFeedbackHandle
{
	class UClass* FeedbackBank;
	struct FName EventName;
	bool bReadOnly;
	bool bBankDefined;
	EFortFeedbackBroadcastFilter BroadcastFilterOverride;
};

struct FFortSentenceAudio
{
	struct TSoftClassPtr<UObject> Audio;
	struct FortFeedbackHandle Handle;
};

struct FFortConversationSentence
{
	struct FortSentenceAudio SpeechAudio;
	struct FText SpeechText;
	struct TSoftClassPtr<UObject> TalkingHeadTexture;
	struct FText TalkingHeadTitle;
	struct TSoftClassPtr<UObject> AnimMontage;
	float PostSentenceDelay;
	float DisplayDuration;
};

struct FAthenaBatchedDamageGameplayCues_Shared
{
	struct Vector_NetQuantize10 Location;
	struct Vector_NetQuantizeNormal Normal;
	float Magnitude;
	bool bWeaponActivate;
	bool bIsFatal;
	bool bIsCritical;
	bool bIsShield;
	bool bIsShieldDestroyed;
	bool bIsShieldApplied;
	bool bIsBallistic;
	bool bIsBeam;
	struct Vector_NetQuantize10 NonPlayerLocation;
	struct Vector_NetQuantizeNormal NonPlayerNormal;
	float NonPlayerMagnitude;
	bool NonPlayerbIsFatal;
	bool NonPlayerbIsCritical;
	bool bIsValid;
};

struct FAthenaBatchedDamageGameplayCues_NonShared
{
	class UClass* HitActor;
	class UClass* NonPlayerHitActor;
};

struct FClientAILODSettings
{
	struct ScalableFloat ScoreMultiplier;
	struct ScalableFloat PreloadingPriorityOverride;
	struct ScalableFloat bSupportCharacterMovementOptimization;
};

struct FRecordedGunshot
{
	class UClass* Weapon;
	class UClass* InstigatingFortPawn;
	struct Vector WorldLocation;
	float Strength;
	float Time;
};

struct FFortAIAttributeReplicationProxy
{
	uint32_t Health;
	uint32_t MaxHealth;
};

struct FMinimapGoalByTagColorsData
{
	struct GameplayTagContainer GoalTags;
	struct LinearColor MinimapColor;
};

struct FFortAIAppearanceOverrideEntry
{
	struct FName AppearanceName;
	bool bIsFemale;
	struct TSoftClassPtr<UObject> SkeletalMesh;
	struct TSoftClassPtr<UObject> FeedbackBank;
};

struct FFortBuddyTagListener
{
	class UClass* Actor;
};

struct FAIDirectorEventData
{
	EFortAIDirectorEvent Event;
	struct CurveTableRowHandle DataMax;
	struct CurveTableRowHandle CoolDownRate;
	EFortAIDirectorEventContribution ContributionType;
	EFortAIDirectorEventParticipant OwnerParticipantType;
};

struct FFortAIDirectorFactorContribution
{
	EFortAIDirectorEvent AIDirectorEvent;
	float MaxContribution;
	EFortAIDirectorFactorContribution ContributionType;
};

struct FFortAIDirectorFactorData
{
	EFortAIDirectorFactor AIDirectorFactor;
	TArray<struct FortAIDirectorFactorContribution> ContributingEvents;
	float MaxValue;
};

struct FFortCurveSequenceInstanceInfo
{
};

struct FFortEncounterSettingsFixedPace
{
	float RiftSpawnInterval;
	int RiftSpawnCount;
	int AIMaxCount;
	float SpawnAIIntervalTime;
	int SpawnAIIntervalCount;
};

struct FAIEncounterSpawnGroupWeights
{
};

struct FFortAIPawnUpgradeData
{
	struct CurveTableRowHandle SpawnPointsMultiplierCurve;
	struct CurveTableRowHandle LifespanMultiplierCurve;
	struct CurveTableRowHandle ScoreMultiplierCurve;
	class UClass* ModifierDefinition;
	TArray<class UClass*> AdditionalModifiers;
};

struct FSpawnGroupInstanceInfo
{
	class UClass* SpawnGroup;
	int NumActiveAlive;
	int TotalGroupCost;
	int SpawnPointsUsed;
	int NumEngaged;
	bool bReadyToSpawn;
	bool bFinishedSpawning;
	struct Guid GroupGuid;
	int EnemySpawnDataIndex;
	float TimeSelected;
	int NextEnemyToSpawnIndex;
	struct GameplayTagContainer UpgradeTags;
	struct FortAISpawnGroupUpgradeUIData UpgradeUIData;
	TArray<struct FortAIPawnUpgradeData> PawnUpgrades;
	TArray<class UClass*> ModifiersForAllPawns;
	TArray<class UClass*> PawnList;
};

struct FFortAIEncounterWaveProgressEstimation
{
	float SectionProgressEstimate;
	float SectionStartTime;
	float LastWaveProgressUpdateTime;
	float PeakAndFadeWavePercentage;
	float MaxAdjustmentPerSecond;
	EFortAIWaveProgressSection CurrentSection;
	int NumberOfWaveSegments;
};

struct FFortGoalActorEncounterDataManagerPair
{
	class UClass* GoalActor;
	class UClass* EncounterDataManager;
};

struct FFortSpawnAIRequest
{
	struct Guid SpawnGroupInstanceGuid;
	int EnemyIndex;
	struct Vector SpawnLocation;
	struct Rotator SpawnRotation;
	class UClass* SpawnPoint;
	TArray<class UClass*> AbilitySetsToGrantOnSpawn;
	bool bIgnoreCollisionWhenSpawning;
};

struct FFortAISpawnerData
{
	struct Guid SpawnGroupInstanceGuid;
	struct FortSpawnAIRequest ReservedSpawnRequest;
};

struct FFortAIEncounterQueryData
{
	TArray<struct Vector> QueryLocations;
	TArray<class UClass*> QueryActors;
};

struct FFortAIEncounterTimedModifierTags
{
	float TimeSeconds;
	struct GameplayTagContainer GameplayTags;
};

struct FEncounterEnvironmentQueryInstance
{
	struct EncounterEnvironmentQueryInfo EnvironmentQueryInfo;
	int QueryID;
	bool bIsWaitingForQueryResults;
	EFortEncounterDirection ChosenDirection;
	TArray<struct Vector> QueryLocations;
	TArray<class UClass*> FoundRifts;
	int NumTimesUsed;
};

struct FFortAIEncounterRift
{
	int QueryID;
	struct Vector RiftLocation;
	class UClass* RiftActor;
	struct FortRiftReservationHandle RiftReservationHandle;
};

struct FFortAIEncounterSpawnArea
{
	TArray<struct EncounterEnvironmentQueryInstance> QueryInstances;
	TArray<struct FortAIEncounterRift> PendingRifts;
	TArray<struct FortAIEncounterRift> Rifts;
	TArray<class UClass*> PathEstimators;
	bool bIsActive;
	bool bUsingFallbackQuery;
	EFortEncounterSpawnLocationManagementMode SpawnLocationManagementMode;
};

struct FFortAIEncounterQueryDirectionTracker
{
	bool bHasTriedPreviousDirections;
	TArray<EFortEncounterDirection> PreviousQueryDirections;
	TArray<EFortEncounterDirection> ChosenDirections;
	TArray<EFortEncounterDirection> FailedDirections;
	TArray<EFortEncounterDirection> AvailableDirections;
};

struct FFortEncounterTransitionSettings
{
	bool bShouldMaintainEncounterState;
};

struct FFortGeneratedEncounterSequence
{
	struct FortEncounterTransitionSettings TransitionSettings;
	int StartingGeneratedEncounterProfileIndex;
	int NumEncountersInSequence;
	struct GameplayTagContainer EncounterSequenceTags;
};

struct FEncounterGoalSelectionTableEntry
{
	struct GameplayTagContainer RequiredGameplayTags;
	struct GoalSelectionCriteria GoalSelectionCriteria;
};

struct FFortAIPawnMaterialDefinition
{
	struct TSoftClassPtr<UObject> Material;
	bool bRequireDynamicInstance;
};

struct FFortAIPawnVariantDefinition
{
	class UClass* PawnClass;
	struct CurveTableRowHandle VariantWeightCurve;
	float CurrentWeight;
	struct GameplayTagQuery RequiredTagsQuery;
};

struct FSpawnGroupEnemy
{
	class UClass* EnemyVariantClass;
	bool bOverrideVariantSpawnPointValue;
	int SpawnValue;
};

struct FFortSpawnGroupEncounterTypeData
{
	struct GameplayTagContainer EncounterTypeTags;
	struct CurveTableRowHandle MaxGroupCategoryPopulationDensityCurve;
	struct CurveTableRowHandle RespawnDelayCurve;
};

struct FSpawnGroupProgression
{
	class UClass* SpawnGroup;
};

struct FFortAIPawnUpgrade
{
	struct GameplayTagQuery TagQuery;
	TArray<struct FortAIPawnUpgradeData> PawnUpgradeDataPerPlayerCount;
};

struct FFortAIPawnUpgradeProbability
{
	struct GameplayTagQuery TagQuery;
	struct CurveTableRowHandle UpgradeProbability;
};

struct FFortCosmeticModification
{
	struct TSoftClassPtr<UObject> CosmeticMaterial;
	struct TSoftClassPtr<UObject> AmbientParticleSystem;
	struct TSoftClassPtr<UObject> MuzzleParticleSystem;
	struct TSoftClassPtr<UObject> MuzzleNiagaraSystem;
	struct TSoftClassPtr<UObject> ReloadParticleSystem;
	struct TSoftClassPtr<UObject> BeamParticleSystem;
	struct TSoftClassPtr<UObject> BeamNiagaraSystem;
	struct TSoftClassPtr<UObject> ImpactPhysicalSurfaceEffects;
	TArray<struct TSoftClassPtr<UObject>> ImpactNiagaraPhysicalSurfaceEffects;
	struct TSoftClassPtr<UObject> TracerTemplate;
	bool bModifyColor;
	struct LinearColor ColorAlteration;
	struct FName ColorParameterName;
	bool bModifyDecalColour;
	struct LinearColor DecalColourAlterationStart;
	struct LinearColor DecalColourAlterationEnd;
	bool bModifyShellColour;
	struct LinearColor ShellColourAlteration;
};

struct FFortConditionalCosmeticModification
{
	struct FortCosmeticModification CosmeticModification;
	struct GameplayTagContainer ConditionalTags;
};

struct FFortConditionalIncludeTags
{
	struct GameplayTagContainer ConditionTags;
	struct GameplayTagContainer IncludeTags;
};

struct FFortMultiSizeBrush
{
	struct SlateBrush Brush_XXS;
	struct SlateBrush Brush_XS;
	struct SlateBrush Brush_S;
	struct SlateBrush Brush_M;
	struct SlateBrush Brush_L;
	struct SlateBrush Brush_XL;
};

struct FFortAnalyticsEventBlacklistEntry
{
	EFortAnalyticsEventBlacklistPlaylistKey Type;
	struct FName PlaylistKey;
	struct FString EventName;
	double Probability;
};

struct FCapMipsTextureGroup
{
	ETextureGroup Group;
	int MaxLODMipCount;
};

struct FFortBasicAudioParam
{
	struct FName Name;
	int Value;
};

struct FFortCosmeticVariantTransition
{
	struct GameplayTag ChannelWithin;
	struct GameplayTag VariantFrom;
	struct GameplayTag VariantTo;
};

struct FFortCosmeticVariantSwapData
{
	struct TSoftClassPtr<UObject> ItemToManage;
	TArray<struct FortCosmeticVariantTransition> Transitions;
};

struct FFortMontageInputAction
{
	struct GameplayTag TriggerAbilityTag;
	struct FName NextSection;
	EFortMontageInputType InputType;
};

struct FEmoteRetargetingNotifyParameters
{
	EFortPlayerAnimBodyType BodyTypeToAffect;
	EFortHandIKOverrideType LeftHandIK;
	EFortHandIKOverrideType RightHandIK;
};

struct FSyncedMontageParams
{
	class UClass* SyncedMontage;
	EMontageSyncTargetType MontageTarget;
	EFortCustomPartType PartType;
	float MontageStopBlendTime;
	bool bSyncMontage;
};

struct FEmotePropMaterialScalarParam
{
	struct FName ParamName;
	float ParamValue;
};

struct FFortAnimSetRandomAnimation
{
	struct GameplayTagQuery Requirements;
	struct RandomPlayerSequenceEntry RandomEntry;
};

struct FFortAppActivationSoundMixPair
{
	class UClass* TrueMix;
	class UClass* FalseMix;
};

struct FOverlapRestrictions
{
	int OverlapsPerActor;
	struct GameplayTag OverlapActorTagRestrictions;
};

struct FFortSpawnContext
{
	byte Team;
	struct GameplayTagContainer Tags;
};

struct FWorldHLODStreamingData
{
	Unknown MediumLevelHLODPackageNames;
	Unknown SplineHLODPackageNames;
	TArray<struct FName> MapsToRebuildHLODs;
};

struct FTieredWaveSetData
{
	int EDOIdx;
	float BreatherBetweenWaves;
	EWaveRules WaveRules;
	struct GameplayTag EnemyTypeToKillMod;
	TArray<struct TSoftClassPtr<UObject>> WaveMissions;
	float WaveLengthMod;
	float NumKillsMod;
	float KillPointsMod;
	float DifficultyAddMod;
	bool bDeferTemporaryModifiers;
	struct TSoftClassPtr<UObject> OverrideSpawnPointsMultiplier;
	struct TSoftClassPtr<UObject> OverrideSpawnPointsCurve;
	struct TSoftClassPtr<UObject> OverrideSpawnProgression;
	struct TSoftClassPtr<UObject> OverrideUtilitiesAdjustment;
	struct TSoftClassPtr<UObject> OverrideUtilitiesFree;
	struct TSoftClassPtr<UObject> OverrideUtilitiesLocked;
	struct TSoftClassPtr<UObject> OverrideDistance;
	struct TSoftClassPtr<UObject> OverrideDirectionNumber;
	struct TSoftClassPtr<UObject> OverrideModifierTags;
	struct TSoftClassPtr<UObject> OverrideTimedModifierTags;
};

struct FChoiceDataEntry
{
	struct FText ButtonText;
	struct FText ButtonDescription;
	struct FText ConfirmText;
	bool bEnabled;
	bool bRequireConfirmation;
	bool bCloseAfterSelection;
};

struct FChoiceData
{
	int MenuIdentifier;
	bool bShowCloseButton;
	struct FText Title;
	TArray<struct ChoiceDataEntry> Items;
};

struct FDigestedFocusSetting
{
	struct GameplayTagQuery WeaponTagQuery;
	bool bRequireAmmoToMatch;
	float IgnoreThreatTimeWhenNotAttacking;
	float IgnoreThreatDeviationWhenNotAttacking;
	float IgnoreThreatDuration;
	float IgnoreThreatDurationDeviation;
};

struct FLookAtDigestedSetting
{
	float LookAtDuration;
	float LookAtDurationDeviation;
	float LookAtDelay;
	float LookAtDelayDeviation;
};

struct FDigestedWeaponAccuracy
{
	struct ScalableFloat TrackingOffsetError;
	struct ScalableFloat TargetingTrackingOffsetError;
	struct ScalableFloat TrackingDistanceFarError;
	struct ScalableFloat TargetingTrackingDistanceFarError;
	struct ScalableFloat TrackingDistanceNearError;
	struct ScalableFloat TargetingTrackingDistanceNearError;
	struct ScalableFloat TrackingDistanceNearErrorProbability;
	struct ScalableFloat TargetingActivationProbability;
	struct ScalableFloat FiringRestrictedToTargetingActive;
	float IdealAttackRange;
	float TargetingIdealAttackRange;
	float MaxAttackRange;
	float ChanceToAimAtTargetsFeet;
	struct ScalableFloat ShouldUseProjectileArcForAiming;
	bool bKeepAimingOnSameSideWhileFiring;
	float MaxTrackingHeightOffsetError;
	float MinRotationInterpSpeed;
	float MaxRotationInterpSpeed;
};

struct FDigestedWeaponAccuracyCategorySpecialization
{
	struct GameplayTagContainer Tags;
	struct DigestedWeaponAccuracy WeaponAccuracy;
};

struct FDigestedWeaponAccuracyCategory
{
	struct GameplayTagContainer Tags;
	struct DigestedWeaponAccuracy WeaponAccuracy;
	TArray<struct DigestedWeaponAccuracyCategorySpecialization> Specializations;
};

struct FDigestedTargetBasedAccuracy
{
	float AimTrackingOffsetErrorMultiplier;
	float AimTrackingHeightOffsetErrorMultiplier;
	float AimTrackingDistanceErrorMultiplier;
	float AimTrackingReactionTimeMultiplier;
	float AimTrackingInterpTimeMultiplier;
	float AimTrackingInAirVelocityThresholdMultiplier;
	float AimTrackinginAirHeightDataThresholdMultiplier;
};

struct FDigestedTargetBasedAccuracyCategory
{
	struct GameplayTagContainer Tags;
	struct DigestedTargetBasedAccuracy TargetBasedAccuracy;
};

struct FDigestedTrackingOffsetModifierCurves
{
	int Values;
};

struct FDigestedTrackingOffsetModifiers
{
	float CombatStartDuration;
	float TargetLowHealthThreshold;
	struct DigestedTrackingOffsetModifierCurves Curves;
	struct DigestedTrackingOffsetModifierCurves DistanceCurves;
	struct DigestedTrackingOffsetModifierCurves InAirHeightCurve;
};

struct FFocusSetting
{
	struct GameplayTagQuery WeaponTagQuery;
	bool bRequireAmmoToMatch;
	struct ScalableFloat IgnoreThreatTimeWhenNotAttacking;
	struct ScalableFloat IgnoreThreatDeviationWhenNotAttacking;
	struct ScalableFloat IgnoreThreatDuration;
	struct ScalableFloat IgnoreThreatDurationDeviation;
};

struct FLookAtSetting
{
	struct ScalableFloat LookAtDuration;
	struct ScalableFloat LookAtDurationDeviation;
	struct ScalableFloat LookAtDelay;
	struct ScalableFloat LookAtDelayDeviation;
};

struct FWeaponAccuracy
{
	struct ScalableFloat MaxTrackingOffsetError;
	struct ScalableFloat TargetingMaxTrackingOffsetError;
	struct ScalableFloat MaxTrackingDistanceFarError;
	struct ScalableFloat TargetingMaxTrackingDistanceFarError;
	struct ScalableFloat MaxTrackingDistanceNearError;
	struct ScalableFloat TargetingMaxTrackingDistanceNearError;
	struct ScalableFloat TrackingDistanceNearErrorProbability;
	struct ScalableFloat TargetingActivationProbability;
	struct ScalableFloat FiringRestrictedToTargetingActive;
	struct ScalableFloat IdealAttackRange;
	struct ScalableFloat TargetingIdealAttackRange;
	struct ScalableFloat MaxAttackRangeFactor;
	struct ScalableFloat ShouldAimAtTargetsFeet;
	struct ScalableFloat ShouldUseProjectileArcForAiming;
	struct ScalableFloat KeepAimingOnSameSideWhileFiring;
	struct ScalableFloat MaxTrackingHeightOffsetError;
	struct ScalableFloat MinRotationInterpSpeed;
	struct ScalableFloat MaxRotationInterpSpeed;
};

struct FWeaponAccuracyCategorySpecialization
{
	struct GameplayTagContainer Tags;
	struct WeaponAccuracy WeaponAccuracy;
};

struct FWeaponAccuracyCategory
{
	struct GameplayTagContainer Tags;
	struct WeaponAccuracy WeaponAccuracy;
	TArray<struct WeaponAccuracyCategorySpecialization> Specializations;
};

struct FTargetBasedAccuracy
{
	struct ScalableFloat AimTrackingOffsetErrorMultiplier;
	struct ScalableFloat AimTrackingHeightOffsetErrorMultiplier;
	struct ScalableFloat AimTrackingDistanceErrorMultiplier;
	struct ScalableFloat AimTrackingReactionTimeMultiplier;
	struct ScalableFloat AimTrackingInterpTimeMultiplier;
	struct ScalableFloat AimTrackingInAirVelocityThresholdMultiplier;
	struct ScalableFloat AimTrackinginAirHeightDataThresholdMultiplier;
};

struct FTargetBasedAccuracyCategory
{
	struct GameplayTagContainer Tags;
	struct TargetBasedAccuracy TargetBasedAccuracy;
};

struct FTrackingOffsetModifiers
{
	struct ScalableFloat CombatStartDuration;
	struct ScalableFloat TargetLowHealthThreshold;
	struct ScalableFloat Curves;
	struct ScalableFloat DistanceCurves;
	struct ScalableFloat InAirHeightCurve;
};

struct FTrackingOffsetModifierInfo
{
	struct ScalableFloat MinSkill;
	struct ScalableFloat MaxSkill;
	struct TrackingOffsetModifiers Modifiers;
};

struct FFortAthenaAIBotBuildDescriptor
{
	struct IntVector Location;
	EOrientedConstructionBuildingType BuildingType;
};

struct FAthenaFortAIBotDigestedWeightedBuildingList
{
	float Weight;
	TArray<struct FortAthenaAIBotBuildDescriptor> BuildDescriptors;
};

struct FAthenaFortAIBotWeightedBuildingList
{
	struct ScalableFloat Weight;
	TArray<struct FortAthenaAIBotBuildDescriptor> BuildDescriptors;
};

struct FFortBotDigestedHealingItems
{
	float UseItemResourceThreshold;
	struct GameplayTagContainer ItemTags;
};

struct FFortBotDigestedHealingItemsList
{
	TArray<struct FortBotDigestedHealingItems> HealthItems;
};

struct FFortBotDigestedHealingItemsSpec
{
	struct GameplayTagQuery TagQuery;
	struct FortBotDigestedHealingItemsList HealthItemsList;
};

struct FFortBotHealingItems
{
	struct ScalableFloat UseItemResourceThreshold;
	struct GameplayTagContainer ItemTags;
};

struct FFortBotHealingItemsList
{
	TArray<struct FortBotHealingItems> HealthItems;
};

struct FFortBotHealingItemsSpec
{
	struct GameplayTagQuery TagQuery;
	struct FortBotHealingItemsList HealthItemsList;
};

struct FDigestedBotEquipWeaponInfo
{
	struct GameplayTagQuery TagQuery;
	struct ScalableFloat DistanceEffectivenessWithThreat;
	struct ScalableFloat DistanceEffectivenessNoThreat;
};

struct FBotEquipWeaponInfo
{
	struct GameplayTagQuery TagQuery;
	struct ScalableFloat DistanceEffectivenessWithThreat;
	struct ScalableFloat DistanceEffectivenessNoThreat;
};

struct FWeaponAmmoCheat
{
	struct GameplayTag WeaponTag;
	struct ScalableFloat HasInfiniteAmmo;
	struct ScalableFloat CheckLoadedAmmoForInfiniteAmmo;
};

struct FDigestedBotKnockbackSettings
{
	struct GameplayTagQuery TagQuery;
	bool bShouldAllowCharacterToBeLaunched;
	bool bShouldStopActiveMovement;
	float IgnoreMoveInputDuration;
};

struct FBotKnockbackSettings
{
	struct GameplayTagQuery TagQuery;
	struct ScalableFloat ShouldAllowCharacterToBeLaunched;
	struct ScalableFloat ShouldStopActiveMovement;
	struct ScalableFloat IgnoreMoveInputDuration;
};

struct FFortAthenaAIBotNameRegionData
{
	struct FString RegionId;
	struct TSoftClassPtr<UObject> NameDataTable;
};

struct FDigestedSightReactionSpecialization
{
	struct GameplayTagContainer Tags;
	struct ScalableFloat SightMinAdditionTime;
	struct ScalableFloat SightMaxAdditionTime;
};

struct FDigestedPerceptionStateSettings
{
	float ForgetTime;
	float ForgetTimeDeviation;
	float ForgetDistance;
	float ForgetDistanceDeviation;
};

struct FSoundPerceptionDigestedSetting
{
	float Loudness;
	float IgnoreTime;
	float OverrideReactionDistanceSq;
};

struct FTrapPerceptionSettings
{
};

struct FSightReactionSpecialization
{
	struct GameplayTagContainer Tags;
	struct ScalableFloat SightMinAdditionTime;
	struct ScalableFloat SightMaxAdditionTime;
};

struct FPerceptionStateSettings
{
	struct ScalableFloat ForgetTime;
	struct ScalableFloat ForgetTimeDeviation;
	struct ScalableFloat ForgetDistance;
	struct ScalableFloat ForgetDistanceDeviation;
	struct ScalableFloat ThreatDistanceWeight;
};

struct FSoundPerceptionSetting
{
	struct ScalableFloat Loudness;
	struct ScalableFloat IgnoreTime;
	struct ScalableFloat OverrideReactionDistance;
};

struct FPlaystyleSwitchToAggressiveDataDigested
{
	struct GameplayTagQuery TagQueryToMatch;
	float TurnToAggressiveMinimumDistanceSquared;
	float TurnToAggressiveTime;
	float DamageThresholdToSwitchToAggressive;
	float SwitchBackToDefensivePreventionTime;
};

struct FPlaystyleSwitchToAggressiveData
{
	struct GameplayTagQuery TagQueryToMatch;
	struct ScalableFloat TurnToAggressiveMinimumDistance;
	struct ScalableFloat TurnToAggressiveTime;
	struct ScalableFloat SwitchBackToDefensivePreventionTime;
	struct ScalableFloat DamageThresholdToSwitchToAggressive;
};

struct FDigestedRangedWeaponSkill
{
	float DelayBetweenShots;
	float DelayDeviationTimeBetweenShots;
	float TriggerHoldDuration;
	float TriggerHoldDeviationTime;
	float DelayBeforeFirstShot;
	float ShotDelayAfterTargeting;
};

struct FDigestedRangedWeaponSkillCategorySpecialization
{
	struct GameplayTagContainer Tags;
	struct DigestedRangedWeaponSkill WeaponSkill;
};

struct FDigestedRangedWeaponSkillCategory
{
	struct GameplayTagContainer Tags;
	struct DigestedRangedWeaponSkill WeaponSkill;
	TArray<struct DigestedRangedWeaponSkillCategorySpecialization> Specializations;
};

struct FRangedWeaponSkill
{
	struct ScalableFloat DelayBetweenShots;
	struct ScalableFloat DelayDeviationTimeBetweenShots;
	struct ScalableFloat TriggerHoldDuration;
	struct ScalableFloat TriggerHoldDeviationTime;
	struct ScalableFloat DelayBeforeFirstShot;
	struct ScalableFloat ShotDelayAfterTargeting;
};

struct FRangedWeaponSkillCategorySpecialization
{
	struct GameplayTagContainer Tags;
	struct RangedWeaponSkill WeaponSkill;
};

struct FRangedWeaponSkillCategory
{
	struct GameplayTagContainer Tags;
	struct RangedWeaponSkill WeaponSkill;
	TArray<struct RangedWeaponSkillCategorySpecialization> Specializations;
};

struct FFortAthenaAILODSetting
{
	bool bIsValid;
};

struct FAITrackedObjectsSet
{
};

struct FAircraftFlightInfo
{
	struct Vector_NetQuantize100 FlightStartLocation;
	struct Rotator FlightStartRotation;
	float FlightSpeed;
	float TimeTillFlightEnd;
	float TimeTillDropStart;
	float TimeTillDropEnd;
};

struct FSpawnerDataComponentAffiliationSharedBBConfiguration
{
	struct GameplayTag FactionTag;
	struct ScalableFloat MaximumDistance;
	struct GameplayTag SharedBlackboardTag;
	class UClass* SharedBlackboard;
};

struct FConstructionBuildingList
{
	struct ConstructionBuildingInfo BuildingList;
};

struct FInitialGameplayEffectInfo
{
	class UClass* GameplayEffect;
	float Level;
};

struct FWeightedAIInventoryLoadout
{
	struct ScalableFloat Weight;
	TArray<struct ItemAndCount> Items;
};

struct FLocalizedStringPair
{
	struct FString Locale;
	struct FString TextLiteral;
};

struct FCreativeIslandMatchmakingSettings
{
	int MinimumNumberOfPlayers;
	int MaximumNumberOfPlayers;
	struct FString MmsType;
	struct FString MmsPrivacy;
	struct FString Override_Playlist;
	int PlayerCount;
	int NumberOfTeams;
	int PlayersPerTeam;
	bool bAllowJoinInProgress;
	EJoinInProgress JoinInProgressType;
	byte JoinInProgressTeam;
};

struct FCreativeLoadedLinkData
{
	struct FString CreatorName;
	struct FString SupportCode;
	struct FString Mnemonic;
	EMMSPrivacy Privacy;
	int Version;
	TArray<struct LocalizedStringPair> LinkTitle;
	struct FText AltTitle;
	TArray<struct LocalizedStringPair> LinkTagline;
	TArray<struct FString> DescriptionTags;
	TArray<struct LocalizedStringPair> IslandIntroduction;
	struct FString LinkYoutubeId;
	struct FString ImageUrl;
	struct FString IslandType;
	struct FString QuestContextTag;
	struct FString AccountId;
	struct CreativeIslandMatchmakingSettings MatchmakingSettings;
	TArray<struct FString> PlaylistOptions;
	struct FString LinkType;
};

struct FAileronRoll
{
	struct FortRechargingActionTimer Action;
	EAileronRollDirection Direction;
};

struct FActiveVehicleUI
{
	class UClass* ActiveWidget;
};

struct FVehicleSpecificUIDetails
{
	class UClass* WidgetClass;
	EUIExtensionSlot Slot;
	TArray<EVehicleSeats> ValidSeats;
};

struct FReplicatedControlState
{
	struct Vector_NetQuantizeNormal Up;
	struct Vector_NetQuantizeNormal Forward;
	bool bIsEngineOn;
};

struct FLocationLerpData
{
	struct Vector PositionLerp_Start;
	struct Vector PositionLerp_End;
	struct Vector PositionLerp_Target;
	float TotalLerpTime;
};

struct FRotationLerpData
{
	struct Quat RotationLerp_Start;
	struct Quat RotationLerp_End;
	struct Quat RotationLerp_Target;
	float TotalLerpTime;
};

struct FExitCraftInfo
{
	struct GameplayTag RequiredExitCraftTag;
	struct ScalableFloat ExitCraftSpawnerZOffset;
	struct ScalableFloat ExitCraftSpawnDelay;
	struct ScalableFloat SpawnDestructionInitialDelay;
	struct ScalableFloat SpawnDestructionDelayBetweenPieces;
	struct ScalableFloat ExitCraftZOffset;
	struct ScalableFloat ExitCraftTargetZOffset;
	struct ScalableFloat ExitCraftTimeToHoverLocation;
	struct ScalableFloat ExitCraftTimeToHoverRotation;
	struct ScalableFloat MinLandingSpeed;
	struct ScalableFloat ExitZOffset;
	struct ScalableFloat ExitTime;
	struct ScalableFloat InteractionTime;
};

struct FFortAthenaTeamHUDInfo
{
	struct FText DisplayName;
	struct LinearColor Color;
};

struct FAthenaLoadoutEntry
{
	class UClass* ItemToGrant;
	struct ScalableFloat NumberOfItemsToGrant;
	int DesiredSlot;
};

struct FFortTreasureChestSpawnInfo
{
	class UClass* TreasureChestClass;
	struct ScalableFloat TreasureChestMinSpawnPercent;
	struct ScalableFloat TreasureChestMaxSpawnPercent;
	struct ScalableFloat GoldenPoiTreasureChestMinSpawnPercent;
	struct ScalableFloat GoldenPoiTreasureChestMaxSpawnPercent;
};

struct FFortAmmoBoxSpawnInfo
{
	class UClass* AmmoBoxClass;
	struct ScalableFloat AmmoBoxMinSpawnPercent;
	struct ScalableFloat AmmoBoxMaxSpawnPercent;
	struct ScalableFloat GoldenPoiAmmoBoxMinSpawnPercent;
	struct ScalableFloat GoldenPoiAmmoBoxMaxSpawnPercent;
};

struct FBuildingGameplayActorSpawnDetails
{
	class UClass* BuildingGameplayActorClass;
	class UClass* TargetActorClass;
	struct ScalableFloat SpawnHeight;
	struct ScalableFloat GlobalMaxBGAs;
	struct ScalableFloat MinNumToSpawnPerPhase;
	struct ScalableFloat MaxNumToSpawnPerPhase;
	struct ScalableFloat MinTimeToStartSpawnningBGAs;
	struct ScalableFloat MaxTimeToStartSpawnningBGAs;
	struct ScalableFloat MinTimeBetweenBGASpawns;
	struct ScalableFloat MaxTimeBetweenBGASpawns;
	struct ScalableFloat MinTimeToRepeatSpawningBGAs;
	struct ScalableFloat MaxTimeToRepeatSpawningBGAs;
};

struct FFortSafeZoneVolumeDefinition
{
	class UClass* Volume;
	struct ScalableFloat RejectionChance;
};

struct FFortSafeZoneDefinition
{
	struct ScalableFloat Count;
	struct ScalableFloat Radius;
	struct ScalableFloat ForceDistanceMin;
	struct ScalableFloat ForceDistanceMax;
	struct ScalableFloat RejectRadius;
	struct ScalableFloat RejectOuterDistance;
	struct ScalableFloat WaitTime;
	struct ScalableFloat ShrinkTime;
	struct ScalableFloat MegaStormGridCellThickness;
	struct ScalableFloat PlayerCapSolo;
	struct ScalableFloat PlayerCapDuo;
	struct ScalableFloat PlayerCapSquad;
};

struct FAshtonStoneData
{
	EAshtonStoneType StoneType;
	class UClass* StoneItemDefinition;
	struct SlateBrush StoneIconBrush;
	struct Vector2D MapIconScale;
	struct GameplayTag GameplayTag;
	struct GameplayTag PickupTag;
	EAshtonStoneStateType InitialStoneState;
};

struct FFortPieSliceSpawnData
{
	struct ScalableFloat SpawnDirection;
	struct ScalableFloat SpawnDirectionDeviation;
	struct ScalableFloat MinSpawnDistanceFromCenter;
	struct ScalableFloat MaxSpawnDistanceFromCenter;
};

struct FAshtonStoneState
{
	EAshtonStoneType StoneType;
	EAshtonStoneStateType StoneState;
	struct GameplayTag GameplayTag;
	float SpawnTime;
	bool bHasEverSpawned;
	int SpawnDataIdx;
};

struct FFortRespawnLogicData
{
	struct ScalableFloat DirectionDeviation;
	struct ScalableFloat MinDistFromCenterPercent;
	struct ScalableFloat MaxDistFromCenterPercent;
	struct ScalableFloat SingleBusRespawnNearDeathLocation;
	struct ScalableFloat MinHeightFromGround;
	struct ScalableFloat MinHeightFromZero;
	struct ScalableFloat CameraDistance;
	struct ScalableFloat RespawnTraceHeight;
};

struct FBagelAreaSpecialActorData
{
	struct GameplayTag SpecialActorTag;
	struct SlateBrush SpecialActorMinimapIconBrush;
	struct Vector2D SpecialActorMinimapIconScale;
	struct SlateBrush SpecialActorCompassIconBrush;
	struct Vector2D SpecialActorCompassIconScale;
	bool bShouldDrawCompassIcon;
};

struct FBagelDifficultySettings
{
	struct ScalableFloat SpawnCountPlayerCountMultiplier;
	struct ScalableFloat SpawnCountDifficultyMultiplier;
	struct ScalableFloat AIEffectPlayerCountMultiplier;
	struct ScalableFloat AIEffectDifficultyMultiplier;
	float SpawnCountMultiplier;
	float AIEffectMultiplier;
};

struct FBagelLootTierOverrideAssetData
{
	int SafeZoneIndex;
	struct FName TieredGroup;
};

struct FBagelScoreData
{
	class UClass* ActorClass;
	struct GameplayTagContainer Tags;
	struct GameplayTagContainer ExclusionTags;
	struct ScalableFloat ScoreValue;
	struct FText ScoreText;
};

struct FBagelScoreMultiplierInstanceData
{
	class UClass* Actor;
	struct FName SpecialActorID;
};

struct FBagelObjectiveAreaInstanceData
{
	struct Vector Location;
	class UClass* AreaActor;
	struct FName SpecialActorID;
	TArray<struct BagelScoreMultiplierInstanceData> ScoreMultipliers;
};

struct FRiftDamagerInfo
{
	class UClass* Rift;
	TArray<class UClass*> Damagers;
};

struct FBagelLeaderboardEntry
{
	struct FString DisplayName;
	int Value;
	int Rank;
	struct UniqueNetIdRepl UserNetId;
	bool bIsSpecialEntry;
	bool bIsLocalPlayer;
};

struct FBagelFriendsLeaderboard
{
	struct FString Name;
	TArray<struct BagelLeaderboardEntry> LeaderBoard;
};

struct FBagelLeaderboardQuery
{
	struct UniqueNetIdRepl RequestingId;
};

struct FBarrierHeadData
{
	TArray<class UClass*> PartsToSwapIn;
};

struct FBarrierMountedTurretData
{
	TArray<class UClass*> MaterialOverrides;
	class UClass* BaseMaterialOverride;
};

struct FBarrierTeamState
{
	byte TeamNum;
	EBarrierFoodTeam FoodTeam;
	class UClass* ObjectiveFlag;
	class UClass* ObjectiveObject;
	bool bRespawnEnabled;
};

struct FLimitedLifeByTeamData
{
	bool bUseTeamPooledLives;
	int Lives;
};

struct FObjectiveSpecialActorContainer
{
	class UClass* TheSpawnedObjective;
};

struct FSpawningInfo
{
	class UClass* ItemSpawnData;
};

struct FMMRSpawningBaseRuntimeInfo
{
};

struct FCenterOnTaggedPOI
{
	struct ScalableFloat Enabled;
	struct GameplayTag TagForPOI;
	struct ScalableFloat WithinRadius;
};

struct FTaggedPOIList
{
	struct ScalableFloat Enabled;
	struct ScalableFloat ChanceToApply;
	TArray<struct CenterOnTaggedPOI> List;
};

struct FCenterOnLocationPOI
{
	struct ScalableFloat Enabled;
	struct ScalableFloat RelativeToPreviousChosenLocation;
	struct Vector LocationForPOI;
	struct ScalableFloat WithinRadius;
};

struct FLocationPOIList
{
	struct ScalableFloat Enabled;
	struct ScalableFloat ChanceToApply;
	TArray<struct CenterOnLocationPOI> List;
};

struct FFortSquadStartSearchParamData
{
	struct GameplayTagContainer FilterTags;
	struct ScalableFloat DistanceMinimum;
	struct ScalableFloat DistanceMaximum;
	struct ScalableFloat bDistance2D;
	struct ScalableFloat bShuffle;
};

struct FChromeRoute
{
	struct FortSquadStartSearchParamData SquadStartSearchParam;
	TArray<struct GameplayTag> FinishPOITags;
};

struct FTaggedPOI
{
	struct GameplayTag POITag;
	struct ScalableFloat Enabled;
	struct Vector SpawnOffset;
	struct ScalableFloat SelectFromAvailableShrinkLocation;
	TArray<struct Vector2D> AvailableShrinkLocations;
	struct FText POIOverrideName;
};

struct FCobaltWidgetMatchData
{
	TArray<byte> Teams;
	struct GameplayTag POITag;
};

struct FCobaltWidgetRoundData
{
	int8_t RoundNumber;
	int8_t FirstRound;
	TArray<struct CobaltWidgetMatchData> CurrentRoundMatches;
	TArray<byte> PreviousRoundWinners;
};

struct FCosmeticOverrideData
{
	EAthenaCustomizationCategory SlotName;
	class UClass* CosmeticItem;
};

struct FStatEventFilter
{
	EFortQuestObjectiveStatEvent StatEvent;
	struct GameplayTagContainer TargetTagsQuery;
	struct GameplayTagContainer SourceTagsQuery;
	struct GameplayTagContainer ContextTagsQuery;
};

struct FCrucibleSegmentData
{
	bool bRegistered;
	int NumAI;
	int NumTargets;
	float MissedTargetPenalty;
	EFortCrucibleStatType BackendStatType;
};

struct FCrucibleCourseData
{
	TArray<struct CrucibleSegmentData> SegmentsData;
};

struct FCrucibleStatValue
{
	float BestTime;
	int64_t RawBestTime;
	EFortCrucibleStatSource Source;
	struct FString BackendStatName;
};

struct FCrucibleSegmentResults
{
	int SegmentId;
	float CalculatedScore;
	float CalculatedPenalty;
	int CalculatedMissedTargets;
	float StartTime;
	float FinishTime;
	float CancelTime;
	int NumAIElims;
	int NumTargetElims;
};

struct FCrucibleCourseResults
{
	float CalculatedScore;
	float CalculatedTotalPenalty;
	int CalculatedMissedTargets;
	int CalculatedSpawnedTargets;
	TArray<struct CrucibleSegmentResults> SegmentResults;
	float StartTime;
	float FinishTime;
	float CancelTime;
	bool bInputMethodWasKBMAtAnyPoint;
};

struct FPartOverrideData
{
	EFortCustomGender Gender;
	TArray<class UClass*> PartsToSwapIn;
	TArray<class UClass*> DefaultParts;
	struct GameplayTag CosmeticSwapTag;
};

struct FCustomCharacterPartsByKillOverrideData
{
	struct ScalableFloat KillThreshold;
	TArray<struct PartOverrideData> PartOverrideData;
};

struct FSafeZoneRoute
{
	struct ScalableFloat bIsEnabled;
	struct ScalableFloat bUsePOIStartLocation;
	struct ScalableFloat bUsePOINameOverride;
	struct FText POINameOverride;
	struct ScalableFloat StartRadius;
	struct ScalableFloat EndRadius;
	struct GameplayTag StartPOITag;
	struct GameplayTag EndPOITag;
	struct Vector StartLocation;
	struct Vector EndLocation;
};

struct FFortDadbroPickupDespawnData
{
	float DespawnTime;
	class UClass* PickUp;
};

struct FDBNOCustomSettings
{
	EDBNOMutatorType Enabled;
	float TenacityDepletionRate;
	float ReviveHealthPercentage;
	bool bAllowRevives;
	bool bAllowCarry;
	float TimeToRevive;
	bool bAlertTeam;
	bool LastManStandingMode;
	bool bAllowInterrogation;
	bool bAllowInterrogationReveal;
	int SelectedTeam;
	bool bTeamAffectsAllButSelected;
	int SelectedClass;
	bool bClassAffectsAllButSelected;
};

struct FDBNOSettingsByActor
{
	class UClass* Actor;
	struct DBNOCustomSettings Settings;
};

struct FFortAthenaMutator_SurvivalObjectiveData
{
	class UClass* BuildingActorObjectiveClass;
	struct ScalableFloat SpawnDistanceFromGround;
	int ActivationSafeZoneIndex;
	bool bEndMatchOnDestroy;
	bool bSpawnOnPOI;
	bool bClearAreaOnSpawn;
	float ClearAreaRadiusOverride;
	float ClearAreaHalfHeightOverride;
	bool bIsSpecialActor;
	struct GameplayTagQuery POIFilterQuery;
	int RandomizedPOICount;
	TArray<int> ExtraSafezoneIndexes;
	class UClass* SpawnedBuildingActorObjective;
	struct GameplayTag SpecialActorTag;
	struct SlateBrush SpecialActorMinimapIconBrush;
	struct Vector2D SpecialActorMinimapIconScale;
	struct SlateBrush SpecialActorCompassIconBrush;
	struct Vector2D SpecialActorCompassIconScale;
	struct FName SpecialActorID;
};

struct FControlPointAssetData
{
	class UClass* CapturePointClass;
	struct ScalableFloat SpawnDistanceFromGround;
	struct Vector2D MiniMapIconScale;
	struct Vector2D CompassIconScale;
};

struct FControlPointInstanceData
{
	class UClass* ControlPoint;
	EControlPointState ControlPointState;
	int SpawnDataIdx;
	float SpawnTime;
	float EnableTime;
	float DisableTime;
	byte PrevOwningTeam;
	class UClass* CachedOwningTeamInfo;
	float PointAccrualTime;
	float PointsRemainder;
	float BonusPointAccrualTime;
	float BonusPointsRemainder;
	float CachedPointAccrualValue;
	float CachedBonusPointAccrualValue;
	bool bPointFinished;
	int CachedSafeZonePhaseWhenToSpawn;
	bool bIgnoreForOrderMessaging;
	bool bAlwaysInPlay;
	float TimeOfShutdown;
};

struct FEQSActorSpawnerClassToSpawnData
{
	struct FName SharedAssetID;
	struct ScalableFloat Weight;
	EEQSActorSpawnerSpawnType SpawnActorType;
	class UClass* ActorClassToSpawn;
	float AdjustSpawnedActorToGroundLocationTraceZOffset;
	class UClass* AISpawnerData;
	class UClass* PickupClassToSpawn;
	class UClass* PickupItemDefinition;
};

struct FEQSActorSpawnerData
{
	struct FName NameId;
	bool bEnabled;
	EEQSActorSpawnerTriggerType SpawningTrigger;
	EFortSafeZoneState SafeZoneStateToStartSpawning;
	int SafeZoneIndexToSpawnIn;
	struct ScalableFloat DelayBeforeSpawning;
	struct ScalableFloat TimeBetweenSpawns_Min;
	struct ScalableFloat TimeBetweenSpawns_Max;
	struct ScalableFloat TotalActorsToSpawn_Min;
	struct ScalableFloat TotalActorsToSpawn_Max;
	struct ScalableFloat DelayBeforeSpawning_BySafeZoneIndex;
	struct ScalableFloat TimeBetweenSpawns_Min_BySafeZoneIndex;
	struct ScalableFloat TimeBetweenSpawns_Max_BySafeZoneIndex;
	struct ScalableFloat TotalActorsToSpawn_Min_BySafeZoneIndex;
	struct ScalableFloat TotalActorsToSpawn_Max_BySafeZoneIndex;
	struct ScalableFloat bSpawnInCenterOfBuildingGridCell;
	struct Vector SpawnLocationOffset;
	TArray<class UClass*> SpawnerHelperDestroyActorsInAreaClasses;
	struct ScalableFloat TotalRetryAttemptsOnSpawnFailure;
	bool bAvoidRepeatClassSpawning;
	TArray<struct EQSActorSpawnerClassToSpawnData> SpawnClassesData;
	class UClass* BasePlacementQuery;
	class UClass* SpawnActorPlacementQuery;
	bool bIncludeMutatorBasePlacementQueryResults;
	TArray<struct EnvNamedValue> SpawnActorPlacementQueryParams;
	class UClass* SpawnedActorRemovalQuery;
	float BaseQueryingAttemptIntervalTimeSeconds;
	float SpawnedActorRemovalQueryInterval;
};

struct FEmoteActionBinding
{
	struct PrimaryAssetId EmoteId;
	bool bOnlyDisplayForDiscoverability;
	struct FName ActionName;
	TArray<struct InputActionKeyMapping> Input;
};

struct FExternalEmoteCategory
{
	struct FName CategoryName;
	struct FText CategoryTitle;
	struct FText CategoryTitleMultipage;
	TArray<struct EmoteActionBinding> Emotes;
	bool bExclusive;
};

struct FTargetDataEntry
{
	TArray<class UClass*> Targets;
	struct GameplayTag FoundationTag;
	struct ScalableFloat HealthPercentRequiredToMoveOn;
	bool bFindInStormCircle;
	TArray<int> StormCircleIndices;
	struct ScalableFloat NumberOfFoundationsToFind;
};

struct FTeamSetupDataEntry
{
	byte TeamNum;
	TArray<class UClass*> PartsToSwapIn;
};

struct FHotfixableInventoryOverrideItem
{
	struct ScalableFloat Count;
	class UClass* Item;
};

struct FItemLoadoutContainer
{
	struct ScalableFloat bEnabled;
	TArray<struct ItemAndCount> Loadout;
	TArray<struct HotfixableInventoryOverrideItem> LoadoutList;
};

struct FItemLoadoutTeamMap
{
	byte TeamIndex;
	byte LoadoutIndex;
	EAthenaInventorySpawnOverride UpdateOverrideType;
	EAthenaLootDropOverride DropAllItemsOverride;
};

struct FInventoryOverrideLoadoutRandomization
{
};

struct FFortAthenaMutator_GamePhaseMessageData
{
	bool bSendIfPhaseSkipped;
	TArray<struct AthenaGameMessageData> Messages;
};

struct FGunGameGunEntry
{
	class UClass* Weapon;
	struct PrimaryAssetId WeaponAssetId;
	struct ScalableFloat Enabled;
	struct ScalableFloat AwardAtElim;
	bool bShowHarvestingToolOnLadder;
};

struct FGunGameGunEntries
{
	TArray<struct GunGameGunEntry> Entries;
};

struct FGunGamePlayerData
{
	TArray<class UClass*> CurrentlyAssignedWeapons;
};

struct FCosmeticsToApplyOnItemPickupData
{
	struct ScalableFloat bEnabled;
	class UClass* GadgetItemDefinition;
	class UClass* PickaxeItemDefinition;
	struct GameplayTag GameplayCueTag;
};

struct FItemsToGiveAtPhase
{
	class UClass* ItemToDrop;
	struct ScalableFloat NumberToGive;
};

struct FItemsToGive
{
	class UClass* ItemToDrop;
	struct ScalableFloat NumberToGive;
};

struct FGravityMovementData
{
	float GravityZScale;
	float VehicleGravityZScale;
	float JumpZVelocityOverride;
	float JumpHorizontalAccelerationOverride;
	float JumpHorizontalVelocityOverride;
};

struct FHeistExitCraftData
{
	class UClass* ExitCraftSpawner;
	class UClass* SpawnedExitCraft;
	EHeistExitCraftState ExitCraftState;
	TArray<class UClass*> DepartedPawns;
	float SpawnTime;
	bool bIsUsed;
	bool bHasDeparted;
};

struct FHeistTeamHoldingJewelInfo
{
	int JewelsHeld;
	float TimeStartedHoldingJewel;
	float AccumulatedTotalTime;
};

struct FHUDElementsToHideData
{
	EAthenaGamePhase StartHidingGamePhase;
	int StartHidingSafeZonePhase;
	EAthenaGamePhase StopHidingGamePhase;
	int StopHidingSafeZonePhase;
	struct GameplayTagContainer HUDElementsToHide;
};

struct FHUDElementVisibilityRepData
{
	struct GameplayTagContainer HiddenHUDElements;
	struct GameplayTagContainer ShownHUDElements;
	TArray<EFortHUDElementVisibiltyOption> WorldResourceVisibilities;
	EFortReticleVisibiltyOption ReticleVisibilityOption;
};

struct FInfiltrationTeamInfo
{
	byte TeamNum;
	struct GameplayTag TeamTag;
};

struct FIconDisplayData
{
	struct SlateBrush MiniMap_Brush;
	struct SlateBrush Compass_Brush;
	struct Vector2D MapSize;
	struct Vector2D CompassSize;
};

struct FIntelState
{
	bool bInRange;
	float TimeRemaining;
	float ServerEndTime;
	float ServerGroundTimerEnd;
	EIntelStateEnum IntelState;
	byte WinningTeam;
	byte AttackingTeam;
	byte DefendingTeam;
};

struct FInfiltrationModeState
{
	int IntelDownloaded;
	int IntelCaptured;
	float TotalTime;
	float TotalGroundTime;
	TArray<struct IntelState> IntelStates;
	int CurrentRound;
	bool bGameOver;
	struct GameplayTag CurrentPOITag;
};

struct FInfiltrationPOIInfo
{
	struct GameplayTag POITag;
	TArray<class UClass*> IntelSpawnPoints;
	TArray<class UClass*> IntelCapturePoints;
	struct FText POIOverrideName;
};

struct FRoundCosmeticInfo
{
	struct FText RoundNameText;
	class UClass* RoundSound;
};

struct FItemLoadoutBucket
{
	struct ScalableFloat bEnabled;
	TArray<struct ItemLoadoutContainer> Loadouts;
};

struct FTeamBucketPartOverrideData
{
	EFortCustomGender Gender;
	EFortCustomBodyType BodyType;
	TArray<class UClass*> PartsToSwapIn;
	bool bShouldRemoveExtras;
	TArray<class UClass*> PartsToAddIfSpecialTags;
	struct GameplayTag CosmeticSwapTag;
};

struct FTeamBucketCosmeticLoadoutContainer
{
	struct ScalableFloat bEnabled;
	struct GameplayTagContainer SkinMetaTagsToSkip;
	TArray<class UClass*> PartsToSwapInToRemoveExtras;
	struct GameplayTagContainer SpecialTags;
	struct GameplayTagContainer ExtraSpecialTags;
	TArray<struct TeamBucketPartOverrideData> PartOverrides;
};

struct FTeamBucketLoadout
{
	struct TeamBucketCosmeticLoadoutContainer CosmeticLoadout;
	struct ItemLoadoutContainer ItemLoadout;
};

struct FTeamBucketDefinition
{
	struct ScalableFloat bEnabled;
	TArray<struct TeamBucketLoadout> Loadouts;
};

struct FItemsToDropOnDeath
{
	class UClass* ItemToDrop;
	struct ScalableFloat NumberToDrop;
};

struct FAthenaJumpPenalty
{
	float JumpScalar;
	float MovementScalar;
};

struct FPlayerLoudoutEntry
{
	class UClass* ItemToGrant;
	struct ScalableFloat NumberOfItemsToGrant;
	struct ScalableFloat RemoveItemOnNextSwap;
	struct ScalableFloat RandomWeight;
	int DesiredSlot;
};

struct FRandomItemEntries
{
	TArray<struct PlayerLoudoutEntry> WeightedEntries;
};

struct FPlayerLoadout
{
	TArray<struct PlayerLoudoutEntry> AlwaysGrantedLoadoutItems;
	TArray<struct RandomItemEntries> RandomlyGrantedLoadoutItems;
	struct ScalableFloat LoadoutDuration;
};

struct FCustomLootOverrideData
{
	ECustomLootSelection CustomLootType;
	class UClass* LootTierData;
	class UClass* LootPackages;
	class UClass* ExperimentalLootTierData;
	class UClass* ExperimentalLootPackages;
	struct GameplayTagContainer PlaylistContextTagsToAdd;
	struct GameplayTagContainer PlaylistContextTagsToRemove;
};

struct FFortAthenaCompassIcon
{
	struct SlateBrush Brush;
	float Scale;
	float MaxPawnDistanceForScaling;
	float DistanceForScalingMultiplier_Min;
	float DistanceForScalingMultiplier_Max;
	float YOffset;
};

struct FMarkerUtilitiesMapIcon
{
	struct SlateBrush MapIcon;
	struct ScalableFloat MapIconScale;
};

struct FMarkerUtilitiesMapPlacementIcon
{
	struct MarkerUtilitiesMapIcon PlacementMapIcon;
	struct MarkerUtilitiesMapIcon SquadmatePlacementMapIcon;
	struct ScalableFloat MinimumTeamScoreToShow;
	bool bMinimumTeamScoreToShowIsPercent;
};

struct FMarkerUtilitiesCompassPlacementIcon
{
	struct FortAthenaCompassIcon PlacementCompassIcon;
	struct FortAthenaCompassIcon SquadmatePlacementCompassIcon;
	struct ScalableFloat MinimumTeamScoreToShow;
	bool bMinimumTeamScoreToShowIsPercent;
};

struct FMarkerUtilitiesTeamPlacement
{
	byte TeamId;
	int Placement;
};

struct FMashAreaSpecialActorData
{
	struct GameplayTag SpecialActorTag;
	struct SlateBrush SpecialActorMinimapIconBrush;
	struct Vector2D SpecialActorMinimapIconScale;
	struct SlateBrush SpecialActorCompassIconBrush;
	struct Vector2D SpecialActorCompassIconScale;
	bool bShouldDrawCompassIcon;
};

struct FMashDifficultySettings
{
	struct ScalableFloat SpawnCountPlayerCountMultiplier;
	struct ScalableFloat SpawnCountDifficultyMultiplier;
	struct ScalableFloat AIEffectPlayerCountMultiplier;
	struct ScalableFloat AIEffectDifficultyMultiplier;
	float SpawnCountMultiplier;
	float AIEffectMultiplier;
};

struct FMashLootTierOverrideAssetData
{
	int SafeZoneIndex;
	struct FName TieredGroup;
};

struct FMashScoreData
{
	class UClass* ActorClass;
	struct GameplayTagContainer Tags;
	struct GameplayTagContainer ExclusionTags;
	struct ScalableFloat ScoreValue;
	struct FText ScoreText;
};

struct FMashScoreMultiplierInstanceData
{
	class UClass* Actor;
	struct FName SpecialActorID;
};

struct FMashObjectiveAreaInstanceData
{
	struct Vector Location;
	class UClass* AreaActor;
	struct FName SpecialActorID;
	TArray<struct MashScoreMultiplierInstanceData> ScoreMultipliers;
};

struct FMashLeaderboardEntry
{
	struct FString DisplayName;
	int Value;
	int Rank;
	struct UniqueNetIdRepl UserNetId;
	bool bIsSpecialEntry;
	bool bIsLocalPlayer;
};

struct FMashFriendsLeaderboard
{
	struct FString Name;
	TArray<struct MashLeaderboardEntry> LeaderBoard;
};

struct FMashLeaderboardQuery
{
	struct UniqueNetIdRepl RequestingId;
};

struct FMatchConditionMutatorTeamData
{
	byte TeamNum;
	EMatchConditionMutatorTeamStatus TeamStatus;
};

struct FFortMutatorMusicEvent
{
	class UClass* Sound;
	float LoopTime;
	float FadeTime;
};

struct FFortMutatorGamePhaseMusicEvent
{
	EAthenaGamePhase Phase;
	struct FortMutatorMusicEvent Event;
};

struct FFortMutatorGamePhaseStepMusicEvent
{
	EAthenaGamePhaseStep Step;
	struct FortMutatorMusicEvent Event;
};

struct FOmahaCharacterVariantInfoData
{
	TArray<struct McpVariantChannelInfo> VariantChannelInfo;
};

struct FPaybackMutatorEffectData
{
	class UClass* KillerPlayerState;
	byte VictimTeam;
	TArray<class UClass*> VictimPlayerStates;
	float StartTime;
	float EndTime;
};

struct FPerkMutatorData
{
	struct GameplayTag PerkTag;
	struct GameplayTag ShowPerkSelectTag;
	struct GameplayTag PassivePerkTag;
	struct GameplayTag ItemPerkTag;
	struct GameplayTag FirstRollTag;
	struct GameplayTag BlockRespawnTag;
	class UClass* PerkUnlockedGameplayEffectClass;
	class UClass* ShowPerkSelectGameplayEffectClass;
	class UClass* BlockRespawnGameplayEffectClass;
	class UClass* PerkScreenIntroWidgetClass;
	struct ScalableFloat StartingRerollCount;
	struct ScalableFloat RerollsToGivePerPerkUnlock;
	struct ScalableFloat MaxRerollsPerPlayer;
	Unknown FactionItemMapping;
	bool bShouldShowBackgroundImage;
};

struct FMutatorPlayerSettingsData
{
	class UClass* ScopeSettings;
};

struct FBonePlayerDamageMultiplier
{
	struct FName BoneName;
	struct ScalableFloat DamageMultiplier;
};

struct FPlayerMarkerMutatorEffectData
{
	class UClass* MarkedPlayerState;
};

struct FMapLocation
{
	struct FText Text;
	struct Vector2D Position;
	struct SlateFontInfo Font;
	struct LinearColor Color;
	struct GameplayTag LocationTag;
};

struct FReroutePlayerEventDefinition
{
	struct GameplayTagContainer IncomingEventTags;
	struct GameplayTagContainer IncludeTags;
	struct GameplayTagContainer ExcludeTags;
	struct GameplayTag ImmediateOutgoingEventTag;
	struct GameplayTag MergedOutgoingEventTag;
};

struct FRespawnAndSpectateTargetData
{
	Unknown OwningMutator;
	bool bEnabled;
	bool bPrevAvailableOnClient;
	bool bPrevEnabledOnClient;
	bool bShouldBeSelectedByDefault;
	bool bHiddenAndAutoSelectedFallback;
	float PostDeathDisableTime;
	int ID;
	int DisplayPriority;
	byte Team;
	class UClass* RespawnTargetActor;
	class UClass* CameraActor;
};

struct FFortAthenaMutator_RespawnWaveTeamData
{
	float ReplicatedTimeStamp;
	TArray<Unknown> PlayerStates;
};

struct FPlayerStartInfo
{
	byte TeamNum;
	struct GameplayTagContainer RequiredTags;
	TArray<class UClass*> PlayerStartActors;
};

struct FPOIRoundInfo
{
	struct ScalableFloat bIsPOIEnabled;
	struct GameplayTag POITag;
	struct FText POIOverrideName;
	TArray<struct PlayerStartInfo> PlayerStartInfos;
	class UClass* CameraActor;
};

struct FHotfixableBlacklistLiteralLocations
{
	struct Vector2D Position;
	float Radius;
};

struct FSafeZoneOrderOptimizeMutatorRouteDefinition
{
	struct Vector StartLocation;
	struct Vector EndLocation;
	struct FText DebugName;
};

struct FSafeZoneOrderOptimizeMutatorRouteOrder
{
	TArray<int> RouteIDs;
	float TotalDistance;
};

struct FSkyCapTargetData
{
	float TargetHeight;
	float MoveTime;
};

struct FSkyCapPositionData
{
	struct ScalableFloat MoveTime;
	struct ScalableFloat Height;
	struct ScalableFloat WaitTime;
};

struct FFortItemDeliverySupplyDropMutatorData
{
	bool bShouldApplyMutator;
	struct ScalableFloat NumDeliveryItemsToSpawn;
	class UClass* SupplyDropPlacementQuery;
};

struct FFortSupplyDropMutatorData
{
	struct FName SupplyDropID;
	bool bShouldCenterGroundCheckAtFoundLocation;
	TArray<struct FortItemDeliverySupplyDropMutatorData> ItemDeliveryMutatorPerSafeZonePhase;
};

struct FBeginGroupTeleportParams
{
	struct FText HUDReasonText;
	bool bRespawnPlayers;
	bool bForceKillPlayers;
	bool bAutoReleaseFromStasis;
	float FadeTime;
	ESynchronizedTeleportHealthAndShieldResetType HealthAndShieldResetType;
	bool bResetPlayerInventories;
	bool bRandomizePlayerInventories;
	bool bReinitializePlayerAbilities;
	bool bBlockPickupsDuringTeleport;
	bool bFadeSound;
};

struct FTeamPawnColor_VisualData
{
	class UClass* GE_Glow;
	struct GameplayTag GlowTag;
	struct LinearColor MinimapColor;
};

struct FPartSwapData
{
	TArray<class UClass*> PartsToSwapIn;
};

struct FTeamPlacementData
{
	byte TeamId;
	int TeamPlacement;
	int TeamScore;
	class UClass* TeamInfoAthena;
};

struct FTimeOfDayPhase
{
	float Time;
	struct FText DisplayName;
};

struct FTimeOfDaySpeed
{
	float Speed;
	struct FText DisplayName;
};

struct FTournamentWeaponKillStat
{
	struct GameplayTagContainer TagList;
	struct FString StatName;
	struct FName StatDisplayName;
	Unknown PlayerStats;
};

struct FUraniumPOIData
{
	struct GameplayTag POITag;
	Unknown SplineActor;
	Unknown CameraActor;
};

struct FUraniumSingleRoundInfo
{
	byte RoundTeamWinner;
	struct FText RoundName;
	class UClass* RoundSound;
	int PointsForWinning;
};

struct FUraniumRoundData
{
	int8_t CurrentRoundNumber;
	int8_t CurrentRoundCheckPoint;
	TArray<struct UraniumSingleRoundInfo> SingleRoundInfos;
};

struct FWaxNoStormZone
{
	struct Vector Location;
	struct ScalableFloat Radius;
};

struct FWaxVisibilityModifiers
{
	struct ScalableFloat UI_VisibilityMode;
	struct SlateBrush CompassIndicator;
	struct SlateBrush MinimapIndicator;
	TArray<class UClass*> TemporarilyGrantedEffects;
	TArray<class UClass*> PermanentlyGrantedEffects;
	struct ScalableFloat DistanceConsideredCloseForUI;
};

struct FWaxRespawnLogicData
{
	struct FortRespawnLogicData BaseRespawnData;
	struct ScalableFloat TryPlaceByTeammates;
	struct ScalableFloat AlsoPlaceBehindTeammates;
	struct ScalableFloat MaxDistanceFromTeammates;
	struct ScalableFloat MinDistanceFromTeammates;
	struct ScalableFloat TryPlaceAwayFromLeaders;
	struct ScalableFloat OnlyCountLeadersAboveWaxState;
	struct ScalableFloat UseDirectionFromTeam;
	struct ScalableFloat SpawnBehindTeammateBias;
};

struct FWaxPartOverrideData
{
	EFortCustomGender Gender;
	TArray<class UClass*> PartsToSwapIn;
	TArray<class UClass*> DefaultParts;
	struct GameplayTag CosmeticSwapTag;
};

struct FFortAthenaTutorial_TargetInfo
{
	class UClass* TargetMarker;
	class UClass* TargetActor;
};

struct FVehicleWeightedDef
{
	struct TSoftClassPtr<UObject> VehicleItemDef;
	struct ScalableFloat Weight;
};

struct FReplicatedAthenaVehicleState
{
	struct Vector ForwardVectorTarget;
};

struct FFortProjectileCues
{
	struct GameplayCueTag Spawn;
	struct GameplayCueTag HitPawn;
	struct GameplayCueTag HitWorld;
	struct GameplayCueTag HitWater;
	bool bOrientHitGCsToProjectileVelocity;
	float MaxSurfaceNormalDeviationAngle;
	struct GameplayCueTag Bounce;
	struct GameplayCueTag Explosion;
	struct GameplayCueTag UnderwaterExplosion;
};

struct FAttachedInfo
{
	struct HitResult Hit;
	class UClass* AttachedToActor;
	struct Vector_NetQuantize10 AttachOffset;
	struct Vector_NetQuantizeNormal VelocityNormalized;
	float NarrowPlacementAgainstVelocityThreshold;
	float StickyOffsetFromPhysicsMesh;
	float StickyOffsetFromBoneCenter;
};

struct FMyFortCategoryData
{
	struct FText CategoryName;
	struct GameplayTag TooltipTag;
	struct GameplayTagContainer ModifiedTagContainer;
	bool bIsCore;
	TArray<struct GameplayAttribute> Attributes;
};

struct FFortAttributeDetailsInfo
{
	struct GameplayTagContainer RequiredTags;
	struct FortMultiSizeBrush Icon;
	struct FText DisplayName;
	struct FText Description;
};

struct FFortAttributeInfo
{
	struct GameplayAttribute Attribute;
	EFortAttributeDisplay DisplayMethod;
	struct FText UnitDisplayName;
	float DisplayScalingFactor;
	struct FText FormatText;
	TArray<struct FortAttributeDetailsInfo> AttributeDetails;
	bool bShowInSummaries;
	bool bShowInDifferences;
	bool bShowAsBuffInFE;
	bool bNegativeValuesShouldBeDisplayedPositively;
};

struct FAudioAnalysisParameterScalar
{
	struct FName Name;
	class UClass* MaterialCollection;
	class UClass* NiagaraCollection;
	bool bDebug;
};

struct FAudioAnalysisParameterVector
{
	TArray<struct FName> ParamNames;
	class UClass* MaterialCollection;
	class UClass* NiagaraCollection;
	bool bDebug;
};

struct FAudioAnalysisSpectralAnalysisSettings
{
	TArray<struct SoundSubmixSpectralAnalysisBandSettings> BandSettings;
	float UpdateRate;
	float DecibalNoiseFloor;
	bool bDoNormalize;
	bool bDoAutoRange;
	float AutoRangeAttackTime;
	float AutoRangeReleaseTime;
};

struct FFortSubmixAnalyzerData
{
	TArray<struct AudioAnalysisParameterScalar> ScalarParameters;
	TArray<struct AudioAnalysisParameterVector> VectorParameters;
	struct AudioAnalysisSpectralAnalysisSettings SpectralAnalysisConfig;
};

struct FAudioCurveInfo
{
	struct RichCurve Curve;
};

struct FChannelData
{
	struct FName Name;
	float MaxMagnitude;
	float Value;
};

struct FBangCheckData
{
	class UClass* AlterQuest;
	class UClass* EgoQuest;
	class UClass* BundleSeeRoom;
	struct GameplayTag IntroTag;
	struct GameplayTag RoomBangTag;
};

struct FAthenaQuickChatActiveEntry
{
	Unknown Bank;
	Unknown ContextObject;
	int16_t ContextValue;
	int8_t Index;
};

struct FFortBroadcastInfoPerPlayer
{
	class UClass* PlayerState;
	class UClass* PlayerInventory;
	class UClass* PlayerClientInfo;
};

struct FDistanceToTargetComparison
{
	bool bUseOverriddenValue;
	float OverriddenValue;
	struct GameplayTagContainer DistanceDataTags;
	EArithmeticKeyOperation Operator;
	ETargetDistanceComparisonType ComparisonType;
};

struct FFortBuildingSoundsPerAffiliation
{
	class UClass* SoundFriendly;
	class UClass* SoundEnemy;
};

struct FFortBuildingSoundsPerResourceType
{
	struct FortBuildingSoundsPerAffiliation OnConstruction;
	struct FortBuildingSoundsPerAffiliation OnGenericDestruction;
	struct FortBuildingSoundsPerAffiliation OnPlayerBuiltDestruction;
};

struct FFortActorRecord
{
	struct Guid ActorGuid;
	EFortBuildingPersistentState ActorState;
	class UClass* ActorClass;
	struct Transform ActorTransform;
	bool bSpawnedActor;
	TArray<byte> ActorData;
};

struct FFortBuildingInstruction
{
	struct FortActorRecord ActorRecord;
};

struct FSmartBuildingActor
{
	struct TSoftClassPtr<UObject> BuildingMetaData;
	int RotationIterations;
};

struct FSmartBuildSelection
{
	float IdealViewPitch;
	TArray<struct SmartBuildingActor> BuildingActors;
};

struct FFilledGadgetSlot
{
	struct FString Gadget;
	int slot_index;
};

struct FFortCrewSlotInformation
{
	struct FText DisplayName;
	struct FName SlotName;
	struct GameplayTagContainer SlotTags;
	float SlotStatContribution;
};

struct FCarriedObjectAttachmentInfo
{
	class UClass* AttachParent;
	struct FName SocketName;
	struct Vector RelativeTranslation;
	struct Rotator RelativeRotation;
};

struct FFortChallengeBundleQuestEntry
{
	struct TSoftClassPtr<UObject> QuestDefinition;
	EChallengeBundleQuestUnlockType QuestUnlockType;
	bool bStartActive;
	bool bIsPrerequisite;
	int UnlockValue;
	struct ChallengeGiftBoxData RewardGiftBox;
	struct FortItemQuantityPair MenuOverrideRewardPreview;
};

struct FFortChallengeBundleRewards
{
	int CompletionCount;
	bool bBundlePrestige;
	TArray<struct AthenaRewardItemReference> Rewards;
};

struct FFortChallengeBundleLevelReward
{
	struct AthenaRewardItemReference RewardItem;
	int NumObjectivesNeeded;
};

struct FFortChallengeBundleLevel
{
	TArray<struct FortChallengeBundleLevelReward> BundleLevelRewardEntries;
};

struct FFortChallengeBundleSpecialOffer
{
	struct FString Storefront;
	struct FText RichText;
	struct TSoftClassPtr<UObject> OfferImage;
};

struct FFortChallengeSetStyle
{
	struct LinearColor PrimaryColor;
	struct LinearColor SecondaryColor;
	struct LinearColor AccentColor;
	struct LinearColor Context_LimitedTimeColor;
	struct LinearColor Context_BaseColor;
	struct TSoftClassPtr<UObject> DisplayImage;
	struct TSoftClassPtr<UObject> CustomBackground;
};

struct FFortChallengeBundleScheduleEntry
{
	struct TSoftClassPtr<UObject> ChallengeBundle;
	EChallengeScheduleUnlockType UnlockType;
	int UnlockValue;
};

struct FFortAnimInput_STWHoverBoard
{
	class UClass* HoverCycleVelocityCurve;
	class UClass* HoverHeightCurve;
	class UClass* HoverLeanCurve;
	class UClass* HoverPitchCurve;
	struct Rotator HoverTransformRotation;
	struct Vector HoverTransformTranslation;
	float HoverTransformAlpha;
	float HoverCycle;
	float HoverHeight;
	float HoverLeanAngle;
	float HoverPitchAngle;
	float HoverYaw;
	float HoverYawCurrent;
	float HoverIdleLeanAlpha;
	bool bIsUsingHoverboard;
};

struct FPartStackEntry
{
	TArray<class UClass*> PartList;
};

struct FFortAthenaVehicleInputStateReliable
{
	bool bIsSprinting;
	bool bIsJumping;
	bool bIsBraking;
	bool bIsHonking;
	bool bIgnoreForwardInAir;
	bool bMovementModifier0;
	bool bMovementModifier1;
	bool bMovementModifier2;
};

struct FFortAthenaVehicleInputStateUnreliable
{
	float ForwardAlpha;
	float RightAlpha;
	float PitchAlpha;
	float LookUpDelta;
	float TurnDelta;
	float SteerAlpha;
	float GravityOffset;
	struct Vector_NetQuantize100 MovementDir;
};

struct FZiplinePawnState
{
	class UClass* Zipline;
	bool bIsZiplining;
	bool bJumped;
	bool bReachedEnd;
	int AuthoritativeValue;
	struct Vector SocketOffset;
	float TimeZipliningBegan;
	float TimeZipliningEndedFromJump;
};

struct FVehiclePawnState
{
	class UClass* Vehicle;
	float VehicleApexZ;
	byte SeatIndex;
	byte ExitSocketIndex;
	bool bOverrideVehicleExit;
	struct Vector SeatTransitionVector;
	float EntryTime;
};

struct FFortAppliedSwapItemAndVariantData
{
	struct Guid SwapId;
	EFortAppliedSwapItemAndVariantState SwapState;
	TArray<struct FortSwapItemAndVariantData> SwapData;
};

struct FFortPawnMaterialOverrideCopiedParameters
{
	TArray<struct FName> ScalarParamNames;
	TArray<struct FName> VectorParamNames;
	TArray<struct FName> TextureParamNames;
};

struct FFortPawnMaterialOverride
{
	struct Guid OverrideId;
	struct TSoftClassPtr<UObject> Material;
	struct FortPawnMaterialOverrideCopiedParameters MaterialParamsToCopy;
	float Priority;
	bool bHideParticleSystems;
	bool bApplyToWeapon;
};

struct FFortPawnMaterialOverrideState
{
	class UClass* SceneComp;
	TArray<class UClass*> OriginalMaterials;
	TArray<class UClass*> AppliedMaterials;
	TArray<class UClass*> FXComps;
};

struct FFortCharacterPartsRepMontageInfo
{
	TArray<struct FortCharacterPartMontageInfo> CharPartMontages;
	class UClass* PawnMontage;
	bool bPlayBit;
};

struct FIgnoreCollisionActor
{
	class UClass* IgnoreActor;
	float TimeIgnoreStarted;
	float IgnoreDuration;
};

struct FFortDBNOCarryHoisterData
{
	class UClass* DBNOHoister;
	EFortDBNOCarryEvent DBNOCarryEvent;
};

struct FPreviouslyAppliedVariantData
{
	class UClass* Character;
	class UClass* Contrail;
	class UClass* Pickaxe;
	class UClass* Backpack;
	TArray<struct McpVariantChannelInfo> Variants;
};

struct FRepFortMeshAttachment
{
	class UClass* SkeletalMesh;
	class UClass* AnimBP;
};

struct FVortexParams
{
	float GravityFloorAltitude;
	float GravityFloorWidth;
	float GravityFloorGravityScalar;
	float GravityFloorTerminalVelocity;
};

struct FSlipperySlopeParams
{
	struct ScalableFloat SlopeForceAcceleration;
	struct ScalableFloat MaxLateralSpeed;
	struct ScalableFloat MaxLateralSpeedMultiplierInWater;
	struct ScalableFloat BrakingDecelerationInWater;
	struct ScalableFloat MaxVerticalLaunchSpeed;
	struct ScalableFloat SlopeLandingForceScalar;
	struct ScalableFloat SlopeLandingMaxHorizontalForce;
	struct ScalableFloat SlopeLaunchMinRequiredSpeed;
	struct ScalableFloat SlopeLaunchMinRequiredAngleChange;
	struct ScalableFloat SlopeLaunchVerticalVelocityBoost;
	struct ScalableFloat SlopeLaunchVerticalVelocityBoostMultiplierJumping;
};

struct FFortPlayerPawnObjectReference
{
	class UClass* TrackedObject;
};

struct FFortPlayerAthenaAttributeReplicationProxy
{
	float WalkSpeed;
	float RunSpeed;
	float SprintSpeed;
	float FlySpeed;
	float CrouchedRunSpeed;
	float CrouchedSprintSpeed;
};

struct FFortPlayerAthenaGravityAttributeReplicationProxy
{
	float GravityZScale;
	float JumpZVelocity;
	float JumpHorizontalAcceleration;
	float JumpHorizontalVelocity;
};

struct FAthenaPawnReplayData
{
	float HealthRatio;
	float ShieldRatio;
	TArray<byte> CipherText;
	class UClass* World;
};

struct FOstrichWeapon_RetainedData
{
	int LoadedShotgunAmmo;
	float RocketsCooldownElapsed;
	bool bHasPrevious;
};

struct FCharmPreviewEntry
{
	struct TSoftClassPtr<UObject> PreviewObject;
	struct Transform PreviewTransform;
	bool bPreviewUsingVehicleShader;
};

struct FCharmSlotMetadata
{
	EFortCustomPartType AttachToPart;
	bool WeaponCharm;
	bool BackPresentedCharm;
	struct FName AttachSocket;
	struct GameplayTagQuery MatchCriteria;
	TArray<struct CharmPreviewEntry> PreviewList;
};

struct FFortClientAnnouncementData
{
};

struct FActionTextPair
{
	struct FName Action;
	struct FText Text;
};

struct FFortClientAnnouncementQueue
{
	TArray<class UClass*> Announcements;
};

struct FFortEventName
{
	struct FName CategoryName;
	struct FName EventName;
};

struct FReplicatedStatValues
{
	int StatValue;
	int ScoreValue;
};

struct FFortExperienceDelta
{
	int Level;
	int XP;
	int BaseXPEarned;
	int BonusXPEarned;
	int BoostXPEarned;
	int BoostXPMissed;
	int RestXPEarned;
	int GroupBoostXPEarned;
	EFortIsFinalXpUpdate IsFinalXpUpdate;
};

struct FFortPlayerScoreReport
{
	struct UniqueNetIdRepl PlayerID;
	struct FString PlayerName;
	Unknown PlayerState;
	EFortTeam PlayerTeam;
	struct ReplicatedStatValues ReplicatedStats_Campaign;
	struct ReplicatedStatValues ReplicatedStats_Zone;
	int InitialLevel;
	int InitialExperienceAmount;
	struct FortExperienceDelta ExperienceInfoDelta;
	int LastExperienceDeltaAmount;
	int LastScoreDeltaAmount;
};

struct FPlayerReportingInfoContainer
{
	class UClass* Owner;
	TArray<class UClass*> AllPlayerInfo;
	TArray<class UClass*> AllIslandInfo;
};

struct FFortGamepadBasicOptions
{
	EFortGamepadSensitivity LookSensitivityPreset;
	EFortGamepadSensitivity LookSensitivityPresetAds;
	float LookBuildModeMultiplier;
	float LookEditModeMultiplier;
	bool bUseAdvancedOptions;
};

struct FFortGamepadAdvancedOptions
{
	uint32_t LookHorizontalSpeed;
	uint32_t LookVerticalSpeed;
	uint32_t LookHorizontalSpeedAds;
	uint32_t LookVerticalSpeedAds;
	uint32_t LookHorizontalBoostSpeed;
	uint32_t LookVerticalBoostSpeed;
	float LookBoostAccelerationTime;
	uint32_t LookHorizontalBoostSpeedAds;
	uint32_t LookVerticalBoostSpeedAds;
	float LookBoostAccelerationTimeAds;
	bool bInstantBoostWhenBuilding;
	float LookEaseTime;
	EFortGamepadLookInputCurve LookInputCurve;
	uint32_t AimAssistStrength;
	bool bUseLegacyControls;
};

struct FFortGamepadUserOptions
{
	struct FortGamepadBasicOptions Basic;
	struct FortGamepadAdvancedOptions Advanced;
};

struct FFortActionKeyMapping
{
	struct FName ActionName;
	EFortInputActionGroup ActionGroup;
	ESubGame SubGameUsedIn;
	struct FText LocalizedCategory;
	struct FText LocalizedName;
	struct Key KeyBind1;
	struct Key KeyBind2;
	float InputScale;
	bool bIsAxisMapping;
};

struct FUserActionBindings
{
	TArray<struct FortActionKeyMapping> UserActionBindings;
};

struct FHUDLayoutDataEntry
{
	struct GameplayTag VisualType;
	struct AnchorData AnchroData;
	int ZOrder;
	EBacchusHUDStateType BuildVisibility;
	EBacchusHUDStateType CombatVisibility;
	EBacchusHUDStateType EditVisibility;
	EBacchusHUDStateType CreativeVisibility;
	float Property_0;
	float Property_1;
	float Property_2;
	float Property_3;
	Unknown FloatProperties;
};

struct FHUDLayoutData
{
	TArray<struct HUDLayoutDataEntry> LayoutEntries;
};

struct FFortMobileHUDProfileIdentifier
{
	struct GameplayTag HUDProfileType;
	struct Guid Guid;
};

struct FFortMobileHUDProfileBase
{
	struct FortMobileHUDProfileIdentifier HUDProfileIdentifier;
	struct FText HUDProfileName;
	struct GameplayTag HUDProfileBaseType;
};

struct FFortMobileHUDWidgetCustomPropertySave
{
	struct GameplayTag PropertyTag;
	struct FString PropertyValue;
};

struct FFortMobileHUDWidgetLayoutSave
{
	struct AnchorData LayoutData;
	TArray<struct FortMobileHUDWidgetCustomPropertySave> CustomProperties;
};

struct FFortMobileHUDWidgetSchemaSave
{
	struct GameplayTag HUDWidgetClassTag;
	struct GameplayTag BehaviorClassTag;
	struct Guid Guid;
	struct FortMobileHUDWidgetLayoutSave HUDWidgetLayout;
};

struct FFortMobileSchemaModification
{
	struct Guid Guid;
	ESchemaModificationType ModificationType;
	struct GameplayTag OwningContextTag;
	struct FortMobileHUDWidgetSchemaSave ModifiedSchema;
};

struct FFortMobileSchemaModificationContainer
{
	TArray<struct FortMobileSchemaModification> Modifications;
};

struct FFortMobileHUDProfileContainer
{
	struct FortMobileHUDProfileIdentifier ActiveHUDProfileIdentifier;
	TArray<struct FortMobileCustomHUDProfile> CustomHUDProfiles;
};

struct FPlayerLastSelectedPreferredProvider
{
	struct UniqueNetIdRepl LocalUserId;
	EAppStore PreferredProvider;
};

struct FFortPendingSlottedItemOperation
{
	struct FString SlottedItemId;
	struct FName SlotRowName;
};

struct FFortCollectionBookSectionState
{
	struct FString Section;
	EFortCollectionBookState State;
};

struct FFortCollectionDataMapping
{
	struct FString CollectionType;
	bool bEnsureAllTaggedItemsAreInTheCollection;
	struct TSoftClassPtr<UObject> Collection;
};

struct FFortCollisionAudioTriggerData
{
	class UClass* Asset;
	struct Vector2D ImpulseMagnitudeRange;
	bool bImpulseMagnitudeLowerBound;
	bool bImpulseMagnitudeUpperBound;
	struct Vector2D VolumeRange;
	struct Vector2D PitchRange;
	float MinRetriggerTime;
	float MaxTriggerDistance;
};

struct FBASEGameplayEffect
{
	class UClass* Effect;
	int LevelOverride;
};

struct FPatternBASEEffect
{
	class UClass* Pattern;
	class UClass* Mesh;
};

struct FConsumeEffectData
{
	struct TSoftClassPtr<UObject> EffectClass;
	struct ScalableFloat Level;
};

struct FFortContentEncryptionCollection
{
	bool bEnabled;
	struct CollectionReference Collection;
	EFortContentEncryptionCollectionGrouping Grouping;
	EFortContentEncryptionAllowedReferences AllowedReferences;
};

struct FDirectiveInput
{
	ECommonInputType Input;
	struct FText Text;
	TArray<struct FText> ABTestingTexts;
};

struct FHUDElementToHighlight
{
	struct GameplayTagContainer HUDElementToHighlight;
	struct GameplayTagContainer HUDSubElementToHighlight;
	struct GameplayTagContainer UniqueHUDElementToHighlight;
};

struct FFortBotTeleportInfo
{
	bool bTeleportSuccess;
	struct Vector TeleportFrom;
	struct Rotator RotationFrom;
	struct Vector TeleportTo;
	struct Rotator RotationTo;
	ETeleportReason TeleportReason;
	struct FString BTNodeNameCausingTeleportation;
};

struct FPlayerFishingTelemetryData
{
	float TimeFishingSessionBegan;
	Unknown Item;
	Unknown ItemUsedToFish;
	bool bFromFishingPool;
	bool bBestCollected;
	struct FString FishVariantTag;
	float FishPropertyLength;
};

struct FFortItemsConsumedInfo
{
	class UClass* WeaponData;
	struct Vector Location;
	float Health;
	float Shield;
	int ItemQuantity;
};

struct FFortRevivedInfo
{
	struct Vector RevivedLocation;
	struct FString ReviverUniqueID;
	byte ReviverTeam;
};

struct FPawnConvertedInfos
{
	struct FString ConvertedPawnUniqueID;
};

struct FFortControllerAIBotComponentTelemetryData_Disguise
{
	bool bWasDisguised;
	bool bWasRevealed;
	EFortPawnComponent_DisguiseRevealReason RevealReason;
};

struct FFortControllerAIBotComponentTelemetryData_PassiveHealer
{
	bool bIsPassiveHealer;
	int InteractionCount;
};

struct FFortChangeMonitoringStruct
{
	bool bAnyValueDirty;
};

struct FComponentWidgetPairings
{
	EUIExtensionSlot Slot;
	class UClass* Class;
};

struct FIndicatedActorScaleAndOpacityData
{
	float SmallSizeDistance;
	float LargestSizeDistance;
	float SmallScale;
	float LargestScale;
	float FarAwayScale;
	float FarAwayOpacity;
	float MaxScaleAndFadePercent;
};

struct FIndicatedActorParticleSystemData
{
	class UClass* ParticleSystem;
	struct FName ParticleSystemActorParamName;
	struct Vector ParticleSystemOffset;
	struct Vector ParticleSystemDBNOOffset;
	struct FName ParticleSystemVectorParamName;
};

struct FIndicatedActorData
{
	struct FString GroupIdentifier;
	float Duration;
	float StepTime;
	struct Vector IndicatorOffset;
	struct Vector IndicatorDBNOOffset;
	EShareActorWith ShareActorWith;
	byte ShareActorWithMask;
	byte DisplayTeamOverride;
	bool bClampToScreen;
	struct IndicatedActorScaleAndOpacityData ScaleAndOpacityData;
	struct IndicatedActorParticleSystemData ParticleSystemData;
	class UClass* Sound;
	EIndicatorStateImage StateImageOverride;
	struct FDelegate OnActorAdded;
	struct FDelegate OnActorRemoved;
};

struct FStenciledActorData
{
	struct FString GroupIdentifier;
	float Duration;
	float StepTime;
	EShareActorWith ShareActorWith;
	byte ShareActorWithMask;
	byte DisplayTeamOverride;
	struct IndicatedActorParticleSystemData ParticleSystemData;
	class UClass* Sound;
	struct FName FriendlyStencilName;
	struct FName EnemyStencilName;
	byte FriendlyStencilIndex;
	byte EnemyStencilIndex;
};

struct FFortGlobalCurrencyStash
{
	int Count;
};

struct FRepGlobalCurrencyStash
{
	EStashInventoryServiceSyncState SyncState;
	struct FortGlobalCurrencyStash Currency;
};

struct FGlobalCurrencyTransactionData
{
	float ServerTime;
	int Amount;
	struct FString SourceType;
};

struct FGlobalCurrencyTrackedData
{
	int TotalCurrencyEarnedInMatch;
	int PhaseStartCurrencyBalance;
	int PhaseEndCurrencyBalance;
	TArray<struct GlobalCurrencyTransactionData> TransactionData;
};

struct FCachedRechargeAmmoData
{
	float ServerStartTime;
	float ChargeRate;
	int AmountToRecharge;
	int ItemLevel;
	struct Guid WeaponItemGuid;
	struct TSoftClassPtr<UObject> AmmoItemDefinition;
	bool bShouldRechargeAmmoToClip;
};

struct FUrgentQuestData
{
	struct GameplayTag EventTag;
	float TotalEventTime;
	bool bQuestCompleteOnTimeExpiration;
	bool bDisplayHUDData;
	struct FText EventTitle;
	struct FText EventSubtitle;
	struct FText EventSubtitleSecondary;
	struct FText EventDescription;
	int SortPriority;
	bool bReverseProgressBar;
	bool bShowBountyThreatInformation;
	bool bShowBountyPriceAndDistance;
	class UClass* DisplayPlayer;
	class UClass* AcceptingPlayer;
	struct TSoftClassPtr<UObject> SocialAvatarBrushPtr;
	class UClass* BountyPriceImage;
	struct TSoftClassPtr<UObject> AlertIcon;
	struct TSoftClassPtr<UObject> SpecialEventStartedSound;
	struct DateTime EventStartTime;
	struct TimerHandle EventTimerHandle;
	struct LinearColor AlertIconBorderColor;
	struct Vector2D AlertIconBrushSize;
};

struct FTagModificationRequest
{
	struct GameplayTagContainer TagsToModify;
	bool bAddTag;
	float Delay;
	float FiniteLifetime;
};

struct FFortWeightedActorTypeList
{
	Unknown WeightedClassTypes;
};

struct FThresholdTestConfig
{
	EThresholdRequirement Requirement;
	struct ScalableFloat Quantity;
};

struct FRotatorErrorCheck
{
	bool bEnabled;
	float TestFromAngle;
	float ToleratedErrorInDegrees;
};

struct FConversationGiftTypeDefinition
{
	EContextRequirementMatchPolicy ServiceProviderRequirementMatchPolicy;
	TArray<class UClass*> ServiceProviderRequirements;
	EContextRequirementMatchPolicy RequirementMatchPolicy;
	TArray<class UClass*> Requirements;
	TArray<class UClass*> Effects;
	class UClass* UINotificationType;
};

struct FConversationParticipantRequirement
{
	struct GameplayTag ParticipantID;
	class UClass* Requirement;
	EConversationRequirementResult FailureNodeBehaviour;
	struct FString CannotUseReasonParameter;
};

struct FEffectRecipientConfig
{
	EDataDrivenEffectRecipient Recipient;
	TArray<class UClass*> Effects;
};

struct FFindActorByClassData
{
	class UClass* ClassFilter;
	TArray<EObjectTypeQuery> ObjectTypes;
	struct GameplayTagQuery TargetFilter;
	struct ScalableFloat MaxToFind;
	struct ScalableFloat MaxRadius;
	struct ScalableFloat ShowToSquad;
	struct ScalableFloat ShowMarkerDetails;
	struct ScalableFloat Price;
	struct ScalableFloat Rarity;
};

struct FFortConversationContextRequirement
{
	struct GameplayTag ParticipantID;
	class UClass* Requirement;
};

struct FContextualMessageCandidate
{
	struct FText Message;
	EContextRequirementMatchPolicy RequirementMatchPolicy;
	TArray<struct FortConversationContextRequirement> ContextRequirements;
	struct ScalableFloat Priority;
	struct ScalableFloat Weight;
};

struct FContextualMessageConfig
{
	TArray<struct ContextualMessageCandidate> ContextualMessages;
	struct FText DefaultMessage;
};

struct FBaseVariantDef
{
	bool bStartUnlocked;
	bool bIsDefault;
	bool bHideIfNotOwned;
	struct GameplayTag CustomizationVariantTag;
	struct FText VariantName;
	struct TSoftClassPtr<UObject> PreviewImage;
	struct FText UnlockRequirements;
	struct TSoftClassPtr<UObject> UnlockingItemDef;
};

struct FMaterialVariants
{
	struct TSoftClassPtr<UObject> MaterialToSwap;
	struct FName ComponentToOverride;
	struct FName CascadeMaterialName;
	int MaterialOverrideIndex;
	struct TSoftClassPtr<UObject> OverrideMaterial;
};

struct FMaterialParamName
{
	struct FName ParamName;
};

struct FMaterialParamterDef
{
	struct TSoftClassPtr<UObject> MaterialToAlter;
	struct FName CascadeMaterialName;
	TArray<struct MaterialVectorVariant> ColorParams;
	TArray<struct MaterialTextureVariant> TextureParams;
	TArray<struct MaterialFloatVariant> FloatParams;
};

struct FVariantParticleSystemInitializerData
{
	struct FName ParticleComponentName;
	struct TSoftClassPtr<UObject> ParticleSystem;
	struct TSoftClassPtr<UObject> MeshToBindTO;
	struct FName AttachSocketName;
	EAttachmentRule LocationRule;
	EAttachmentRule RotationRule;
	EAttachmentRule ScaleRule;
	bool bWeldSimulatedBodies;
};

struct FParticleVariant
{
	struct TSoftClassPtr<UObject> ParticleSystemToAlter;
	struct FName ComponentToOverride;
	struct TSoftClassPtr<UObject> OverrideParticleSystem;
};

struct FParticleParamterVariant
{
	struct TSoftClassPtr<UObject> ParticleSystemToAlter;
	TArray<struct MaterialVectorVariant> ColorParams;
	TArray<struct VectorParamVariant> VectorParams;
	TArray<struct MaterialFloatVariant> FloatParams;
};

struct FFortPortableSoftParticles
{
	EFXType FXType;
	struct TSoftClassPtr<UObject> NiagaraVersion;
	struct TSoftClassPtr<UObject> CascadeVersion;
};

struct FManagedParticleSwapVariant
{
	struct GameplayTag ParamGroupTag;
	struct FortPortableSoftParticles ParticleToOverride;
};

struct FManagedParticleParamVariant
{
	struct GameplayTag ParamGroupTag;
	TArray<struct MaterialVectorVariant> ColorParams;
	TArray<struct VectorParamVariant> VectorParams;
	TArray<struct MaterialFloatVariant> FloatParams;
};

struct FSoundVariant
{
	struct TSoftClassPtr<UObject> SoundToSwap;
	struct FName ComponentToOverride;
	struct TSoftClassPtr<UObject> OverrideSound;
};

struct FFoleySoundVariant
{
	TArray<class UClass*> LibrariesToAdd;
	TArray<class UClass*> LibrariesToRemove;
};

struct FSocketTransformVariant
{
	struct FName SourceSocketName;
	struct FName OverridSocketName;
	struct TSoftClassPtr<UObject> SourceObjectToModify;
};

struct FScriptedActionVariant
{
	struct GameplayTag ActionTag;
};

struct FMeshVariant
{
	struct TSoftClassPtr<UObject> MeshToSwap;
	struct FName ComponentToOverride;
	struct TSoftClassPtr<UObject> OverrideMesh;
	EAnimInstanceClassSwapType AnimInstanceClassSwapType;
	struct TSoftClassPtr<UObject> AnimInstanceClassToSwap;
	struct TSoftClassPtr<UObject> OverrideAnimInstanceClass;
};

struct FCosmeticMetaTagContainer
{
	struct GameplayTagContainer MetaTagsToApply;
	struct GameplayTagContainer MetaTagsToRemove;
};

struct FEmoteMontageSwap
{
	struct TSoftClassPtr<UObject> ToSwapFrom;
	struct TSoftClassPtr<UObject> ToSwapTo;
};

struct FItemTextureVariant
{
	TArray<struct TSoftClassPtr<UObject>> MaterialsToAlter;
	struct FName ParamName;
	struct FString DefaultSelectedItem;
};

struct FFortCosmeticLockerSlotsActiveVariants
{
	TArray<struct McpVariantReader> Variants;
};

struct FFortCosmeticLockerSlots
{
	TArray<struct FString> Items;
	TArray<struct FortCosmeticLockerSlotsActiveVariants> ActiveVariants;
};

struct FFortCosmeticLockerSlotData
{
	Unknown Slots;
};

struct FFortCosmeticLockerSlotInformation
{
	EAthenaCustomizationCategory CustomizationCategory;
	int NumSlotsOfCategory;
	bool bCanBeBlank;
	bool bMustBeUniqueInArray;
};

struct FFCRP_LoopingUpdate
{
	class UClass* Requester;
};

struct FRichColorVariant
{
	struct LinearColor DefaultStartingColor;
	struct TSoftClassPtr<UObject> ColorSwatchForChoices;
	bool bVariantPickerShouldShowHSV;
	TArray<struct TSoftClassPtr<UObject>> MaterialsToAlter;
	TArray<struct TSoftClassPtr<UObject>> ParticlesToAlter;
	struct FName ColorParamName;
};

struct FApplyWrapVariant
{
	TArray<struct FName> ComponentNameAllowList;
	int WrapSectionMask;
	Unknown CustomSectionMaskByMeshName;
	struct FString DefaultSelectedItem;
	EItemWrapMaterialType WrapMaterialType;
};

struct FCowVehicleMapRangeClamped
{
	float InRangeA;
	float InRangeB;
	float OutRangeA;
	float OutRangeB;
};

struct FCowVehicleImpactPointInterp
{
	float Target;
	float InterpSpeed;
};

struct FCowVehicleFloatSpringInterp
{
	float Stiffness;
	float CriticalDampingFactor;
	float Mass;
};

struct FFortCreativeBudgetOverride
{
	struct FString ActorClass;
	int AssetCost;
	float AssetCostMultiplier;
	int InstanceCost;
	float InstanceCostMultiplier;
	int SimulationCost;
	int DrawCall;
	int AudioCost;
	int NetworkCost;
};

struct FFortCreativeBudgetClassInstanceLimit
{
	struct TSoftClassPtr<UObject> ActorClass;
	int MaxNumberOfInstances;
};

struct FFortCreativeBudgetPickupInstanceLimit
{
	struct TSoftClassPtr<UObject> ItemDefinition;
	int MaxNumberOfInstances;
};

struct FFortCreativeBudgetComponentSimulationCost
{
	struct TSoftClassPtr<UObject> ComponentClass;
	int SimulationCost;
};

struct FFortCreativeBudget
{
	int TotalBudget;
	int UsedBudget;
	int PreviewBudget;
	EFortBudgetCategory Category;
	bool bCritical;
	int BudgetLowend;
	int FixedInstanceCost;
};

struct FFortCreativeGridCellBudget
{
	int CellBudget;
	EFortBudgetCategory Category;
};

struct FFortCreativeBudgetPlotBudgetOverride
{
	struct TSoftClassPtr<UObject> PlotItemDefinitionClass;
	TArray<struct FortCreativeBudget> Budgets;
	bool bCreativeHeatmapThermometerEnabled;
	float CreativeHeatmapThermometerCellSize;
	float CreativeHeatmapThermometerInfluenceDistanceMultiplier;
	TArray<struct FortCreativeGridCellBudget> CreativeHeatmapThermometerBudgetOverrides;
};

struct FCreativeAssetMetaData
{
	struct FString StringObjectPtr;
	struct FName NameObjectPtr;
	struct TSoftClassPtr<UObject> ObjectPtr;
	int AssetSize;
};

struct FCreativeActorMetaData
{
	int AssetSize;
	int InstanceSize;
	int SimulationCost;
	int DrawCall;
	int AudioCost;
	int NetworkCost;
	bool bCostOverridden;
	TArray<struct CreativeAssetMetaData> AssetDependencies;
};

struct FFortCreativeAssetCostData
{
	Unknown MetaDataMap;
	Unknown NameMetaDataMap;
};

struct FFortCreativeDiscoverySplicedEntry
{
	struct FName EntryName;
	int EntryIndex;
	struct FString LinkCode;
	EFortCreativeDiscoveryDeterminism VisibilitySelector;
	float VisibilityChance;
	bool bPushDownExistingEntry;
};

struct FFortCreativeDiscoveryContentPanel
{
	struct FName PanelName;
	struct FText PanelDisplayName;
	EFortCreativeDiscoveryPanelType PanelType;
	int PageSize;
	int NumPages;
	TArray<struct FString> PanelLinkCodeBlacklist;
	TArray<struct FString> PanelLinkCodeWhitelist;
	EFortCreativeDiscoverySkippedEntries EntrySkippingMethod;
	int SkippedEntriesCount;
	float SkippedEntriesPercent;
	TArray<struct FString> CuratedListOfLinkCodes;
	struct FString MetricName;
	struct FString MetricInterval;
	bool bLowestToHighest;
	struct FString ModelName;
	TArray<struct FortCreativeDiscoverySplicedEntry> SplicedEntries;
};

struct FFortCreativeDiscoveryTestCohort
{
	struct FName TestName;
	EFortCreativeDiscoveryDeterminism CohortSelector;
	float SelectionChance;
	TArray<struct FortCreativeDiscoveryContentPanel> ContentPanels;
};

struct FFortCreativeHeatmapThermometerPreviewData
{
	struct TSoftClassPtr<UObject> ActorClass;
	struct Vector Location;
	struct Vector RelativeLocation;
	struct Rotator RelativeRotation;
	float InfluenceDistance;
	Unknown OwningPlayerState;
};

struct FOriginalAndSpawnedPair
{
	class UClass* OriginalActor;
	class UClass* SpawnedActor;
	bool bSpawnedActorIsForPreview;
};

struct FActorAndTransformPair
{
	class UClass* Actor;
	struct Transform Transform;
	bool bHasValidTransform;
};

struct FCreativeSelectedActorInfo
{
	class UClass* Actor;
	struct Transform UnscaledActorToSelectionAtDragStart;
	struct Vector DragStartGridSnapPoint;
	float OriginalRelevancyDistance;
	bool bWasCollisionEnabled;
	bool bWasDormant;
	bool bSpawnedFromSaveRecord;
	int LogicalConnectionChainIndex;
};

struct FLogicalConnectionChain
{
	TArray<class UClass*> LogicalConnectedActors;
};

struct FValidPlacementPair
{
	class UClass* Actor;
	bool bIsPlacementValid;
};

struct FCreativePooledMID
{
	class UClass* Mid;
	class UClass* OriginalMaterial;
};

struct FFlashCountedActorInfo
{
	TArray<struct CreativeSelectedActorInfo> Actors;
	int8_t FlashCounter;
};

struct FActorAndTimePair
{
	class UClass* Actor;
	struct DateTime SpawnTime;
};

struct FAgingActorArray
{
	TArray<struct ActorAndTimePair> AgingActors;
};

struct FPlaysetInfo
{
	class UClass* Playset;
	int8_t FlashCounter;
};

struct FCreativeOptionData
{
	struct FText DisplayText;
	int Value;
};

struct FFortInventory_SaveEntrySettings
{
	bool bIncludeHealthAndShield;
	bool bIncludeClassSlot;
	bool bIncludeTeamIndex;
	Unknown MinigameScoreStats;
	bool bIncludeInventory;
	bool bIncludeResources;
	bool bIncludeGold;
	bool bIncludeCheckpoints;
	bool bIncludePlayerLocation;
};

struct FFortInventory_SaveEntryBase
{
	bool bHasBeenStored;
};

struct FFortInventory_CheckpointEntry
{
	struct Guid LevelSpawnableGuid;
	struct DateTime MostRecentUseTime;
};

struct FFortInventory_PlayerStatEntry
{
	class UClass* StatFilter;
	int Value;
};

struct FFortInventory_SaveEntry
{
	struct DateTime MostRecentUseTime;
	struct FortInventory_SaveEntryFloat Health;
	struct FortInventory_SaveEntryFloat Shield;
	struct FortInventory_SaveEntryUint8 ClassSlot;
	struct FortInventory_SaveEntryUint8 TeamIndex;
	struct FortInventory_SaveEntryItems Items;
	struct FortInventory_SaveEntryItems Resources;
	struct FortInventory_SaveEntryItems Gold;
	struct FortInventory_SaveEntryCheckpoints Checkpoints;
	struct FortInventory_SaveEntryTransform PlayerTransform;
	struct FortInventory_MinigameStats MinigameStats;
};

struct FPendingToApplyData
{
	Unknown PlayerController;
	Unknown PlayerState;
	Unknown Minigame;
	struct FortInventory_SaveEntry SaveEntry;
};

struct FFortCreativePlotPermissionData
{
	EFortCreativePlotPermission Permission;
	TArray<struct FString> WhiteList;
};

struct FActiveRealEstatePlotInfo
{
	class UClass* Plot;
	struct Vector Position;
};

struct FFortSourDropperColorData
{
	Unknown TypeColors;
};

struct FTeleporterGroup
{
	Unknown Teleporters;
};

struct FTimerObjectiveHUDData
{
	float TimeLeft;
	bool bIsPaused;
};

struct FFortCurieCableSocketIdentifier
{
	struct GameplayTag Tag;
	int Index;
};

struct FFortCurieCableSocketConnection
{
	class UClass* ConnectedActor;
	struct FortCurieCableSocketIdentifier ConnectedSocketIdentifier;
	class UClass* ConnectedSocketComponent;
};

struct FFortCurieCableSocket
{
	struct FortCurieCableSocketIdentifier SocketIdentifier;
	struct Vector Location;
	struct FortCurieCableSocketConnection SocketConnection;
	bool bAutoSendSignalOnElementAttachment;
	bool bAutoSendSignalOnElementDetachment;
	bool bAutoRouteToCurieComponentOnReceive;
	Unknown ReceivedElementToMagnitudeMap;
};

struct FFortCurieStateSuggestionData
{
	struct GameplayTag StateTag;
	struct GameplayTagQuery AttachedStatesQuery;
	bool bIncludeStateParams;
	bool bDiscardOnResolve;
};

struct FFortCurieActorFireEntry
{
	TArray<class UClass*> AvailablePropagationNeighbors;
};

struct FFortCurieAmbientAudioLocationKey
{
};

struct FFortCurieActiveAmbientAudio
{
	class UClass* AudioComponent;
	class UClass* OwningComponent;
	int AudioClusterCount;
};

struct FFortCurieGlowFadeRequest
{
	class UClass* CurieComponent;
	EFortCurieNativeFXType FXType;
	float StartTimestamp;
	bool bIsFadeIn;
};

struct FFortCurieActiveElectricityArc
{
	struct Vector TargetActorSmallestAxis;
	class UClass* Mesh;
	class UClass* OwningComponent;
	float StartTime;
	bool bPlayedImpact;
};

struct FFortCuriePendingElectricityArcRequest
{
	class UClass* RequestingComponent;
	float ExecutionTime;
	bool bIsFirstIteration;
};

struct FFortCurieFireParticleActorData
{
	class UClass* AssociatedComponent;
};

struct FFortCurieFireParticleGrassData
{
};

struct FFortSpatialCellIndex
{
	int X;
	int Y;
	int Z;
};

struct FFortCurieWorldFirePlayerSystem
{
	class UClass* ActiveWorldFireSystem;
	float LastFireParticleSampleTimestamp;
};

struct FNativeCurieFXTypeSettings
{
	struct RuntimeFloatCurve GlowFadeInCurve;
	struct RuntimeFloatCurve GlowFadeOutCurve;
	int GlowPriority;
	float GlowMaterialIdx;
	bool bNeedsSignificanceTracking;
	bool bNeedsGlow;
	bool bNeedsAmbientAudio;
};

struct FFortCurieExecutionFilter
{
	struct GameplayTag RequiredState;
};

struct FFortCurieExecutionParams
{
	struct GameplayTag ElementIdentifier;
	struct ScalableFloat Magnitude;
	struct ScalableFloat LandscapeEffectRadius;
	struct ScalableFloat PropagationFuelOverride;
};

struct FFortCurieExecutionEntry
{
	EFortCurieExecutionType ExecutionType;
	EFortCurieApplicationEvent ApplicationEvent;
	struct FortCurieExecutionFilter ExecutionFilter;
	struct FortCurieExecutionParams ExecutionParameters;
};

struct FCurieToggleComponentGroup
{
	TArray<class UClass*> GroupComponents;
	struct FScriptMulticastDelegate OnToggleGroupFullyActive;
	struct FScriptMulticastDelegate OnToggleGroupFullyInactive;
	struct FScriptMulticastDelegate OnToggleGroupMemberStateChange;
};

struct FFortSpatialGrid
{
};

struct FFortCustomizationAssetsToLoad
{
	TArray<class UClass*> HeroTypes;
	TArray<class UClass*> CharacterParts;
	TArray<class UClass*> ItemDefs;
	TArray<struct SoftObjectPath> VariantAssets;
};

struct FFortDailyRewardScheduleDisplayData
{
	struct FText Title;
	struct FText Description;
	struct FText ItemDescription;
	struct FText EpicItemDescription;
};

struct FFortDailyRewardScheduleDefinition
{
	struct FName ScheduleName;
	struct TSoftClassPtr<UObject> EnablingToken;
	class UClass* Rewards;
	struct FortDailyRewardScheduleDisplayData DisplayData;
	struct DateTime BeginDate;
	struct DateTime EndDate;
};

struct FFortDamageNumberColorInfo
{
	struct FString DisplayText;
	struct LinearColor Color;
	struct LinearColor CriticalColor;
	struct GameplayTagContainer Tags;
};

struct FPooledDamageNumberComponents
{
	TArray<class UClass*> Components;
};

struct FLiveDamageNumberComponent
{
	class UClass* Component;
};

struct FFortBasicAudioParamList
{
	TArray<struct FortBasicAudioParam> Params;
};

struct FSkyLightValues
{
	struct LinearColor SkyLightColor;
	struct LinearColor SkyLightOcclusionTint;
	float SkyLightMinOcclusion;
	float VolumetricScatteringIntensity;
	class UClass* Cubemap;
	class UClass* DestinationCubemap;
};

struct FCloudColorState
{
	struct LinearColor BottomEmissive;
	struct LinearColor TopEmissive;
	struct LinearColor BottomLightning;
	struct LinearColor TopLightning;
	struct LinearColor InternalColor;
};

struct FThreatCloudValues
{
	struct CloudColorState CloudActivated;
	struct CloudColorState CloudDeactivated;
};

struct FElementalCharValues
{
	struct LinearColor FireCharColor;
	float ElectricalCharEmissive;
};

struct FDirectionalLightValues
{
	struct Color LightColor;
	float Brightness;
	float VolumetricScatteringIntensity;
};

struct FExponentialHeightFogValues
{
	float FogDensity;
	float FogHeightFalloff;
	float FogMaxOpacity;
	float StartDistance;
	float DirectionalInscatteringExponent;
	float DirectionalInscatteringStartDistance;
	struct LinearColor DirectionalInscatteringColor;
	struct LinearColor FogInscatteringColor;
	float VolumetricFogScatteringDistribution;
	struct LinearColor VolumetricFogAlbedo;
	float VolumetricFogExtinctionScale;
	float VolumetricFogDistance;
	struct ExponentialHeightFogData SecondFogData;
};

struct FSkyAtmosphereValues
{
	float RayleighScatteringScale;
	struct LinearColor RayleighScattering;
	float RayleighExponentialDistribution;
	float MieScatteringScale;
	struct LinearColor MieScattering;
	float MieAbsorptionScale;
	struct LinearColor MieAbsorption;
	float MieAnisotropy;
	float MieExponentialDistribution;
	float OtherAbsorptionScale;
	struct LinearColor OtherAbsorption;
	struct LinearColor SkyLuminanceFactor;
	float AerialPespectiveViewDistanceScale;
	float HeightFogContribution;
};

struct FDayPhaseInfo
{
	struct FString PhaseStartAnnouncement;
	float TimePhaseBegins;
	float PhaseLengthInHours;
	float PercentageTransitionIn;
	float TransitionInTimeInMinutes;
	float PercentageTransitionOut;
	float TransitionOutTimeInMinutes;
	struct SkyLightValues SkyLightValues;
	struct ThreatCloudValues ThreatCloudValues;
	struct ElementalCharValues ElementalCharValues;
	struct DirectionalLightValues DirectionalLightValues;
	struct ExponentialHeightFogValues ExpHeightFogValues;
	struct SkyAtmosphereValues SkyAtmosphereValues;
	class UClass* LowPriPostProcessComponent;
	class UClass* SkyMaterialInstance;
	class UClass* StarMapMaterialInstance;
};

struct FCameraAltitudeAdjustments
{
	float Altitude;
	float FogHeightFalloff;
	float HeightFogZOffset;
	float FogDensity;
};

struct FFortDeathCauseInfoVariant
{
	struct FString DisplayText;
	struct GameplayTag DeathCauseTag;
	TArray<struct FText> SelfInflictedMessages;
	TArray<struct FText> SelfInflictedDBNOMessages;
	TArray<struct FText> EliminatedMessages;
	TArray<struct FText> EliminatedDBNOMessages;
};

struct FFortDeathCauseInfo
{
	struct FString DisplayText;
	struct GameplayTag DeathCauseTag;
	TArray<struct FText> SelfInflictedMessages;
	TArray<struct FText> SelfInflictedDBNOMessages;
	TArray<struct FText> EliminatedMessages;
	TArray<struct FText> EliminatedDBNOMessages;
	TArray<struct FortDeathCauseInfoVariant> DeathCauseInfoVariants;
};

struct FFortTagToDeathCause
{
	struct GameplayTag DeathTag;
	EDeathCause DBNOCause;
	EDeathCause DeathCause;
};

struct FDecoPlacementState
{
	struct Vector Start;
	struct Vector End;
	struct Vector RawLocation;
	struct Vector Normal;
	struct Quat AbsoluteRotation;
	struct Vector GridLocation;
	struct Vector PreviousLocation;
	struct Vector FallbackLocation;
	Unknown LastHitActor;
	Unknown CurrentBuildingActorAttachment;
	struct Vector CreateBuildingLocation;
	struct Rotator CreateBuildingRotation;
	EFortDecoPlacementQueryResults CanPlaceState;
};

struct FFortTierProgressionInfo
{
	struct FString ProgressionLayoutGuid;
	int HighestDefeatedTier;
};

struct FFortTierProgression
{
	TArray<struct FortTierProgressionInfo> ProgressionInfo;
};

struct FFortUserCloudRequestHandle
{
	uint64_t Handle;
};

struct FPendingDeployableBaseUser
{
	struct UniqueNetIdRepl UserNetId;
	struct FortUserCloudRequestHandle LoadingCloudRequestHandle;
	class UClass* BaseRecord;
	class UClass* BasePlot;
};

struct FPendingDeployableManagerAction
{
	EQueueActionType ActionType;
	TArray<class UClass*> PendingPlots;
	int CurrentPlotRunningIndex;
	EDeployableBaseBuildingState DesiredPlotState;
	class UClass* Manager;
};

struct FEnvironmentBuildingRestorationRecord
{
	class UClass* ActorClass;
	struct Transform ActorTransform;
	struct FName QuotaSelectedLootTierKey;
	int QuotaSelectedLootTier;
};

struct FFortPickupLocationData
{
	class UClass* PickupTarget;
	class UClass* CombineTarget;
	class UClass* ItemOwner;
	struct Vector_NetQuantize10 LootInitialPosition;
	struct Vector_NetQuantize10 LootFinalPosition;
	float FlyTime;
	struct Vector_NetQuantizeNormal StartDirection;
	struct Vector_NetQuantize10 FinalTossRestLocation;
	EFortPickupTossState TossState;
	bool bPlayPickupSound;
	struct Guid PickupGuid;
};

struct FFortTaggedUnlockBase
{
	struct GameplayTag RequiredTag;
};

struct FFortDialogDescription
{
	struct SlateBrush Icon;
	struct FText MessageHeader;
	struct FText MessageBody;
	struct FText AcceptButtonText;
	struct FText IgnoreButtonText;
	struct FText DismissButtonText;
	float DisplayTime;
	class UClass* AdditionalContent;
	EFortDialogFeedbackType FeedBackType;
	bool Dismissable;
	class UClass* NotificationHandler;
};

struct FFortEncounterLockedUtility
{
	EFortAIUtility Utility;
	EFortEncounterUtilityDesire UtilityDesire;
};

struct FFlightParams
{
	float TopSpeedKmh;
	float LiftoffSpeedKmh;
	float LandingSpeedKmh;
	float ControlSpeedKmh;
	float HeadingStabilizationRate;
	float HeadingStabilizationMaxForwardVelocityKmh;
	float HeadingStabilizationMaxDegPerSecond;
	float VerticalStabilizationDrag;
	float HorizontalStabilizationDrag;
	float VerticalStabilizationTorque;
	float MaxVerticalStabilizationTorque;
	float HorizontalStabilizationTorque;
	float MaxHorizontalStabilizationTorque;
	float RotationalDampingCoefficient;
	float MaxRotationalDampingTorque;
	float TailLength;
	float LowSpeedThrust;
	float HighSpeedThrust;
	float AntigravityHorizontal;
	float AntigravityUp;
	float AntigravityDown;
	float ControlFrameHeight;
	float ControlFrameDistance;
	float ControlFrameDistanceInterpPerSecond;
	float ControlFrameOrbitInterpPerSecond;
	float ControlFrameRollInterpPerSecond;
	struct Vector ControlFrameDefaultRollUp;
	float ControlFrameRollUpAcceleration;
	float ControlFrameRollUpMaxVelocity;
	float ControlFrameRollUpDamping;
	float ControlFrameMinUpNudge;
	float ControlFrameMaxUpNudge;
	float ControlFrameUpsideDownIgnoreNudgePercent;
	float SteerPitchRate;
	float SteerYawRate;
	float SteerMaxHeadingDiffDegrees;
	float RollPerHeadingDiff;
	float HeadingMatchRate;
	float RollMatchRate;
	float MatchingTorqueCap;
	float StallVelocityLow;
	float StallVelocityHigh;
	float MinStallYawMultiplier;
	float MaxStallYawMultiplier;
	float StallHighVelocityDeviationAngle;
};

struct FFortRechargingActionTimerConfig
{
	float ChargeRate;
	float ActiveExpenseRate;
	float PassiveExpenseRate;
	float MinActiveDuration;
	float MinActivationCharge;
	float ActiveCooldownTime;
	float ChargeThreshold;
};

struct FFortEmoteVolumePlayerTrackingInfo
{
};

struct FCameraSequence
{
	struct TSoftClassPtr<UObject> CameraSequencer;
	bool PlayFromStart;
	bool UseCinematiceMode;
};

struct FNavWidgetSettings
{
	bool bDrawNavWidget;
	float Distance;
	float Angle;
	float MinRandomNavAngle;
	float MaxRandomNavAngle;
	struct GameplayTagContainer WordSetsToUse;
};

struct FNavWidgetSettingContainer
{
	struct GameplayTagContainer CameraTags;
	struct NavWidgetSettings WidgetSettings;
};

struct FNavOptionFallback
{
	ENavOptionFallbackDir NavDir;
	class UClass* NavObj;
};

struct FNavOptions
{
	struct GameplayTag CameraTag;
	class UClass* NavObjToLeft;
	class UClass* NavObjToRight;
	class UClass* NavObjToUp;
	class UClass* NavObjToDown;
	TArray<struct NavOptionFallback> NavFallbacks;
};

struct FFactionData
{
	bool bActive;
	struct FText Name;
	struct FText Description;
	struct GameplayTag FactionTag;
	EFortFactionAttitude DefaultAttitude;
	bool bAreFactionMemberAllies;
	bool bShouldSpecificRelationsApplyToEntireFaction;
	bool bMembersCanResetSpecificRelation;
	struct ScalableFloat ResetSpecificRelationUnawareDelay;
	struct ScalableFloat ResetSpecificRelationNoDamageDelay;
	struct GameplayTagContainer RelationExceptions;
};

struct FFortFeedbackResponse
{
	struct FortFeedbackHandle Handle;
	EFortFeedbackContext Context;
};

struct FFortFeedbackLine
{
	struct TSoftClassPtr<UObject> Audio;
	struct TSoftClassPtr<UObject> AnimSequence;
	struct TSoftClassPtr<UObject> AnimMontage;
	EFortFeedbackAddressee Addressee;
	EFortFeedbackContext Context;
	TArray<struct FortFeedbackResponse> ResponseEvents;
	bool bInterruptCurrentLine;
	bool bCanBeInterrupted;
	bool bCanQue;
};

struct FFortFeedbackAction
{
	struct FortFeedbackHandle Handle;
	TArray<struct FortFeedbackLine> Lines;
};

struct FFortFeedbackEventData
{
	struct FortFeedbackHandle Handle;
	float ChanceToPlay;
	float MinReplayTime;
	float MinReplayTimeForSpeaker;
	float MaxWitnessDistance;
	bool bInterruptCurrentLine;
	bool bCanBeInterrupted;
	bool bCanQue;
	EFortFeedbackBroadcastFilter MultiplayerBroadcastFilter;
	EFortFeedbackSelectionMethod ContextSelectionMethod;
	float FeedbackDelay;
	float TimeLastPlayed;
};

struct FFortFootstepSurfaceAudioData
{
	class UClass* SoundAssets;
	class UClass* SoundAssetsAbove;
	class UClass* SoundAssetsBelow;
};

struct FFortFootstepAttenuationData
{
	class UClass* SoundAttenuation;
	class UClass* SoundAttenuationAbove;
	class UClass* SoundAttenuationBelow;
	class UClass* SoundAttenuationAboveOrBelowAndVisible;
	float VolumeMultiplier;
};

struct FFortChallengeMapPoiData
{
	struct FText Text;
	struct Vector WorldLocation;
	struct GameplayTag LocationTag;
	TArray<struct FString> CalendarEventsRequired;
	struct TSoftClassPtr<UObject> DiscoveryQuest;
	struct FName DiscoverObjectiveBackendName;
	TArray<int> ActiveDiscoverableBlodIds;
};

struct FFortFXAnimationInfoBase
{
	class UClass* LerpCurve;
};

struct FFortSplineMeshAnimSet
{
	class UClass* SplineMesh;
	TArray<struct FortSplineMeshScaleAnimationInfo> ScaleAnims;
	TArray<struct FortSplineMeshSnapAnimationInfo> SnapAnims;
};

struct FFortMIDAnimSet
{
	class UClass* Mid;
	TArray<struct FortFloatParamAnimationInfo> FloatParamAnims;
	TArray<struct FortLinearColorParamAnimationInfo> ColorParamAnims;
	TArray<struct FortLinearColorCurveParamAnimationInfo> ColorCurveParamAnims;
};

struct FFortParticleAnimSet
{
	class UClass* PSC;
	TArray<struct FortFloatParamAnimationInfo> ParamAnims;
};

struct FFortLightAnimSet
{
	class UClass* LightComp;
	TArray<struct FortFloatAnimationInfo> IntensityAnims;
};

struct FFortHighlightParamProfile
{
	struct FName UniqueProfileName;
	struct FName StencilParamName;
	struct FName OutlineColorParamName;
	struct FName SceneModulationColorOneParamName;
	struct FName SceneModulationColorTwoParamName;
};

struct FFortHighlightColors
{
	struct LinearColor OutlineColor;
	struct LinearColor SceneModulationColor1;
	struct LinearColor SceneModulationColor2;
};

struct FFortHighlightColorsContainer
{
	struct FortHighlightColors ValidHighlight;
	struct FortHighlightColors InvalidHighlight;
	struct FName ParamProfileName;
};

struct FFortQuestPackInfo
{
	struct FString Name;
	struct FString DefaultQuestPack;
	int MaxActiveDailyQuests;
	int MaxRerollsPerDay;
	int DaysToKeepClaimedQuests;
	int DaysToKeepCompletedQuests;
	int MaxUnclaimedQuests;
	bool IsStreamingQuestPack;
};

struct FDamageDistanceTagEval
{
	float DistanceLimit;
	struct GameplayTag DistanceTag;
};

struct FSubGameInfo
{
	class UClass* AccessToken;
	bool RequiredFullInstall;
	bool bCanPartyWithoutFullInstall;
};

struct FPartnerPcbInfo
{
	struct GameplayTag PartnerTag;
	struct GameplayTagContainer ValidPartnerBundles;
};

struct FCumulativeFrameTimeWithoutSleepLimits
{
	double FrameTimeWithoutSleep;
	double MaxCumulativeFrameTimeAboveThreshold;
	double MaxNumberOfFramesAboveThreshold;
};

struct FHotfixVolumePlacement
{
	struct FString Map;
	struct Vector Center;
	struct Vector Extent;
	struct FString GameInclude;
	struct FString GameExclude;
	bool bNeededOnClient;
};

struct FTimeOfDayOverride
{
	struct FName PlaylistName;
	float TimeOfDay;
	float TimeOfDaySpeed;
};

struct FGCSettingsOverride
{
	struct FName PlaylistName;
	bool bEnableGCOnServerDuringMatch;
	float GCFrequency;
};

struct FScorePlacementTable
{
	TArray<float> Solo;
	TArray<float> Duos;
	TArray<float> Squads;
	TArray<float> FiftyFifty;
	TArray<float> LargeTeam;
	TArray<float> MediumTeam;
	TArray<float> QuickSolo;
	TArray<float> QuickDuo;
	TArray<float> QuickSquad;
	TArray<float> QuickLargeTeam;
	TArray<float> QuickTwoTeam;
	TArray<float> QuickMediumTeam;
	TArray<float> SinglePlacement;
	TArray<float> Default;
};

struct FSupplyDropZoneBasedSpawnData
{
	TArray<class UClass*> SpawnedSupplyDrops;
	int NumDropsRemainingInWave;
	float NextWaveSpawnTime;
	float NextSpawnTime;
	EAthenaGamePhase CurrGamePhase;
	int CurrSubPhase;
	int TotalSupplyDropsSpawnedInSubPhase;
};

struct FSupplyDropItemDeliverySpawnData
{
	int NumItemsToDeliver;
	TArray<float> QueuedSpawnTimes;
	float NextSpawnTime;
	int NumInitialSpawns;
};

struct FSupplyDropSpawnData
{
	class UClass* SupplyDropInfo;
	struct SupplyDropZoneBasedSpawnData ZoneBasedData;
	struct SupplyDropItemDeliverySpawnData ItemDeliveryData;
};

struct FBuildingGameplayActorSpawnData
{
	int BGAIndex;
	int NumSpawnedBGAs;
	int NumBGAsToSpawn;
	float NextSpawnTime;
	EBuildingGameplayActorAircraftSpawnSide LastSpawnSide;
};

struct FExitCraftSpawnData
{
	class UClass* ExitCraftInfo;
};

struct FFortSpawnActorData
{
	class UClass* SpawnActorInfo;
	int NumSpawnsRemaining;
	float TimeUntilNextSpawn;
	TArray<class UClass*> SpawnedFortSpawnActors;
};

struct FFortCustomRepNodeClassMapping
{
	class UClass* NodeClass;
	TArray<class UClass*> ClassesToRoute;
	bool bShouldDisableSquadNodes;
	bool bShouldDisableHighFrequencyPawnRouting;
};

struct FAthenaStreamIdOverride
{
	struct FString SourceName;
	struct FString OverriddenId;
};

struct FFortPlacedBuilding
{
	struct FString BuildingTag;
	struct FString PlacedTag;
};

struct FFortOutpostCoreInfo
{
	TArray<struct FortPlacedBuilding> PlacedBuildings;
	TArray<struct FString> AccountsWithEditPermission;
	uint32_t HighestEnduranceWaveReached;
};

struct FFortAimAssist2D_Settings
{
	struct ScalableFloat AssistInnerReticleWidth;
	struct ScalableFloat AssistInnerReticleHeight;
	struct ScalableFloat AssistOuterReticleWidth;
	struct ScalableFloat AssistOuterReticleHeight;
	struct ScalableFloat TargetingReticleWidth;
	struct ScalableFloat TargetingReticleHeight;
	struct ScalableFloat TargetRange;
	class UClass* TargetWeightCurve;
	struct ScalableFloat TargetWeightDBNOScale;
	struct FortTargetFilter TargetFilter;
	struct ScalableFloat PullInnerStrengthHip;
	struct ScalableFloat PullOuterStrengthHip;
	struct ScalableFloat PullInnerStrengthAds;
	struct ScalableFloat PullOuterStrengthAds;
	struct ScalableFloat PullLerpInRate;
	struct ScalableFloat PullLerpOutRate;
	struct ScalableFloat PullMaxRotationRate;
	struct ScalableFloat SlowInnerStrengthHip;
	struct ScalableFloat SlowOuterStrengthHip;
	struct ScalableFloat SlowInnerStrengthAds;
	struct ScalableFloat SlowOuterStrengthAds;
	struct ScalableFloat SlowLerpInRate;
	struct ScalableFloat SlowLerpOutRate;
	struct ScalableFloat SlowMinRotationRate;
};

struct FFortFollowCam_Settings
{
	struct ScalableFloat TurnDampeningStart;
	struct ScalableFloat TurnFullRate;
	struct ScalableFloat TurnFullDeflectionThreshold;
	struct ScalableFloat TurnBoostRate;
	struct ScalableFloat TurnDampeningRecoveryRate;
	struct ScalableFloat TurnDeadZone;
	struct ScalableFloat HorizonPullDelay;
	struct ScalableFloat HorizonPullDampeningTime;
	struct ScalableFloat HorizonPullHalfTime;
};

struct FTurnFloatRange
{
	float Min;
	float Max;
};

struct FTurnTransitionData
{
	bool bUseMontageForTurnTransition;
	struct FName MontageSectionName;
	float MinYawAngle;
	float MaxYawAngle;
	float TurnRate;
	struct GameplayTagContainer RequiredGameplayTags;
	int PriorityLevel;
	struct TurnFloatRange SpeedConstraintRange;
	bool bEnableSpeedConstraint;
	bool bSkipTransitionInCrowd;
};

struct FFortPawnSpinParams
{
	float LocalSpin;
	bool bOnlySpinWhenFalling;
};

struct FSpyTechUpgradeData
{
	struct GameplayTagContainer UpgradeTags;
	struct ScalableFloat IsUpgradeEnabled;
	struct PrimaryAssetId UpgradedItemAssetID;
};

struct FSpyTechLevelUpgradeData
{
	struct GameplayTagContainer LevelUpgradeTags;
	struct ScalableFloat IsLevelUpgradeEnabled;
	struct ScalableFloat MaxLevelUpgrade;
};

struct FActiveItemGrantInfo
{
	class UClass* Item;
	struct ScalableFloat AmountToGive;
	struct ScalableFloat MaxAmount;
};

struct FFortParticleSystemParamBucket
{
	TArray<struct ParticleSysParam> Parameters;
};

struct FFortGameplayDataTrackerEventConfiguration
{
	struct GameplayTag EventTag;
	struct ScalableFloat CoolDownRate;
	EFortGameplayDataTrackerEventContributionType ContributionType;
};

struct FFortGameplayDataTrackerActorStateConfig
{
	struct GameplayTag StateTag;
	struct GameplayTag EventTag;
	struct ScalableFloat ShouldUseDistanceCheck;
	struct ScalableFloat ShouldInterpolateValueOverDistance;
	struct ScalableFloat DistanceNear;
	struct ScalableFloat DistanceFar;
	struct ScalableFloat DistancePercentLerpExponent;
	struct ScalableFloat ShouldUseDotProductTest;
	struct ScalableFloat DotProductComparisonValue;
	struct ScalableFloat DotProductFalureMultiplier;
	struct ScalableFloat ShouldHaveFirstEntryEvent;
	struct GameplayTag FirstEntryEvent;
	struct ScalableFloat FirstEntryEventGuaranteedRange;
	struct ScalableFloat FirstEntryEventNearComparisonRange;
	struct ScalableFloat FirstEntryEventTrackedActorFacingNearDotProductComparisonValue;
	struct ScalableFloat FirstEntryEventTrackedActorFacingFarDotProductComparisonValue;
	struct ScalableFloat FirstEntryEventAvatarActorFacingNearDotProductComparisonValue;
	struct ScalableFloat FirstEntryEventAvatarActorFacingFarDotProductComparisonValue;
};

struct FFortGameplayDataTrackerTrackedActorState
{
	class UClass* TrackedActor;
	class UClass* TrackedActorAsPawn;
	class UClass* TrackedActorAsBuilding;
	struct GameplayTag CurrentState;
	bool bHasUnprocessedStateEntry;
};

struct FFortGameplayDataTrackerActorStateGroupConfig
{
	TArray<struct TSoftClassPtr<UObject>> RelevantActorClasses;
	bool bAllowDBNOPawns;
	TArray<struct FortGameplayDataTrackerActorStateConfig> StateConfigs;
	TArray<struct FortGameplayDataTrackerTrackedActorState> ActorStates;
};

struct FFortGameplayDataTrackerAccumulationContributor
{
	struct GameplayTag EventTag;
	struct ScalableFloat EventValueMultiplier;
	float CurrentValue;
};

struct FFortGameplayDataTrackerThreshold
{
	struct ScalableFloat EnterValue;
	struct ScalableFloat ExitValue;
};

struct FFortGameplayDataTrackedRange
{
	struct GameplayTag RangeTag;
	struct FortGameplayDataTrackerThreshold LowerThreshold;
	struct FortGameplayDataTrackerThreshold UpperThreshold;
	struct FScriptMulticastDelegate RangeChangedInternalDelegate;
	bool bIsCurrentlyInRange;
};

struct FFortGameplayDataTrackerAccumulation
{
	struct GameplayTag AccumulationTag;
	TArray<struct FortGameplayDataTrackerAccumulationContributor> Contributions;
	TArray<struct FortGameplayDataTrackedRange> TrackedRanges;
	float CurrentValue;
};

struct FFortGameplayDataTrackerEventValue
{
	struct GameplayTag EventTag;
	float Value;
};

struct FGameplayFeedbackEventInfo
{
	struct FString DisplayText;
	struct GameplayTag EventTag;
	EAthenaGameMsgType MsgType;
	bool bTeamBasedEvent;
	TArray<struct FText> FriendlyEventMessages;
	TArray<struct FText> HostileEventMessages;
	TArray<struct FText> NeutralEventMessages;
};

struct FFortGameplayMessageReceiverList
{
	TArray<class UClass*> Receivers;
};

struct FFortGameplayMessageTriggerList
{
	TArray<class UClass*> Triggers;
};

struct FFortTriggeredGameplayMessage
{
	class UClass* Sender;
	struct GameplayTag ChannelId;
	Unknown TriggerInstigator;
};

struct FFortEncounterMutatorReplacedTag
{
	struct GameplayTagContainer TagsToRemove;
	struct GameplayTagContainer TagsToAdd;
};

struct FSetCVarParams
{
	struct FName CVarName;
	ESetCVarType Type;
	float NumberValue;
	struct FString StringValue;
};

struct FTierTags
{
	struct GameplayTag TagForTier;
};

struct FFortHostSessionParams
{
	struct FName SessionName;
	int ControllerId;
};

struct FFortDisconnectedPlayerReservation
{
	struct FName SessionName;
	struct UniqueNetIdRepl PlayerID;
};

struct FClientIdRestrictions
{
	struct FString ClientId;
	TArray<struct FString> RestrictedPlatforms;
};

struct FAdditionalLevelStreamed
{
	struct FName LevelName;
	bool bIsServerOnly;
};

struct FOnTimeHitInfo
{
	struct FDelegate TimeCallback;
};

struct FTeamChangeRequest
{
	class UClass* RequestingController;
	byte DesiredTeam;
};

struct FAppliedHomebaseData
{
	class UClass* Source;
	class UClass* Target;
	TArray<struct ActiveGameplayEffectHandle> AppliedEffects;
	TArray<struct FortAbilitySetHandle> AppliedAbilitySets;
};

struct FGameStateLootInfo
{
	TArray<struct FortTreasureChestSpawnInfo> TreasureChestSpawnInfos;
	TArray<struct FortAmmoBoxSpawnInfo> AmmoBoxSpawnInfos;
};

struct FBuildingContainerDebugInfo
{
	struct Vector Location;
	struct GameplayTagContainer LocationTags;
};

struct FReplicatedMontageIndexPair
{
	class UClass* Montage;
	int Index;
};

struct FReplicatedMontageMap
{
	TArray<struct ReplicatedMontageIndexPair> Mappings;
};

struct FActiveGameplayModifierHandle
{
	int Handle;
};

struct FFortZoneDifficultyIncreaseRewardData
{
	TArray<struct FortItemQuantityPair> Rewards;
};

struct FFortZoneMissionAlertData
{
	TArray<struct FortItemQuantityPair> MissionAlertRewards;
	struct FString MissionAlertCategoryName;
	struct FString MissionAlertID;
};

struct FPlayerBuildableClassContainer
{
	TArray<class UClass*> BuildingClasses;
};

struct FFortVoteConfig
{
	int NumVoteOptions;
	float VoteDuration;
	float FailedVoteLockOutDuration;
	int MaxVotesAllowedPerPlayer;
	EFortVoteArbitratorType VoteArbitratorType;
};

struct FVoter
{
	int VoteDecision;
	int LastVoteDecision;
	struct UniqueNetIdRepl NetId;
	int NumVotesCast;
};

struct FVoteData
{
	EFortVoteType VoteType;
	struct UniqueNetIdRepl VoteInstigatorNetId;
	struct FortVoteConfig VoteConfig;
	float VoteStartTime;
	float VoteEndTime;
	TArray<struct Voter> Voters;
	int NumVotersWithMaxVotes;
	struct TimerHandle EndVoteTimerHandle;
	EFortVoteStatus VoteStatus;
};

struct FStormCapDamageThresholdInfo
{
	float ThresholdFloor;
	float ThresholdCeiling;
};

struct FFortWinnerPlayerData
{
	int PlayerID;
};

struct FDynamicLandData
{
	struct FName Name;
	struct Vector WorldPosition;
	struct SlateBrush LandBrush;
};

struct FAICharacterPartsPreloadData
{
	float Priority;
	class UClass* CharacterPart;
};

struct FAIPawnCustomizationPreloadData
{
	float Priority;
	class UClass* Customization;
};

struct FPropertyOverrideScope
{
	TArray<struct PropertyOverride> PropertyOverrides;
};

struct FGameplayMutatorEventData
{
	int EventId;
	int EventParam1;
	int EventParam2;
	int EventParam3;
};

struct FMeshNetworkStatus
{
	bool bEnabled;
	bool bConnectedToRoot;
	EMeshNetworkNodeType GameServerNodeType;
};

struct FVariantUsageReportInner
{
	struct GameplayTag ChannelTag;
	struct GameplayTag VariantTag;
	int UseCount;
};

struct FVariantUsageReport
{
	TArray<struct VariantUsageReportInner> VariantUsage;
	int TotalUses;
};

struct FCosmeticUsageReport
{
	TArray<class UClass*> TrackedCosmetics;
	TArray<struct VariantUsageReport> VariantUsageByCosmetic;
	TArray<int> PrioritizedCosmeticIndices;
};

struct FFortFactionSharedBBInfo
{
	TArray<class UClass*> Members;
	struct GameplayTag BlackboardTag;
	class UClass* BlackboardComp;
};

struct FFortFactionInfo
{
	struct GameplayTag FactionTag;
	EFortFactionAttitude DefaultAttitude;
	bool bHasSpecificRelations;
	bool bAreFactionMemberAllies;
	bool bShouldSpecificRelationsApplyToEntireFaction;
	bool bMembersCanResetSpecificRelation;
	float ResetSpecificRelationUnawareDelay;
	float ResetSpecificRelationNoDamageDelay;
	struct GameplayTagContainer FactionRelations;
	struct FortAffiliationActorIdentifierList SpecificRelations;
	TArray<class UClass*> Members;
	TArray<struct FortFactionSharedBBInfo> SharedBBInfos;
};

struct FDynamicCompositeWorld
{
	struct TSoftClassPtr<UObject> MainCompositeWorld;
	TArray<struct TSoftClassPtr<UObject>> AdditionalCompositeWorlds;
	struct FString CalendarEventName;
	struct FName CameraName;
	EFrontEndCamera CameraOverride;
	float GetDefaultLevelTransitionTime;
	bool bStreamInOnDemand;
	struct GameplayTag StreamingInLevelTag;
	struct GameplayTag StreamingOutLevelTag;
	bool bStreamInLevel;
	bool bLevelStreamedIn;
};

struct FHiddenActors
{
	struct GameplayTag RewardEventGraphNode;
	TArray<Unknown> HiddenActors;
};

struct FAnalyticsNavActorClickedData
{
	struct GameplayTag InteractionCamera;
	struct GameplayTagContainer InteractionActor;
	struct FName HumanReadable;
};

struct FFortFrontendCompositeLevel
{
	struct FName StreamingLevelPackageName;
	TArray<class UClass*> StreamingLevels;
};

struct FMiniMapDataOverride
{
	struct GameplayTagContainer ContextTags;
	struct SlateFontInfo LocationFont;
	EMapLocationStateType StateType;
};

struct FMiniMapDataOverrideRepData
{
	struct GameplayTagContainer ContextTags;
	struct GameplayTagContainer LocationTags;
};

struct FPickupManagementSettings
{
	int PickupsAllowedMax;
	int PickupsDesiredSlack;
	float PickupDespawnDelaySeconds;
	bool bDebugPickupManagement;
	bool bEnablePickupManagement;
	EFortRarity NotJunkPickupThreshold;
	EFortRarity ImportantPickupThreshold;
	bool bFlagPlayerDropsAsImportant;
};

struct FHordeDifficultyTierInfo
{
	struct FName DifficultyTierName;
	class UClass* QuestPrerequisite;
};

struct FIronCityDifficultyInfo
{
	int AccountLevel;
	int Difficulty;
	int LootLevel;
	struct FName StatClamp;
	struct FString MissionRewards;
	struct GameplayTag RewardBadgeTag;
};

struct FPlaylistData
{
	struct FName PlaylistName;
	struct FString TournamentId;
	struct FString EventWindowId;
	struct FString RegionId;
};

struct FSavedCustomMatchOptions
{
	Unknown CustomMatchOptions;
};

struct FFortPortalOriginInfo
{
	struct FName PlaylistName;
	struct FString LinkCode;
};

struct FFireModeData
{
	bool bAutoFireIsEnabled;
	bool b3DTouchEnabled;
	bool bTapToShootEnabled;
	bool bAlwaysShowDedicatedButton;
	EFireModeType FireModeType;
};

struct FFortSimpleGameStats
{
	int GamesPlayed;
	int SecondsPlayed;
	int KillCount;
	int BestResult;
	struct DateTime LastReviewPromptDay;
	int CampaignGamesPlayed;
};

struct FSavedCredentials
{
	ESavedAccountType Type;
	struct FString ID;
	struct FString Token;
};

struct FRecentPlayerEncounter
{
	struct UniqueNetIdRepl UserId;
	struct DateTime EncounterTime;
};

struct FFortPresenceSocialStatus
{
	Unknown AttendingRTIds;
};

struct FSocialStatusSerialized
{
	struct UniqueNetIdRepl LocalUserId;
	struct UniqueNetIdRepl OtherUserId;
	struct FortPresenceSocialStatus SocialStatus;
};

struct FFortGenericDataStoreMapWrapper
{
	Unknown NamedStoreMap;
};

struct FFortLayeredAudioOneshotGate
{
	class UClass* Sound;
	float GateValue;
	ELayeredAudioTriggerDir Direction;
	bool FadeWhenOutsideGate;
	float MinTimeSinceTrigger;
	float InterruptFadeTime;
	class UClass* AudioComp;
};

struct FFortLayeredAudioFloatParam
{
	bool bEnabled;
	struct FName Name;
	float Value;
	ELayeredAudioInterpolationType InterpType;
	class UClass* Curve;
	float AttackSpeed;
	float ReleaseSpeed;
	struct Vector2D ParamRange;
	TArray<struct FortLayeredAudioOneshotGate> Oneshots;
	class UClass* Owner;
};

struct FChallengeObjectiveHotfix
{
	struct FName Quest;
	struct FName Stat;
	int Count;
	int NewCount;
};

struct FChallengeSuppressedHotfix
{
	struct FName Quest;
	bool bSuppressed;
	struct FName Replacing;
};

struct FXPEventEntryHotfix
{
	struct FName Entry;
	int CountThreshhold;
	int MaxCount;
};

struct FLoginFailureLogSubmitOptions
{
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	int LogTailKb;
	float LogSubmitChance;
	TArray<ELoginResult> DoNotUploadReasons;
};

struct FLogoutLogSubmitOptions
{
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	bool bSubmitLogsDuringLogin;
	int LogTailKb;
	float LogSubmitChance;
	TArray<struct FString> DoNotUploadReasons;
};

struct FPurchaseFailureLogSubmitOptions
{
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	int LogTailKb;
	float LogSubmitChance;
	TArray<struct FString> DoNotUploadReasons;
};

struct FPartyFailureLogSubmitReason
{
	struct FString Reason;
	struct FString SubReason;
	float LogSubmitChance;
};

struct FPartyFailureLogSubmit
{
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	int LogTailKb;
	TArray<struct PartyFailureLogSubmitReason> Reasons;
};

struct FKairosSubmitLogOptions
{
	struct FString Type;
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	int LogTailKb;
	float LogSubmitChance;
	TArray<struct FString> SubmitErrors;
};

struct FVoiceChatLogUploadRule
{
	float LogSubmitChance;
	struct FString VoiceEvent;
	struct FString ErrorCode;
};

struct FVoiceChatLogSubmitOptions
{
	bool bSubmitLogs;
	bool bSubmitSecondaryLogs;
	int LogTailKb;
	TArray<struct VoiceChatLogUploadRule> IncludeRules;
	TArray<struct VoiceChatLogUploadRule> ExcludeRules;
};

struct FSubGameAccess
{
	ESubGame SubGame;
	ESubGameAccessStatus AccessStatus;
	ESubGameMatchmakingStatus MatchmakingStatus;
};

struct FGoatVehicleBoostLevel
{
	float AccumulationPercent;
	float BoostTime;
};

struct FGravityGunVelocityAudioMapRangeClamped
{
	struct Vector2D InRange;
	struct Vector2D OutRange;
};

struct FHeldObjectThrownData
{
	struct ScalableFloat MaxSpeed;
	struct ScalableFloat GravityScale;
};

struct FHeldObjectMovementReplicatedData
{
	byte RepIncrement;
	struct Vector DetachLocation;
	struct Rotator DetachRotation;
	struct Vector MovementVelocity;
};

struct FCollisionResponseRestoreState
{
	byte OverlapResponse;
	byte BlockingResponse;
};

struct FFortHelpAdditionalContent
{
	EFortHelpContentLocation ContentLocation;
	bool ShowAdditionalImage;
	struct SlateBrush ImageToDisplay;
	bool ShowAdditionalWidget;
	class UClass* WidgetToDisplay;
};

struct FFortSavedModeLoadout
{
	struct FString LoadoutName;
	TArray<struct FString> SelectedGadgets;
};

struct FHeroPerkDefaultRequirements
{
	EFortItemTier MinimumHeroTier;
	int MinimumHeroLevel;
	EFortRarity MinimumHeroRarity;
};

struct FHeroSubclassRarityAttributeData
{
	TArray<struct FortAttributeInitializationKey> AttributeInitKeysPerTier;
};

struct FHeroSubclassAttributeData
{
	struct GameplayTagContainer HeroClassAndSubclassTags;
	TArray<struct HeroSubclassRarityAttributeData> AttributeInitKeysPerRarity;
};

struct FHeroKeywordDisplayData
{
	struct GameplayTag KeyWordGameplayTag;
	struct FText KeywordDisplayName;
};

struct FFortHeroTierAbilityKit
{
	struct TSoftClassPtr<UObject> GrantedAbilityKit;
	EFortRarity MinimumHeroRarity;
};

struct FFortHeroGameplayPiece
{
	struct TSoftClassPtr<UObject> GrantedAbilityKit;
	struct GameplayTagQuery RequiredCommanderTagQuery;
	struct FText CommanderRequirementsText;
	bool bUseGlobalDefaultMinima;
	EFortItemTier MinimumHeroTier;
	int MinimumHeroLevel;
	EFortRarity MinimumHeroRarity;
};

struct FHeroSpecializationAttributeRequirement
{
	struct GameplayAttribute Attribute;
	float MinimumValue;
};

struct FFortSpecializationSlot
{
	struct TSoftClassPtr<UObject> GrantedAbilityKit;
	struct TSoftClassPtr<UObject> RemovedAbilityKit;
	TArray<struct HeroSpecializationAttributeRequirement> AttributeRequirements;
	struct GameplayTagContainer RequiredTags;
	int MinimumHeroLevel;
};

struct FHighlightObjectData
{
	struct GameplayTagContainer HighlightTags;
	struct FName FriendlyStencilName;
	struct FName EnemyStencilName;
	byte FriendlyStencilIndex;
	byte EnemyStencilIndex;
	class UClass* Effect;
	float OverlapRadius;
	TArray<EObjectTypeQuery> ObjectTypes;
	class UClass* ActorClassFilter;
	TArray<class UClass*> PreviouslyOverlappingActors;
	bool bIgnoreDistanceCheck;
	bool bOnlyHighlightOwningActor;
};

struct FHomebaseBannerColor
{
	struct LinearColor PrimaryColor;
	struct LinearColor SecondaryColor;
};

struct FWorkerPortraitData
{
	struct TSoftClassPtr<UObject> Portrait;
};

struct FWorkerGenderData
{
	EFortCustomGender Gender;
	TArray<struct WorkerPortraitData> PotraitData;
};

struct FWorkerPersonalityData
{
	struct GameplayTag PersonalityTypeTag;
	struct FText PersonalityName;
	int SelectionWeight;
	TArray<struct WorkerGenderData> GenderData;
};

struct FWorkerSetBonusData
{
	struct GameplayTag SetBonusTypeTag;
	struct FText DisplayName;
	int RequiredWorkersCount;
	class UClass* SetBonusEffect;
	int SelectionWeight;
	int PowerPoints;
};

struct FManagerSynergyData
{
	struct GameplayTag SynergyTypeTag;
	TArray<struct WorkerGenderData> GenderData;
};

struct FHomebaseSquadSlotId
{
	struct FName SquadId;
	int SquadSlotIndex;
};

struct FHomebaseNodeLevel
{
	struct FName DisplayDataId;
	int MinCommanderLevel;
	TArray<struct FortItemQuantityPair> Cost;
	TArray<struct FName> GameplayEffectRowNames;
	struct TSoftClassPtr<UObject> AbilityKit;
	TArray<struct HomebaseSquadSlotId> UnlockedSquadSlots;
};

struct FHookGunRopeRepData
{
	Unknown Weapon;
	Unknown Projectile;
};

struct FFortHUDTagPromptData
{
	struct GameplayTag VolumeTag;
	struct FText Text_PromptTitle;
	struct FText Text_PromptDescription;
	bool bIsEnterData;
	class UClass* WidgetClass;
};

struct FInputEventBinding
{
	struct FName ActionName;
	EInputEvent InputEvent;
};

struct FScreenLabelText
{
	struct FText NormalText;
	struct FText RichText;
};

struct FFortInstensityCurveSequenceProgression
{
	class UClass* CurveSequence;
	struct CurveTableRowHandle SelectionWeight;
};

struct FInteractionType
{
	ETInteractionType InteractionType;
	EInteractionBeingAttempted InteractionBeingAttempted;
	Unknown RequestingPawn;
	Unknown RequestingPlayerController;
	Unknown InteractComponent;
	Unknown OptionalObjectData;
	struct Vector InteractPoint;
};

struct FInteriorAudioBuildingRotationConstraint
{
	EInteriorAudioBuildingDefaultRotation StartingOrientation;
	float XScaleFlipRotation;
	float YScaleFlipRotation;
	int Quadrant;
	struct Vector DotProductComparisons;
};

struct FInteriorAudioDictionaryEntry
{
	struct FString EditCode;
	int Tags;
	EInteriorAudioBuildingEvaluation DefaultEvaluation;
	EInteriorAudioBuildingEvaluation ConditionalEvaluation;
	struct InteriorAudioBuildingRotationConstraint RotationConstraint;
	int BuildingWeights;
};

struct FInteriorAudioDictionary
{
	TArray<struct InteriorAudioDictionaryEntry> Entries;
};

struct FInteriorAudioRoomSizeInfo
{
	float SizeRequired;
	struct GameplayTag SizeTag;
};

struct FInteriorAudioBuildingInfo
{
	class UClass* Actor;
};

struct FInteriorAudioDirectionScanInfo
{
	struct InteriorAudioBuildingInfo Building;
	class UClass* SourceBusComponent;
	class UClass* SourceBusActor;
};

struct FInteriorAudioPlayerInfo
{
	EInteriorAudioState CurrentState;
	struct GameplayTag CurrentRoomSizeTag;
	class UClass* CurrentAmbientBank;
	class UClass* PreviousAmbientBank;
	Unknown Directions;
	Unknown StartingWalls;
	class UClass* CenterCellActor;
	EInteriorAudioQuadrant Quadrant;
};

struct FItemCategoryMappingData
{
	EFortItemType CategoryType;
	struct FText CategoryName;
};

struct FItemCategory
{
	struct GameplayTagContainer TagContainer;
	struct FText CategoryName;
	struct FortMultiSizeBrush CategoryBrush;
};

struct FFortItemEntryAbilityData
{
	Unknown AbilityCooldownMap;
};

struct FFortItemEntryTaggedIntegerData
{
	Unknown TaggedIntegerMap;
};

struct FFortItemEntryTaggedFloatData
{
	Unknown TaggedFloatMap;
};

struct FFortMeleeDeflectTransitionAnimData
{
	class UClass* TransitionAnim;
	int NextDeflectDataIndex;
};

struct FFortMeleeDeflectAnimData
{
	class UClass* EntryFromGuardAnim;
	class UClass* HoldAnim;
	class UClass* ExitToGuardAnim;
	TArray<struct FortMeleeDeflectTransitionAnimData> TransitionsToNextDeflection;
};

struct FFortColorPalette
{
	struct LinearColor Color1;
	struct LinearColor Color2;
	struct LinearColor Color3;
	struct LinearColor Color4;
	struct LinearColor Color5;
};

struct FEventItemNamedWeight
{
	struct FName WeightName;
	struct ScalableFloat WeightValue;
};

struct FEventHotfixForPlaylistData
{
	struct GameplayTagContainer PlaylistTags;
	TArray<struct FString> TableHotfixSyntax;
};

struct FItemsForEventData
{
	struct FString RequiredEventName;
	struct EventItemNamedWeight EventItemNamedWeight;
	TArray<struct PrimaryAssetId> ItemsToLoad;
	TArray<struct FString> GlobalTableHotfixSyntax;
	TArray<struct EventHotfixForPlaylistData> PlaylistTableHotfixSyntax;
};

struct FFortVariantData
{
	struct FText VariantName;
	TArray<struct TSoftClassPtr<UObject>> OverrideMaterials;
	struct ScalableFloat Weight;
	struct GameplayTag ItemFilterTag;
	struct GameplayTag CollectionTag;
	struct GameplayTag AnalyticsTag;
	struct GameplayTagContainer POITags;
	struct GameplayTagContainer TODTags;
	struct GameplayTagContainer RequiredTags;
};

struct FItemDefToItemVariantDataMapping
{
	struct GameplayTagContainer ItemDefinitionTags;
	class UClass* ItemVariantData;
	struct GameplayTag ItemVariantTag;
};

struct FItemWrapPreviewEntry
{
	struct TSoftClassPtr<UObject> PreviewObject;
	struct TSoftClassPtr<UObject> PreviewObjectAnimation;
	struct TSoftClassPtr<UObject> PreviewObjectAnimInstance;
	struct Transform PreviewTransform;
	int WrapSectionMask;
	bool bPreviewUsingVehicleShader;
	struct TSoftClassPtr<UObject> PreviewObjectFiringAnimation;
	struct TSoftClassPtr<UObject> PreviewObjectMuzzleFlashParticleSystem;
};

struct FItemWrapSlotMetadata
{
	struct GameplayTagQuery MatchCriteria;
	TArray<struct ItemWrapPreviewEntry> PreviewList;
	EFortItemType PreviewListItemDefinitionType;
	struct TSoftClassPtr<UObject> SlotImage;
	int PreviewListSortOrder;
};

struct FFortMaterialsPriorToOverride
{
	TArray<class UClass*> OwnedMaterials_AssumeExternalOrder;
};

struct FServerLaunchInfo
{
	float LaunchServerTime;
	class UClass* LaunchedPawn;
};

struct FLeaderboardRowData
{
	int Rank;
	struct FString User;
	int Value;
	struct UniqueNetIdRepl PlatformAccountId;
};

struct FFortSourceWorldAndOverlayWorld
{
	bool bSpecifySourceWorld;
	struct TSoftClassPtr<UObject> SourceWorld;
	struct TSoftClassPtr<UObject> OverlayWorld;
	bool bServerOnly;
};

struct FFortLevelStreamingData
{
	struct FName PackageName;
	struct FName UniquePackageName;
};

struct FLightOverrideSettings
{
	EFLightOverrideLevel OverrideLevel;
	float Intensity;
	float AttenuationRadius;
	bool CastsShadows;
	bool UseInverseSquaredFalloff;
	float LightFalloffExponent;
	float ShadowResolutionScale;
	float ShadowBias;
};

struct FNamePlateFilter
{
	bool bIsSet;
	Unknown NamePlatesToShow;
};

struct FSavedSpectatorCameraState
{
	bool bDataIsValid;
	ESpectatorCameraType CameraType;
	struct UniqueNetIdRepl FollowedPlayerUniqueId;
	struct CameraFilmbackSettings FilmbackSettings;
	struct CameraLensSettings LensSettings;
	struct CameraFocusSettings FocusSettings;
	float CurrentFocalLength;
	float CurrentAperture;
	bool bAutoFocus;
	float CurrentFocusDistance;
	bool bAutoExposure;
	float ManualExposureBias;
	struct Transform Transform;
	struct Rotator ControlRotation;
	float ThirdPersonNormalizedDistance;
	EThirdPersonAutoFollowMode ThirdPersonAutoFollowMode;
	float DroneSpeedIndex;
	float ReplayPlaybackSpeed;
	bool bNamePlatesEnabled;
	bool bPlayerOutlinesEnabled;
	bool bThirdPersonCamCollide;
	bool bFollowDroneDoTest;
	bool bBattleMapIsOnTimelineMode;
	struct NamePlateFilter NamePlatesFilter;
};

struct FSavedPlayerSpectatorCameraData
{
	Unknown PlayerToCameraStateMap;
};

struct FNotificationUISettings
{
	float DisplayTime;
	bool bShouldOverrideVisibilitySettings;
};

struct FSavedSpectatorCameraShot
{
	struct SavedSpectatorCameraState CameraState;
	bool bIsShotStart;
	float Timestamp;
	float ShotLength;
	struct FString Message;
	struct FText LocalisedMessage;
	ECameraShotNotificationTypes NotificationType;
	struct NotificationUISettings MessageUISettings;
};

struct FSavedSpectatorCameras
{
	TArray<struct SavedSpectatorCameraShot> Shots;
};

struct FRemoteViewRotSnapshotManager
{
	int BufferSize;
	float TimeDelay;
	bool bUseVariableTimeDelay;
	float VariableTimeDelayMultiplier;
	float TimeBeforeDormant;
	float InterpSpeedWhenNoSample;
};

struct FFortLogoLoadingScreen
{
	struct LinearColor LogoColor;
	struct Margin LogoPadding;
	float LogoSize;
};

struct FFortLoadingBlockScreen
{
	struct ProgressBarStyle LoadingProgressBar;
	struct Margin LoadingProgressBarPadding;
	struct TextBlockStyle LoadingTextStyle;
	struct Margin LoadingTextPadding;
};

struct FFortSubGameLoadingScreen
{
	TArray<class UClass*> Tips;
	struct TSoftClassPtr<UObject> BackgroundImage;
	struct Vector2D BackgroundDesiredSize;
	struct LinearColor BackgroundColor;
	struct FortLogoLoadingScreen LogoLoadingScreen;
	struct FortLoadingBlockScreen LoadingBlockScreen;
	struct Margin SafeAreaMargin;
};

struct FFortEarlyAcessLoadingScreen
{
	struct SlateBrush EABackground;
	struct TextBlockStyle EATextStyle;
};

struct FFortPSALoadingScreen
{
	int PercentChance;
	int MinimumGames;
	struct TSoftClassPtr<UObject> WidgetClass;
};

struct FReplicatedTranslatedStrings
{
	struct FString ID;
	struct FString Name;
	TArray<struct LocalizedStringPair> Translations;
};

struct FReplicatedLocalizedDocument
{
	struct FString ID;
	struct FString SourceLang;
	TArray<struct ReplicatedTranslatedStrings> Strings;
};

struct FProcessedTranslations
{
};

struct FFortPartyMatchmakingInfo
{
	int BuildId;
	int HotfixVersion;
	struct FString RegionId;
	struct FName PlaylistName;
	int PlaylistRevision;
	struct FString TournamentId;
	struct FString EventWindowId;
	struct FString LinkCode;
};

struct FFortCreativeDiscoverySurfaceRevision
{
	struct FName SurfaceName;
	int Revision;
};

struct FFortPartySquadAssignment
{
	struct UniqueNetIdRepl MemberId;
	int AbsoluteMemberIdx;
};

struct FFortPartySquad
{
	TArray<Unknown> SquadMembers;
};

struct FPartyMemberCampaignHero
{
	struct FString HeroItemInstanceId;
	struct TSoftClassPtr<UObject> HeroType;
};

struct FPartyMemberCampaignInfo
{
	int MatchmakingLevel;
	struct FString ZoneInstanceId;
	int64_t HomeBaseVersion;
};

struct FPartyMemberFrontendEmote
{
	struct TSoftClassPtr<UObject> EmoteItemDef;
	struct FString EmoteEKey;
	int8_t EmoteSection;
};

struct FPartyMemberLobbyState
{
	EFortPartyMemberReadyCheckStatus InGameReadyCheckStatus;
	EGameReadiness GameReadiness;
	ECommonInputType ReadyInputType;
	ECommonInputType CurrentInputType;
	int HiddenMatchmakingDelayMax;
	bool HasPreloadedAthena;
};

struct FPartyMemberAssistedChallengeInfo
{
	class UClass* QuestItemDef;
	int ObjectivesCompleted;
};

struct FPartyMemberSquadAssignmentRequest
{
	int StartingAbsoluteIdx;
	int TargetAbsoluteIdx;
	struct UniqueNetIdRepl SwapTargetMemberId;
	byte Version;
};

struct FPartyMemberFrontEndMapMarker
{
	struct Vector2D MarkerLocation;
	bool bIsSet;
};

struct FPartyMemberScratchEntry
{
	int T;
	int V;
};

struct FPartyMemberCosmeticLoadout
{
	struct TSoftClassPtr<UObject> CharacterDef;
	struct FString CharacterEKey;
	struct TSoftClassPtr<UObject> BackpackDef;
	struct FString BackpackEKey;
	struct TSoftClassPtr<UObject> PickaxeDef;
	struct FString PickaxeEKey;
	struct TSoftClassPtr<UObject> ContrailDef;
	struct FString ContrailEKey;
	TArray<struct McpVariantChannelInfo> CosmeticVariants;
	TArray<struct PartyMemberScratchEntry> Scratchpad;
};

struct FPartyVariantRep
{
	struct FString C;
	struct FString V;
	int dE;
};

struct FNestedPartyVariantRep
{
	TArray<struct PartyVariantRep> I;
};

struct FPartyMemberVariantCache
{
	Unknown vL;
	bool fT;
};

struct FPartyMemberAthenaBannerInfo
{
	struct FString BannerIconId;
	struct FString BannerColorId;
	int SeasonLevel;
};

struct FPartyMemberBattlePassInfo
{
	bool bHasPurchasedPass;
	int PassLevel;
	int SelfBoostXp;
	int FriendBoostXp;
};

struct FFortLootGroupWeightData
{
	struct ScalableFloat Weight;
	struct GameplayTag LootRowRequiredTag;
};

struct FFortMangSentryInfo
{
	class UClass* Sentry;
	EAlertLevel CurrentSentryAlertLevel;
};

struct FMatchmakingParams
{
	int ControllerId;
	int PartySize;
	struct FString DatacenterId;
	int PlaylistId;
	int MatchmakingLevel;
	int MissionDifficultyMin;
	int MissionDifficultyMax;
	EStormShieldDefense StormShieldDefenseType;
	struct FString TheaterId;
	struct FString ZoneInstanceId;
	struct FString WUID;
	struct UniqueNetIdRepl WorldOwnerId;
	struct FString SessionId;
	EMatchmakingStartLocation StartWith;
	EMatchmakingFlags Flags;
	float ChanceToHostOverride;
	float ChanceToHostIncrease;
	int NumAttempts;
	int MaxSearchResultsOverride;
	int MaxProcessedSearchResults;
};

struct FFortCachedMatchmakingSearchParams
{
	EFortMatchmakingType MatchmakingType;
	struct MatchmakingParams MatchmakingParams;
	bool bValid;
};

struct FFortMatchmakingConfig
{
	float ChanceToHostOverride;
	float ChanceToHostIncrease;
	int MaxSearchResultsOverride;
	int MaxProcessedSearchResults;
};

struct FFortInviteSessionParams
{
	EMatchmakingState State;
	struct FText FailureReason;
	EPartyReservationResult LastBeaconResponse;
};

struct FMMSAltDomainRecord
{
	struct FString OriginalDomain;
	struct FString AltDomain;
};

struct FMaterialReservation
{
	class UClass* MaterialInstance;
};

struct FFortMcpCollectedItemProperties
{
	struct FString VariantTag;
	TArray<struct FString> ContextTags;
	struct JsonObjectWrapper Properties;
	EFortCollectedState SeenState;
	int Count;
};

struct FMcpPrivacySettings
{
	bool OptOutOfPublicLeaderboards;
	bool OptOutOfFriendsLeaderboards;
};

struct FFriendCode
{
	struct FString Code;
	struct FString CodeType;
};

struct FFortSubgameClientSettings
{
	TArray<struct FString> PinnedQuestInstances;
};

struct FFortWorldProfileUpdateRequest
{
	TArray<class UClass*> WorldProfilesToSave;
	int NumberOfRequests;
};

struct FQuickBarSlot
{
	TArray<struct Guid> Items;
	bool bEnabled;
	bool bIsDirty;
	bool bIsReserved;
	bool bIsOccupied;
	int UsedBySlotIndex;
	struct Guid UsedByItemGuid;
};

struct FQuickBar
{
	int CurrentFocusedSlot;
	int PreviousFocusedSlot;
	int SecondaryFocusedSlot;
	TArray<struct QuickBarSlot> Slots;
	struct QuickBarData DataDefinition;
	Unknown EquippedItemDefinitions;
	TArray<int> SharedVisibleSlotIndicesWhenUsingGamepad;
};

struct FFortWorldPlayerLoadout
{
	bool bPlayerIsNew;
	struct QuickBar PrimaryQuickBarRecord;
	struct QuickBar SecondaryQuickBarRecord;
	int ZonesCompleted;
};

struct FEventDependentTag
{
	struct FString RequiredEventFlag;
	struct GameplayTag RelatedTag;
};

struct FFortRequirementsInfo
{
	int CommanderLevel;
	int PersonalPowerRating;
	int MaxPersonalPowerRating;
	int PartyPowerRating;
	int MaxPartyPowerRating;
	TArray<class UClass*> ActiveQuestDefinitions;
	class UClass* QuestDefinition;
	struct DataTableRowHandle ObjectiveStatHandle;
	class UClass* UncompletedQuestDefinition;
	class UClass* ItemDefinition;
	struct FString EventFlag;
};

struct FFortTheaterColorInfo
{
	bool bUseDifficultyToDetermineColor;
	struct SlateColor Color;
};

struct FFortMissionAlertRuntimeData
{
	struct FName MissionAlertCategoryName;
	bool bRespectTileRequirements;
	bool bAllowQuickplay;
};

struct FFortTheaterGameplayModifier
{
	struct FString EventFlagName;
	class UClass* GameplayModifier;
};

struct FFortTheaterRuntimeData
{
	EFortTheaterType TheaterType;
	struct GameplayTagContainer TheaterTags;
	TArray<struct EventDependentTag> EventDependentTheaterTags;
	struct FortRequirementsInfo TheaterVisibilityRequirements;
	struct FortRequirementsInfo Requirements;
	ESubGame RequiredSubGameForVisibility;
	bool bOnlyMatchLinkedQuestsToTiles;
	class UClass* WorldMapPinClass;
	class UClass* TheaterImage;
	struct FortMultiSizeBrush TheaterImages;
	struct FortTheaterColorInfo TheaterColorInfo;
	struct FName Socket;
	struct FortRequirementsInfo MissionAlertRequirements;
	TArray<struct FortMissionAlertRuntimeData> MissionAlertCategoryRequirements;
	TArray<struct FortTheaterGameplayModifier> GameplayModifierList;
	float HighestDifficulty;
};

struct FFortLinkedQuest
{
	class UClass* QuestDefinition;
	struct DataTableRowHandle ObjectiveStatHandle;
};

struct FFortTheaterMissionWeight
{
	struct TSoftClassPtr<UObject> MissionGenerator;
	float Weight;
};

struct FFortTheaterDifficultyWeight
{
	struct DataTableRowHandle DifficultyInfo;
	struct FString LootTierGroup;
	float Weight;
};

struct FFortTheaterMapTileData
{
	EFortTheaterMapTileType TileType;
	struct TSoftClassPtr<UObject> ZoneTheme;
	struct FortRequirementsInfo Requirements;
	TArray<struct FortLinkedQuest> LinkedQuests;
	int XCoordinate;
	int YCoordinate;
	TArray<struct FortTheaterMissionWeight> MissionWeightOverrides;
	TArray<struct FortTheaterDifficultyWeight> DifficultyWeightOverrides;
	bool CanBeMissionAlert;
	bool bDisallowQuickplay;
	struct GameplayTagContainer TileTags;
};

struct FFortTheaterMapMissionData
{
	TArray<struct FortTheaterMissionWeight> MissionWeights;
	TArray<struct FortTheaterDifficultyWeight> DifficultyWeights;
	int NumMissionsAvailable;
	int NumMissionsToChange;
	float MissionChangeFrequency;
};

struct FFortMissionAlertRequirementsInfo
{
	struct FName CategoryName;
	struct FortRequirementsInfo Requirements;
};

struct FFortTheaterMapRegionData
{
	struct FText DisplayName;
	struct FName UniqueId;
	struct GameplayTagContainer RegionTags;
	TArray<int> TileIndices;
	struct TSoftClassPtr<UObject> RegionThemeIcon;
	struct FortTheaterMapMissionData MissionData;
	struct FortRequirementsInfo Requirements;
	TArray<struct FortMissionAlertRequirementsInfo> MissionAlertRequirements;
};

struct FFortTheaterMapData
{
	struct FText DisplayName;
	struct FString UniqueId;
	int TheaterSlot;
	bool bIsTestTheater;
	bool bHideLikeTestTheater;
	struct FString RequiredEventFlag;
	struct FName MissionRewardNamedWeightsRowName;
	struct FText Description;
	struct FText ThreatDisplayName;
	struct FortTheaterRuntimeData RuntimeInfo;
	TArray<struct FortTheaterMapTileData> Tiles;
	TArray<struct FortTheaterMapRegionData> Regions;
};

struct FFortAvailableMissionData
{
	struct FString MissionGuid;
	struct McpLootResult MissionRewards;
	struct McpLootResult BonusMissionRewards;
	struct TSoftClassPtr<UObject> MissionGenerator;
	struct DataTableRowHandle MissionDifficultyInfo;
	int TileIndex;
	struct DateTime AvailableUntil;
	TArray<class UClass*> ItemDefinitionRefCache;
};

struct FFortAvailableTheaterMissions
{
	struct FString TheaterId;
	TArray<struct FortAvailableMissionData> AvailableMissions;
	struct DateTime NextRefresh;
};

struct FFortAvailableMissionAlertData
{
	struct FString CategoryName;
	struct FString SpreadDataName;
	struct FString MissionAlertGuid;
	int TileIndex;
	struct DateTime AvailableUntil;
	struct DateTime RefreshSpreadAt;
	struct McpLootResult MissionAlertRewards;
	struct McpLootResult MissionAlertModifiers;
	TArray<class UClass*> ItemDefinitionRefCache;
};

struct FFortAvailableMissionAlerts
{
	struct FString TheaterId;
	TArray<struct FortAvailableMissionAlertData> AvailableMissionAlerts;
	struct DateTime NextRefresh;
};

struct FFortActiveTheaterInfo
{
	TArray<struct FortTheaterMapData> Theaters;
	TArray<struct FortAvailableTheaterMissions> Missions;
	TArray<struct FortAvailableMissionAlerts> MissionAlerts;
};

struct FVehicleEffectsPontoonParamSet
{
	struct FName PontoonName;
	struct FName EffectsName;
	struct FName ImpulseParamName;
	struct FName LocationParamName;
	struct FName RotationParamName;
	struct FName WaterBodyIndex;
	struct FName WaterHeightParamName;
	struct FName DepthParamName;
	struct FName WaterPlaneLocation;
	struct FName WaterPlaneNormal;
};

struct FCardSlotMedalData
{
	class UClass* AccoladeForSlot;
	int SlotIndex;
	bool bLoadedFromMcp;
	bool bPunched;
};

struct FBuildingSupportCellIndex
{
	int X;
	int Y;
	int Z;
};

struct FMegaStormCircle
{
	struct BuildingSupportCellIndex GridCenter;
	int NumCellsFromCenter;
	int CurrentQuadrant;
	struct BuildingSupportCellIndex GridAt;
	int RadiusInGridCells;
	int XAdvanceAccumulation;
	int YAdvanceAccumulation;
	int GridRadiusSquaredX4;
	int NumPlots;
	struct Vector WorldCenter;
	float WorldRadius;
	TArray<Unknown> ActorsInMegaStorm;
	EMegaStormState MegaStormState;
};

struct FEventResponderTrackingItem
{
	struct FName EventName;
	TArray<class UClass*> EventResponders;
};

struct FNavAgentData
{
	struct FName AgentName;
	struct CurveTableRowHandle BuildingActorHealthToNavAreaStrengthHandle;
	struct CurveTableRowHandle PlayerBuiltBuildingActorHealthToNavAreaStrengthHandle;
};

struct FFortMinigameStatQuery
{
	class UClass* Stat;
	EFortMinigameStatScope Scope;
	bool bAnyMatch;
	EFortMinigameStatOperation Operation;
	int Value;
	bool bStaticCount;
};

struct FMinigameEndCondition
{
	struct FortMinigameStatQuery StatQuery;
	EMinigameTeamListType TeamListType;
	TArray<byte> TeamList;
};

struct FMinigameTeam
{
	byte TeamIndex;
	struct FString TeamName;
	int TeamColorIndex;
	int MaxInitTeamSize;
	int InitTeamSizeWeight;
	bool bHasBucketAvailable;
	byte EliminatedCount;
	byte TeamSize;
};

struct FMinigameClassSlot
{
	byte ClassSlotIndex;
	struct FString ClassName;
	TArray<struct ItemAndCount> InventoryItems;
};

struct FFortMinigameStat
{
	class UClass* Filter;
	int Count;
};

struct FFortMinigameGroupStats
{
	TArray<struct FortMinigameStat> Stats;
};

struct FFortMinigameStatCollection
{
	struct FortMinigameGroupStats GroupStats;
	TArray<struct FortMinigamePlayerStats> PlayerStats;
	TArray<struct FortMinigamePlayerBucketStats> PlayerBucketStats;
};

struct FFortMinigameTimes
{
	float SetupTime;
	float WarmupTime;
	float StartTime;
	float EndTime;
	float ResetTime;
};

struct FMinigamePlayerBucket
{
	byte TeamIdAtGameStart;
	byte TeamIdAtRoundStart;
	float DesiredTeamSizePercent;
	TArray<struct UniqueNetIdRepl> PlayerIds;
};

struct FMinigamePlayerPersistentStartPoint
{
	struct Vector Location;
	struct Quat Rotation;
	struct UniqueNetIdRepl UniqueId;
	bool bConsumed;
};

struct FPickupInstigatorData
{
	byte TeamIndex;
	ETeamAttitude TargetAttitude;
	float AccentColorParam;
	int ScoreValue;
	struct PropertyOverrideData ItemOptionData;
	class UClass* OverridePickupClass;
};

struct FMinigameItemData
{
	struct TSoftClassPtr<UObject> ItemDefinition;
	int ItemQuantity;
	struct PropertyOverrideData ItemOptionData;
	int TrackedIndex;
};

struct FMinigameScoreTemplate
{
	EMinigameScoreType ScoreType;
	int NumHighScores;
	bool bAscending;
};

struct FMinigameSpawnerSpawnParams
{
	struct TSoftClassPtr<UObject> PickupToSpawn;
	int PickupQuantity;
	int PickupInstigatorHandle;
	struct Transform SpawnTransform;
};

struct FFortMissionPlacementActorPreferredTagInfo
{
	struct DataTableRowHandle DifficultyInfo;
	struct GameplayTagContainer PlacementActorPreferredTags;
	float Difficulty;
};

struct FFortMissionFocusDisplayData
{
	struct FText CurrentFocusDisplayText;
	float CurrentFocusPercentage;
};

struct FFortGeneratedMissionOption
{
	class UClass* MissionOptionCategory;
	class UClass* MissionOption;
	float RangeLerpValue;
};

struct FFortGeneratedEncounterOption
{
	class UClass* EncounterOptionCategory;
	class UClass* EncounterOption;
	class UClass* EncounterOptionInstance;
	float RangeLerpValue;
	bool bChangedSinceLastVLog;
};

struct FFortEncounterModeSettings
{
	EFortEncounterPacingMode PacingMode;
	EFortEncounterSpawnLocationManagementMode SpawnLocationManagementMode;
	EFortEncounterSpawnLocationPlacementMode SpawnLocationMode;
	EFortEncounterUtilitiesMode UtilitiesMode;
	EFortEncounterSpawnLimitType SpawnLimitMode;
};

struct FFortGeneratedEncounterProfile
{
	float EncounterDifficultyLevel;
	TArray<struct FortGeneratedEncounterOption> EncounterOptions;
	struct FortEncounterModeSettings EncounterModeSettings;
	struct GameplayTagContainer EncounterTypeTags;
	int DifficultyOptionPointsAvailableAtGeneration;
	int MinDifficultyOptionPointsToUse;
	bool bShouldReselectOptionsPerInstance;
	int GeneratedEncounterIndex;
};

struct FFortGeneratedDifficultyOptions
{
	float GameDifficultyAtGeneration;
	int DifficultyOptionPointsAvailableAtGeneration;
	int MaxEncounterSpawnPointsAtGeneration;
	int MinDifficultyOptionPointsToUse;
	TArray<struct FortGeneratedEncounterSequence> GeneratedEncounterSequences;
	TArray<struct FortGeneratedMissionOption> MissionOptions;
	TArray<struct FortGeneratedEncounterProfile> GeneratedEncounterProfiles;
};

struct FFortMissionInstancedConfigDataBucket
{
	struct GameplayTag Tag;
	class UClass* ConfigData;
};

struct FFortMissionInstancedConfigData
{
	TArray<struct FortMissionInstancedConfigDataBucket> ConfigDataBuckets;
};

struct FFortActiveThreatPlayerData
{
	class UClass* PlayerController;
	class UClass* Encounter;
};

struct FFortReactiveQuestDialogue
{
	class UClass* Conversation;
	int PlayOnObjectiveCount;
};

struct FFortRiftSpawnerData
{
	class UClass* Rift;
	class UClass* Encounter;
	class UClass* EncounterSequence;
	TArray<class UClass*> PlayersInRange;
	float TriggerDamagePercentage;
	class UClass* KillingInstigator;
	class UClass* KillingDamageCauser;
	struct TimerHandle DeactivationTimerHandle;
	struct TimerHandle BurstFallbackTimerHandle;
	struct FortEncounterSettings OverrideSettings;
	bool bUseOverrideSettings;
};

struct FMissionVehicleSpawnSet
{
	class UClass* VehicleClass;
	int MinSpawnCount;
	int MaxSpawnCount;
	struct Vector SpawnOffset;
	class UClass* OccupiedLocationGameplayEffect;
	int OccupiedLocationGameplayEffectLevel;
	struct GameplayTagContainer OccupiedLocationAdditionalContextTags;
};

struct FFortDifficultyOptionBudget
{
	struct GameplayTagContainer BudgetTypeTags;
	struct CurveTableRowHandle DifficultyOptionPointsCurve;
};

struct FMissionGenerationInfo
{
	int NumMissionsRequired;
	int MaxMissionsAllowed;
	int NumMissionsGeneratedMatchingRequirements;
	struct GameplayTagQuery MissionTagRequirements;
};

struct FPerDifficultyMissionGenerationInfo
{
	struct DataTableRowHandle MinDifficulty;
	struct DataTableRowHandle MaxDifficulty;
	TArray<struct MissionGenerationInfo> MissionGenerationInfos;
};

struct FZoneLoadingScreenHeadingConfig
{
	class UClass* HeadingImage;
	struct FText Heading;
	struct FText HeadingDescription;
};

struct FZoneLoadingScreenConfig
{
	struct TSoftClassPtr<UObject> BackgroundImage;
	struct FText TitleDescription;
	struct FText Title;
	struct ZoneLoadingScreenHeadingConfig Headings;
	TArray<class UClass*> LoadingTips;
	class UClass* LoadingMusic;
};

struct FFortAthenaLTMConfig
{
	struct TSoftClassPtr<UObject> SplashImage;
	struct FText FrontEndDescription;
	struct FText DisabledMessage;
};

struct FFortPossibleMission
{
	struct TSoftClassPtr<UObject> MissionInfo;
	float Weight;
	int MinAlwaysGenerated;
	bool bIsPrototype;
};

struct FFortPlayerSpawnPadPlacementData
{
	class UClass* PlacementQuery;
	struct TSoftClassPtr<UObject> ActorToPlace;
	bool bSnapToGrid;
	bool bAdjustPlacementForFloors;
	struct GameplayTagContainer TagsToAddToChosenPlacementActor;
};

struct FFortObjectiveEntry
{
	struct TSoftClassPtr<UObject> ObjectiveRef;
	class UClass* ObjectiveRewardBadge;
	EFortObjectiveRequirement MissionRequirement;
	struct GameplayTagContainer ObjectiveHandle;
};

struct FFortObjectiveBlock
{
	TArray<struct FortObjectiveEntry> ObjectiveEntries;
};

struct FFortMissionPlacementFoundationItem
{
	struct GameplayTagContainer ItemIdentifyingTags;
	struct GameplayTagContainer TagsToAddToChosenPlacementActorOrFoundationActor;
	class UClass* PlacementQuery;
	struct TSoftClassPtr<UObject> BuildingFoundationToPlace;
	int NumLocationsToFind;
	bool bAdjustFoundationPlacementForFloors;
};

struct FFortMissionPlacementActorItem
{
	struct GameplayTagContainer ItemIdentifyingTags;
	struct GameplayTagContainer TagsToAddToChosenPlacementActor;
	class UClass* PlacementQuery;
	struct TSoftClassPtr<UObject> ActorToPlace;
	int NumLocationsToFind;
	bool bSpawnActorAutomatically;
	bool bShouldReserveLocations;
	bool bSnapToGrid;
	bool bAdjustPlacementForFloors;
	bool bDontCreateSpawnRiftsNearby;
};

struct FFortMissionPlacementItems
{
	struct GameplayTagContainer TagsToAddToChosenPlacementActors;
	TArray<struct FortMissionPlacementFoundationItem> AdditionalWorldFoundations;
	TArray<struct FortMissionPlacementActorItem> ActorsAndLocations;
};

struct FFortFinderProperty
{
	Unknown Property;
	struct FString Value;
};

struct FFortPlacementActorFinderEntry
{
	struct TSoftClassPtr<UObject> BuildingToSpawn;
	class UClass* BuildingClassToFind;
	TArray<class UClass*> BuildingClassesToFind;
	struct GameplayTagContainer RequiredTags;
	struct GameplayTagContainer PreferredTags;
	struct GameplayTagContainer ExlusionTags;
	TArray<struct FortFinderProperty> RequiredProperties;
	bool bIgnoreCollisionCheck;
	bool bSnapToGrid;
};

struct FFortPlacementActorFinderInfo
{
	float DistanceRangeMin;
	float DistanceRangeMax;
	TArray<struct FortPlacementActorFinderEntry> BuildingData;
};

struct FFortMissionPopupWidgetData
{
	struct FText DisplayName;
	bool bShowDescription;
	struct FText Description;
	struct SlateBrush DescriptionIcon;
	struct SlateBrush AvailableIcon;
	struct SlateBrush UnavailableIcon;
};

struct FFortEncounterProfile
{
	struct TSoftClassPtr<UObject> EncounterOptions;
	TArray<class UClass*> OverrideCategories;
	bool bShouldReselectOptionsPerInstance;
	struct GameplayTagContainer EncounterTypeTags;
};

struct FFortEncounterSequenceSettings
{
	struct FortEncounterTransitionSettings TransitionSettings;
	TArray<struct FortEncounterProfile> EncounterSequence;
	struct GameplayTagContainer SequenceTags;
};

struct FFortMissionTaggedRewards
{
	struct GameplayTag Tag;
	TArray<struct TSoftClassPtr<UObject>> WorldItemDefinitions;
};

struct FFortTimeOfDayTheme
{
	TArray<class UClass*> TimeOfDayCollections;
	TArray<struct TSoftClassPtr<UObject>> AdditionalTimeOfDayManagers;
	TArray<struct TSoftClassPtr<UObject>> ProhibitedTimeOfDayManagers;
	TArray<struct TSoftClassPtr<UObject>> ValidTimeOfDayManagers;
};

struct FFortWindIntensityAndDirection
{
	float WindIntensity;
	float WindHeading;
};

struct FFortGlobalWindInfo
{
	TArray<struct FortWindIntensityAndDirection> ValidWindInfos;
	int WindIndex;
};

struct FMissionPerDifficultyProperties
{
	TArray<struct DataTableRowHandle> ValidDifficulties;
	struct FortTimeOfDayTheme OverrideTimeOfDayTheme;
	struct FortGlobalWindInfo OverrideGlobalWindInfo;
};

struct FFortMissionEventReceiverByGameplayTagQuery
{
	struct GameplayTagQuery TagQueryEventDiscriminator;
	class UClass* DelegateHolder;
};

struct FFortMissionUIActorHandle
{
	Unknown AttachedActor;
	struct Vector AttachmentOffset;
	float MaxVisibleDistance;
	struct Guid MissionGuid;
	Unknown MissionUIIndicator;
};

struct FEarnedBadgePlayerData
{
	struct UniqueNetIdRepl PlayerID;
	int Count;
};

struct FMissionTimeDisplayData
{
	float LessThanTimeValue;
	bool bHideTimer;
	struct LinearColor BaseColor;
	struct LinearColor PulseColor;
	float ColorPulsesPerSecond;
};

struct FMissionTimerData
{
	bool bTimerIsPaused;
	float OriginalTimePeriod;
	float ReplicatedRemainingTime;
	float TimeAddedOrRemoved;
	float LastTimeAddedOrRemoved;
	float ClientRemainingTime;
};

struct FFortMissionWeightedReward
{
	struct FName LootTierGroup;
	struct SlateBrush LootIcon;
	float Weight;
};

struct FFortMobileHUDContextLayoutSave
{
	struct GameplayTag ContextLayoutTag;
	TArray<struct FortMobileHUDWidgetSchemaSave> HUDWidgetSchemas;
};

struct FFortMobileHUDLayoutProfileSave
{
	struct FText ProfileName;
	TArray<struct FortMobileHUDContextLayoutSave> ContextLayouts;
};

struct FFortMobileHUDPresetExtension
{
	struct GameplayTag HUDPresetToExtend;
	struct FortMobileHUDLayoutProfileSave HUDPresetExtensionSave;
};

struct FMontageLookupData
{
	struct GameplayTagContainer GameplayTags;
	struct TSoftClassPtr<UObject> AnimMontage;
};

struct FTurretCosmeticData
{
	class UClass* OverheatSmoke;
	class UClass* OverheatScreenL;
	class UClass* OverheatScreenR;
	TArray<class UClass*> OverheatParticleSystems;
	class UClass* OverheatAudio;
	class UClass* OnOverheatAudio;
	class UClass* OverheatAudioCurve;
	class UClass* ServoAudio;
	class UClass* AimingWeaponComponent;
	struct FName OverheatParamName;
	float OverheatParamOnOverheatValue;
	struct FName OverheatColorParamName;
	struct FName OverheatAlphaParamName;
	struct FName RedBlinkParamName;
	float RotatationAudioRangeInA;
	float RotatationAudioRangeInB;
	float RotatationAudioRangeOutA;
	float RotatationAudioRangeOutB;
	float HotThreshold;
	float WarmThreshold;
	float AudioFadeinDuration;
	float AudioFadeoutDuration;
	struct LinearColor WarmColor;
	struct LinearColor HotColor;
	float OverheatColorExponent;
	float ReplicateOverheatMax;
};

struct FAirControlParams
{
	struct ScalableFloat MaxAcceleration;
	struct ScalableFloat LateralFriction;
	struct ScalableFloat MaxLateralSpeed;
	struct ScalableFloat TerminalVelocity;
	struct ScalableFloat UpwardTerminalVelocity;
	struct ScalableFloat GravityScalar;
	struct ScalableFloat CustomGravityCeiling;
	struct ScalableFloat CustomGravityCeilingWidth;
};

struct FGoopControlParams
{
	struct ScalableFloat DragHorizontalPercent;
	struct ScalableFloat DragVerticalPercent;
	struct ScalableFloat DragHorizontalFlat;
	struct ScalableFloat DragVerticalFlat;
};

struct FWaterControlParams
{
	struct ScalableFloat MinImmersionDepth;
	struct ScalableFloat TargetImmersionDepth;
	struct ScalableFloat MaxImmersionDepth;
	struct ScalableFloat CancelCrouchImmersionDepth;
	struct ScalableFloat DBNOSwimImmersionDepth;
	struct ScalableFloat MaxAcceleration;
	struct ScalableFloat MaxAccelerationSprinting;
	struct ScalableFloat BrakingDeceleration;
	struct ScalableFloat MaxNormalSpeed;
	struct ScalableFloat MaxSprintSpeed;
	struct ScalableFloat MinSprintJumpSpeed;
	struct ScalableFloat SprintJumpAirAccelAngleLimit;
	struct ScalableFloat SprintRetriggerDelay;
	struct ScalableFloat SprintDelayAfterFiring;
	struct ScalableFloat MaxTargetingSpeed;
	struct ScalableFloat BackwardsSpeedMultiplier;
	struct ScalableFloat BackwardsSpeedCosAngle;
	struct ScalableFloat AngledSpeedMultiplier;
	struct ScalableFloat AngledSpeedCosAngle;
	struct ScalableFloat Friction;
	struct ScalableFloat FrictionSprinting;
	struct ScalableFloat FrictionDirectionChangeDot;
	struct ScalableFloat FrictionDirectionChangeMultiplier;
	struct ScalableFloat MaxSpeedUp;
	struct ScalableFloat MaxSpeedDown;
	struct ScalableFloat MaxHorizontalEntrySpeed;
	struct ScalableFloat WaterForceMultiplier;
	struct ScalableFloat WaterForceSecondMultiplier;
	struct ScalableFloat MaxWaterForce;
	struct ScalableFloat WaterVelocityDepthForMax;
	struct ScalableFloat WaterVelocityMinMultiplier;
	struct ScalableFloat WaterSimMaxTimeStep;
	struct ScalableFloat WaterSimSubStepTime;
	struct ScalableFloat BobbingMaxForce;
	struct ScalableFloat BobbingIdealDepthTolerance;
	struct ScalableFloat BobbingFrictionDown;
	struct ScalableFloat BobbingExpDragDown;
	struct ScalableFloat BobbingFrictionDownSubmerged;
	struct ScalableFloat BobbingExpDragDownSubmerged;
	struct ScalableFloat BobbingFrictionUp;
	struct ScalableFloat BobbingExpDragUp;
	struct ScalableFloat BoostDragMultiplier;
};

struct FWaterJumpParams
{
	struct ScalableFloat WaterJumpMinVelocityZ;
	struct ScalableFloat WaterJumpMaxVelocityZ;
	struct ScalableFloat WaterJumpForceZ;
	struct ScalableFloat JumpForceDuration;
};

struct FWaterSprintBoostParams
{
	struct ScalableFloat RequiredDepth;
	struct ScalableFloat TriggerAboveDepth;
	struct ScalableFloat TriggerAtVelocity;
	struct ScalableFloat PreventAtVelocity;
	struct ScalableFloat Duration;
	struct ScalableFloat MaxSpeed;
};

struct FGroundSplineLocationData
{
	float DistanceAlongSpline;
	class UClass* SplineComponent;
	int TeleportRequestNum;
	float Timestamp;
};

struct FGroundSplineSpeedData
{
	float Speed;
	float Acceleration;
	float YawSpeed;
	float YawAcceleration;
	int SnapToNewSpeedRequestNum;
	float Timestamp;
};

struct FFortMtxPlatformList
{
	TArray<EFortMtxPlatform> MtxPlatformList;
};

struct FFortMusicSection
{
	class UClass* Sound;
	float FadeInTime;
	float FadeOutTime;
	float InitialOffset;
	float Duration;
};

struct FObjectiveRequirement
{
	struct FName ObjectiveBackendName;
	bool bCompleted;
};

struct FQuestRequirement
{
	class UClass* QuestDef;
	EFortQuestState DesiredState;
	TArray<struct ObjectiveRequirement> Objectives;
};

struct FItemRequirement
{
	class UClass* ItemDef;
	bool bMustOwnItem;
};

struct FCalendarRequirement
{
	struct FName CalendarEventName;
	bool bActive;
	int DaysFromEeventStart;
};

struct FProfileStatRequirement
{
	struct FString ProfileStat;
};

struct FFortNavLinkPattern
{
	int PatternBits;
	int WildcardBits;
};

struct FRestrictedCountry
{
	bool bHealthWarningShown;
	bool bAntiAddictionMessageShown;
	bool bRealMoneyStoreRestriction;
	bool bGameplayRestrictions;
};

struct FFortCachedFloatCurve
{
	struct FName CurveName;
};

struct FFortAnimInput_TurnCorrection
{
	struct FortCachedFloatCurve RotationCurve;
	float YawCorrectionOffset;
	float MaxYawCorrectionOffset;
	float CharacterMeshYawOffset;
	float CurveMultiplier;
	float CharacterInitialWorldYaw;
	float CharacterWorldYawLastFrame;
	float AccumulatedAnimationYaw;
	float TotalYawFromCurve;
	float RotationCurveYawLastFrame;
	bool bEnableDebug;
	bool bIsTurnFinished;
};

struct FFortDirectionalAnimRef
{
	class UClass* NorthAnim;
	class UClass* SouthAnimLeft;
	class UClass* SouthAnimRight;
	class UClass* EastAnim;
	class UClass* WestAnim;
};

struct FOutpostBuildingData
{
	struct TSoftClassPtr<UObject> ItemDefinition;
};

struct FOutpostUpgradesPerTheaterData
{
	int TheaterSlot;
	class UClass* OutpostUpgradesData;
};

struct FOutpostPrestigeEffects
{
	TArray<class UClass*> EnemyPrestigeEffect;
};

struct FOutpostPrestigeEffectsPerTheater
{
	int TheaterSlot;
	struct OutpostPrestigeEffects PrestigeEffects;
};

struct FOutpostPOSTBoost
{
	struct CurveTableRowHandle PlayerStructureHealthModPerPOSTLevel;
};

struct FOutpostPOSTPerTheaterData
{
	int TheaterSlot;
	struct OutpostPOSTBoost POSTData;
};

struct FEmptyServerReservation
{
	int PlaylistId;
	struct FString ZoneInstanceId;
	struct FString WUID;
	struct UniqueNetIdRepl WorldDataOwner;
	bool bMakePrivate;
	EFortMatchmakingPool MatchmakingPool;
	bool bUsesMatchmakingV2;
};

struct FPlayerSquadMapping
{
	struct FName PlaylistName;
	struct FString PlayerID;
	struct FString CustomMatchKey;
	uint32_t TeamId;
	uint32_t PartyId;
	int SeatId;
};

struct FFortHomeBaseInfo
{
	struct FString BannerIconId;
	struct FString BannerColorId;
	struct FText Name;
	bool ValidData;
};

struct FFortTeamMemberInfo
{
	struct UniqueNetIdRepl MemberUniqueId;
	struct UniqueNetIdRepl PartyLeaderUniqueId;
	struct UniqueNetIdRepl ConsoleUniqueId;
	struct FString Platform;
	struct FText PlayerName;
	bool bPartyLeader;
	bool bIsInZone;
	bool bHasBoostXp;
	bool bHasRestXp;
	struct FString BannerIconId;
	struct FString BannerColorId;
	bool bBattlePassPurchased;
	int BattlePassLevel;
	int BattlePassSelfXpBoost;
	int BattlePassFriendXpBoost;
	int NumPlayersInParty;
	int PlayerIndex;
	byte TeamAffiliation;
	struct FText HeroClass;
	struct FText HeroLevel;
	int HeroXP;
	class UClass* HeroItem;
	TArray<class UClass*> SelectedGadgetItems;
	struct SlateBrush HeroIcon;
	struct FortHomeBaseInfo HomeBaseInfo;
};

struct FFortAnimInput_PatrolAnimSet
{
	class UClass* PatrolIdlePose;
	class UClass* PatrolIdleAdditive;
	class UClass* AdditiveHitReactMontage;
	class UClass* FullBodyHitReactMontage;
	class UClass* PatrolToAlertMontage;
	class UClass* AlertToCombatMontage;
	class UClass* WalkBlendSpaceCore;
	class UClass* WalkBlendSpaceAdditive;
};

struct FBuildingHitTime
{
	class UClass* HitBuilding;
};

struct FFortPassiveHealerPawnInfo
{
	class UClass* Pawn;
	bool bEmoteStarted;
};

struct FScriptedAction
{
	struct FString Template;
	TArray<struct FString> Params;
	Unknown ParamMap;
};

struct FScriptedBehavior
{
	struct FString ScriptName;
	TArray<struct ScriptedAction> Actions;
};

struct FFortBadMatchTrigger
{
	struct FName Key;
	EFortBadMatchTriggerOperation Operation;
	double Value;
	TArray<struct FName> Playlists;
	EFortBadMatchTriggerType Type;
};

struct FFortPetOffsets
{
	struct Vector DefaultIdle;
	struct Vector DefaultDownsights;
	struct Vector CrouchIdle;
	struct Vector CrouchDownsights;
	struct Vector DBNOIdle;
	struct Vector MeleePlaying;
	struct Vector EmotePlaying;
	struct Vector DefaultSkydive;
	struct Vector SurfaceSwimmingIdle;
	struct Vector SurfaceSwimmingDownsights;
	struct Vector SurfaceSwimmingMeleePlaying;
	struct Vector DiveJumping;
	struct Vector CrouchTurning;
	struct Vector DivingSkydive;
};

struct FPetResponseFromQuestSystem
{
	struct DataTableRowHandle ObjectiveStatHandle;
	struct GameplayTag ResponseTag;
	float ResponseDuration;
};

struct FPhysicsObjectMovementStateData
{
	EFortPhysicsObjectMovementState MovementState;
	EFortPhysicsObjectMovementState PreviousMovementState;
	struct HitResult LastMovingHitResult;
	float TimeMovementStateUpdated;
	float MovingStopTime;
};

struct FPhysicsObjectImpactInstigatorData
{
	class UClass* ImpactInstigator;
	float ServerTimeAssigned;
};

struct FImpulseDamageData
{
	struct ScalableFloat MinVelocityForDamage;
	struct ScalableFloat PercentageOfDamage;
};

struct FReplicatedPhysicsPawnState
{
	struct Vector_NetQuantize100 Translation;
	struct Quat Rotation;
	struct Vector_NetQuantize10 LinearVelocity;
	struct Vector_NetQuantize10 AngularVelocity;
	uint16_t SyncKey;
};

struct FTimeStampedPhysicsPawnState
{
	float WorldTime;
	struct ReplicatedPhysicsPawnState PhysicsPawnState;
};

struct FHitData
{
	float PingOfHitter;
	class UClass* HittingPawn;
	struct Vector HitNormal;
	struct TimeStampedPhysicsPawnState ObjectState;
};

struct FPhysicsPawnObjectInitialParameters
{
	struct Vector InitialVelocity;
	float OverrideMassInKG;
	float OverrideAngularDampening;
	float OverrideLinearDampening;
	float PlayerForceMultiplier;
	float PawnKnockbackMultiplier;
	bool bEnableGravity;
	bool bEnableGravityOnHit;
	bool bShouldKillPawnOnHit;
};

struct FFortPlayerAttributeSets
{
	class UClass* HealthSet;
	class UClass* ControlResistanceSet;
	class UClass* DamageSet;
	class UClass* MovementSet;
	class UClass* AdvancedMovementSet;
	class UClass* ConstructionSet;
	class UClass* PlayerAttrSet;
	class UClass* CharacterAttrSet;
	class UClass* WeaponAttrSet;
	class UClass* HomebaseSet;
};

struct FActiveFortCamera
{
	class UClass* Camera;
	class UClass* ViewTarget;
	float TransitionAlpha;
	float TransitionUpdateRate;
	float BlendWeight;
};

struct FFortCameraInstanceEntry
{
	class UClass* CameraClass;
	class UClass* ViewTarget;
	class UClass* Camera;
};

struct FFortCameraModeOverride
{
	class UClass* OriginalClass;
	class UClass* ClassOverride;
};

struct FFortCameraPrototype
{
	struct FName PrototypeName;
	struct FString PrototypeDescription;
	TArray<struct FortCameraModeOverride> ModeOverrides;
};

struct FAthenaAccolades
{
	class UClass* AccoladeDef;
	struct FString TemplateId;
	int Count;
};

struct FFortDepositedResources
{
	struct FString TemplateId;
	int Quantity;
};

struct FFortTagToGliderAnimSetPair
{
	struct GameplayTag MatchingTag;
	struct FortAnimInput_PlayerGliderAnimAsset OverrideAnimAsset;
};

struct FFortInputActionGroupContext
{
	struct FName ActionName;
	EFortInputActionGroup InputActionGroup;
};

struct FFortInputActionKeyAlias
{
	struct FName ActionName;
	struct Key KeyAlias;
	EFortInputActionType InputActionType;
};

struct FFortInputActionThatShouldAlwaysBeBound
{
	struct FName ActionName;
	TArray<struct FName> AlternateActionNames;
};

struct FMorphValuePair
{
	struct FName MorphName;
	float MorphValue;
};

struct FPlayerPerkLevel
{
	TArray<class UClass*> PlayerPerks;
};

struct FTechCurrentLevelCap
{
	struct FString CalendarEvent;
	struct FString EventFlag;
	int LevelCap;
};

struct FPetStimuliRepData
{
	struct GameplayTag Stimuli;
	float GameTimeEnd;
};

struct FTeamRoles
{
	byte PusherTeam;
	byte DefenderTeam;
};

struct FPlayerBannerInfo
{
	struct FString IconId;
	struct FString ColorId;
	int Level;
};

struct FCustomCharacterData
{
	byte WasPartReplicatedFlags;
	int RequiredVariantPartFlags;
	class UClass* Parts;
	class UClass* Charms;
	TArray<class UClass*> VariantRequiredCharacterParts;
	bool bReplicationFailed;
};

struct FAccumulatedItemEntry
{
	class UClass* ItemDefinition;
	int Quantity;
};

struct FSimulatedAttributeEntry
{
	struct GameplayAttribute Attribute;
	float CurrentValue;
};

struct FFortRespawnData
{
	bool bRespawnDataAvailable;
	bool bClientIsReady;
	bool bServerIsReady;
	struct Vector RespawnLocation;
	struct Rotator RespawnRotation;
	float RespawnCameraDistance;
};

struct FMetricStateInformation
{
	struct FString Name;
	struct FString Description;
	EFortBudgetCategory Category;
	int Cost;
	int Budget;
};

struct FDetailedMetricInformation
{
	TArray<struct MetricStateInformation> MetricStates;
};

struct FSimpleMetricInformation
{
	TArray<EFortBudgetCategory> CategoryNames;
	TArray<int> CategoryValues;
	TArray<int> CategoryBudgets;
	struct Vector Location;
	float NormalizedGroupBudgetValue;
	int GroupTotalBudget;
	struct Vector NeighbourGroupCenter;
	TArray<byte> NeighbourGroupBudgetValue;
};

struct FDeathInfo
{
	class UClass* FinisherOrDowner;
	class UClass* Downer;
	bool bDBNO;
	EDeathCause DeathCause;
	byte DeathClassSlot;
	float Distance;
	struct Vector DeathLocation;
	bool bInitialized;
	struct GameplayTagContainer DeathTags;
	struct GameplayTagContainer FinisherOrDownerTags;
	struct GameplayTagContainer VictimTags;
};

struct FChangeTeamInfo
{
	class UClass* Instigator;
	struct GameplayTagContainer ChangeTeamTags;
};

struct FFortResurrectionData
{
	bool bResurrectionChipAvailable;
	float ResurrectionExpirationTime;
	float ResurrectionExpirationLength;
	struct Vector WorldLocation;
};

struct FFortPlayerSurveyAnswer
{
};

struct FFortPlayerSurveyDescriptionMessage
{
	struct FText Title;
	struct FText Message;
};

struct FFortPlayerSurveyDescription
{
	struct FText SurveyTitle;
	struct FortPlayerSurveyDescriptionMessage CancelConfirmationMessage;
	int DefaultAnswer;
};

struct FFortPlayerSurveyQuestionChoice
{
	struct FText ChoiceText;
};

struct FFortPlaylistBaseCurveTableOverride
{
	struct TSoftClassPtr<UObject> BaseTable;
	struct TSoftClassPtr<UObject> OverrideTable;
};

struct FFortPlaylistBaseDataTableOverride
{
	struct TSoftClassPtr<UObject> BaseTable;
	struct TSoftClassPtr<UObject> OverrideTable;
};

struct FRatingExpansion
{
	int Priority;
	int RatingDelta;
};

struct FAthenaScoreData
{
	EAthenaScoringEvent ScoringEvent;
	struct FText ScoreNameText;
	struct GameplayTagContainer EventInclusionTags;
	int NumOccurrencesForScore;
	int NumOccurrencesPermitted;
	struct ScalableFloat ScoreAwarded;
};

struct FWinConditionScoreData
{
	struct ScalableFloat GoalScore;
	struct ScalableFloat BigScoreThreshold;
	TArray<struct AthenaScoreData> ScoreDataList;
};

struct FSupplyDropSubPhaseModifier
{
	EAthenaGamePhase GamePhase;
	int SubPhaseIndex;
	float SpawnInPreviousZonePercentChance;
};

struct FSupplyDropModifierData
{
	struct FName SupplyDropID;
	TArray<struct SupplyDropSubPhaseModifier> ModifierList;
};

struct FCharacterPreloadBlock
{
	bool bShouldGoInNPCBudget;
	struct TSoftClassPtr<UObject> CID;
};

struct FCharacterFallbackPreloadBlock
{
	bool bShouldGoInNPCBudget;
	struct GameplayTag FallbackTag;
};

struct FFortOptionsMenuData
{
	ESettingTab OptionTabType;
	bool bDisplayOption;
	struct FText DisplayName;
	class UClass* PageWidgetClass;
	struct SlateBrush TabBrush;
};

struct FPostGameScreenTagClassPair
{
	struct GameplayTag ScreenType;
	class UClass* PostGameScreenClass;
};

struct FFortPreviewActorData
{
	struct TSoftClassPtr<UObject> ActorClass;
	struct Vector RelativeLocation;
	struct Rotator RelativeRotation;
	float InfluenceDistance;
};

struct FFortPlayspaceConfigData
{
	struct TSoftClassPtr<UObject> PlayspaceClass;
	EPlayspaceCreationType CreationType;
	bool bSpawnAtPlayspaceSpawnActors;
};

struct FEventDrivenDiscoveryID
{
	struct FString CalendarEventName;
	bool bRequireEventActive;
	int ActiveBitId;
};

struct FFortPOIAmbientAudioLoop
{
	struct TSoftClassPtr<UObject> LoopingSound;
	float CrossfadeTime;
};

struct FFortPOIAmbientAudioOneShot
{
	struct TSoftClassPtr<UObject> OneShotSound;
	struct Vector2D RetriggerTimeRange;
	struct Vector2D TriggerDistanceRange;
	TArray<EFortDayPhase> AllowedDayPhases;
};

struct FFortPoiGridInfo
{
	struct Vector2D WorldGridStart;
	struct Vector2D WorldGridEnd;
	struct Vector2D WorldGridSpacing;
	int GridCountX;
	int GridCountY;
	struct Vector2D WorldGridTotalSize;
};

struct FFortPoiVolumeGridCell
{
};

struct FPreferredItemSlotSettingData
{
	struct GameplayTag OverrideTag;
	struct GameplayTagContainer GameplayTags;
	class UClass* Icon;
};

struct FProfileGoScenario
{
	struct FString Name;
	struct Vector Position;
	struct Rotator Orientation;
	struct FString OnBegin;
	struct FString OnEnd;
	bool AutoGenerated;
	bool UseSetupCheats;
};

struct FProfileGoCollection
{
	struct FString Name;
	struct FString Scenarios;
};

struct FProfileGoCommand
{
	struct FString Group;
	struct FString Command;
	float Wait;
	struct FString Log;
	bool CopyOutputToGameLog;
};

struct FProjectileHomingData
{
	EFortHomingStyle HomingStyle;
	float MinTurnSpeed;
	float MaxTurnSpeed;
	float RampTimeFromMinToMaxTurnSpeed;
	float LockTargetDistanceThreshold;
	int RandomSeed;
	Unknown LockedOnTarget;
	struct Vector LockedOnTargetPosition;
	struct Vector TargetOffset;
	bool ResetTurnSpeedTimer;
};

struct FFortReplicatedVelocityData
{
	byte RepIncrement;
	struct Vector ReplicatedVelocity;
};

struct FFortStopSimulatingRepData
{
	byte RepIncrement;
	struct Vector StopLocation;
};

struct FProjectileMovementDrunkConfig
{
	class UClass* DrunkSpeedScaleCurve;
	class UClass* DrunkGravityScaleCurve;
	float InitialDelay;
	float Duration;
	float DirectionChangeRate;
	float TurnAngle;
	float MinPitch;
	float TurnAngleClamp;
};

struct FDrunkHomingConfig
{
	class UClass* DrunkOverrideSpeedCurve;
	float DirectionChangeRate;
	float LookaheadDist;
	float TurnAngle;
	float TurnAngleBlendOut;
	float DrunkDuration;
	float MinPitch;
	float RandomTargetPositionRadius;
	float PassedTargetSlackTime;
	float TurnAngleClamp;
	float DrunkBlendOutRange;
	float DrunkBlendOutTimeThreshold;
	float DrunkBlendOutTime;
	float DrunkBlendOutTurnSpeed;
	float AimPointMaxRange;
};

struct FPropertyOverrideRedirect
{
	struct FString OldPropertyScope;
	struct FString OldPropertyName;
	struct FString NewPropertyScope;
	struct FString NewPropertyName;
};

struct FPropertyOverridePropertyDataRedirect
{
	struct FString PropertyKnobName;
	struct FString OldPropertyData;
	struct FString NewPropertyData;
};

struct FFortPointOnCurveRange
{
	float MinPercentage;
	float MaxPercentage;
};

struct FFortPointsOnCurve
{
	struct TSoftClassPtr<UObject> Curve;
	TArray<struct FortPointOnCurveRange> RangesForPointsOnCurve;
};

struct FFortGameplayTagQueryPerDifficulty
{
	struct DataTableRowHandle DifficultyInfo;
	struct GameplayTagQuery TagQueryToMatch;
	float Difficulty;
};

struct FGoalDistanceData
{
	bool bIgnoreScreeningDistance;
	struct AIDataProviderFloatValue ScreeningTestMaxDistance;
	struct TSoftClassPtr<UObject> TestScoreCurve;
	struct AIDataProviderFloatValue CurveDistanceScale;
};

struct FCompletionCountEntry
{
};

struct FFortQuestObjectiveCompletion
{
	struct FString StatName;
	int Count;
	int TimestampOffset;
};

struct FSharedQuestData
{
	TArray<class UClass*> SharedQuests;
	class UClass* PinnedQuest;
};

struct FFortDisplayQuestUpdateData
{
	struct FortUpdatedObjectiveStat ObjectiveUpdated;
	class UClass* QuestOwner;
	class UClass* AssistingPlayer;
};

struct FFortRarityItemData
{
	struct FText Name;
	struct LinearColor Color1;
	struct LinearColor Color2;
	struct LinearColor Color3;
	struct LinearColor Color4;
	struct LinearColor Color5;
	float Radius;
	float Falloff;
	float Brightness;
	float Roughness;
	float Glow;
};

struct FFortQuestEarnedBadgeData
{
	struct FString TemplateId;
	int Count;
};

struct FEndZoneScoreAndAwards
{
	bool bResultsPendingSave;
	int TotalScore;
	bool bCriticalMatchBonus;
	bool bDidLeech;
	TArray<struct FortQuestEarnedBadgeData> EarnedBadgeData;
	TArray<struct FString> EarnedItemCaches;
	int NumMissionPoints;
	float MissionLeechScaling;
};

struct FRepeatableDailiesCardDateOverride
{
	struct TSoftClassPtr<UObject> Quest;
	struct DateTime Start;
	struct DateTime End;
};

struct FFortReplayPlaybackState
{
	float StartTime;
	float EndTime;
	float TimeNow;
	bool bIsPaused;
	float PlaybackSpeedMultiplier;
	EHudVisibilityState HUDVisibility;
	bool bLevelStreaming;
	bool bHasRelevancyZone;
};

struct FFortReplayFXState
{
	int DefaultParticleLODBias;
	int DefaultDepthOfFieldQuality;
	int OverrideParticleLODBias;
	int OverrideDepthOfFieldQuality;
};

struct FRepGraphActorSettingsBase
{
	bool bAddClassRepInfoToMap;
	bool bUseCustomClassRepInfo;
	bool bAddToExplicitCSVStatTracker;
	bool bAddToImplicitCSVStatTracker;
	bool bAddToRPC_Multicast_OpenChannelForClassMap;
	bool bRPC_Multicast_OpenChannelForClass;
	EClassRepNodeMapping ClassNodeMapping;
	struct ClassReplicationInfo ClassRepInfo;
	struct FName CSVStatNamePrefix;
};

struct FRepGraphClassTracking
{
	struct FString ClassName;
	struct FString CSVStatNamePrefix;
	bool bIncludeFastPath;
};

struct FFortAlwaysRelevantActorInfo
{
	class UClass* Connection;
	class UClass* LastPawn;
	class UClass* LastTetherPawn;
};

struct FCreativeBetaPermission
{
	struct PrimaryAssetId PrimaryAssetId;
	TArray<struct FString> PermissionTagContainer;
};

struct FCreativeIslandDescriptionTag
{
	struct FString Name;
	struct FText DisplayText;
};

struct FExperimentalCohortPercent
{
	int ExperimentNum;
	int CohortPercent;
	struct FString Name;
};

struct FRiftTourInfo
{
	int SlotId;
	struct DateTime StartsAtUTC;
};

struct FRuntimeOptionSpectateAPartyMemberOverride
{
	struct FName PropertyName;
	bool bEnabled;
};

struct FRuntimeOptionLocalizableStringEntry
{
	struct FString Culture;
	struct FString Text;
};

struct FRuntimeOptionLocalizableString
{
	TArray<struct RuntimeOptionLocalizableStringEntry> Entries;
};

struct FRuntimeOptionReviewPromptCriteria
{
	int MinutesPlayed;
	int GamesPlayed;
	int BestResult;
	int KillCount;
	bool RequireAll;
};

struct FRuntimeOptionTabStateInfo
{
	struct FName TabName;
	EFortRuntimeOptionTabState TabState;
	EFortRuntimeOptionTabStateTarget TargetPlayer;
};

struct FRuntimeOptionDisabledGameplayMessage
{
	struct FName MessageOwnerClassName;
	struct FName MessageName;
};

struct FRuntimeOptionTournamentScoreThreshold
{
	int StartingPlacement;
	int PointsIncrement;
};

struct FPlayerMarkerConfig
{
	float DoubleClickTime;
	bool bShowMarkerDetailsWidget;
	bool bCreateMarkerActors;
	bool bCreateMarkerWidgets;
	bool bClampEnemyMarkers;
	bool bClampItemMarkers;
	bool bShowLocationMarkersOnCompass;
	int LocalPlaceableMarkersPerRate;
	float LocalPlaceableMarkersRechargeRate;
	int RemotePlayableMarkerSoundsPerRate;
	int RemotePlayableMarkerSoundsRechargeRate;
	int RemotePlayableMarkerSoundsByPlayerIDPerRate;
	int RemotePlayableMarkerSoundsByPlayerIDRechargeRate;
	int RemotePlayableMarkerSoundsByPlayerIDRechargeRateCap;
	bool EnableDoubleClickAction;
	bool EnableItemMarking;
	bool EnableInteractionMarking;
	float ScreenPercentageDistanceToShowMarkerInfo;
	float EnemyMarkerTLL;
	float ItemMarkerTTL;
	float SpecialServerMarkerTTL;
	float EliminationMarkerTTL;
	float SelfEliminationMarkerTTL;
	int MaxItemMarkers;
	int MaxEnemyMarkers;
	int MaxEliminationMarkers;
	int MaxSpecialLocalMarkers;
	int MaxSpecialServerMarkers;
};

struct FFortPlaylistCuratedContent
{
	TArray<struct FString> CuratedLinkCodes;
};

struct FAthenaItemShopSectionPriority
{
	EFortItemShopSection Section;
	int Priority;
};

struct FAthenaItemShopSectionOverrideDisplayData
{
	EFortItemShopSection Section;
	bool bHideTitle;
	bool bNoSectionTab;
};

struct FRuntimeOptionScheduledNotification
{
	struct DateTime FireDateTime;
	bool LocalTime;
	struct RuntimeOptionLocalizableString Title;
	struct RuntimeOptionLocalizableString Body;
};

struct FSanitizationData
{
	class UClass* TaskQueue;
};

struct FFortScoreStylingInfo
{
	struct FText Name;
	struct FortMultiSizeBrush Icon;
	struct LinearColor Color;
};

struct FFortScriptedActionParams
{
	class UClass* Player;
	EFortScriptedActionSource SourceType;
	class UClass* SourceItem;
	struct DataTableRowHandle SourceData;
	struct FName SourceName;
};

struct FFortAvailableScriptedAction
{
	struct FortScriptedActionParams Params;
	class UClass* ActionDefaults;
};

struct FFortSearchPassParams
{
	int ControllerId;
	struct FName SessionName;
	struct FString BestDatacenterId;
	int MaxProcessedSearchResults;
};

struct FFortSearchPassState
{
	int BestSessionIdx;
	bool bWasCanceled;
	EFortSessionHelperJoinResult FailureType;
	EMatchmakingState MatchmakingState;
	EPartyReservationResult LastBeaconResponse;
};

struct FFortNonPrimaryMission
{
	struct TSoftClassPtr<UObject> MissionInfo;
	struct GameplayTagQuery ContextTagQuery;
	struct DataTableRowHandle MinDifficulty;
	struct DataTableRowHandle MaxDifficulty;
	bool bSatisfiesCurrentRequirement;
	TArray<int> RequirementIndicesSatisfied;
	TArray<int> DistributionCategoryIndicesSatisfied;
};

struct FFortMissionDistributionCategory
{
	struct GameplayTagQuery CategoryTagQuery;
};

struct FFortGlobalMission
{
	struct TSoftClassPtr<UObject> MissionInfo;
	TArray<EFortTheaterType> AllowedTheaterTypes;
	struct GameplayTagQuery TheaterTagQuery;
	struct GameplayTagQuery RegionTagQuery;
	TArray<EFortZoneType> AllowedZoneTypes;
	struct GameplayTagQuery ZoneTagQuery;
	struct GameplayTagQuery PrimaryMissionTagQuery;
	float MaxDifficultyLevel;
	float MinDifficultyLevel;
	bool bIsPrototype;
	bool bAllowInTestMaps;
	bool bEnabled;
};

struct FQuestDrivenMissionSubList
{
	struct FName MissionSubListName;
	bool bEnabled;
	TArray<struct FortQuestDrivenMission> QuestDrivenMissionList;
};

struct FSkeletalAudioBoneConfig
{
	struct FName BoneName;
	class UClass* SoundLoop;
	class UClass* SoundMediumDelta;
	class UClass* SoundHighDelta;
	float ThresholdLoop;
	float ThresholdMedium;
	float ThresholdHigh;
	float RetriggerDelay;
	ESkeletalAudioBoneSpace TrackingSpace;
	ESkeletalAudioBoneVelocityType VelocityTrackingType;
};

struct FSkeletalAudioBoneInstance
{
	class UClass* LoopInstance;
	float Delta;
};

struct FSoundIndicatorInitializationData
{
	struct FName EmitterName;
	struct Vector Offset;
	float LifeTime;
};

struct FSpecialActorStatData
{
	struct GameplayTag CategoryTag;
	TArray<struct SpecialActorSingleStatData> Stats;
};

struct FSpecialGameplayAreaOverrideBodyPartsExtraSpecial
{
	TArray<class UClass*> OverrideBodyPartsForExtraSpecial;
	struct GameplayTagContainer SkinMetaTagsForExtraSpecial;
};

struct FSpecialGameplayAreaLootData
{
	struct GameplayTag LootSourceTag;
	struct FName LootTierGroup;
};

struct FFortSplineBase
{
	float StartTime;
	float Duration;
};

struct FFortEvenlySizedSegment
{
	class UClass* SplineMeshComponent;
	class UClass* CapsuleComponent;
};

struct FReppedLastServerIndexToIndex
{
	int ReppedPointIdx;
};

struct FRawPointToLastServerIndexPlusAlpha
{
	float ReppedLastServerIndexPlusAlpha;
};

struct FFortSprayDecalRepPayload
{
	class UClass* SprayAsset;
	struct FName BannerName;
	struct FName BannerColor;
	int SavedStatValue;
};

struct FFortEventConditional
{
	EFortEventConditionType ConditionalType;
	struct FName StatToCompare;
	EStatRecordingPeriod RelevantPeriod;
	EFortCompare ComparisonType;
	int Value;
	class UClass* Stat;
	class UClass* FPC;
};

struct FFortStatEvent
{
	struct FName StatEventName;
	EFortEventRepeat RepeatType;
	TArray<struct FName> StatsToMonitor;
	TArray<struct FortEventConditional> Conditions;
	class UClass* AnnouncementToDisplay;
	class UClass* NotificationParameter;
	class UClass* AssociatedStat;
	class UClass* FPC;
};

struct FFortStatEventSequence
{
	struct FName StatEventName;
	EFortEventRepeat RepeatType;
	TArray<struct FortStatEvent> EventSequence;
	class UClass* AssociatedStat;
	class UClass* FPC;
	TArray<struct FName> StatsToMonitor;
	TArray<struct FortEventConditional> Conditions;
};

struct FTransformableNavLinkClass
{
	struct Vector Translation;
	struct Rotator Rotation;
	class UClass* NavigationLinksClass;
};

struct FFortSupplyDropSubPhaseData
{
	struct ScalableFloat SupplyDropMinCount;
	struct ScalableFloat SupplyDropMaxCount;
	struct ScalableFloat SupplyDropCap;
};

struct FFortSupplyDropGamePhaseData
{
	EAthenaGamePhase GamePhase;
	struct ScalableFloat SupplyDropMinPlacementHeight;
	struct ScalableFloat SupplyDropMaxPlacementHeight;
	struct ScalableFloat SupplyDropTimeInterval;
	struct ScalableFloat SupplyDropTimeDeviation;
	struct ScalableFloat SupplyDropSpawnMinWaitTime;
	struct ScalableFloat SupplyDropSpawnMaxWaitTime;
	struct ScalableFloat SupplyDropMinSpawnHeight;
	struct ScalableFloat SupplyDropMaxSpawnHeight;
	struct ScalableFloat SupplyDropMinSpeed;
	struct ScalableFloat SupplyDropMaxSpeed;
	struct FortSupplyDropSubPhaseData SubPhaseData;
};

struct FFortSurfaceTypeToSurfaceTypeTag
{
	EFortFootstepSurfaceType FootSurfaceType;
	EPhysicalSurface SurfaceType;
	struct GameplayTag SurfaceTypeTag;
	bool bAllowsSurfaceRetriggerOfEvents;
};

struct FFortTagUIData
{
	struct GameplayTag Tag;
	struct FortMultiSizeBrush Icon;
	struct FText DisplayName;
	struct FText Description;
};

struct FFortTaggedSoundCue
{
	struct GameplayTagQuery Requirements;
	class UClass* Sound;
};

struct FTaggedSoundBankResponseList
{
	TArray<struct FortTaggedSoundCue> CueList;
};

struct FFortBotReservedLoot
{
	uint32_t LootId;
	class UClass* Owner;
};

struct FFortBotTeamInfoAthena
{
	TArray<struct FortBotReservedLoot> ReservedLoots;
};

struct FFortTeamPerkLoadoutCondition
{
	int NumTimesSatisfiable;
	struct GameplayTagQuery RequiredTagQuery;
	bool bConsiderMinimumTier;
	bool bConsiderMaximumTier;
	bool bConsiderMinimumLevel;
	bool bConsiderMaximumLevel;
	bool bConsiderMinimumRarity;
	bool bConsiderMaximumRarity;
	EFortItemTier MinimumHeroTier;
	EFortItemTier MaximumHeroTier;
	int MinimumHeroLevel;
	int MaximumHeroLevel;
	EFortRarity MinimumHeroRarity;
	EFortRarity MaximumHeroRarity;
};

struct FStreamingTestPOIVector
{
	struct FString POI;
	struct Vector Location;
};

struct FStreamingTestSkydivePath
{
	struct StreamingTestPOIVector DropLocation;
	struct StreamingTestPOIVector GlideLocation;
	struct StreamingTestPOIVector LandingLocation;
	struct FString TargetPOI;
	struct FString CalendarEvent;
};

struct FFortMissionAlertAvailableData
{
	struct FName MissionAlertCategoryName;
	int NumMissionAlertsAvailable;
};

struct FFortMissionAlertRegionData
{
	TArray<struct FString> RegionUniqueIds;
	TArray<struct FortMissionAlertAvailableData> AvailabilityDataPerCategory;
};

struct FFortTheaterMapMissionAlertData
{
	TArray<struct FortMissionAlertRegionData> AvailabilityDataPerRegion;
};

struct FFortEditorTheaterMapRegionColor
{
	class UClass* Region;
	struct LinearColor RegionColor;
};

struct FFortTheaterTileEditorData
{
	int XCoordinate;
	int YCoordinate;
	class UClass* ZoneTheme;
	class UClass* Region;
	struct FortRequirementsInfo Requirements;
	TArray<struct FortLinkedQuest> LinkedQuests;
	EFortTheaterMapTileType TileType;
	struct GameplayTagContainer TileTags;
	TArray<struct FortTheaterMissionWeight> MissionWeightOverrides;
	TArray<struct FortTheaterDifficultyWeight> DifficultyWeightOverrides;
	bool bCanBeMissionAlert;
	bool bDisallowQuickplay;
};

struct FFortHexMapCoord
{
	int Horizontal;
	int Vertical;
	int Depth;
};

struct FFortWindImpulseHandle
{
	int UID;
};

struct FTieredCollectionProgressionDataBase
{
	ECollectionSelectionMethod SelectionMethod;
};

struct FDifficultyRowProgression
{
	struct FName DifficultyRowName;
	struct ScalableFloat AdditiveDifficultyMod;
};

struct FRewardBadgesProgression
{
	TArray<class UClass*> RewardBadges;
};

struct FWorldAmbientPropertiesMatchingGameContext
{
	struct GameplayTagQuery QueryToMatch;
	struct FortTimeOfDayTheme TimeOfDayTheme;
	struct FortGlobalWindInfo GlobalWindInfo;
};

struct FTimeOfDayDirectOverrides
{
	bool bOverrideLightIntensity;
	bool bOverrideLightColor;
	bool bOverrideSkyLightIntensity;
	bool bOverrideSkyLightColor;
	bool bOverrideFogDensity;
	bool bOverrideFogColor;
	bool bOverrideFogStartDistance;
	float OverriddenLightIntensity;
	struct LinearColor OverriddenLightColor;
	float OverriddenSkyLightIntensity;
	struct LinearColor OverriddenSkyLightColor;
	float OverriddenFogDensity;
	float OverriddenFogStartDistance;
	struct LinearColor OverriddenFogColor;
	class UClass* OverriddenPostProcessActorClass;
};

struct FSkylightWeatherData
{
	class UClass* SkyLightColor;
	class UClass* SkyLightColorWeight;
};

struct FDirectionalLightWeatherData
{
	class UClass* DirectionalLightColor;
	class UClass* DirectionalLightColorWeight;
	class UClass* DirectionalLightColorBrightness;
	class UClass* VolumetricScatteringIntensity;
};

struct FExponentialHeightFogWeatherData
{
	class UClass* FogDensityScale;
	class UClass* FogHeightFalloffScale;
	class UClass* SecondFogDensityScale;
	class UClass* SecondFogHeightFalloffScale;
	class UClass* SecondHeightFogOffsetBias;
};

struct FSkyAtmosphereWeatherData
{
	class UClass* MieScatteringScaleScale;
	class UClass* MieAbsorptionScaleScale;
	class UClass* HeightFogContributionScale;
};

struct FHeightFogAltitudeWeatherData
{
	class UClass* HeightFogZOffset;
};

struct FMaterialWeatherData
{
	class UClass* SkyMaterialInstance;
	class UClass* DynamicSkyMaterialInstance;
};

struct FWindWeatherData
{
	class UClass* WindDirection;
	class UClass* WindStrength;
};

struct FGlobalWeatherData
{
	struct FString WeatherEventName;
	struct FString RequiredCalendarEvent;
	struct GameplayTagContainer WeatherEventTags;
	struct ScalableFloat EnableChanceMin;
	struct ScalableFloat EnableChanceMax;
	class UClass* Frequency;
	struct ScalableFloat FrequencyMin;
	struct ScalableFloat FrequencyMax;
	class UClass* Chance;
	struct ScalableFloat ChanceMin;
	struct ScalableFloat ChanceMax;
	struct ScalableFloat IntensityMin;
	struct ScalableFloat IntensityMax;
	struct ScalableFloat DurationMin;
	struct ScalableFloat DurationMax;
	struct ScalableFloat CooldownMin;
	struct ScalableFloat CooldownMax;
	struct ScalableFloat ChanceToRemainActiveMin;
	struct ScalableFloat ChanceToRemainActiveMax;
	struct ScalableFloat BlendInMin;
	struct ScalableFloat BlendInMax;
	struct ScalableFloat BlendOutMin;
	struct ScalableFloat BlendOutMax;
	struct GameplayTagContainer Tags;
	class UClass* BotVisibilityScale;
	struct PostProcessSettings PostProcessSettings;
	class UClass* PostProcessBlendWeight;
	struct SkylightWeatherData SkylightWeatherData;
	struct DirectionalLightWeatherData DirectionalLightWeatherData;
	struct ExponentialHeightFogWeatherData ExponentialHeightFogWeatherData;
	struct SkyAtmosphereWeatherData SkyAtmosphereWeatherData;
	struct HeightFogAltitudeWeatherData HeightFogAltitudeWeatherData;
	struct MaterialWeatherData MaterialWeatherData;
	struct WindWeatherData WindWeatherData;
	float TimeForNextAttempt;
	bool bIsEnabled;
};

struct FFortTokenContextInfo
{
	struct GameplayTagContainer RequiredContextTags;
	struct FText Text;
};

struct FFortTooltipTokenInfo
{
	struct GameplayTag Token;
	TArray<struct FortTokenContextInfo> ContextDetails;
};

struct FFortTooltipDisplayStatInfo
{
	struct GameplayTag Token;
	struct GameplayAttribute Attribute;
	struct GameplayTagContainer ContextTags;
	bool bLowerIsBetter;
};

struct FFortTooltipDisplayStatsCategory
{
	struct FText CategoryName;
	TArray<struct FortTooltipDisplayStatInfo> TooltipStats;
};

struct FFortTooltipDisplayInfo
{
	class UClass* PrimaryObjectClass;
	class UClass* SecondaryObjectClass;
	struct GameplayTagContainer DescriptionStatsTags;
	class UClass* TooltipStatsList;
};

struct FFortTooltipMapEntry
{
	class UClass* ObjectClass;
	class UClass* SecondaryObjectClass;
	class UClass* TooltipClass;
};

struct FFortTouchAimAssistSettings
{
	float AssistReticleWidth;
	float AssistReticleHeight;
	float AutoFireReticleWidth;
	float AutoFireReticleHeight;
	struct Vector2D AutofireTargetSizeReduction;
	float AutoFireTrackingReticleWidth;
	float AutoFireTrackingReticleHeight;
	float TargetingReticleWidth;
	float TargetingReticleHeight;
	float TargetRange;
	class UClass* TargetWeightCurve;
	class UClass* PullStrengthYawCurve;
	class UClass* PullStrengthPitchCurve;
	float PullMaxRate;
	float AutoTrackDuration;
	float AutoTrackPullStrength;
	float ProjectileMinSpeedForAssist;
	float ProjectileMaxLookAheadTime;
};

struct FTrackConnectorMeshConfig
{
	ETrackIncline InclineSideA;
	ETrackIncline InclineSideB;
	class UClass* Mesh;
};

struct FTrackMovement
{
	class UClass* CurrentSpline;
	float DistanceAlongSpline;
	bool bReverseDirectionAlongSpline;
};

struct FFortUIFeedback
{
	class UClass* Audio;
	bool bLooping;
	float FadeIn;
	float FadeOut;
};

struct FFortLevelStreamingInfo
{
	struct FName PackageName;
	EFortLevelStreamingState LevelState;
	bool bFailedToLoad;
};

struct FFortPlacementLocationTagHandler
{
	class UClass* SpawnLocationBuildingActor;
	class UClass* SpawnedActor;
	struct GameplayTagContainer TagsToRemove;
};

struct FFortTileLootData
{
	struct FortLootQuotaData LootQuotas;
	int LootDrops;
};

struct FWorldTileSubArray
{
	TArray<class UClass*> X;
};

struct FFortStartingMissionInfo
{
	TArray<class UClass*> StartingMissions;
	bool bDisableSharedMissionLoading;
};

struct FFortMissionPlacementItemLookupData
{
	struct GameplayTagContainer ItemIdentifyingTags;
	struct GameplayTagContainer TagsAddedToPlacementActors;
	class UClass* ActorToPlace;
	class UClass* ActorToUseForSpawnLocation;
	class UClass* SpawnedActor;
	struct Vector SpawnLocation;
	struct Rotator SpawnRotation;
	bool bDontCreateSpawnRiftsNearby;
	bool bShouldFreeLocationsOnDeath;
};

struct FFortMissionEntry
{
	float Weight;
	int WorldMinLevel;
	int WorldMaxLevel;
	struct DataTableRowHandle MinDifficultyInfoRow;
	class UClass* MissionGenerator;
	class UClass* MissionInfo;
	EMissionGenerationCategory GenerationCategory;
	struct FortGeneratedDifficultyOptions GeneratedDifficultyOptions;
	TArray<struct FortMissionPlacementItemLookupData> BlueprintLookupData;
};

struct FFortObjectiveRecord
{
	class UClass* ObjectiveClass;
	TArray<byte> ObjectiveData;
};

struct FFortMissionRecord
{
	struct FortMissionEntry MissionEntry;
	class UClass* MissionGenerator;
	struct FortGeneratedDifficultyOptions GeneratedMissionOptions;
	int DayGenerated;
	int UIIndex;
	struct UniqueNetIdRepl QuestOwnerAccount;
	EFortMissionStatus MissionStatus;
	TArray<struct FortObjectiveRecord> ObjectiveRecords;
	TArray<byte> MissionData;
	struct Guid MissionGuid;
};

struct FFortMissionManagerRecord
{
	class UClass* MissionManagerClass;
	TArray<struct FortMissionRecord> MissionRecords;
	int NumRequiredMissionsOfType;
	TArray<byte> MissionManagerData;
};

struct FFortDeferredNewActorData
{
	class UClass* BuildingActor;
	int SavedLevelIndex;
};

struct FFortPlayerEarnedItemCaches
{
	struct UniqueNetIdRepl PlayerID;
	TArray<class UClass*> EarnedItemCaches;
};

struct FWeaponSeatDefinition
{
	int SeatIndex;
	class UClass* VehicleWeapon;
	TArray<struct ActionDefForUI> WeaponActionDefForUI;
	class UClass* VehicleWeaponOverride;
	class UClass* LastEquippedVehicleWeapon;
};

struct FFortCachedWeaponOverheatData
{
	float TimeWeaponWasUnequipped;
	float OverheatValueAtUneqip;
	float OverheatValue;
	float TimeOverheatedBegan;
	float TimeHeatWasLastAdded;
};

struct FVehicleWeapon_RetainedData
{
	int AmmoInClip;
	float LastFireTime;
	bool bHasPrevious;
};

struct FFortVehicleAudioOneshotGate
{
	float GateValue;
	EVehicleAudioTriggerDir Direction;
	bool FadeWhenOutsideGate;
	class UClass* Sound;
	float MinTimeSinceTrigger;
	float InterruptFadeTime;
	class UClass* AudioComp;
};

struct FFortVehicleAudioParam
{
	float Value;
	EVehicleAudioInterpolationType InterpType;
	class UClass* Curve;
	float AttackSpeed;
	float ReleaseSpeed;
};

struct FFortVehicleAudioFloatParam
{
	struct FName Name;
	struct FortVehicleAudioParam Data;
};

struct FPotentiallyDestroyedBuilding
{
	class UClass* BuildingActor;
	float TimeSinceCollision;
};

struct FFortVehicleIncrementTrick
{
	struct FText Name;
	int HalfSpinsNeeded;
	int BaseScore;
	int Repeats;
	int RepeatsHalfSpinsPerTrick;
	int MultiplierIncrement;
};

struct FVolumeActorStats
{
	Unknown BuildingTypeCounts;
};

struct FVolumePerformanceMetrics
{
	int PerformanceValue;
	int PerformanceMaxValue;
	int PerformanceLowendThreshold;
	int PreviewDeltaValue;
	EFortBudgetCategory Category;
};

struct FFortCreativeClassInstanceTracker
{
	TArray<class UClass*> ChildClassesAtLimit;
	TArray<class UClass*> ParentClassesAtLimit;
};

struct FFortCreativeBudgetTracker
{
	TArray<struct FortCreativeBudget> Budgets;
	TArray<uint32_t> AssetInstances;
	TArray<uint32_t> AssetLastInstances;
	Unknown UsedAssetDependencies;
};

struct FFortCreativePersistenceOptions
{
	bool bAllowPlayerToClearData;
};

struct FFortVolumeTimeOfDayConfig
{
	bool bShouldOverrideTimeOfDay;
	bool bUseRandomTimeOfDay;
};

struct FPlayerWaypointContext
{
	class UClass* PlayerState;
	class UClass* Waypoint;
};

struct FFortWeakPointTypeData
{
	struct GameplayTag WeakPointsTag;
	struct ScalableFloat StartActive;
	struct ScalableFloat PassDamageToOwner;
	struct ScalableFloat ResetHealthOnActivation;
	struct ScalableFloat AllowWeakPointDestruction;
	struct ScalableFloat DestroyedWeakPointAutoRegenerationTime;
	class UClass* PassthroughDamageGEClass;
	struct GameplayTag SetByCallerDamageTag;
	TArray<class UClass*> WeakPointActors;
};

struct FAttachedParticleComponentDef
{
	struct Transform Transform;
	struct FName ParentSocket;
	struct TSoftClassPtr<UObject> Template;
	EDetailMode DetailMode;
};

struct FWeaponPickupAmmoCountData
{
	struct GameplayTag AmmoItemDefinitionTag;
	struct ScalableFloat SpawnCount;
};

struct FHelperGameplayTagToAmmoCountMultiplier
{
	struct GameplayTag Tag;
	struct ScalableFloat CountMultiplier;
};

struct FFortBulletPatternEntry
{
	struct FName SocketName;
	struct Rotator RelativeRotation;
};

struct FMountedWeaponInfo
{
	TArray<class UClass*> TraceIgnoreActors;
	float ThirdPersonDistanceCorrection;
	float ThirdPersonDistanceCorrectionPawn;
	bool bDamageStartFromWeaponTowardFocus;
	bool bUseMountedWeaponAimRotOverride;
	bool bUseAimingClampAngles;
	float MaximumPitchAimingAngle;
	float MinimumPitchAimingAngle;
	bool bTargetSourceFromVehicleMuzzle;
	float MinReticleAlphaForAimInterpolation;
	float MinAimAngleDiffForReticleAlpha;
	float MaxAimAngleDiffForReticleAlpha;
	bool bNeedsVehicleAttachment;
	int AttachAttemptCount;
};

struct FMountedWeaponInfoRepped
{
	class UClass* HostVehicleCachedActor;
	int HostVehicleSeatIndexCached;
};

struct FWeaponRechargeAmmoMappingData
{
	struct GameplayTag TagOnPlayer;
	TArray<struct PrimaryAssetId> WeaponIds;
};

struct FFortWindImpulseRadius
{
	struct Vector Location;
	float Radius;
	float CurrentRadius;
	float PreviousRadius;
	float Magnitude;
	float CurrentMagnitude;
	float PreviousMagnitude;
	float BlendTime;
	float CurrentBlendTime;
	struct Box WorldBounds;
	struct FortWindImpulseHandle Handle;
};

struct FFortWindImpulseCylinderDelta
{
	struct Vector DeltaCenter;
	bool bInitialized;
	bool bRippleOutward;
	float SectionWidth;
	float InnerSectionRadius;
	float OuterSectionRadius;
	float MaximumRadius;
	float DesiredOverallBlendTime;
	float SectionBlendTime;
	float SectionCurrentBlendTime;
	float PreviousMagnitude;
	float SectionCurrentMagnitude;
	float DesiredMagnitude;
	struct Box OuterWorldBounds;
	struct Box InnerWorldBounds;
	struct Box WindImpulseBounds;
	struct FortWindImpulseHandle WindImpulseHandleToModify;
};

struct FFortWindImpulseCylinderRadial
{
	struct Vector Location;
	float InnerRadius;
	float OuterRadius;
	float Magnitude;
	struct Box WorldBounds;
	bool bIsChanging;
	bool bIsChangePending;
	struct FortWindImpulseHandle Handle;
};

struct FWindScalarMaterialInterpolationData
{
	struct FName MaterialParameterName;
	int MaterialParameterIndex;
	float LerpFromValue;
	float LerpToValue;
};

struct FWindVectorMaterialInterpolationData
{
	struct FName MaterialParameterName;
	int MaterialParameterIndex;
	struct LinearColor LerpFromValue;
	struct LinearColor LerpToValue;
};

struct FFortMaterialParameterID
{
	int VariableIndex;
	struct FName VariableName;
};

struct FFortWindMaterialParameterPairID
{
	int PairIndex;
	struct FortMaterialParameterID SpeedParameter;
	struct FortMaterialParameterID OffsetParameter;
};

struct FFortWindMaterialData
{
	class UClass* Mid;
	class UClass* IntenseStateMID;
	int MaterialParameterPairIndices;
	int WindVectorParameterIndex;
	TArray<struct WindScalarMaterialInterpolationData> ScalarInterpolationData;
	TArray<struct WindVectorMaterialInterpolationData> VectorInterpolationData;
	TArray<struct FortWindMaterialParameterPairID> ParametersToSet;
};

struct FFortWindResponderMaterialVariablePairData
{
	float PreviousSpeed;
	float PreviousOffset;
	float MaterialsPreviousTime;
	float DeltaTimeModifiedByMaterialSpeed;
	int MaterialVariableIndex;
	struct FName SpeedVariableName;
	struct FName TimeOffsetVariableName;
};

struct FFortWindResponder
{
	class UClass* WindUpdatingBuildingSMActor;
	class UClass* WindSpeedCurve;
	class UClass* WindPannerSpeedCurve;
	class UClass* WindAudio;
	TArray<class UClass*> MildWindMaterialInstances;
	TArray<class UClass*> IntenseWindMaterialInstances;
	TArray<struct FortWindMaterialData> MaterialsData;
	int MaterialParameterPairIndices;
	TArray<struct FortWindResponderMaterialVariablePairData> PairedVariablesData;
	float WindSpeed;
	bool bHasSetupAnimatingMaterials;
};

struct FFortWorldMultiItemInfo
{
	bool bEnableXPLimitsPerLevel;
	struct TSoftClassPtr<UObject> ItemDefinition;
	class UClass* ComponentClassForXPLogic;
	float RequiredXPForNextLevel;
};

struct FPlayerStatsRecord
{
	int Stats;
};

struct FStatRecord
{
	struct FName StatName;
	int StatValue;
};

struct FStatManagerPeriodRecord
{
	TArray<struct StatRecord> StatRecords;
};

struct FFortPlayerRecord
{
	struct FString DisplayName;
	struct FString UniqueId;
	TArray<byte> BackpackData;
	bool bPlayerIsNew;
	struct PlayerStatsRecord PlayerStatsData;
	struct StatManagerPeriodRecord CampaignPeriodRecord;
};

struct FFortZoneInstanceInfo
{
	struct FString WorldId;
	struct FString TheaterId;
	struct FString TheaterMissionId;
	struct FString TheaterMissionAlertId;
	struct TSoftClassPtr<UObject> ZoneThemeClass;
};

struct FFortLevelRecord
{
	int ParentLevelIndex;
	struct Guid BoundActorGuid;
	struct FName PackageName;
	TArray<struct FortActorRecord> SavedActors;
	int X_Loc;
	int Y_Loc;
	byte Rotation;
};

struct FDeployableBaseSupportSettings
{
	bool bUseDeployableBases;
	struct TSoftClassPtr<UObject> DeployableBaseCloudSaveItemDef;
	struct TSoftClassPtr<UObject> DeployableBasePlot;
	struct TSoftClassPtr<UObject> SupportedUnlocks;
	bool bDeployableBasesReadOnly;
	EDeployableBaseUseType SupportedUseType;
	TArray<class UClass*> TieredCollectionLayouts;
};

struct FZoneThemeDifficultyProperties
{
	TArray<struct DataTableRowHandle> ValidDifficulties;
	struct FortTimeOfDayTheme TimeOfDayTheme;
	struct FortGlobalWindInfo GlobalWindInfo;
};

struct FCameraPair
{
	EFrontEndCamera Type;
	class UClass* Camera;
};

struct FGameplayTagMessage
{
	struct GameplayTagContainer Tags;
	struct FText Text;
};

struct FSettingsHUDVisibilityAndText
{
	struct GameplayTag HUDVisibilityGameplayTag;
	ESlateVisibility DefaultHUDVisibility;
	struct FText DisplayText;
	struct FText ToolTipText;
	bool bPlatformConstraintPC;
};

struct FFallbackCharacterPartsMapper
{
	struct GameplayTag RequiredTag;
	TArray<struct TSoftClassPtr<UObject>> CharacterParts;
};

struct FFallbackAIPawnCustomizationMapper
{
	struct GameplayTag RequiredTag;
	struct TSoftClassPtr<UObject> CustomizationFallback;
};

struct FFortHarvestingToolMontageOverrideData
{
	struct GameplayTag CosmeticTag;
	struct TSoftClassPtr<UObject> MontageToPlay;
};

struct FFortHarvestingToolMontageOverrides
{
	TArray<struct FortHarvestingToolMontageOverrideData> MontageOverrideData;
};

struct FEmoteActivationTrigger
{
	struct GameplayTagQuery EmoteTagQuery;
	struct GameplayTagQuery PlayerTagQuery;
	struct GameplayTagContainer TagsToApply;
	float Duration;
};

struct FItemDefinitionAndCount
{
	int Count;
	struct TSoftClassPtr<UObject> ItemDefinition;
};

struct FFortReplicatedStatMapping
{
	EStatCategory StatCategory;
	struct FText DisplayName;
};

struct FConditionalFoundationQuotaTier
{
	TArray<struct TSoftClassPtr<UObject>> FoundationClasses;
	int MinFoundations;
	int MaxFoundations;
};

struct FConditionalFoundationQuota
{
	TArray<struct ConditionalFoundationQuotaTier> Tiers;
};

struct FDefaultCharacterPartsForPawnClass
{
	struct GameplayTag ClassTag;
	TArray<struct TSoftClassPtr<UObject>> CharacterParts;
};

struct FTagRestrictedTable
{
	struct TSoftClassPtr<UObject> SoftTablePtr;
	struct GameplayTagQuery TagQuery;
};

struct FIronCityRowToRating
{
	int Difficulty;
	int RecommendedRating;
	int MinRating;
	int MaxRating;
};

struct FIronCityMatchmakingBuckets
{
	int Difficulty;
	int RecommendedRating;
};

struct FPhoneixXPStats
{
	struct FString CalendarEventFlag;
	int MaxLevelXP;
	int MaxLevel;
	int NumOverlevelRewards;
};

struct FFortMobileHUDPresetHotfix
{
	struct GameplayTag HUDPresetTag;
	struct FortMobileSchemaModificationContainer Modifications;
};

struct FFortAnimInput_AimScrambleDataHelper
{
	float AimPitch;
	float AimYaw;
	bool bIsAimDataScambled;
};

struct FComponentRecordRedirect
{
	struct FString ClassName;
	struct FString OldComponentClass;
	struct FString OldComponentName;
	struct FString NewComponentClass;
	struct FString NewComponentName;
};

struct FActorTemplateRecord
{
};

struct FActorInstanceRecord
{
};

struct FLevelStreamedDeleteActorRecord
{
};

struct FDeleteActorRecord
{
};

struct FRecordBucket
{
	TArray<int> RecordIndices;
};

struct FRecordBucketMap
{
	Unknown PositionToRecord;
	TArray<int> DuplicateRecords;
};

struct FLevelSaveRecordCollectionItem
{
	struct TSoftClassPtr<UObject> LevelSaveRecord;
	struct FName RecordUniqueName;
	struct Transform Transform;
};

struct FVersionedMetricWrapper
{
	ELevelSaveRecordVersion IntroducedVersion;
	ELevelSaveRecordVersion DeprecatedVersion;
	struct TSoftClassPtr<UObject> Class;
};

struct FWidgetMapping
{
	struct GameplayTagContainer KeyTags;
	struct GameplayTag ReplacementTag;
	bool bUseLegacyTagAsBehavior;
	struct GameplayTagContainer AlternativeWidgetTags;
	struct GameplayTag ClassTag;
};

struct FWidgetMappingContainer
{
	struct GameplayTag LegacyTag;
	struct WidgetMapping DefaultMapping;
	TArray<struct WidgetMapping> ContextMappings;
};

struct FWidgetVisibilityOverride
{
	struct GameplayTag Key;
	struct GameplayTagContainer VisibleOverrides;
	struct GameplayTagContainer AllowedOverrides;
	struct GameplayTagContainer HideOverrides;
};

struct FWidgetPropertyUpgradeData
{
	struct GameplayTag Key;
	class UClass* InstancedPropertyData0;
	class UClass* InstancedPropertyData1;
	class UClass* InstancedPropertyData2;
	class UClass* InstancedPropertyData3;
	Unknown InstancedFloatPropertyToTagPropertyMap;
};

struct FWidgetLayoutProxy
{
	struct GameplayTag Key;
	struct AnchorData AnchorData;
	struct Vector2D DefaultContentSize;
};

struct FMyTownWorkerPortraitData
{
	struct TSoftClassPtr<UObject> Portrait;
	int SelectionWeight;
};

struct FMyTownWorkerGenderData
{
	EFortCustomGender Gender;
	int SelectionWeight;
	TArray<struct MyTownWorkerPortraitData> PotraitData;
};

struct FMyTownWorkerPersonalityData
{
	struct GameplayTagContainer PersonalityTypeTag;
	struct FText PersonalityName;
	int SelectionWeight;
	TArray<struct MyTownWorkerGenderData> GenderData;
};

struct FMyTownWorkerSetBonusData
{
	struct GameplayTagContainer SetBonusTypeTag;
	struct FText DisplayName;
	int RequiredWorkersCount;
	class UClass* SetBonusEffect;
	int SelectionWeight;
};

struct FServerMigrationAlertData
{
	int SecondsRemainingStart;
	int SecondsRemainingEnd;
	struct TimerHandle TimerHandle;
};

struct FHeroSlotInfo
{
};

struct FPerkItemSet
{
	TArray<class UClass*> Items;
	float Time;
};

struct FPerkAccoladeInfo
{
	int Index;
	class UClass* AccoladeDef;
	int PipCount;
};

struct FFutureTechData
{
	int UnlockLevel;
	int XpToGetThisLevelFromRoundStartLevel;
	int SingleLevelRequiredXp;
	class UClass* PerkItemDef;
};

struct FRoundTechDataCache
{
	TArray<struct FutureTechData> FuturePerks;
	int LevelAtRoundStart;
	int FactionXPAtRoundStart;
	bool bDataReady;
	int MaxCalandarLevel;
	int MaxLevel;
};

struct FFortPlaysetStreamingData
{
	struct FName PackageName;
	struct FName UniquePackageName;
	struct Vector Location;
	struct Rotator Rotation;
	bool bValid;
};

struct FPairedWeightEntry
{
	struct FName Name;
	struct ScalableFloat Value;
};

struct FPairedWeightContainer
{
	TArray<struct PairedWeightEntry> Weights;
};

struct FIndicatedActorDataWithFilter
{
	struct GameplayTagContainer IndicatedActorTags;
	struct GameplayTagQuery IndicatedActorTagQuery;
	TArray<EObjectTypeQuery> ObjectTypes;
	class UClass* ActorClassFilter;
	TArray<EFortTeamAffiliation> WithAffiliation;
	struct IndicatedActorData IndicatedData;
	struct StenciledActorData StenciledData;
	float OverlapRadius;
};

struct FQuickHealPriority
{
	float MinHealth;
	float MaxHealth;
	float MinShields;
	float MaxShields;
	TArray<struct GameplayTag> GameplayTags;
};

struct FSpecialRelevancyGroup
{
	TArray<class UClass*> Controllers;
	TArray<class UClass*> SpecialActors;
};

struct FSpecialRelevancyModeData
{
	ESpecialRelevancyMode Mode;
	int NumberOfSquads;
};

struct FSpecialRelevancyMultiSquadControllerGroup
{
	TArray<class UClass*> Controllers;
};

struct FSpecialRelevancyMultiSquadControllerGroupsContainer
{
	TArray<struct SpecialRelevancyMultiSquadControllerGroup> ControllerGroups;
};

struct FTileGroupInfo
{
	class UClass* TileGroup;
	int Weight;
	int MinTiles;
	int MaxTiles;
	bool bPlaceAdjacent;
};

struct FTileGroupSelection
{
	TArray<struct TileGroupInfo> TileGroupOptions;
};

struct FTileGroupMapInfo
{
	struct TSoftClassPtr<UObject> GroupWorld;
	float Weight;
	struct FName QuotaCategory;
};

struct FPawnDamageData
{
	class UClass* DamagedActor;
	float Damage;
	class UClass* InstigatedBy;
	class UClass* DamageCauser;
	struct Vector HitLocation;
	class UClass* HitComponent;
	struct FName BoneName;
	struct Vector Momentum;
};

struct FProjectileEventData
{
	class UClass* SpawnedProjectile;
	TArray<struct HitResult> Hits;
	TArray<class UClass*> ExplodedActors;
};

struct FFortAIDirectorEvent
{
	EFortAIDirectorEvent Event;
	class UClass* EventSource;
	class UClass* EventTarget;
	float EventValue;
};

struct FPurchasedItemInfo
{
	class UClass* Item;
	int Quantity;
};

struct FFortPublicAccountInfo
{
	int Level;
	int MaxLevel;
	int LevelXp;
	int LevelXpForLevel;
	struct AthenaLevelInfo BattleRoyaleLevel;
};

struct FFortItemInstanceQuantityPair
{
	class UClass* Item;
	EFortInventoryType InventoryType;
	int Quantity;
};

struct FMcpBanInfo
{
	TArray<EPlayerBanReasons> BanReasons;
	struct FString ExploitProgramName;
	struct FString AdditionalInfo;
	struct DateTime BanStartTimeUtc;
	float BanDurationDays;
	EPlayerCompetitiveBanReasons CompetitiveBanReason;
	bool bRequiresUserAck;
	bool bBanHasStarted;
};

struct FFortCampaignLoadout
{
	class UClass* PersonalVehicle;
};

struct FAthenaMatchLootReward
{
	struct FString TemplateId;
	int Amount;
};

struct FAthenaMatchXpReward
{
	struct FText Text;
	int Amount;
};

struct FAsyncTaskResult
{
	bool bSucceeded;
	struct FString ErrorCode;
	struct FText ErrorMessage;
};

struct FAttributeModifierInfo
{
	class UClass* InstantGEs;
};

struct FBuildingClassData
{
	class UClass* BuildingClass;
	int PreviousBuildingLevel;
	byte UpgradeLevel;
};

struct FCreateBuildingActorData
{
	uint32_t BuildingClassHandle;
	struct Vector_NetQuantize10 BuildLoc;
	struct Rotator BuildRot;
	bool bMirrored;
	float SyncKey;
	struct BuildingClassData BuildingClassData;
};

struct FFortPickupTossOverrideData
{
	bool bIsValid;
	float MinTossDist;
	float MaxTossDist;
	float SpawnDirectionConeHalfAngle;
};

struct FEndOfDayRecap
{
	int DayNumber;
	int TeamScoreAtStartOfDay;
	int TeamScoreAtEndOfDay;
	TArray<struct FortPlayerScoreReport> ScoreReports;
};

struct FEvaluationResult
{
	bool bSucceeded;
	struct FString ErrorCode;
	struct FText ErrorMessage;
};

struct FPlayersLeft
{
	int Humans;
	int Bots;
	int Total;
};

struct FTotalPlayers
{
	int Humans;
	int Bots;
	int Total;
};

struct FFortCreativeMessageDispatcherErrorMessage
{
	EMessageDispatcherErrorMessageType ErrorMessageType;
	struct FText MessageName;
	struct GameplayTag ChannelId;
	int LimitValue;
};

struct FEventTournamentIds
{
	struct FString EventId;
	struct FString WindowId;
	struct FString GroupId;
	struct FString SubGroupId;
};

struct FFortTournamentStatInfo
{
	struct FString StatName;
	struct FName StatDisplayName;
	int StatValue;
};

struct FXPEventInfo
{
	struct FName EventName;
	struct FText SimulatedText;
	class UClass* QuestDef;
	EXPEventPriorityType Priority;
	int EventXpValue;
	int TotalXpEarnedInMatch;
	struct PrimaryAssetId Accolade;
	int RestedValuePortion;
	int SeasonBoostValuePortion;
	int RestedXPRemaining;
};

struct FGiftUINotificationInfo
{
	struct TSoftClassPtr<UObject> SoftItemPtr;
	int Count;
};

struct FPlayerStateEncryptionKey
{
	TArray<byte> Key;
};

struct FCreativePublishOptions
{
	struct FString UserTitle;
	struct FString UserDescription;
	struct FString UserYoutubeVideoId;
	TArray<struct FString> DescriptionTags;
	bool bActivateLink;
	bool bClearPersistentData;
	struct FString UserLocale;
};

struct FCreativeIslandInfo
{
	struct FString IslandTitle;
	struct FString IslandIntroduction;
	struct FString UserLocale;
};

struct FChaseExternalForce
{
	float Duration;
	struct Vector Direction;
	float Magnitude;
	class UClass* MagnitudeScaleCurve;
};

struct FFortPlayerSurveyAnswerContainerChangeEventInfo
{
	EFortPlayerSurveyAnswerContainerChangeReason Reason;
};

struct FFortSelectableRewardOption
{
	TArray<struct FortItemQuantityPair> Rewards;
};

struct FFortRewardInfo
{
	TArray<struct FortSelectableRewardOption> SelectableRewards;
	TArray<struct FortItemQuantityPair> StandardRewards;
	TArray<struct FortHiddenRewardQuantityPair> HiddenRewards;
};

struct FFortCollectionBookRewards
{
	ECollectionBookRewardType RewardType;
	struct FName PageId;
	struct FName SectionId;
	int XpRequired;
	struct FText Description;
	bool bIsMajorReward;
	struct FortRewardInfo Rewards;
};

struct FFortAthenaLoadoutData
{
	EAthenaCustomizationCategory SlotName;
	struct FString ItemToSlot;
	int IndexWithinSlot;
};

struct FAccountIdAndScore
{
	struct FString AccountId;
	int TotalScore;
	int IndividualContribution;
	bool bCriticalMatchBonus;
	bool bIsLeecherExempt;
};

struct FAccountIdAndMatchEndData
{
	struct FString AccountId;
	TArray<struct FortQuestObjectiveCompletion> Advance;
	TArray<struct FortQuestEarnedBadgeData> EarnedBadgeData;
	TArray<struct FString> EarnedItemCaches;
	int TotalScore;
	bool bCriticalMatchBonus;
	bool bDidLeech;
	int NumMissionPoints;
	TArray<struct FString> ShuffledLoadoutUsed;
	int ShuffledLockerUsedIndex;
	int TheaterNum;
	struct McpProfileChangeRequest TheaterItemUpdate;
	int OutpostNum;
	struct McpProfileChangeRequest OutpostItemUpdate;
	struct FString OutpostId;
	struct FortOutpostCoreInfo OutpostInfo;
	struct FString DeployableBaseItemId;
	struct FortCloudSaveInfo CloudSaveInfo;
	struct FString LockCode;
	bool bAbandoning;
	float MissionLeechScaling;
};

struct FFortVehicleInPersistent
{
	class UClass* FortPhysicsVehicleConfigs;
	bool bUseForceHeading;
	bool bHasDriver;
	bool bHasPassengers;
	bool bIsTouchingAnything;
	bool bAttemptAsyncOrientationCorrection;
	float WaterLevel;
	struct Vector LocalFrontFrictionPt;
	struct Vector LocalRearFrictionPt;
	float FrontMassRatio;
	float RearMassRatio;
	struct ReplicatedAthenaVehicleAttributes VehicleAttributes;
};

struct FFortVehicleOutContinuous
{
	float SteeringAngle;
	struct Vector AverageSpringNormal;
};

struct FFortVehicleOutPersistent
{
	bool bCanDriveOnIncline;
	bool bWheelsOnGround;
	bool bAnyWheelsOnGround;
	bool bIsTouchingDrivableGround;
	bool bIsAsyncCorrectingOrientation;
	bool bIsTouchingGroundWithoutWheels;
};

struct FFortVehicleInternalPersistent
{
};

struct FDuelOverlayData
{
	struct FText ChallengerName;
	struct FText ChallengedNPCName;
	struct TSoftClassPtr<UObject> ChallengerSocialAvatarBrushPtr;
	struct TSoftClassPtr<UObject> ChallengedNPCSocialAvatarBrushPtr;
	class UClass* ChallengedNPCLootDef;
};

struct FGameSummaryInfo
{
	struct FString GameSessionId;
	bool Completed;
};

struct FFortItemViewSettings
{
	bool UsesPlacementActor;
	bool UsesFixedCamera;
	bool SupportsZooming;
	float DefaultZoomLevel;
	struct FloatRange ZoomRange;
	EFortItemViewRotationMode RotationMode;
	struct Rotator CameraRotationOffset;
	struct Vector MeshBoundsCenterOffset;
};

struct FConfirmationDialogAction
{
	struct FText DisplayName;
	struct FName ResultName;
	struct SlateBrush Icon;
	struct FName ActionName;
};

struct FFortDialogDescription_NUI
{
	struct SlateBrush Icon;
	struct FText MessageHeader;
	struct FText MessageBody;
	TArray<struct ConfirmationDialogAction> ConfirmButtonInputActions;
	struct FName DeclineButtonInputAction;
	class UClass* AdditionalContent;
	class UClass* LeftAdditionalContent;
	float DisplayTime;
	bool Dismissable;
	bool Cancelable;
	bool bShouldWaitForLatentActionOnConfirmAction;
	class UClass* NotificationHandler;
	class UClass* ShowSound;
};

struct FFortDialogExternalLatentActionHandle
{
	int Handle;
};

struct FFortGlobalActionDetailsFunctionContext
{
	ECommonInputType OverrideInputType;
};

struct FFortItemHeaderInformation
{
};

struct FHomebaseSquadAttributeBonus
{
	struct GameplayAttribute AttributeGranted;
	struct CurveTableRowHandle BonusCurve;
};

struct FHomebaseSquadSlot
{
	struct FText DisplayName;
	TArray<EFortItemType> ValidSlottableItemTypes;
	struct GameplayTagContainer TagFilter;
	TArray<struct HomebaseSquadAttributeBonus> SlottingBonuses;
	class UClass* PersonalityMatchBonusTable;
	ESquadSlotType SlotType;
};

struct FFortReleaseVersion
{
	struct FName VersionName;
};

struct FAIPopulationCountSnapshot
{
	int NumTotalSpawnedBots;
	int NumAliveBots;
	int NumAlivePlayerBots;
	int NumAliveNPCBots;
	int NumAliveAIPawns;
};

struct FFortBotFaction
{
	struct GameplayTagContainer FactionTags;
	TArray<class UClass*> BotPlayerStates;
	TArray<class UClass*> NoneParticipantPlayerStates;
	TArray<class UClass*> AllBotPlayerStates;
};

struct FAthenaBroadcastKillFeedEntryInfo
{
	struct FString InstigatorPlayerName;
	struct FString VictimPlayerName;
	EAthenaBroadcastKillFeedEntryType EntryType;
};

struct FAthenaCosmeticItemDefinitionWrapper
{
	Unknown ItemDef;
};

struct FApplyVariantsAdditionalParams
{
	Unknown WeakPlayerPawn;
	TArray<class UClass*> AdditionalVariantComponents;
	bool bApplyToAdditionalVariantComponentsOnly;
	bool bDeriveMIDNameFromParent;
	bool bShouldResetOverrideMaterialsOnMeshSwap;
	TArray<struct CosmeticVariantInfo> PreviouslyActiveVariants;
	bool bBackpackReliesOnVariantsFromCID;
	bool bGliderReliesOnVariantsFromCID;
	bool bForbidParticleSwapping;
	struct GameplayTagContainer MetaTags;
};

struct FPlayerWithIndicatorState
{
	Unknown PlayerState;
	EPlayerIndicatorFlags IndicatorState;
};

struct FFortFrontEndMarkerData
{
	struct UniqueNetIdRepl PlayerID;
	struct Vector CurrentLocation;
};

struct FPetStimuliResponse
{
	struct GameplayTag ResponseTag;
	float ResponseDuration;
	float ResponseWeight;
};

struct FAthenaSeasonRewardLevelInfo
{
	EAthenaSeasonRewardTrack Track;
	int Level;
	int XpToNextLevel;
	TArray<struct AthenaRewardItemReference> Rewards;
};

struct FBattlePassOfferData
{
	struct FString OfferId;
	TArray<struct ItemQuantity> RewardItemQuantity;
	bool bIsFreePassReward;
	int Cost;
	struct FString CurrencyId;
	int SectionOffersNeededForUnlock;
	int CategoryOffersNeededForUnlock;
	TArray<struct FString> RequiredItemsForUnlock;
};

struct FBattlePassOfferSectionData
{
	int CategoryIndex;
	int SectionIndex;
	int SectionRewardCount;
	int CategoryRewardCount;
	int NumLevelsNeededForSectionUnlock;
	int NumCategoryRewardsNeededForSectionUnlock;
	TArray<struct BattlePassOfferData> SectionOfferList;
};

struct FBattlePassRewardInfo
{
	class UClass* ItemDef;
	int Level;
	bool bIsFree;
	int QuantityRewarded;
	EBattlePassRewardSource UnlockableSource;
	class UClass* UnlockableSourceItemDef;
};

struct FTraversePointSpawnData
{
	class UClass* PointClass;
	struct Vector Location;
	struct Rotator Rotation;
};

struct FBacchusActionIconMapping
{
	struct FName Action;
	class UClass* Sprite;
};

struct FBattleMapSharedData
{
};

struct FBuildingActorClassData
{
	int MaximumBuildingLevel;
};

struct FBuildingFoundationLODActorData
{
	class UClass* VisibilityMaterial;
	class UClass* VisibilityTexture;
};

struct FChaserMarkerPosition
{
	float MarkerPosition;
};

struct FCollectedItemValue
{
	class UClass* CollectedItem;
	int DepositAmount;
	int DepositGoal;
	int CaptureCount;
};

struct FBuildingDuplicationData
{
	class UClass* ClassData;
	class UClass* TextureData;
};

struct FLogicalBuilding
{
};

struct FBuildingNavigationCellInfo
{
};

struct FNeighboringWallInfo
{
	Unknown NeighboringActor;
	struct BuildingSupportCellIndex NeighboringCellIdx;
	EStructuralWallPosition WallPosition;
};

struct FNeighboringFloorInfo
{
	Unknown NeighboringActor;
	struct BuildingSupportCellIndex NeighboringCellIdx;
	EStructuralFloorPosition FloorPosition;
};

struct FNeighboringCenterCellInfo
{
	Unknown NeighboringActor;
	struct BuildingSupportCellIndex NeighboringCellIdx;
};

struct FBuildingNeighboringActorInfo
{
	TArray<struct NeighboringWallInfo> NeighboringWallInfos;
	TArray<struct NeighboringFloorInfo> NeighboringFloorInfos;
	TArray<struct NeighboringCenterCellInfo> NeighboringCenterCellInfos;
};

struct FBuildingGridActorFilter
{
	bool bIncludeWalls;
	bool bIncludeFloors;
	bool bIncludeFloorInTop;
	bool bIncludeCenterCell;
};

struct FBuildingValueRules
{
	int CellsAbove;
	int CellsBelow;
	int CellHorizontalRadius;
	float DistanceFromObjectiveWeight;
	float AttackWeight;
	float StructuralWeight;
	float TrapWeight;
};

struct FTimeOfDayBlueprintDefaultVariables
{
	class UClass* AlternateShadowStaticMesh;
	float VolumetricLightScatteringIntensity;
	bool bDisableTODLightsAndMaterialEmissiveValues;
	bool bDisableStaticMeshShadowCastingWhenLightsAreActive;
	bool bUseAnAlternateShadowMeshWhenTheLightIsOff;
	bool bCastVolumetricShadows;
};

struct FPointConfiguration
{
	TArray<struct Vector> PointLocations;
};

struct FPlayerBindTracking
{
	Unknown PlayerController;
};

struct FModuleTracker
{
	struct FString ContentTrackerID;
	bool bShouldBeInstalled;
	bool bHasAttemptedActivation;
};

struct FFortCreativeAnimateFloatCurveResult
{
	float Value;
	bool bComplete;
};

struct FCreativeDeviceExportData
{
};

struct FCreativeItemInfo
{
	class UClass* ItemDefinition;
	struct Guid Guid;
	int DesiredSlot;
	bool bUseVolumeToSpawn;
};

struct FDebugNativeActionInfo
{
};

struct FAttributeInfo
{
};

struct FFortGameplayEffectModifierDescription
{
	struct GameplayAttribute ModAttribute;
	struct FText ModDescription;
	bool bIsBuff;
	EFortAttributeDisplay MagnitudeFormat;
	EFortStatDisplayType DisplayType;
	float Magnitude;
};

struct FFortGameplayEffectDescription
{
	struct FText EffectDisplayName;
	struct FText EffectWrittenDescription;
	TArray<struct FortGameplayEffectModifierDescription> ModDescriptions;
	TArray<struct FText> GrantedTagDescriptions;
};

struct FVisibilityTestPoint
{
	struct Vector Location;
	class UClass* Component;
};

struct FFortAbilityCanHitParameters
{
};

struct FAbilityTrackedActorEntry
{
};

struct FLODAIUpdateInfo
{
};

struct FFortEncounterGroupLimitData
{
	int DesiredPawnNumCap;
	int RemainingDesiredLimit;
	int CurrentEncounterLimit;
};

struct FFortEncounterAIDirectorFactor
{
	float CurrentValue;
	float AccumulatedPeriodValue;
	float TotalPeriodTime;
};

struct FCurrentIntensityAnalyticsBucket
{
};

struct FFortAIEncounterRiftManagerInitializationData
{
	class UClass* EncounterInfo;
	struct FortEncounterSettings EncounterSettings;
	struct EncounterEnvironmentQueryInfo CurrentEnvironmentQueryInfo;
	struct EncounterEnvironmentQueryInfo FallbackEnvironmentQueryInfo;
	class UClass* RiftClassTemplate;
	int NumRiftsToUse;
	int MinRiftsToUse;
	float UpdateIntervalTimeSeconds;
};

struct FAIAssignmentInfo
{
	Unknown CurrentAssignment;
	struct FortAIGoalInfo CurrentGoal;
	float TimeCurrentGoalWasChosen;
	float TimeExitedLastAssignmentOfType;
	Unknown PreviousAssignment;
	struct FortAIGoalInfo PreviousGoal;
	bool bWaitingForQueryResponse;
	bool bSuppressGoalUpdates;
	bool bReportEnemyGoalSelection;
};

struct FAIDiscouragedGoalTimer
{
	struct FortAIGoalInfo DiscouragedGoalInfo;
	double ExpirationTime;
	uint32_t NumberOfTimesMarkedForDiscouragement;
};

struct FFortAimAssist2D_InputParams
{
};

struct FFortAimAssist2D_OwnerInfo
{
	class UClass* FortPC;
	class UClass* FortPI;
	class UClass* FortPawn;
};

struct FFortAimAssist2D_Target
{
	class UClass* Actor;
};

struct FFortAimAssist2D
{
	struct FortAimAssist2D_InputParams InputParams;
	struct FortAimAssist2D_OwnerInfo OwnerInfo;
	TArray<struct FortAimAssist2D_Target> TargetCache0;
	TArray<struct FortAimAssist2D_Target> TargetCache1;
};

struct FFortAIPawnSkeletalMeshAsyncMaterialLoadData
{
	struct TSoftClassPtr<UObject> Material;
	bool bRequireDynamicInstance;
};

struct FFortAIPawnSkeletalMeshAsyncLoadData
{
	struct TSoftClassPtr<UObject> SkeletalMesh;
	TArray<struct FortAIPawnSkeletalMeshAsyncMaterialLoadData> OverrideMaterials;
	struct TSoftClassPtr<UObject> AnimationBP;
};

struct FFortAISharedRepMovement
{
	struct RepMovement RepMovement;
	float RepTimeStamp;
	byte RepMovementMode;
	EFortAILODLevel RepCurrentFortAILODLevel;
	struct GameplayAbilityRepSharedAnim_Index RepSharedAnimInfo;
};

struct FFortAIBatchedDamageCues
{
	bool bImpact;
	bool bImpactWeapon;
	bool bDamage;
	bool bDamageShields;
	bool bDamageWeapon;
	bool bFatal;
	bool bWeaponActivated;
	struct Vector_NetQuantize10 HitLocation;
	class UClass* TargetActor;
};

struct FDamagerInfoAnalytics
{
	struct FString DamageCauser;
	int DamageAmount;
};

struct FRunVariationData
{
	Unknown FortAIPawn;
	float Distance;
};

struct FAIScalableFloat
{
	struct ScalableFloat ScalableFloat;
	EAIScalableFloatScalingType ScalingType;
};

struct FAlterationOption
{
	struct TSoftClassPtr<UObject> AlterationDef;
	TArray<struct FortItemQuantityPair> Costs;
};

struct FAlterationSlot
{
	int UnlockLevel;
	EFortRarity UnlockRarity;
	struct FName SlotDefinitionRow;
	bool bRespeccable;
	struct FName SlotRarityInitRow;
	EFortRarity SlotInitMin;
	EFortRarity SlotInitMax;
	int SlotInitIndex;
};

struct FAlterationWeightData
{
	struct FString AID;
	int InitialRollWeight;
	TArray<struct FString> ExclusionNames;
};

struct FAlterationWeightSet
{
	TArray<struct AlterationWeightData> WeightData;
};

struct FFortAnalyticsClientEngagement
{
};

struct FAnimSpinner
{
	struct Rotator Rotation;
	struct Vector RotationAxis;
	float BaseRotationSpeed;
	float RotationRate;
	float InterpolationRateSpeedUp;
	float InterpolationRateSlowDown;
	float RotationAngle;
};

struct FFortAnimInput_SpaghettiVehicle
{
	bool bIsUsingSpaghettiVehicle;
};

struct FSourceDriver
{
	struct BoneReference SourceBone;
	EComponentType SourceComponent;
	bool UseQuaternion;
	class UClass* DrivingCurve;
	float Multiplier;
	bool bUseRange;
	float RangeMin;
	float RangeMax;
	float RemappedMin;
	float RemappedMax;
};

struct FOrientationWarpingSpineBoneSettings
{
	struct BoneReference Bone;
};

struct FOrientationWarpingSettings
{
	EAxis YawRotationAxis;
	float BodyOrientationAlpha;
	TArray<struct OrientationWarpingSpineBoneSettings> SpineBones;
	struct BoneReference IKFootRootBone;
	TArray<struct BoneReference> IKFootBones;
};

struct FOrientationWarpingSpineBoneData
{
};

struct FSlopeWarpingFootDefinition
{
	struct BoneReference IKFootBone;
	struct BoneReference FKFootBone;
	int NumBonesInLimb;
};

struct FSlopeWarpingFootData
{
};

struct FSpeedWarpingFootDefinition
{
	struct BoneReference IKFootBone;
	struct BoneReference FKFootBone;
	int NumBonesInLimb;
};

struct FSpeedWarpingFootData
{
};

struct FWeightedLookAtBoneWeightDefinition
{
	float Weight;
};

struct FAntelopeVehicleBoostLevel
{
	float AccumulationPercent;
	float BoostTime;
};

struct FFortAthenaAIBotRunTimeCustomizationData
{
	struct GameplayTag PredefinedCosmeticSetTag;
	float CullDistanceSquared;
	bool bCheckForOverlaps;
	bool bHasCustomSquadId;
	byte CustomSquadId;
};

struct FAILootInfoRowEntry
{
	struct GameplayTagQuery OptionalTagQuery;
	bool bShouldDropInventoryOnDeath;
	bool bShouldDropLootOnDeath;
	TArray<struct FName> LootTiers;
};

struct FBotPerceivedSound
{
	class UClass* SourceActor;
};

struct FFlightControlSurfaces
{
	float RudderAngle;
	float AileronAngle;
	float ElevatorAngle;
	float FlapAngle;
};

struct FItemsToSpawn
{
	class UClass* ItemToDrop;
	struct ScalableFloat NumberToDrop;
};

struct FUpdateBotSkillInfo
{
};

struct FOverrideTablePair
{
	struct FString ParentTablePath;
	struct FString OverrideTablePath;
};

struct FMMRSpawningDataTableInfo
{
	float Skill;
	float Weight;
	int SpawingItemIndex;
};

struct FMMRPhaseSpawningDataTableInfo
{
	int GamePhaseIndexToSpawn;
	TArray<struct MMRSpawningDataTableInfo> SpawningItems;
};

struct FCrucibleWorkManager
{
};

struct FCrucibleLeaderboardEntry
{
	struct FString DisplayName;
	int Rank;
	int Value;
	bool bIsLocalPlayer;
	struct UniqueNetIdRepl UserNetId;
};

struct FCrucibleLeaderboardData
{
	EFortCrucibleLeaderboardId LeaderboardId;
	EFortCrucibleLeaderboardState CurrentState;
	struct FString LeaderboardName;
	TArray<struct CrucibleLeaderboardEntry> CurrentCompiledResults;
	bool bHasBeenRequestedByUser;
	bool bHasHadWorkQueued;
	int NumQueries;
};

struct FCrucibleParticipantData
{
	Unknown AthenaPC;
	Unknown LatestRetrievedRawStats;
};

struct FHeistPostMatchAnalyticsData
{
	int SupplyDropsOpenedPerMatch;
	int JewelsLostToStorm;
	int HeistDropsLostToStorm;
	int JewelsEquippedPerMatch;
	TArray<float> TimeJewelHeldPerTeam;
	TArray<int> PlayersAlivePerEscape;
	int NumSuccessfulEscapes;
	TArray<byte> WinningTeams;
};

struct FPlayerBucketSelection
{
};

struct FFortMutatorMusicEventPlayInstance
{
	class UClass* AudioComponent;
	struct TimerHandle LoopTimerHandle;
};

struct FForcedPerks
{
	class UClass* ForcedItems;
};

struct FQuickbarRestoreMutatorData
{
};

struct FReroutePlayerEventQueueEntry
{
	struct GameplayEventData GameplayEventData;
};

struct FSynchronizedTeleportPlayer
{
	class UClass* FortPlayerState;
};

struct FFortAthenaTutorial_ShootTargetInfo
{
	class UClass* ShootTargetMarker;
	class UClass* ShootTarget;
};

struct FFortAthenaTutorial_AiTargetInfo
{
	class UClass* TargetMarker;
	class UClass* TargetActor;
};

struct FVehicleImpactBucket
{
};

struct FVehicleBounceState
{
	EBounceCompressionState CompressionState;
	float CompressionValue;
	float StateCooldown;
};

struct FSpringConfig
{
	struct Transform LocalStartTM;
	struct Transform LocalApplyTM;
	float SpringLength;
	float SpringStiff;
	float SpringDamp;
	float MaxAccelChange;
};

struct FGearConfig
{
	float TopSpeed;
	float MinSpeed;
	float PushForce;
	float RampTime;
	float SteeringAngleMultiplier;
	bool bAutoBrake;
	bool bIgnoreGravity;
	bool bForwardGear;
};

struct FFortVehicleDerived
{
	TArray<struct SpringConfig> Springs;
	TArray<struct GearConfig> InternalGears;
	TArray<struct GearConfig> InternalSprintGears;
	struct Vector LocalFrontFrictionPt;
	struct Vector LocalRearFrictionPt;
	float FrontMassRatio;
	float RearMassRatio;
};

struct FVehicleSimAux
{
};

struct FFortAnalyticsEventAttribute
{
};

struct FRecipeDataTableRowHandleQuantityData
{
	struct DataTableRowHandle DataTableRowHandle;
	int Quantity;
	bool ConvertRemainderUp;
};

struct FRecipeData
{
};

struct FFortUICameraFrameTargetSettings
{
	struct Vector FocusPointToCenter;
	EFortUICameraFrameTargetBoundingBehavior BoundingBehavior;
	struct FortUICameraFrameTargetBounds BoundsToFrame;
	struct Rotator FocusObjectRotation;
};

struct FObjectivePartialCompletionData
{
	struct FString ObjectiveName;
	int CompletionCount;
};

struct FChallengePartialCompletionData
{
	struct FString QuestTemplateId;
	TArray<struct ObjectivePartialCompletionData> ObjectivData;
};

struct FFortChallengePunchCardStyles
{
	struct GameplayTag StyleType;
	struct FString EncodedName;
	struct LinearColor BaseColor1;
	struct LinearColor BaseColor2;
	struct LinearColor BaseColor3;
	struct LinearColor BaseColor4;
	struct LinearColor BaseColor5;
	struct LinearColor AccentColor1;
	struct LinearColor AccentColor2;
	struct TSoftClassPtr<UObject> BackgrounTexture;
};

struct FFortCharacterCustomization
{
	TArray<class UClass*> CharacterPartOverrides;
	class UClass* AnimBPClassOverride;
	class UClass* AnimSetOverride;
	TArray<class UClass*> AnimLayersOverride;
	class UClass* FootstepBankOverride;
};

struct FFortCharacterCustomizationHandle
{
	int Handle;
};

struct FSpawnPickupEntry
{
	struct FString Name;
	class UClass* PickupClass;
};

struct FClientBotBuildStep
{
};

struct FMovementTestDefinition
{
	float ForwardMoveStrength;
	float SideMoveStrength;
	float Duration;
	struct FString TestName;
};

struct FConsumableTestDefinition
{
};

struct FFortClientPilotConsumableTestDefinition
{
};

struct FFortClientPilotMovementTestDefinition
{
	float ForwardMoveStrength;
	float SideMoveStrength;
	float Duration;
	struct FString TestName;
};

struct FClientPilotBuildStep
{
};

struct FFortCollectionBookStat
{
	int MaxBookXpLevelAchieved;
};

struct FFortDamageSource
{
	Unknown InstigatorController;
	Unknown DamageCauser;
};

struct FFortCombatManagerEvent
{
	float EventValue;
	EFortCombatEvents Event;
};

struct FFortComponentPropertyProxy
{
};

struct FFortEncryptionKey
{
	struct FName Name;
	struct Guid Guid;
	TArray<byte> Bytes;
	EFortEncryptionStatus Status;
};

struct FFortKeyChain
{
	uint32_t Hash;
	TArray<struct FortEncryptionKey> Keys;
};

struct FInspectorScreenshotContext
{
	float DelaySeconds;
	float FOVAngle;
	uint32_t ResX;
	uint32_t ResY;
	uint32_t SampleMultiplier;
	struct FString ResponseUri;
};

struct FMinigameActivityEndedData
{
	bool bSuccessfullyCompleted;
	EMinigameActivityStat Stat;
	float FinalStatValue;
	float FinalStatBestValue;
	int FinalScore;
	int TotalScore;
	int Rank;
};

struct FMinigameActivityStartedData
{
	struct FText MinigameActivityName;
	EMinigameActivityStat Stat;
	float InitialStatValue;
	int InitialScore;
	int TotalScore;
};

struct FEffectRequestContext
{
	class UClass* InstigatingController;
	class UClass* InstigatingActor;
	struct GameplayTagContainer SourceTags;
	struct GameplayTagContainer TargetTags;
	struct GameplayTagContainer ContextTags;
};

struct FNPCChosenItemData
{
};

struct FEffectUIParameters
{
	Unknown ParameterKeyValuePairs;
};

struct FControllerRequirementTestContext
{
	class UClass* TestSubjectActor;
	class UClass* TestSubjectController;
	class UClass* OtherActor;
	class UClass* OtherController;
};

struct FConversationHistoryData
{
};

struct FDataDrivenServiceTaskMemory
{
};

struct FConversationSellTaskMemory
{
};

struct FPendingSupplyDropInfo
{
};

struct FSupplyDropTaskMemory
{
};

struct FFortConversionTierData
{
	int TierCost;
	int RequiredItemQuantity;
};

struct FDisplayManagerVariantData
{
	class UClass* CosmeticItemDef;
	TArray<struct McpVariantChannelInfo> CosmeticVariantChannels;
};

struct FCosmeticLoadoutPartyReplState
{
	struct FString BackpackPersistentName;
	struct FString BannerIconId;
	struct FString BannerColorId;
	int BattlePassLevel;
	int BattlePassSelfBoostXp;
	int BattlePassFriendBoostXp;
	struct DisplayManagerVariantData AthenaCharacterVariantInfo;
};

struct FMcpVariantWriter
{
	struct GameplayTag VariantChannel;
	struct GameplayTag ActiveVariant;
	struct FString CustomActiveVariant;
	struct GameplayTagContainer UnlockedVariants;
	struct FString CustomUnlockedVariants;
	struct FString ColorSwatch;
	int AntiConflictChannel;
	struct FString AntiConflictRules;
	TArray<struct FString> UnlockRequirements;
};

struct FCosmeticMetaTagStack
{
	struct GameplayTagContainer CurrentState;
};

struct FCreativeActorMemoryCost
{
	TArray<struct CreativeAssetMetaData> AssetDependencies;
	int InstanceMemoryCost;
	int AssetMemoryCost;
	struct TSoftClassPtr<UObject> ActorId;
};

struct FCreativeItemGranterItemEntry
{
	struct TSoftClassPtr<UObject> ItemDefinition;
	int Count;
	int Level;
};

struct FMoveToolSettings
{
	bool bAllowGravityOnPlace;
	bool bIsScalingInsteadOfRotating;
};

struct FCreativeOptionCategoryData
{
	struct FText CategoryDisplayName;
	struct FName CategoryTag;
};

struct FFortCreativeWhitelistUserEntry
{
	struct FString UniqueId;
	struct FString UserName;
};

struct FFortInteractionExecutionInteractData
{
};

struct FFortNativeCurieFXResponse
{
	struct GameplayTag OverrideGameplayCue;
	EFortNativeCurieFXCueResponse GameplayCueResponse;
	bool bShouldPlayGeneralVFX;
	bool bShouldPlayAmbientAudio;
	bool bShouldPlayGlow;
	bool bShouldPlayFXAsAOE;
};

struct FFortCurieMaterialElementIntensityDecayMultiplier
{
	struct GameplayTag Element;
	float DecayMultiplier;
};

struct FFortDailyLoginRewardStat_ScheduleClaimed
{
	int RewardsClaimed;
	bool ClaimedToday;
};

struct FFortDailyLoginRewardStat
{
	int NextDefaultReward;
	int TotalDaysLoggedIn;
	struct DateTime LastClaimDate;
	Unknown AdditionalSchedules;
};

struct FFortDailyRewardsNotification
{
	int DaysLoggedIn;
	TArray<struct McpLootEntry> Items;
};

struct FFortTierCollectionLayoutOutput
{
	struct FName SpawnCollectionName;
	struct FName DifficultyRowName;
	struct FName ModifierProgressionName;
	float AdditiveDifficultyMod;
	TArray<class UClass*> RewardBadges;
	TArray<struct FName> StartTierLootTierGroups;
	TArray<struct FName> WaveCompleteLootTierGroups;
};

struct FAvailableTierLayout
{
	class UClass* Layout;
	TArray<struct FortTierCollectionLayoutOutput> AvailableTiers;
	TArray<struct FortTierCollectionLayoutOutput> LockedTiers;
	bool bLocked;
};

struct FDevPartyMember
{
	struct FString ContextName;
	struct FString Email;
	struct FString Password;
	struct FortAthenaLoadout AthenaLoadout;
	class UClass* Emote;
	int SeasonLevel;
	ECrossplayPreference CrossplayPreference;
	struct FString JsonPartyMemberData;
	bool bIsEmbedded;
	struct FString Platform;
	ECommonInputType InputType;
};

struct FMeshNetworkEditorSettings
{
	bool bEnableMeshNetwork;
	int BaseMeshPort;
	int BaseMeshGamePort;
	int NumRootClients;
	int RootLoginStartIndex;
	struct FString MeshMetadata;
};

struct FFortEventLevelNavigationDisplayDetails
{
	struct FString NavActorName;
};

struct FFortFeedbackEvent
{
	class UClass* Instigator;
	class UClass* Recipient;
	struct FortFeedbackHandle Handle;
	float Delay;
	bool bOverriddenQueuing;
};

struct FFerretVehicleBoostLevel
{
	float AccumulationPercent;
	float BoostTime;
};

struct FFortFlightModel
{
};

struct FFortFootstepSurfaceInfo
{
};

struct FAttributeClamp
{
	struct GameplayAttribute Attribute;
	EClampType ClampType;
	float ClampValue;
};

struct FContestedBuildingRequest
{
};

struct FAthenaServerStartAircraftStats
{
	float WarmupDurationSec;
	int NumPlayersMissing;
	int NumPlayersQuitting;
	bool bStartedEarly;
	EAircraftLaunchReason StartReason;
	int ExpectedPlayers;
	int PlayersReadied;
	int PlayersLoadingScreenDropped;
	struct GameplayTagContainer GoldenPoisTags;
};

struct FClearAreaParams
{
	struct Vector Location;
	float CapsuleHalfHeight;
	float CapsuleRadius;
	struct GameplayTagContainer IgnoreTags;
};

struct FRejoinScoreMemory
{
};

struct FBCActionInfo
{
	int Type;
	int Action;
};

struct FProjectileTrajectorySplineInfo
{
	class UClass* Instigator;
	struct Vector InitialLocation;
	struct Vector InitialVelocity;
	float MaxSpeed;
	float Gravity;
	float Friction;
	float Bounciness;
	float TimeStep;
	float TraceExtent;
	ECollisionChannel TraceChannel;
	struct FName CollisionProfile;
	int MaxBounces;
	int MaxSteps;
	float MaxDistanceBetweenSplinePoints;
	float InitialDistance;
	TArray<class UClass*> ExtraActorsToIgnore;
	float LinearDamping;
};

struct FFortGameplayDataTrackerRegisteredActorData
{
	struct GameplayTag CurrentStateTag;
};

struct FGameplayFeedbackEventParams
{
	struct GameplayTag EventTag;
	struct UniqueNetIdRepl InstigatorPlayerId;
	struct Vector EventLocation;
};

struct FFortPendingReceiverReg
{
	struct GameplayTagContainer ChannelIds;
};

struct FFortMessageChannelInfo
{
};

struct FMeshServiceMetadata
{
};

struct FFortBanHammerStrike
{
	struct UniqueNetIdRepl AccountId;
	struct FString Reason;
	EFortBanHammerNotificationAction Action;
	struct FString Source;
	struct FString Offense;
};

struct FPlaylistStreamedLevelData
{
	bool bIsFinishedStreaming;
	bool bIsServerOnly;
};

struct FNavIndicatorTextList
{
	struct GameplayTag ListName;
	TArray<struct FText> IndicatorTextList;
};

struct FDynaimcLevelStreamingLocations
{
	struct Transform LoadLocation;
	struct FName StreamingLevelPackageName;
};

struct FDynamicStreamingLevelData
{
	struct FName LevelPackageName;
	EFrontEndCamera CameraOverride;
	struct FName CameraName;
};

struct FPickupManagedActorBucket
{
};

struct FManagedActorInfo
{
};

struct FBountyQuestSettings
{
	bool bSetTargetForSquad;
	struct TSoftClassPtr<UObject> TargetQuestToGrantPtr;
	bool bGrantProtectorQuests;
	struct TSoftClassPtr<UObject> ProtectorQuestToGrantPtr;
	struct ScalableFloat MediumThreatRange;
	struct ScalableFloat HighThreatRange;
	struct ScalableFloat ThreatRefreshRate;
	struct TSoftClassPtr<UObject> NPCIcon;
};

struct FRadiusTrackingInitializer
{
	float MaxCenterOffset;
	float MinimumRadius;
	float MaximumRadius;
	class UClass* CircleDrawingMaterial;
	float TimeBetweenUpdatesInSeconds;
	float RadiusAmountToShrinkEachUpdate;
	struct LinearColor CircleColor;
	ERadiusTrackingGroupingType GroupingType;
};

struct FRadiusTrackingInfo
{
	struct RadiusTrackingInitializer RadiusTrackingStaticValues;
};

struct FRadiusTrackingGroupKey
{
};

struct FPlayerBuildableClassFilter
{
	EFortResourceType ResourceType;
	EFortBuildingType BuildingType;
	int Level;
	class UClass* EditModeMetadata;
};

struct FFortScalabilityModeSettings
{
};

struct FFortGlobalActionDetails
{
	struct FName ActionName;
	bool HoldStatus;
};

struct FContentInstallError
{
	struct FString ErrorCode;
	struct FText ErrorText;
};

struct FGameplayMessageRouterBool
{
	bool bValue;
};

struct FFortHealthBarComponentData
{
	struct FText DisplayText;
};

struct FFortTeamPerkSlotFilteringData
{
};

struct FHeroItem
{
	struct TSoftClassPtr<UObject> Item;
	int Quantity;
	EFortReplenishmentType Replenishment;
	struct GameplayTagContainer RequiredGPTags;
	struct GameplayTagContainer SwapTag;
	bool bShowInAbilityScreen;
};

struct FHeroAbilityKit
{
	struct TSoftClassPtr<UObject> InherentAbilityKit;
	struct GameplayTagContainer RequiredGPTags;
	bool bShowInAbilityScreen;
};

struct FFortZoneEvent
{
	struct FName EventType;
	class UClass* EventFocus;
	class UClass* EventContent;
	class UClass* EventInstigator;
};

struct FHomebaseNodeState
{
	bool bIsOwned;
	bool bAreCostsPayable;
	int Level;
};

struct FWorkerSetBonusState
{
	struct GameplayTag SetBonusTag;
	int CurrentMatchCount;
	int RequiredMatchCountToActivate;
};

struct FTeamMapExplorationEvent
{
	byte TeamId;
	int8_t ExplorationThreshold;
};

struct FInteriorAudioScanResults
{
};

struct FSpawnItemVariantParams
{
	class UClass* WorldItemDefinition;
	struct GameplayTagContainer RequiredTags;
	int NumberToSpawn;
	struct Vector Position;
	struct Vector Direction;
	int OverrideMaxStackCount;
	bool bToss;
	bool bRandomRotation;
	bool bBlockedFromAutoPickup;
	int PickupInstigatorHandle;
	EFortPickupSourceTypeFlag SourceType;
	EFortPickupSpawnSource Source;
	class UClass* OptionalOwnerPC;
	bool bPickupOnlyRelevantToOwner;
};

struct FMcpLeaderboardResultRow
{
	struct UniqueNetIdRepl PlayerUniqueNetId;
	int Rank;
	int Value;
};

struct FReplicatedTranslatedString
{
	struct FString Locale;
	struct FString Literal;
};

struct FAthenaPlaylistEntry
{
	struct FString PlaylistName;
	struct FString DisplayName;
	struct FString DisplaySubName;
	struct FString Description;
	struct FString Violator;
	struct FString Image;
	TArray<struct FString> ExtraImages;
	int CropOffset;
	EFortMatchmakingTileStyle SpecialBorderId;
	bool bShowRevealAnimation;
	bool bIsCMSDataHidden;
};

struct FShowdownTournamentEntry
{
	struct FString TournamentDisplayId;
	struct FString TitleLine1;
	struct FString TitleLine2;
	struct FString ScheduleInfo;
	struct FString PosterFrontImage;
	struct FString PosterBackImage;
	struct FString FlavorDescription;
	struct FString DetailsDescription;
	struct FString ShortFormatTitle;
	struct FString LongFormatTitle;
	struct FString BackgroundTitle;
	int PinScoreRequirement;
	struct FString PinEarnedText;
	struct FString BaseColor;
	struct FString PrimaryColor;
	struct FString SecondaryColor;
	struct FString HighlightColor;
	struct FString TitleColor;
	struct FString ShadowColor;
	struct FString BackgroundLeftColor;
	struct FString BackgroundRightColor;
	struct FString BackgroundTextColor;
	struct FString PosterFadeColor;
	struct FString PlaylistTileImage;
	struct FString LoadingScreenImage;
	struct FString StyleInfoId;
	struct FString AlertText;
	EFortTournamentAlertType AlertType;
};

struct FShowdownLatestTournamentData
{
	struct FString LastModified;
	Unknown Entries;
};

struct FLeaderboardEntry
{
	struct FString LeaderboardId;
	struct FString LeaderboardName;
	bool ShowDetailsPanel;
};

struct FLatestLeaderboardData
{
	struct FString LastModified;
	TArray<struct LeaderboardEntry> Entries;
};

struct FPlaylistFrontEndData
{
	Unknown WeakPlaylistData;
	EPlaylistVisibilityState Visibility;
	bool bDisplayAsDefault;
	EPlaylistAdvertisementType AdvertiseType;
	bool bDisplayAsLimitedTime;
	int CategoryIndex;
	struct DateTime EndDateUTC;
};

struct FFortMatchmakingUtilityRequest
{
	struct PlaylistFrontEndData DesiredPlaylist;
	struct FString PlaylistRegionForTournament;
	EMatchmakingUtilityFlows FlowOverride;
	struct FString LinkCodeOverride;
};

struct FFortMatchmakingErrorInfo
{
	EMatchmakingErrorV2 Error;
	struct FString ErrorCode;
	struct FString ResponseStr;
};

struct FFortMcpCollectedCharacterProperties
{
	int QuestsGiven;
	int QuestsCompleted;
	byte EncounterTypeFlags;
};

struct FFortMcpCollectedFishProperties
{
	float Weight;
	float Length;
};

struct FFortMcpCollectionsVariant
{
	struct FString Category;
	struct FString Variant;
};

struct FFortMcpCollectionsBulkUpdateEntry
{
	struct FString Category;
	struct FString Variant;
	TArray<struct FString> ContextTags;
	struct JsonObjectWrapper Properties;
	EFortCollectedState SeenState;
	int Count;
};

struct FFriendCodeLocString
{
	struct FString Lang;
	struct FString Text;
};

struct FMtxPackage
{
	struct FString StorefrontName;
	struct FString OfferId;
	struct FText Title;
	struct FText Description;
	int TotalAmount;
	int BonusAmount;
	struct FText Price;
	struct FText SaleBasePrice;
	struct FString DisplayAssetPath;
	struct FString BannerOverride;
};

struct FMtxBreakdown
{
	int AvailableTotalMtx;
	int AvailablePremiumMtx;
	int UnavailableTotalMtx;
	int UnavailablePremiumMtx;
};

struct FFortLootNotification
{
	struct FString LootSource;
	struct FString LootSourceInstance;
	struct McpLootResult LootGranted;
};

struct FFortPhoenixLevelUpNotification
{
	int Level;
	struct FortLootNotification Loot;
};

struct FFortEventFlagsNotification
{
	TArray<struct FString> EventFlags;
	struct DateTime RefreshTime;
};

struct FFriendCodeIssuedNotification
{
	struct FString Code;
	struct FString CodeType;
};

struct FFortDifficultyIncreaseRewardEntry
{
	int DifficultyIncreaseTier;
	struct McpLootResult DifficultyIncreaseMissionRewards;
};

struct FFortDifficultyIncreaseRewardRecord
{
	TArray<struct FortDifficultyIncreaseRewardEntry> PendingRewards;
};

struct FFortMissionAlertClaimData
{
	struct FString MissionAlertID;
	struct DateTime RedemptionDateUtc;
	struct DateTime EvictClaimDataAfterUtc;
};

struct FFortMissionAlertRecord
{
	TArray<struct FortMissionAlertClaimData> ClaimData;
	struct McpLootResult PendingMissionAlertRewards;
};

struct FFortCollectionBookResearchItemNotification
{
	struct FString ResearchedItemId;
	struct FString ResearchedTemplateId;
};

struct FFortCollectionBookUnslotItemNotification
{
	struct FString UnslottedItemId;
};

struct FFortCollectionBookClaimRewardNotification
{
	struct McpLootResult Loot;
	struct FString Page;
	struct FString Section;
};

struct FFortCollectionBookSlotItemNotification
{
	struct FString SlottedItemId;
};

struct FFortClaimedDifficultyIncreaseRewardNotification
{
	struct McpLootResult LootGranted;
};

struct FFortMissionAlertCompleteNotification
{
	struct McpLootResult LootGranted;
};

struct FFortTransmogResultNotification
{
	TArray<struct McpLootEntry> TransmoggedItems;
	TArray<struct McpLootEntry> RecycledItems;
};

struct FFortUpgradeItemRarityNotification
{
	TArray<struct McpLootEntry> ItemsGranted;
};

struct FFortConversionResultNotification
{
	TArray<struct McpLootEntry> ItemsGranted;
};

struct FFortCollectedResourceNotification
{
	struct McpLootResult Loot;
};

struct FFortReceivedGiftedBoostXpNotification
{
	int AmountBoostXpGifted;
	struct FString GifterAccountId;
};

struct FFortEarnScoreNotification
{
	int BaseXPEarned;
	int BonusXPEarned;
	int BoostXPEarned;
	int BoostXPMissed;
	int RestXPEarned;
	int GroupBoostXPEarned;
};

struct FFortEndMatchScoreNotification
{
	Unknown scoreNotifications;
};

struct FFortDailyQuestRerollNotification
{
	struct FString NewQuestId;
};

struct FFortQuestRewardData
{
	struct FString QuestId;
	struct McpLootResult Loot;
};

struct FFortQuestRewardNotification
{
	struct FString QuestId;
	struct McpLootResult Loot;
	TArray<struct FortQuestRewardData> QuestsAndRewards;
};

struct FFortExpeditionResultNotification
{
	bool bExpeditionSucceeded;
	TArray<struct McpLootEntry> ExpeditionRewards;
};

struct FFortGetMcpTimeForPlayerNotification
{
	struct DateTime McpTime;
};

struct FCardPackResultNotification
{
	struct McpLootResult LootGranted;
	int DisplayLevel;
};

struct FFortNotificationLevelUp
{
	int Level;
	struct FString HeroId;
	struct FortLootNotification Loot;
};

struct FFortCraftingResultNotification
{
	TArray<struct McpLootEntry> ItemsCrafted;
};

struct FImportFriendsRewardNotification
{
	TArray<struct McpLootEntry> LootGranted;
};

struct FFortMissionCompletionNotification
{
	bool bWasCritical;
	struct FString MissionName;
	struct McpLootResult LootGranted;
};

struct FFortInventorySnapshot
{
	struct JsonObjectWrapper Stash;
};

struct FHardcoreModifierUpdate
{
	struct FString ModifierId;
	bool bEnabled;
};

struct FCompetitiveIdentity
{
	struct DateTime LastUpdated;
	struct FString RegionId;
};

struct FAthenaSeasonEntry
{
	int BookLevel;
	int BookXp;
	int NumHighBracket;
	int NumLowBracket;
	int NumWins;
	bool PurchasedVIP;
	int SeasonLevel;
	int SeasonNumber;
	int SeasonXp;
};

struct FOfferVoteInfo
{
	struct DateTime FirstVoteAt;
	struct DateTime LastVoteAt;
	int VoteCount;
};

struct FCommunityVoteInfo
{
	struct FString ElectionId;
	Unknown VoteHistory;
	struct DateTime LastVoteGranted;
	int VotesRemaining;
};

struct FBattlePassOfferPurchaseRecord
{
	struct FString OfferId;
	struct DateTime PurchaseDate;
	TArray<struct McpLootEntry> LootResult;
	struct PrimaryAssetId CurrencyType;
	int TotalCurrencyPaid;
};

struct FFortAthenaPastSeasonStats
{
	int BookLevel;
	int BookXp;
	int NumHighBracket;
	int NumLowBracket;
	int NumWins;
	int PurchasesVIP;
	int SeasonLevel;
	int SeasonNumber;
	int SeasonXp;
};

struct FFortAthenaSeasonStats
{
	int NumWins;
	int NumHighBracket;
	int NumLowBracket;
};

struct FFortAthenaItemCacheRecord
{
	class UClass* ItemDef;
};

struct FFortAthenaConsumableRecord
{
	class UClass* ItemType;
	int TotalQuantity;
};

struct FMtxPurchaseHistoryEntry
{
	struct FString PurchaseId;
	struct FString OfferId;
	struct DateTime PurchaseDate;
	struct DateTime RefundDate;
	bool FreeRefundEligible;
	bool bHasBeenRefunded;
	TArray<struct McpLootEntry> LootResult;
	int TotalMtxPaid;
};

struct FMtxPurchaseHistory
{
	TArray<struct MtxPurchaseHistoryEntry> Purchases;
	int RefundCredits;
	int RefundsUsed;
};

struct FGiftHistory
{
};

struct FFortCommonPublicPersona
{
	struct FString BannerIconId;
	struct FString BannerColorId;
	struct FString HomebaseName;
};

struct FItemTransferOperation
{
	struct FString ItemId;
	int Quantity;
	bool ToStorage;
	struct FString NewItemIdHint;
};

struct FItemIdAndQuantityPair
{
	struct FString ItemId;
	int Quantity;
};

struct FFortProfileAndQuestSaveIdPair
{
};

struct FFortBatchUpdatePlayer_DeployableBaseUpdate
{
	struct UniqueNetIdRepl AccountId;
	struct FString DeployableBaseItemId;
	struct FortCloudSaveInfo CloudSaveInfo;
};

struct FFortBatchUpdatePlayer_Update
{
	struct UniqueNetIdRepl AccountId;
	int TheaterNum;
	struct McpProfileChangeRequest TheaterItemUpdate;
	int OutpostNum;
	struct McpProfileChangeRequest OutpostItemUpdate;
	TArray<struct FortQuestObjectiveCompletion> QuestObjectiveUpdates;
};

struct FFortActiveSubscription
{
	struct DateTime NextRewardDate;
	struct DateTime NextRenewalRewardDate;
	struct DateTime LastRenewalRewardDate;
	struct DateTime NextMonthlyRewardDate;
	struct FString NextMonthlyRewardLabel;
	struct DateTime CurrentMonthlyRewardDate;
	struct FString CurrentMonthlyRewardLabel;
	struct DateTime LastStatusRefresh;
	struct DateTime MustRefreshAuthBy;
	struct DateTime LastAuthRefresh;
	struct DateTime SubscriptionEndDate;
	bool IsRetryingRenewal;
	bool WillAutoRenew;
	EAppStore AppStore;
	struct FString UniqueSubscriptionId;
};

struct FFortTwitchPendingQuestNotification
{
};

struct FFortTwitchViewerNameAndAccountId
{
	struct FString TwitchViewerName;
	struct FString AccountId;
};

struct FFortTwitchViewerCompletedQuestNotification
{
	TArray<struct FortTwitchViewerNameAndAccountId> ViewerIds;
};

struct FFortTwitchViewerGrantedQuestNotification
{
	struct FString QuestTemplateId;
	TArray<struct FortTwitchViewerNameAndAccountId> ViewerIds;
};

struct FFortCharacterRanges
{
	TArray<uint32_t> Ranges;
	TArray<uint32_t> SinglePoints;
	TArray<uint32_t> ExcludedPoints;
};

struct FMcpMatchResults
{
	int Placement;
	int Kills;
	int Deaths;
};

struct FQueryXboxUserXUIDParams
{
	struct FString UserXSTSToken;
};

struct FIssuedFriendCode
{
	struct FString CodeId;
	struct FString CodeType;
	struct DateTime DateCreated;
};

struct FXboxDedicatedServerSessionCreationParams
{
	struct FString TitleId;
	struct FString SandboxId;
	TArray<struct FString> XboxUserIds;
};

struct FPlayerToxicityReportRequest
{
	struct FString Reason;
	struct FString Details;
	struct FString ReporterGameSessionId;
	struct FString GameSessionId;
	struct FString Token;
	struct FString CreativeIslandSharingLink;
	struct FString CreativeIslandGuid;
	struct FString CreativeIslandOwnerAccountId;
	struct FString SubGameName;
	struct FString PlaylistName;
	bool bIsCompetitiveEvent;
	struct FString ReporterPlatform;
	struct FString OffenderPlatform;
	bool bBlockUserRequested;
	bool bUserMarkedAsKnown;
};

struct FMeshRegionLockData_Tracker
{
	TArray<struct FString> RegionLockDatas;
};

struct FMeshRegionLockData_Base
{
	struct FString RegionData;
	bool Replicated;
};

struct FMinigameStatRow
{
	struct FText PlayerName;
	int TeamColorIndex;
	TArray<struct FText> PlayerStats;
	TArray<struct FText> CumulativePlayerStats;
	bool bIsTeamRow;
};

struct FMinigameScoreData
{
	float Score;
};

struct FMinigameScoreTracker
{
	struct MinigameScoreData GroupScore;
	TArray<struct MinigameTeamScoreData> TeamScores;
	TArray<struct MinigameSoloScoreData> SoloScores;
};

struct FMinigameScoreboardValue
{
	class UClass* StatFilter;
	int Value;
	bool bHighlight;
};

struct FMingiameScoreboardRow
{
	struct FText Name;
	int TeamColorIndex;
	TArray<struct MinigameScoreboardValue> Values;
	int WinCount;
	bool bHighlight;
};

struct FVehiclePlacementInfo
{
	Unknown PlacementActor;
	struct ActiveGameplayEffectHandle AppliedGameplayEffectHandle;
};

struct FFortMissionInfoOption
{
	struct TSoftClassPtr<UObject> MissionInfo;
	float MinDifficultyLevel;
};

struct FQueuedMusic
{
};

struct FMutatorAbilitySetEntry
{
	TArray<struct FortAbilitySetHandle> AppliedAbilitySetHandles;
};

struct FMutatorContext
{
	TArray<class UClass*> MutatorOwners;
};

struct FNativizationCachedFieldsToCompare
{
};

struct FOctopusAsyncDef
{
};

struct FFortOctopusCmd
{
};

struct FVerifyProfileTokenPayload
{
	Unknown ProfileTokens;
};

struct FGeneralChatRoom
{
	struct FString RoomName;
	int CurrentMembersCount;
	int MaxMembersCount;
	struct FString PublicFacingShardName;
};

struct FGeneralChatReturn
{
	TArray<struct GeneralChatRoom> GlobalChatRooms;
	TArray<struct GeneralChatRoom> FounderChatRooms;
	bool bNeedsPaidAccessForGlobalChat;
	bool bNeedsPaidAccessForFounderChat;
	bool bIsGlobalChatDisabled;
	bool bIsFounderChatDisabled;
	bool bIsSubGameGlobalChatDisabled;
};

struct FFortOutpostPOSTInfo
{
	TArray<struct FortDepositedResources> DepositedPostItems;
};

struct FFortPatrolAnimSetWeaponPair
{
	EFortWeaponCoreAnimation WeaponType;
	class UClass* DataAsset;
};

struct FAssetAttachment
{
	struct FName SocketName;
	class UClass* SkeletalMeshAsset;
	class UClass* StaticMeshAsset;
	bool bSkipOnDedicatedServers;
	bool bIsCurrentWeaponSubstitute;
	class UClass* SkelMeshComp;
	class UClass* StaticMeshComp;
};

struct FSavedPosition
{
};

struct FFortPresenceGameplayStats
{
	struct FString State;
	struct FName Playlist;
	int NumKills;
	bool bFellToDeath;
};

struct FFortPresenceBasicInfo
{
	int HomeBaseRating;
};

struct FFortPhysicsSimulationRepData
{
	EFortPhysicsSimulationRepEvent EventType;
	struct Vector RepAppliedVector;
	struct Vector RepLocationVector;
	struct FName RepBoneName;
	bool bRepFlag;
};

struct FCanInteractResult
{
	Unknown RequestingPawn;
	Unknown RequestingPlayerController;
	Unknown InteractComponent;
};

struct FCreativeToolPersistentData
{
	byte GridSnapIndex;
	byte RotationAxisIndex;
	byte SelectedScaleAxis;
	bool bShouldUsePrecisionGridSnapping;
	bool bAllowGravityOnPlace;
	bool bShouldDestroyPropsWhenPlacing;
	byte HitTraceRule;
	bool bIsScalingInsteadOfRotating;
};

struct FServerSideHitMarker
{
};

struct FVoiceChatUsageAnalytics
{
};

struct FCraftingQueueInfo
{
};

struct FXPUIEvent
{
	class UClass* AccoladeDef;
	struct FText SimulatedXpEvent;
	int OldXPValue;
	int EventXpValue;
};

struct FFortPhoenixLevelUpData
{
	int Level;
	TArray<struct FortItemInstanceQuantityPair> Rewards;
};

struct FHighlightReel
{
	int TotalDurationSeconds;
	struct FText Description;
	EHighlightReelTypes HighlightType;
};

struct FFortInputActionDetails
{
	EFortInputActionType InputActionType;
	struct Key ActionKey;
};

struct FFortAxisSmoothing
{
	float ZeroTime;
	struct Vector AverageValue;
	int Samples;
	float TotalSampleTime;
};

struct FFortPickupRequestInfo
{
	struct Guid SwapWithItem;
	float FlyTime;
	struct Vector Direction;
	bool bPlayPickupSound;
	bool bIsAutoPickup;
	bool bUseRequestedSwap;
	bool bTrySwapWithWeapon;
	bool bIsVisualOnlyPickup;
};

struct FFortTetherSessionTelemetryInfo
{
};

struct FAnimLayersOverrideData
{
};

struct FAnimSetOverrideData
{
};

struct FFortFootstepBankOverrideData
{
};

struct FFortAnimBPOverrideData
{
};

struct FFortRedeployGliderTelemetryData
{
};

struct FCustomHighlighting
{
};

struct FSharedRepMovement
{
	struct RepMovement RepMovement;
	float RepTimeStamp;
	float TurretYaw;
	float TurretPitch;
	uint32_t RemoteViewData32;
	uint16_t AccelerationPack;
	int8_t AccelerationZPack;
	byte RepMovementMode;
	byte JumpFlashCountPacked;
	byte LandingFlashCountPacked;
	EFortMovementStyle CurrentMovementStyle;
	bool bProxyIsJumpForceApplied;
	bool bIsCrouched;
	bool bIsSkydiving;
	bool bIsParachuteOpen;
	bool bIsSlopeSliding;
	bool bIsProxySimulationTimedOut;
	bool bIsTargeting;
	bool bIsWaterJump;
	bool bIsWaterSprintBoost;
	bool bIsWaterSprintBoostPending;
};

struct FPersistenceDelegateHandle
{
};

struct FFortVehicleUseTelemetryInfo
{
};

struct FCustomFeedMessageContext
{
	Unknown OwningPlayerState;
	struct Vector OriginLocation;
};

struct FFortPlayerSurveyAnalyticsResponse
{
	uint32_t InitialOrder;
	struct FString ResponseTextID;
	struct FString ResponseTextLocalized;
	struct FString Selection;
};

struct FFortPlayerSurveyAnalyticsQuestion
{
	uint32_t InitialOrder;
	struct FString QuestionTextID;
	struct FString QuestionTextLocalized;
	struct FString QuestionType;
	TArray<struct FortPlayerSurveyAnalyticsResponse> Responses;
	uint32_t TimeTaken;
	bool bAnswered;
};

struct FFortPlayerSurveyAnalyticsTextReplacementEvaluation
{
	struct FString TextReplacementTag;
	struct FString TextReplacementValueID;
	struct FString TextReplacementValueLocalized;
};

struct FFortPlayerSurveyAnalyticsData
{
	struct FString SurveyID;
	TArray<struct FortPlayerSurveyAnalyticsQuestion> Questions;
	TArray<struct FortPlayerSurveyAnalyticsTextReplacementEvaluation> TextReplacementEvaluations;
	struct FString PreviousMatchPlaylist;
	struct FString PreviousMatchGameSessionID;
};

struct FFortPlayerSurveyAnalyticsAnswer
{
};

struct FFortPlayerSurveyAnalyticsSurveyResponse
{
	struct FString SurveyID;
	EFortPlayerSurveyAnalyticsFinishReason FinishReason;
	struct JsonObjectWrapper MetaData;
	TArray<struct FortPlayerSurveyAnalyticsAnswer> Answers;
};

struct FFortPlayerSurveyAnalyticsAnswerBase
{
	uint32_t TimeTaken;
};

struct FFortPlayerSurveyAnalyticsAnswerMultipleSelectionSingleAnswer
{
	bool Selected;
	uint32_t TimeTaken;
};

struct FFortPlayerSurveyAnswerBase
{
	struct Timespan TimeSpentAnswering;
};

struct FFortPlayerSurveyAnswerMultipleSelectionSingleAnswer
{
	bool bSelected;
	struct Timespan TimeTaken;
};

struct FFortPlayerSurveyCMSSurveyKey
{
	struct FString ID;
};

struct FFortPlayerSurveyMcpDataSurveyMetadata
{
	int NumTimesCompleted;
	struct DateTime LastTimeCompleted;
};

struct FFortPlayerSurveyMcpDataRoot
{
	struct FortPlayerSurveyMcpDataSurveyMetadata AllSurveysMetadata;
	Unknown MetaData;
};

struct FVehicleTrickLocalAxisRotInfo
{
	float Angle;
	int AccumulatedHalfSpinCount;
	float AccumulatedAngle;
	float AngleAtFurthestExtent;
	int TrickOrder;
	int Points;
};

struct FQueryName
{
	struct FName QueryName;
};

struct FUserOptionValueDescription
{
	struct FString Value;
	struct FText Description;
};

struct FFortQuestIndicatorCustomCategory
{
	struct GameplayTag CategoryTag;
	struct FText DisplayName;
	struct FText AllChallengesText;
	int Priority;
};

struct FFeatSeriesObjectiveStep
{
	int Count;
};

struct FPartyAssistObjectiveData
{
	struct FName BackendName;
	int Count;
	bool bCompleted;
};

struct FPartyAssistQuestData
{
	class UClass* AssistedQuestDef;
	class UClass* AssistedPlayer;
	int CurrentQuestStage;
	bool QuestCompleted;
	TArray<struct PartyAssistObjectiveData> Objectives;
};

struct FFortQuestMapCosmetic
{
	ECosmeticType CosmeticType;
	struct SlateBrush CosmeticBrush;
	class UClass* WidgetClass;
	TArray<class UClass*> CosmeticDataList;
	struct Vector2D CosmeticPosition;
	struct WidgetTransform CosmeticTransform;
};

struct FFortQuestMapNode
{
	struct PrimaryAssetId QuestItemDefinitionId;
	class UClass* QuestItemDefinition;
	EFortQuestMapNodeType NodeType;
	EFortQuestMapNodeLabelPosition LabelPosition;
	bool UseHighContrastMode;
};

struct FFortChallengeBundleInfoLockedReason
{
	EFortChallengeBundleInfoLockedReasonCode ReasonCode;
	struct FString EventName;
	int RequiredTier;
	struct Timespan UnlockTimespanAfterStart;
};

struct FFortCreateItemDetail
{
	struct FString TemplateId;
	int Quantity;
};

struct FFortQuestPoolStats_PerPool
{
	struct FString PoolName;
	struct DateTime NextRefresh;
	int RerollsRemaining;
};

struct FFortQuestPoolStats
{
	TArray<struct FortQuestPoolStats_PerPool> PoolStats;
};

struct FFortQuestManagerAttributes
{
	struct DateTime DailyLoginInterval;
	int DailyQuestRerolls;
	struct FortQuestPoolStats QuestPoolStats;
};

struct FFortTransientQuestGrant
{
	struct FString TemplateId;
	TArray<struct FortQuestObjectiveCompletion> Objectives;
	struct FString ExpirationTime;
	struct FString CreationTime;
};

struct FSecondaryXpGained
{
	struct FString Type;
	int secondaryXp;
};

struct FQuickBarAndSlot
{
	EFortQuickBars QuickBarType;
	int QuickBarSlot;
};

struct FReflectedEngineVersion
{
	int Major;
	int Minor;
	int Patch;
	int Changelist;
	struct FString Branch;
};

struct FFortRecordVersion
{
	int DataVersion;
	int PackageFileVersion;
	struct ReflectedEngineVersion EngineVersion;
};

struct FFortReplayEvent
{
};

struct FReplayKillSummary
{
	struct UniqueNetIdRepl Victim;
	struct Transform VictimLocation;
	struct UniqueNetIdRepl Instigator;
	struct Transform InstigatorLocation;
	float Timestamp;
	bool bIsDownButNotOut;
	EDeathCause DeathCause;
};

struct FGameStateInformation
{
	struct GameMemberInfoArray PlayerInfos;
	bool bIsTeamBasedGame;
};

struct FFortReplayMetadata
{
	struct FString ReplayName;
	struct FString PlaylistName;
	float ReplayLength;
	struct UniqueNetIdRepl RecordingPlayer;
	Unknown RelevancyList;
	Unknown PlayerIds;
	Unknown PlayerMatchReportToName;
	Unknown PlayerIdToMatchReport;
	Unknown PlayerIdToWorldIdentifier;
	TArray<struct ReplayKillSummary> KillSummaries;
	Unknown PlayerFinalRankings;
	struct GameStateInformation GameInformation;
	TArray<struct AthenaTravelLogEntry> WorldLogEntries;
};

struct FBaseReplayEventInfo
{
	float EventTime;
	int VersionNumber;
};

struct FMinimalHighlightShot
{
	int VersionNumber;
	float StartTimestamp;
	float ShotDuration;
	struct UniqueNetIdRepl PlayerID;
	struct FString PlayerName;
	Unknown FeatureScores;
	int NumEliminations;
	EHighlightSignificances ClipSignificance;
	float FinalScore;
};

struct FReplayDataActorPosition
{
	struct Vector_NetQuantize Position;
};

struct FReplayDataMoveSnapshotContainer
{
};

struct FReplayDataMoveSnapshot
{
	struct Vector_NetQuantize Position;
	EFortMovementStyle MovementStyle;
	uint16_t WorldTime;
};

struct FRuntimeOptionPlaygroundKnobOverride
{
	struct FString KnobID;
	bool bEnabled;
	struct FString OverrideDefault;
};

struct FSanitizedEntry
{
};

struct FFortDestroyedActorRecord
{
	struct Guid ActorGuid;
	class UClass* ActorClass;
	struct Transform ActorTransform;
};

struct FFortBuildingActorArray
{
	TArray<struct FortDestroyedActorRecord> ActorRecords;
};

struct FMMAttemptState
{
	int BestSessionIdx;
	int NumSearchResults;
	EMatchmakingState State;
	EPartyReservationResult LastBeaconResponse;
};

struct FFortTimedKeysState
{
	TArray<struct FString> K;
};

struct FFortMatchmakingRegionState
{
	Unknown EventFlagsForcedOn;
	Unknown EventFlagsForcedOff;
};

struct FFortMatchmakingEventsState
{
	Unknown Region;
};

struct FFortRotationalContentEventsState
{
	Unknown ActiveStorefronts;
	Unknown ActiveEventFlags;
	Unknown EventNamedWeights;
	Unknown ExpirationTimes;
};

struct FFortStandaloneStoreState
{
	struct DateTime StoreEnd;
};

struct FFortCommunityVotesState
{
	struct FString ElectionId;
	TArray<struct CatalogOffer> Candidates;
	struct DateTime ElectionEnds;
	int NumWinners;
	int WinnerStateHours;
};

struct FFortFeaturedIslandsState
{
	TArray<struct FString> IslandCodes;
	Unknown PlaylistCuratedContent;
	Unknown PlaylistCuratedHub;
	TArray<struct FString> IslandTemplates;
};

struct FSpatialGamePlayActorLoadingStateUpdatedContext
{
	class UClass* SpatialGameplayInterfaceActor;
	ESpatialLoadingState State;
};

struct FPlayerExitSpatialActorContext
{
	class UClass* SpatialGameplayInterfaceActor;
	class UClass* ExitingPlayerState;
};

struct FPlayerEnterSpatialActorContext
{
	class UClass* SpatialGameplayInterfaceActor;
	class UClass* EnteringPlayerState;
};

struct FSpatialGameplayInterfaceContext
{
	class UClass* SpatialGameplayInterfaceActor;
};

struct FFortChaseCameraHelper
{
	struct Transform PivotToViewTarget;
	struct Transform PivotToViewTarget_Crouching;
	struct Transform MinCameraToPivot;
	struct Transform MaxCameraToPivot;
	float CameraToPivotAlphaInterpSpeed;
	float CameraCollisionSphereRadius;
	float PivotLocationInterpSpeed;
	float PivotRotationInterpSpeed;
	TArray<class UClass*> IgnoreActors;
	EThirdPersonAutoFollowMode AutoFollowMode;
	float CameraTruckRate;
	float AutoFollowPitch;
	float LazyAutoFollowPitchMin;
	float LazyAutoFollowPitchMax;
};

struct FFortSphericalVehicleAsyncDef
{
};

struct FFortLinearSpline
{
};

struct FSplineWaterAudioZone
{
	struct Vector Position;
	float Radius;
};

struct FFortTaggedActorOctreeFilter
{
	struct BoxSphereBounds Bounds;
	float MinDistanceFromBoundsCenter;
	TArray<class UClass*> OptionalSubclasses;
	TArray<struct FortFinderProperty> RequiredProperties;
	struct GameplayTagContainer TagsToLookFor;
	bool bHasAllTags;
};

struct FSmokeTestResult
{
	struct FString TestStep;
	struct FString TestSummary;
	bool bWasExecuted;
	bool bPassed;
	struct FString ResultMessage;
};

struct FScreenshotTransformPair
{
	struct Transform Left;
	struct Transform Right;
};

struct FCosmeticScreenshotTestConfig
{
	int ScreenshotResX;
	int ScreenshotResY;
	Unknown CosmeticIgnores;
	Unknown VariantIgnores;
	Unknown CosmeticWhitelist;
	Unknown IndividualOverrides;
};

struct FFortFoundQuestMissions
{
	struct FString TheaterId;
	bool bIsValidForAllPlayableMissions;
	TArray<struct FortAvailableMissionData> LinkedMissions;
	TArray<struct FortAvailableMissionData> TagMatchingMissions;
	TArray<struct FortAvailableMissionData> FallbackMatchingMissions;
};

struct FThreatGridIndex
{
	int X;
	int Y;
};

struct FTimeOfDayEditorViewSettings
{
};

struct FFortTooltipValueData
{
	struct FText DisplayName;
	struct FText FormattedValue;
	struct FText ExplanationText;
	float Value;
	struct GameplayTagContainer StateTags;
};

struct FFortTouchAimAssist
{
	struct FortAimAssist2D_InputParams InputParams;
	struct FortAimAssist2D_OwnerInfo OwnerInfo;
	TArray<struct FortAimAssist2D_Target> TargetCache0;
	TArray<struct FortAimAssist2D_Target> TargetCache1;
	class UClass* AutoFireTargetActor;
};

struct FTournamentPayoutData
{
	EPayoutRewardType RewardType;
	struct FString Value;
	int Quantity;
	int TeamSize;
};

struct FTournamentPayoutThresholdData
{
	EPayoutScoringType ScoringType;
	double Threshold;
	TArray<struct TournamentPayoutData> PayoutData;
};

struct FTowhookAttachState
{
};

struct FTrackPieceConfig
{
	ETrackPieceType Type;
	struct Rotator Rotation;
	struct Vector Scale;
};

struct FTrackSplineConfig
{
	bool bUseSpline;
	ETrackDirection Start;
	ETrackDirection End;
};

struct FTrackSwitchStateConfig
{
	struct TrackPieceConfig TrackPiece;
	struct TrackSplineConfig SplineConfig1;
	struct TrackSplineConfig SplineConfig2;
};

struct FTrackConfiguration
{
	TArray<bool> NeighborsByDirection;
	TArray<struct TrackSwitchStateConfig> SwitchStates;
	struct Rotator SwitchRotation;
	struct Vector SwitchOffset;
};

struct FTransactionalAnalyticEvent
{
};

struct FTransformGeneratorResult
{
	struct Transform OutputTransform;
};

struct FTransformGeneratorRequest
{
	struct Transform OriginTransform;
	class UClass* World;
};

struct FNameStringGameplayMessage
{
	struct FName Name;
	struct FString String;
};

struct FAmmoItemState
{
	class UClass* AmmoItemDefintion;
	int AmmoLoadedCount;
	int AmmoMaxCount;
};

struct FItemGuidAndCount
{
	int Count;
	struct Guid ItemGuid;
};

struct FFortGameHotfixUpdateResponse
{
	struct FString Status;
};

struct FFortGameAnalyticsSessionResponse
{
	struct FString SessionId;
	struct FString AppID;
	struct FString AppVersion;
	struct FString DeviceType;
	struct FString DeviceModel;
	struct FString OS;
};

struct FCreativeDeviceMeshSettings
{
	struct TSoftClassPtr<UObject> Mesh;
	struct Vector Scale;
	struct GameplayTag StatTag;
	struct TSoftClassPtr<UObject> Material;
};

struct FRemoteViewRotSnapshot
{
};

struct FFortGameFeatureSizeDetailed
{
	uint64_t DownloadSize;
	uint64_t InstallSize;
	uint64_t InstallOverheadSize;
	uint64_t FreeSpace;
};

struct FFortGameFeatureSize
{
	uint64_t DownloadSize;
	uint64_t InstallSize;
	uint64_t FreeSpace;
};

struct FCombinedFeatureProgressSummary
{
	EFortGameFeature GameFeature;
	struct FString PrimaryTextKey;
	struct FString LocalizedPrimaryText;
	struct FString SecondaryTextKey;
	struct FString LocalizedSecondaryText;
	float CombinedProgress;
	bool bIsPaused;
	bool bCanBePaused;
	bool bIsFinished;
};

struct FFortGameFeatureStatus
{
	EFortGameFeature Feature;
	EFortGameFeatureState CurrentState;
	EFortGameFeatureState RequestedState;
	struct FString ErrorCode;
	struct FText ErrorText;
	bool bIsUsingBackgroundDownloads;
	bool bIsProgressPaused;
	struct FString PauseReason;
	struct CombinedFeatureProgressSummary ProgressSummary;
	Unknown BundleFullProgress;
	Unknown PreloadBundleFullProgress;
	bool IsActive;
	bool IsPendingActive;
	int BundlesToInstall;
	bool bEnoughFreeSpace;
};

struct FFortGameFeatureOptionalInstallStatus
{
	EFortGameFeature Feature;
	bool bContentReady;
	struct FString ErrorCode;
	struct FText ErrorText;
	bool bIsUsingBackgroundDownloads;
	bool bIsProgressPaused;
	struct FString PauseReason;
	struct CombinedFeatureProgressSummary ProgressSummary;
	int BundlesToInstall;
	bool bEnoughFreeSpace;
};

struct FFortGameFeatureStatusList
{
	struct CombinedFeatureProgressSummary ProgressSummary;
	TArray<struct FortGameFeatureStatus> Features;
	TArray<struct FortGameFeatureOptionalInstallStatus> FeatureOptionalInstalls;
	bool bHasNetworkConnection;
	bool bIsUsingCellularConnection;
	bool bAutoLaunchFullGame;
};

struct FFortGameFeatureResponse
{
	struct FString ErrorCode;
	struct FText ErrorText;
	EFortErrorSeverity ErrorSeverity;
};

struct FAthenaPickResult
{
	EAthenaPickerType PickType;
	class UClass* FoundBuildingActor;
	class UClass* FoundPlayer;
};

struct FDataTableRowHandleQuantityPair
{
	struct DataTableRowHandle DataTableRowHandle;
	int Quantity;
};

struct FFortAlterationSlotStatus
{
	class UClass* Alteration;
	int MinRequiredLevel;
	EFortRarity MinHostItemRarity;
};

struct FFortClientEvent
{
	struct FName CategoryName;
	struct FName EventName;
	class UClass* EventSource;
	class UClass* EventFocus;
};

struct FFortCatalogMetaPreload
{
	TArray<struct TSoftClassPtr<UObject>> ChaseItems;
	struct TSoftClassPtr<UObject> PackDefinition;
};

struct FFortCatalogMeta
{
	TArray<class UClass*> ChaseItems;
	class UClass* PackDefinition;
};

struct FFortDayPhaseCallbackHandle
{
};

struct FFortSpecialEventEmoteData
{
	TArray<struct FName> BoundActions;
	TArray<class UClass*> Emotes;
};

struct FFortMultiSizeMargin
{
	struct Margin Margin_XXS;
	struct Margin Margin_XS;
	struct Margin Margin_S;
	struct Margin Margin_M;
	struct Margin Margin_L;
	struct Margin Margin_XL;
};

struct FFortMultiSizeFont
{
	struct SlateFontInfo Font_XXS;
	struct SlateFontInfo Font_XS;
	struct SlateFontInfo Font_S;
	struct SlateFontInfo Font_M;
	struct SlateFontInfo Font_L;
	struct SlateFontInfo Font_XL;
};

struct FPurchasedBattlePassInfo
{
	int Count;
	struct FString ID;
};

struct FFortUserCloudRequestPayload
{
	struct FortUserCloudRequestHandle RequestHandle;
	struct UniqueNetIdRepl UserNetId;
	struct FString Filename;
	EFortUserCloudRequestType RequestType;
	TArray<byte> DataBuffer;
};

struct FFortUserCloudRequest
{
	struct FortUserCloudRequestPayload RequestPayload;
	bool bNeedsFileEnumeration;
	bool bStartedProcessing;
};

struct FFortUserCloudRequestQueue
{
	bool bFreezeIncomingRequests;
	struct FortUserCloudRequestHandle FirstFrozenHandle;
	struct TimerHandle ProcessingTimerHandle;
	TArray<struct FortUserCloudRequest> RequestQueue;
};

struct FDampedSpringConfig
{
	float Stiffness;
	float Damping;
	float MaxAccel;
};

struct FVehicleGamepadLiftInputs
{
	float MoveUpPressTime;
	float MoveDownPressTime;
};

struct FFortAthenaVehicleSessionTelemetryInfo
{
};

struct FPendingSpawnLevelSaveRecord
{
};

struct FWeaponHudKeyActionVisibility
{
	int Index;
	bool bVisibility;
};

struct FFortPIEMissionOverrideData
{
};

struct FFortClientMarkerRequest
{
	int InstanceID;
	EFortWorldMarkerType MarkerType;
	struct Vector BasePosition;
	struct Vector BasePositionOffset;
	struct Vector WorldNormal;
	class UClass* MarkedActor;
	bool bIncludeSquad;
	bool bUseHoveredMarkerDetail;
};

struct FPlaysetPropSettings
{
};

struct FPlaysetSettings
{
};

struct FFortZoneStats
{
};

struct FContainerStatInfo
{
};

struct FEnemyNpcStatInfo
{
};

struct FDefenderNPCStatInfo
{
};

struct FTieredWaveCollectionLootSetData
{
	TArray<struct FName> StartOfCollectionItemTierGroups;
	TArray<struct FName> SuccessfulWaveItemTierGroups;
};

struct FTieredModifierSetData
{
	int WaveNumber;
	int ModifierDuration;
	struct FName ModifierLootTierGroup;
};

struct FTieredWaveSetCollectionData
{
	struct FText DefenseText;
	struct FText LevelText;
	struct FText WaveText;
	struct FText BreatherText;
	int MinLvl;
	int MaxLvl;
	struct FName BaseWaveLengthRowName;
	struct FName BaseNumOfKillsRowName;
	struct FName BaseNumOfKillPointsRowName;
	struct FName WaveSet;
};

struct FWorldItemAndMinMaxCount
{
	struct CurveTableRowHandle MinCurveTable;
	struct CurveTableRowHandle MaxCurveTable;
	class UClass* Item;
};

struct FFortRewardQuantityPair
{
	struct FString TemplateId;
	int Quantity;
};

struct FGenericMessage
{
};

struct FStructRecord
{
};

struct FSaveStructFile
{
	TArray<struct StructRecord> StructRecords;
};

struct FActorComponentRecord
{
	struct FName ComponentName;
	struct TSoftClassPtr<UObject> ComponentClass;
	TArray<byte> ComponentData;
	uint32_t DataHash;
};

struct FFortPlacementDistanceRequirements
{
	float DistanceRangeMin;
	float DistanceRangeMax;
};

struct FFortMissionEvent
{
	struct FName EventType;
	struct GameplayTagContainer ObjectiveHandle;
	class UClass* EventFocus;
	class UClass* EventContent;
	class UClass* EventInstigator;
	int GenericInt;
	float GenericFloat;
	struct FText GenericText;
	struct GameplayTagContainer GameplayTags;
	struct Guid MissionGuid;
	class UClass* Params;
};

struct FObjectTracker_Legacy
{
	class UClass* MetricConfiguration;
};

struct FVersionedCostOverride
{
	struct TSoftClassPtr<UObject> ClassPtr;
	ELevelSaveRecordVersion IntroducedVersion;
	ELevelSaveRecordVersion DeprecatedVersion;
	int OverrideCost;
};

struct FClipInfo
{
	float StartTimestamp;
	float Duration;
	struct UniqueNetIdRepl PlayerID;
	struct FString S3KeyName;
	struct FString S3Bucket;
	struct FString DatabaseId;
	struct FString DatabaseParitionKey;
	ESpectatorCameraType CameraType;
	EClipMessageSettings MessageSettings;
	ECameraShotNotificationTypes MessageNotificationType;
	float MessageDisplayTime;
	struct FString MessageString;
	Unknown FeatureScores;
	struct FString PlayerName;
	struct SavedSpectatorCameraShot CameraShotPrefab;
};

struct FPegasusJob
{
	struct FString ReplayId;
	bool bExportShotsIndividually;
	TArray<struct ClipInfo> ClipsToExport;
	bool bGenerateTimelineEventData;
	bool bJobAllowsFailure;
};

struct FPegasusJobMeta
{
	struct FString PGS_VideoJobId;
	TArray<struct FString> PGS_AdditionalTags;
	struct FString PGS_RenderJobId;
	int PGS_RenderJobCreationEpoch;
};

struct FBackendJobsPayload
{
	TArray<struct PegasusJob> JobsToRun;
	struct PegasusJobMeta PayloadMeta;
};

struct FBackendExportedClipInfo
{
	struct FString PGS_DatabaseId;
	struct FString PGS_DatabaseIdPartitionKey;
	struct FString PGS_S3Bucket;
	struct FString PGS_S3key;
	float PGS_Duration;
};

struct FPegasusAdditionalTagInfo
{
	float PGS_ScalarValue;
	struct FString PGS_StringValue;
	struct FString PGS_TagKeyContext;
};

struct FPegasusTimelineEventHit
{
	int PGS_HitCount;
	float PGS_ScalarValue;
	float PGS_ReplayStartTimeStamp;
	float PGS_ReplayEndTimeStamp;
	Unknown PGS_AdditionalTags;
	float PGS_ClipRelativeStartTime;
	float PGS_ClipRelativeEndTime;
};

struct FPegasusTimelineEvent
{
	struct FString PGS_TagName;
	bool PGS_IsScalarValueRelevant;
	struct FString PGS_ScalarValueDescription;
	EPegasusTimelineCategories PGS_Category;
	TArray<struct PegasusTimelineEventHit> EventHits;
};

struct FBackendExportedTimelineEvents
{
	struct FString PGS_DatabaseId;
	struct FString PGS_DatabaseIdPartitionKey;
	TArray<struct PegasusTimelineEvent> PGS_TimelineEvents;
};

struct FDriverInstanceInfo
{
	struct FString PGS_JobsEndpoint;
	struct FString PGS_LocalInstanceId;
	struct FString PGS_CloudInstanceId;
	struct FString PGS_ASGName;
	struct FString PGS_InstanceType;
	struct FString PGS_CloudOperatingRegion;
};

struct FBackendJobCompletePayload
{
	struct FString PGS_RenderJobMessageHandle;
	struct FString PGS_RenderQueueUrl;
	struct FString PGS_ASGName;
	TArray<struct BackendExportedClipInfo> PGS_ExportedVideos;
	TArray<struct BackendExportedTimelineEvents> PGS_ExportedTimelines;
	struct PegasusJobMeta PGS_PayloadMeta;
	Unknown PGS_VideoManagerStateDurations;
	struct DriverInstanceInfo PGS_InstanceInfo;
};

struct FBackendAlarmPayload
{
	struct FString PGS_Message;
	struct FString PGS_ErrorCode;
	struct BackendJobsPayload PGS_ReceivedJobPayload;
	struct FString PGS_RenderQueueUrl;
	struct FString PGS_ASGName;
	struct BackendJobCompletePayload PGS_JobProgress;
	uint32_t PGS_SQSRecievedCount;
	bool PGS_bIsVideoManagerFinished;
	bool PGS_bIsShuttingDown;
	int PGS_WastedSeconds;
	struct DriverInstanceInfo PGS_InstanceInfo;
};

struct FCachedPlayerReportingMatchInfoAnalytics
{
	struct FString GameSessionId;
	struct FName PlaylistName;
	struct FString TournamentId;
	struct FString EventWindowId;
};

struct FApproachingActorContext
{
	TArray<struct Vector> ApproachingActorLocations;
};

struct FActorTrackerBaseContext
{
	class UClass* SpatialActorTrackerComponent;
};

struct FPrimitiveCrosstalkNameArray
{
	TArray<struct FName> SupportedFunctions;
};

struct FCreativePerfCost
{
};

struct FAnimClassStats
{
	struct FString AnimClassName;
	Unknown StatValues;
};

struct FWatchedReplayShotInfo
{
	int ShotIndex;
	TArray<struct PegasusTimelineEvent> TimelineEvents;
};

struct FVideoManagerExtractionJob
{
};

struct FSolarisPrototype
{
};

struct FRelevantPawnArray
{
	TArray<class UClass*> PlayerPawns;
};

struct FSpecialActorSpawnData
{
	class UClass* ActorClass;
	struct Transform Transform;
	class UClass* Owner;
};

struct FDevHeroClassInfo
{
	struct FString Name;
	int Level;
};

struct FBaseSample
{
	float Timestamp;
	struct Vector Location;
};

struct FGameLogPlayerSampleBase
{
	struct UniqueNetIdRepl PlayerID;
};

struct FUnicornDancePartyInfo
{
	float StartTimestamp;
	float EndTimestamp;
	int PeakMembers;
	float PeakStartTimestamp;
	float PeakEndTimestamp;
	bool bEndsBecauseOfUs;
	bool bIsMovingEmote;
	Unknown LatestMembers;
};

struct FGameLogStream
{
	TArray<struct DeathEvent> DeathEvents;
	TArray<struct GameLogPawnSample> PawnSamples;
	TArray<struct GameLogBuildSample> BuildEvents;
	TArray<struct StormSample> StormSamples;
	TArray<struct GameLogPlayerMetaSample> PlayerMetaInfoSamples;
	TArray<struct GameLogDancePartySample> PlayerDancePartySamples;
};

struct FHighlightsPayloadMeta
{
	struct FString UCRN_ReplayId;
	struct FString UCRN_BuildVersion;
	struct FString UCRN_McpDeployment;
	byte UCRN_PayloadVersion;
	struct DateTime UCRN_GenerationDate;
	struct FString UCRN_McpBackendEnvironment;
	struct FString UCRN_McpAppName;
	struct FString UCRN_McpGameBackendName;
	struct FString UCRN_PlaylistName;
	bool UCRN_bIsCustomMatch;
};

struct FCosmeticLoadoutPayload
{
	Unknown UCRN_CharacterVarients;
};

struct FHighlightReelPlayerInfoPayload
{
	struct CosmeticLoadoutPayload UCRN_CosmeticInfo;
};

struct FUnicornSocialMetaPayload
{
	Unknown UCRN_FeatureScores;
	Unknown UCRN_FriendsInvolved;
	float UCRN_SocialScore;
};

struct FVictimGameplayInfoPayload
{
	Unknown UCRN_DeathCauses;
	Unknown UCRN_VictimTags;
	Unknown UCRN_KillerTags;
	Unknown UCRN_DeathTags;
};

struct FHighlightClipPayload
{
	int UCRN_HighlightId;
	float UCRN_StartTimestamp;
	float UCRN_Duration;
	Unknown UCRN_GameplayFeatures;
	float UCRN_GameplayScore;
	struct UnicornSocialMetaPayload UCRN_SocialMeta;
	float UCRN_NetworkingFidelityAverage;
	TArray<struct FString> UCRN_POITags;
	struct VictimGameplayInfoPayload UCRN_FollowingPlayerKill_VictimGameplayInfo;
	struct VictimGameplayInfoPayload UCRN_FollowingPlayerDeath_VictimGameplayInfo;
	Unknown UCRN_UnicornTags;
};

struct FHighlightReelPayload
{
	struct UniqueNetIdRepl UCRN_PlayerId;
	struct FString UCRN_PlayerName;
	int UCRN_MMR;
	struct HighlightReelPlayerInfoPayload UCRN_PlayerInfo;
	EHighlightReelIds UCRN_HighlightReelId;
	TArray<struct HighlightClipPayload> UCRN_Clips;
};

struct FHighlightsPayload
{
	struct HighlightsPayloadMeta UCRN_Meta;
	TArray<struct HighlightReelPayload> UCRN_HighlightReels;
};

struct FPrimaryContentSetup
{
	bool ShowBottomBar;
};

struct FFortPrioritizedWidgetData
{
	EFortPrioritizedWidgetPriority Priority;
	EFortPrioritizedWidgetContestedBehavior WhenContesting;
};

struct FMutatorAddedWidgets
{
};

struct FTimerDisplayData
{
	struct LinearColor BrushColor;
	struct FText TimeRemainingDisplayText;
	struct LinearColor TextColor;
};

struct FRewardCategoryTabData
{
	class UClass* TabDisplayAsset;
	struct FText DisplayName;
	struct GameplayTag IncludeTag;
	struct GameplayTag ExcludeTag;
	bool bDefaultEntry;
};

struct FAthenaChallengeListVisualOptions
{
	bool bHideHeaders;
	bool bHideCompletionRewards;
	bool bHideQuestRewards;
	bool bUseCompactActionInfo;
	bool bHideLockedQuests;
	bool bHideCompletedQuests;
	bool bShowOnlyCurentBundleLevelChallenges;
	bool bSortCompletedToEnd;
	int PreviewBundleLevel;
};

struct FAthenaCollectionScreenContainerTabInfo
{
	struct FText TabTitle;
	struct TSoftClassPtr<UObject> TabIcon;
	struct TSoftClassPtr<UObject> TabClass;
};

struct FFortTabButtonLabelInfo
{
	struct FText DisplayName;
	struct SlateBrush IconBrush;
};

struct FAthenaCustomizationParams
{
	EAthenaCustomizationCategory Category;
	int CategorySubslotIndex;
	struct FText CategoryDisplayName;
	struct FText ItemDisplayTypeName;
	bool bAllowUnownedItems;
	bool bOneItemPerSlot;
	class UClass* OverrideSlotImage;
};

struct FAthenaWinnerInfo
{
	struct FString BigNameWinnerName;
	TArray<struct FString> WinnerNames;
};

struct FFortItemFilterDefinition
{
	TArray<EFortItemType> ItemTypeFilters;
	bool bRequiresItemDetails;
};

struct FAthenaItemShopReloadMtxInfo
{
	TArray<struct FName> StaticStorefrontNames;
	TArray<struct FName> DynamicStorefrontNames;
	struct FName IncrementalStorefrontName;
	struct FString DynamicOtherOptionOfferId;
	int MaxMtxQuantityToShowDynamicReloadMtx;
	struct TSoftClassPtr<UObject> StaticReloadMtxScreenClass;
	struct TSoftClassPtr<UObject> DynamicReloadMtxScreenClass;
	struct TSoftClassPtr<UObject> ReloadMtxIntroModalPopupClass;
};

struct FUnlockableVariantPreviewInfo
{
	bool bIsValid;
	int SetNumber;
	int SetCount;
	struct FText UnlockCondition;
};

struct FPlaylistFilter
{
	class UClass* PlaylistsData;
};

struct FLevelEffectsData
{
	int MinPlayerLevel;
	struct LinearColor ContentColor;
	struct LinearColor OutlineColor;
	struct TSoftClassPtr<UObject> FlameDisplayObject;
};

struct FAthenaMapScreenContainerTabInfo
{
	struct FText TabTitle;
	struct TSoftClassPtr<UObject> TabIcon;
	struct TSoftClassPtr<UObject> TabClass;
	bool bDefaultFrontendActiveTab;
	bool bDefaultInGameActiveTab;
	EAtheaMapTabType TabType;
};

struct FFortPostGameScreenConfig
{
	EPostGameClickCatcherMode ClickCatcherState;
	EPostGameHUDMode PostGameHUDMode;
	bool bSohwCinematicBars;
	bool bShowPlacementOverlay;
};

struct FAthenaTeamDisplayInfo
{
	struct TSoftClassPtr<UObject> Icon;
	struct TSoftClassPtr<UObject> Ribbon;
	struct FText Name;
	struct LinearColor TextColor;
	struct LinearColor PedestalColor;
};

struct FMarkerLargeIndicatorType
{
	class UClass* MarkerMaterial;
	struct Vector2D ImageSize;
};

struct FFortTopBarTabButtonInfo
{
	struct FName TabId;
	struct FString CalendarEventName;
	struct FText DisplayName;
	struct SlateBrush IconBrush;
	EFortUIFeature LinkedUIFeature;
	EFortBangType BangType;
	bool bForceImage;
	bool bPrimaryPlayerOnly;
	bool bInteractAnalytic;
	struct TSoftClassPtr<UObject> OverrideTabWidget;
};

struct FVideoWidgetConfig
{
	class UClass* StreamingMediaSource;
	struct FName VideoString;
	struct FName VideoDisplayDataID;
	struct FName FallbackVideoID;
	bool bCheckAutoPlay;
	bool bForceAutoPlay;
	bool bStreamingEnabled;
	struct FString VideoUID;
	bool bShouldBeModal;
	bool bUseSkipBoundaries;
	bool bHoldToSkip;
	bool bFadeFromWhite;
	bool bAllowSkipping;
	struct FString mimetype;
	struct FString VideoURL;
	bool bEnableLooping;
	bool bAutoClose;
	bool bHideControls;
	bool bHideBackground;
	bool bStartMTTransparent;
	class UClass* SoundMixOverride;
	bool bOverrideSoundMix;
};

struct FAthenaVariantFilterTabInfo
{
	struct TSoftClassPtr<UObject> Icon;
	struct FText Label;
	struct GameplayTagContainer IncludedVariantChannels;
	float ZoomLevel;
	bool bDisableScrollBox;
};

struct FAthenaVariantFilterTabInfo_Primary
{
	struct AthenaVariantFilterTabInfo PrimaryTab;
	TArray<struct AthenaVariantFilterTabInfo> SubTabs;
};

struct FAthenaVariantFilterSet
{
	TArray<struct AthenaVariantFilterTabInfo_Primary> PrimaryTabs;
};

struct FBarrierObjectState
{
	class UClass* ObjectiveActor;
	byte TeamNum;
	EBarrierFoodTeam FoodTeam;
	EBarrierObjectiveDamageState DamageState;
};

struct FTrackCategoryUI
{
	struct TrackCategory TrackData;
	int LastSelectedIndex;
};

struct FBattlePassPageData
{
};

struct FBattlePassCharacterPreviewDisplayData
{
	class UClass* CharacterItemDef;
	class UClass* PreviewMaterial;
};

struct FPreviewRewardData
{
};

struct FPreviewUnlockRewardData
{
	int Currency;
	int NumAdditionalRewards;
	int TotalRewards;
	int NumCharacters;
	TArray<struct PreviewRewardData> Characters;
	TArray<struct PreviewRewardData> AdditionalRewards;
};

struct FPreviewUnlockData
{
	struct PreviewUnlockRewardData InstantRewards;
	struct PreviewUnlockRewardData EarnedRewards;
	struct FText ChapterNumberText;
	struct FText SeasonNumberText;
	struct FText SummaryText;
	bool bPaysForSelf;
};

struct FContentPushState
{
	bool bHideHeader;
	bool bHideFooter;
	bool bHideChatWidget;
};

struct FShowdownTournamentData
{
	struct FString Tournament_Display_Id;
	struct FText Title_Line_1;
	struct FText Title_Line_2;
	struct FText Schedule_Info;
	struct FString Poster_Front_Image;
	struct FString Poster_Back_Image;
	struct FText Flavor_Description;
	struct FText Details_Description;
	struct FText Short_Format_Title;
	struct FText Long_Format_Title;
	struct FText Background_Title;
	int Pin_Score_Requirement;
	struct FText Pin_Earned_Text;
	struct LinearColor Base_Color;
	struct LinearColor Primary_Color;
	struct LinearColor Secondary_Color;
	struct LinearColor Highlight_Color;
	struct LinearColor Title_Color;
	struct LinearColor Shadow_Color;
	struct LinearColor Background_Left_Color;
	struct LinearColor Background_Right_Color;
	struct LinearColor Background_Text_Color;
	struct LinearColor Poster_Fade_Color;
	struct FString Playlist_Tile_Image;
	struct FString Loading_Screen_Image;
	struct FString Style_Info_Id;
	struct FString Alert_Text;
	EFortTournamentAlertType AlertType;
};

struct FShowdownTournamentJsonObject
{
	TArray<struct ShowdownTournamentData> Tournaments;
};

struct FShowdownTournamentSource
{
	struct ShowdownTournamentJsonObject Tournament_Info;
	struct FString _title;
	struct FString _locale;
};

struct FKoreanCafeData
{
	struct GameplayTag Korean_Cafe;
	struct FText Korean_Cafe_Header;
	struct FText Korean_Cafe_Description;
};

struct FKoreanCafeJsonObject
{
	TArray<struct KoreanCafeData> Cafes;
};

struct FKoreanCafeSource
{
	struct FString _title;
	struct FString __locale;
	struct KoreanCafeJsonObject Cafe_Info;
};

struct FSubgameDisplayData
{
	struct FText Title;
	struct FText Description;
	struct FText SpecialMessage;
	struct FText StandardMessageLine1;
	struct FText StandardMessageLine2;
	struct FString Image;
	struct LinearColor Color;
};

struct FSubgameScreenSource
{
	struct FString _title;
	struct FString __locale;
	struct SubgameDisplayData Creative;
	struct SubgameDisplayData SaveTheWorld;
	struct SubgameDisplayData BattleRoyale;
};

struct FCreativeAdData
{
	struct FString Header;
	struct FString Sub_Header;
	struct FString Description;
	struct FString Creator_Name;
	struct FString Island_Code;
	EFortCreativeAdType Ad_Type;
	EFortCreativeAdColorPreset Ad_Color_Preset;
	struct FString Image;
};

struct FCreativeAdJsonObject
{
	TArray<struct CreativeAdData> Ads;
};

struct FCreativeAdSource
{
	struct FString _title;
	struct FString __locale;
	struct CreativeAdJsonObject Ad_Info;
};

struct FCmsJsonMessages
{
	struct FString _title;
	struct FString _locale;
	struct ShowdownTournamentSource TournamentInformation;
	struct KoreanCafeSource KoreanCafe;
	struct SubgameScreenSource SubGameInfo;
	struct CreativeAdSource CreativeAds;
};

struct FCobaltPlayerPortraitIndexInfo
{
	TArray<int> PortraitIndexArray;
};

struct FSurvivalObjectiveText
{
	TArray<struct FString> SafezoneStateText;
};

struct FDiscoCaptureUIData
{
	EDiscoCaptureUIState CurrDisplayState;
	class UClass* CapturePoint;
	class UClass* CurrPawn;
	class UClass* CurrMID;
	float FillAmount;
	struct FText DisplayText;
};

struct FDiscoCaptureIconData
{
	EDiscoCaptureIconState CurrIconState;
	EDiscoCaptureProgressState CurrProgressState;
	float CurrCapturePercent;
	class UClass* CapturePoint;
};

struct FQuestRecapData
{
	Unknown QuestItem;
	int LastKnownMcpValue;
	int AchivedCount;
	int RequiredCount;
};

struct FFortAthenaTutorialHighlightInfo
{
	struct GameplayTag WidgetToHighlight;
	struct GameplayTag ItemHighlightTag;
	struct Margin HighlightMargin;
	struct TSoftClassPtr<UObject> HighlightWidgetOverride;
	bool bIsLegacyHighlight;
};

struct FFortAthenaTutorialScreenInfo
{
	EFortAthenaTutorialSubstep Substep;
	struct Vector2D TextPromptPosition;
	struct Vector2D TextPromptPosition_AthenaHUD;
	struct Anchors TextPromptAnchors;
	struct Anchors TextPromptAnchors_AthenaHUD;
	struct FText TextPromptText_Touch;
	struct FText TextPromptText_Gamepad;
	struct FText TextPromptText_TouchLegacy;
	struct DataTableRowHandle ActionWidget_DataTableRow;
	TArray<struct FName> KeybindWidget_NameArray;
	TArray<struct FName> KeybindWidget_GamepadNameArray;
	bool ForceSingleInputKeybind;
	TArray<struct FortAthenaTutorialHighlightInfo> HighlightInfo;
	bool DisplayNextButton;
	bool DisplayTextPromptTarget;
	EFortAthenaTutorialScreenExtraWidget ExtraWidget;
};

struct FFortAthenaTutorialStepInfo
{
	EFortAthenaTutorialStep TutorialStep;
	TArray<struct FortAthenaTutorialScreenInfo> StepScreenInfo;
};

struct FFortAthenaTutorialStandaloneStepInfo
{
	EFortAthenaTutorial_StandaloneStep StandaloneStep;
	struct FortAthenaTutorialScreenInfo StepScreenInfo;
};

struct FFortStateStyle
{
	struct FortMultiSizeBrush Brush;
	struct LinearColor PrimaryColor;
	struct LinearColor SecondaryColor;
};

struct FFortBattlePassVideoAnnotation
{
	float StartTime;
	float EndTime;
	struct Vector2D ScreenPosition;
	int EntryAngle;
	float EntryMagnitude;
	class UClass* RewardItemDefinition;
};

struct FFortSwipeDetector
{
	struct Vector2D SwipeThreshold;
};

struct FFortConfirmationButtonInfo
{
	class UClass* Button;
};

struct FCachedIslandsGroup
{
	TArray<class UClass*> IslandLinks;
};

struct FItemListCategoryArray
{
	TArray<struct FortItemEntry> ItemList;
	struct FText CategoryTitle;
};

struct FFortItemListOptionBucket
{
	int ItemIndex;
	TArray<class UClass*> ItemOptionData;
};

struct FRarityArray
{
	TArray<class UClass*> Items;
};

struct FFortDailyRewardsItemData
{
	class UClass* RewardItem;
	int RewardDay;
	bool IsCurrentReward;
	bool IsClaimed;
};

struct FFortDailyRewardsScheduleData
{
	struct FText ScheduleTitle;
	struct FText ScheduleDescription;
	struct FText ScheduleItemDescription;
	struct FText ScheduleEpicItemDescription;
	int WeekOffset;
	int RewardsGiven;
	int RewardsAllowed;
	TArray<struct FortDailyRewardsItemData> CalendarItems;
	TArray<struct FortDailyRewardsItemData> HighValueItems;
	bool ClaimedToday;
};

struct FFortPickerSubCategoryIdentifier
{
	struct GameplayTag Tag;
	struct FText Name;
};

struct FFortLeaderboardRequestIds
{
};

struct FExpeditionTabInfo
{
	struct FName TabNameID;
	struct FortTabButtonLabelInfo TabLabelInfo;
};

struct FFortFrontEndFeatureStruct
{
	EFortFrontEndFeatureState CurrentState;
	EFortFrontEndFeatureState ForcedState;
	EFortFrontEndFeatureStateReason ForcedStateReason;
	struct FScriptMulticastDelegate ChangeDelegate;
};

struct FCheatMenuEntry
{
	struct FString DevName;
	struct FString DisplayName;
	struct FString RichText;
	TArray<struct FString> ConsoleCommands;
};

struct FCheatMenuSection
{
	struct FString DevName;
	struct FString DisplayName;
	struct FString MinVersion;
	TArray<struct CheatMenuEntry> Entries;
};

struct FNotificationWidgetClass
{
	class UClass* NotificationWidgetClass;
	int InitialPoolSize;
	bool bCanIncreasePoolSizeAtRunTime;
	TArray<class UClass*> WidgetPool;
};

struct FGiftingErrorText
{
	EOfferPurchaseError GiftingError;
	struct FText ErrorTitle;
	struct FText ErrorDesc;
};

struct FFortUIFeatureStruct
{
	EFortUIFeatureState CurrentState;
	EFortUIFeatureState ForcedState;
	EFortUIFeatureStateReason ForcedStateReason;
	struct FScriptMulticastDelegate ChangeDelegate;
};

struct FFortInGamePerkDisplayData
{
	EFortHeroPerkDisplayType PerkDisplayType;
	int Index;
	int Row;
	int Column;
};

struct FContextPosition
{
	EContextPositionPlatform Platform;
	struct Vector2D Position;
	EHorizontalAlignment HorizontalAlignment;
	EVerticalAlignment VerticalAlignment;
};

struct FFortReceivedItemLootInfo
{
	struct FortItemHeaderInformation HeaderInformation;
	class UClass* GeneratedItemInstance;
	class UClass* ItemDef;
	struct FString TemplateId;
	int Quantity;
};

struct FItemPresentationRestrictionData
{
	struct FText HeaderText;
	struct TSoftClassPtr<UObject> Icon;
};

struct FMeasuredText
{
};

struct FOfferGroup
{
	struct FString Name;
	int MaxNumberToShow;
};

struct FFortMaterialProgressBarSectionStyle
{
	bool bGradientBar;
	struct FName BarParamName;
	struct FName BarColorOneParamName;
	struct FName BarColorTwoParamName;
	struct LinearColor BarColorOne;
	struct LinearColor BarColorTwo;
};

struct FFortMaterialProgressBarSectionInfo
{
	struct FortMaterialProgressBarSectionStyle SectionStyle;
	float Percent;
};

struct FIconPicker
{
	struct GameplayTagContainer Tags;
	class UClass* Sprite;
};

struct FTagVisibility
{
	struct GameplayTagContainer Tags;
	ETagComparisonType ComparisonType;
	ESlateVisibility Visibility;
};

struct FActionBindingIconPicker
{
	TArray<struct FName> ActionNames;
	class UClass* Sprite;
};

struct FActionBindingVisibility
{
	TArray<struct FName> ActionNames;
	EActionBindingComparisonType ComparisonType;
	ESlateVisibility Visibility;
};

struct FGameplayMessageVisibility
{
	struct EventMessageTag EventMessageTag;
	ESlateVisibility MessageReceivedVisibility;
	ESlateVisibility MessageStateClearedVisibility;
};

struct FFortMobileHUDContextLayout
{
	class UClass* HUDContextModel;
};

struct FFortMobileHUDLayoutProfile
{
	struct FText ProfileName;
	TArray<struct FortMobileHUDContextLayout> ContextLayouts;
};

struct FFortMobileContextFilter
{
	struct GameplayTagContainer ShownInContexts;
	struct GameplayTagContainer HiddenInContexts;
};

struct FFortMobileHUDContextModelExtension
{
	struct GameplayTag ContextLayoutTag;
	struct GameplayTagContainer UseableHUDWidgetTags;
};

struct FHUDLayoutToolV2_PopupContent
{
	EHUDLayoutToolPopupType PopupType;
	struct FText TitleText;
	struct FText DescriptionText;
	struct FText ButtonConfirmText;
	struct FText ButtonCancelText;
};

struct FHUDLayoutToolV2_ToastNotificationContent
{
	EHUDLayoutToolToasterType ToasterType;
	struct FText TitleText;
	struct FText DescriptionText;
};

struct FHUDWidgetBehaviorRegistryEntry
{
	struct GameplayTag HUDWidgetBehaviorTag;
	struct GameplayTag HUDWidgetOverrideTag;
	class UClass* HUDWidgetBehaviorClass;
};

struct FHUDWidgetRegistryEntry
{
	struct GameplayTag HUDWidgetTag;
	bool bIsMandatory;
	float MandatoryOnScreenPercent;
	int MaximumAmountOfInstances;
	int ZOrder;
	class UClass* HUDWidgetClass;
	class UClass* HUDWidgetProxyClass;
	class UClass* HUDWidgetPreviewClass;
	TArray<class UClass*> CustomPropertyModels;
};

struct FHUDWidgetBehaviorExtensionsRegistryEntry
{
	struct GameplayTag HUDWidgetBehaviorTag;
	TArray<class UClass*> HUDWidgetBehaviorExtensions;
};

struct FFortModalContainerSizeEntry
{
	float AbsoluteWidth;
	float TopPercent;
	float MiddlePercent;
	float BottomPercent;
	float VerticalPadding;
	float HorizontalPadding;
	float ContentPadding;
};

struct FFortMtxDetailsAttribute
{
	struct FText Name;
	struct FText Value;
};

struct FFortMtxGradient
{
	struct LinearColor Start;
	struct LinearColor Stop;
};

struct FFortUIPerkTier
{
	class UClass* HeroSpecialization;
	class UClass* CurrentHero;
	EFortItemTier Tier;
	bool bIsUpgrade;
	bool bIsEvolution;
};

struct FFortUIPerk
{
	struct FortSpecializationSlot SpecializationSlot;
	class UClass* Hero;
	EFortItemTier Tier;
	EFortSupportBonusType SupportBonusType;
	bool bIsTierPerk;
	bool bIsUpgrade;
	bool bIsEvolution;
	bool bIsEvolutionBranch;
};

struct FRadialOptionData
{
	struct FText Label;
	struct SlateBrush Brush;
	struct TSoftClassPtr<UObject> SoftIcon;
};

struct FTrackedTrapTimeAndCount
{
};

struct FFortPickerTemporaryWheelItem
{
	struct FText LabelOverride;
	class UClass* Item;
};

struct FFortPickerTemporaryWheel
{
	TArray<struct FortPickerTemporaryWheelItem> Items;
};

struct FFortPlayerSurveyAnswerWidgetDataTableEntry
{
	Unknown WidgetClassMap;
	class UClass* FallbackWidgetClass;
};

struct FFortPlayerSurveyButtonListMultipleSelectionAnswerWidgetBaseInternalButtonData
{
	class UClass* ButtonWidget;
};

struct FFortPlayerSurveyConditionsContextLegacy
{
	class UClass* Player;
	class UClass* Context;
};

struct FPlatformSupportDesc
{
	struct FText DisableDesc;
	EFortLoginAccountType AccountType;
};

struct FForcedIntroEntry
{
	struct FName EntryName;
	struct TSoftClassPtr<UObject> WidgetClass;
};

struct FFortSettingsFilterState
{
	bool bIncludeDisabled;
	bool bIncludeHidden;
	bool bIncludeResetable;
	bool bIncludeNestedPages;
	TArray<class UClass*> SettingRootList;
	TArray<class UClass*> SettingWhiteList;
};

struct FFortSettingClassExtensions
{
	TArray<struct TSoftClassPtr<UObject>> Extensions;
};

struct FFortSettingNameExtensions
{
	bool bIncludeClassDefaultExtensions;
	TArray<struct TSoftClassPtr<UObject>> Extensions;
};

struct FFortSimpleWidgetAnimation
{
	struct FName TargetName;
	class UClass* Translation;
	class UClass* Scale;
	class UClass* Alpha;
	bool bResetOnFinish;
	bool bShouldAnimateTranslation;
	bool bShouldAnimateScale;
	bool bShouldAnimateAlpha;
	class UClass* World;
};

struct FFortSimpleWidgetAnimations
{
	struct FName Name;
	bool bIsEnabled;
	TArray<struct FortSimpleWidgetAnimation> Targets;
	struct FScriptMulticastDelegate OnAnimationsFinished;
};

struct FPlatformPrefixIcon
{
	struct FString Platform;
	class UClass* PrefixIcon;
};

struct FPlatformPrefixIconList
{
	TArray<struct PlatformPrefixIcon> PlatformPrefixIcons;
};

struct FFortUserListHeaderInfo
{
};

struct FPotentialSpectatorTarget
{
	int Rank;
	Unknown PlayerState;
	bool bCurrentViewTarget;
};

struct FSquadSlotSortTypes
{
	TArray<ESquadSlotSortType> SortTypes;
};

struct FCardPackOffer
{
	struct FText Title;
	struct FText Description;
	int MtxPrice;
	ECatalogSaleType SaleType;
	struct FText SaleText;
	int Price;
	int RegularPrice;
	class UClass* CurrencyType;
	int QuantityRemaining;
	bool bTimedOffer;
};

struct FCard
{
	int QuantityReceived;
	class UClass* Item;
	EPauseType PauseType;
};

struct FOpenedCardPack
{
	class UClass* CardPackDefinition;
	int DisplayLevel;
};

struct FSubscriptionContentTabData
{
	struct FText TabName;
	ESubscriptionContentTab TabType;
	bool bDynamicTab;
};

struct FFortUISurvivorSquadStatMatch
{
	struct FortMultiSizeBrush Icons;
	struct FText MagnitudeText;
	struct FText AttributeDisplayName;
	int NumMembersMeetingCriteria;
	int NumMembersRequired;
	EFortUISurvivorSquadMatchType MatchType;
	EFortBuffState PreviewEffect;
};

struct FFortTabListRegistrationInfo
{
	struct FName TabNameID;
	bool bHidden;
	bool bAllowedInZone;
	struct FortTabButtonLabelInfo TabLabelInfo;
	class UClass* TabButtonType;
	class UClass* TabContentType;
	class UClass* CreatedTabContentWidget;
};

struct FCachedComponentMaterials
{
	TArray<class UClass*> OriginalMaterials;
};

struct FTouchInteractionIconOverride
{
	struct TSoftClassPtr<UObject> IconOverride;
	float IconScale;
};

struct FButtonInteractionPair
{
	class UClass* TrackedInteraction;
	class UClass* TrackedButton;
};

struct FStateWidgetEntry
{
	struct TSoftClassPtr<UObject> Class;
	EFortNamedBundle Bundle;
};

struct FFortItemCard_PowerRatingBlock_Configuration
{
	struct SlateBrush MoonbeamBorderBrush;
	struct Margin MoonbeamBorderExteriorPadding;
	struct Margin MoonbeamBorderPadding;
	struct Margin CustomRatingInternalPadding;
	struct Vector2D CustomRatingIconSize;
	struct TSoftClassPtr<UObject> CustomRatingTextStyle;
	struct Vector2D ComparisonIndicatorSize;
};

struct FFortItemCard_NameplateBorder_Configuration
{
	struct Margin Padding;
	struct SlateBrush Brush;
};

struct FFortItemCard_DefenderWeaponTypeIcon_Configuration
{
	struct Vector2D IconConstraints;
};

struct FFortItemCard_DetailAreaBorder_Configuration
{
	float MinimumHeight;
	struct Margin Padding;
};

struct FFortItemCard_TierMeter_Configuration
{
	struct Vector2D PipSize;
	float InterPipPadding;
};

struct FFortItemCard_XL_PersonnelAndSchematics_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_PowerRatingBlock_PersonnelAndSchematics_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_NameplateBorder_Configuration Nameplate;
	struct TSoftClassPtr<UObject> ItemNameTextStyle;
	struct Vector2D LeadSurvivorTypeIconSize;
	struct Vector2D FirstIconSlotSize;
	float PaddingBetweenIconSlots;
	struct Vector2D SecondIconSlotSize;
	struct FortItemCard_DefenderWeaponTypeIcon_Configuration DefenderWeaponTypeIcon;
	struct FortItemCard_DetailAreaBorder_Configuration DetailAreaBorder;
	float RarityNameTextLeftPadding;
	struct TSoftClassPtr<UObject> RarityNameTextStyle;
	float ClassIconImageLeftPadding;
	struct Vector2D ClassIconSize;
	float PaddingBetweenClassIconAndName;
	struct TSoftClassPtr<UObject> ClassNameTextStyle;
	float TierMeterLeftPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
	struct Margin IconSlotOverNameplatePadding;
	struct Vector2D IconSlotOverNameplateSize;
};

struct FFortItemCard_StackCountBlock_Configuration
{
	bool bShowShorthandStackCount;
	struct TSoftClassPtr<UObject> TextStyle;
};

struct FFortItemCard_DurabilityMeter_Configuration
{
	float MeterThickness;
	struct Margin MeterPadding;
};

struct FFortItemCard_XL_ItemInstance_Configuration
{
	struct Margin BackgroundPadding;
	struct Margin PowerRatingBlockPadding;
	struct FortItemCard_PowerRatingBlock_ItemInstance_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Margin TraitBoxPadding;
	struct Vector2D FirstTraitSize;
	float PaddingBetweenTraitIcons;
	struct Vector2D SecondTraitSize;
	struct Margin TierMeterPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
	struct FortItemCard_DurabilityMeter_Configuration DurabilityMeter;
};

struct FFortItemCard_XL_TransformKey_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Vector2D TransformKeyIconSize;
};

struct FFortItemCard_LevelMeter_Configuration
{
	float MeterThickness;
	struct Margin MeterPadding;
};

struct FFortItemCard_L_PersonnelAndSchematics_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_PowerRatingBlock_PersonnelAndSchematics_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_NameplateBorder_Configuration Nameplate;
	struct Vector2D ClassIconSize;
	struct Vector2D LeadSurvivorTypeIconSize;
	struct Vector2D FirstIconSlotSize;
	float PaddingBetweenIconSlots;
	struct Vector2D SecondIconSlotSize;
	struct FortItemCard_DefenderWeaponTypeIcon_Configuration DefenderWeaponTypeIcon;
	struct FortItemCard_DetailAreaBorder_Configuration DetailAreaBorder;
	struct FortItemCard_LevelMeter_Configuration LevelMeter;
	float TierMeterLeftPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
};

struct FFortItemCard_L_ItemInstance_Configuration
{
	struct Margin BackgroundPadding;
	struct Margin PowerRatingBlockPadding;
	struct FortItemCard_PowerRatingBlock_ItemInstance_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Margin TraitBoxPadding;
	struct Vector2D FirstTraitSize;
	float PaddingBetweenTraitIcons;
	struct Vector2D SecondTraitSize;
	struct Margin TierMeterPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
	struct FortItemCard_DurabilityMeter_Configuration DurabilityMeter;
};

struct FFortItemCard_L_TransformKey_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Vector2D TransformKeyIconSize;
};

struct FFortItemCard_M_PersonnelAndSchematics_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_PowerRatingBlock_PersonnelAndSchematics_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_NameplateBorder_Configuration Nameplate;
	struct Vector2D ClassIconSize;
	struct Vector2D LeadSurvivorTypeIconSize;
	struct Vector2D FirstIconSlotSize;
	float PaddingBetweenIconSlots;
	struct Vector2D SecondIconSlotSize;
	struct FortItemCard_DefenderWeaponTypeIcon_Configuration DefenderWeaponTypeIcon;
	struct FortItemCard_DetailAreaBorder_Configuration DetailAreaBorder;
	struct FortItemCard_LevelMeter_Configuration LevelMeter;
	float TierMeterLeftPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
};

struct FFortItemCard_M_ItemInstance_Configuration
{
	struct Margin BackgroundPadding;
	struct Margin PowerRatingBlockPadding;
	struct FortItemCard_PowerRatingBlock_ItemInstance_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Margin TraitBoxPadding;
	struct Vector2D FirstTraitSize;
	float PaddingBetweenTraitIcons;
	struct Vector2D SecondTraitSize;
	struct Margin TierMeterPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
	struct FortItemCard_DurabilityMeter_Configuration DurabilityMeter;
};

struct FFortItemCard_M_TransformKey_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Vector2D TransformKeyIconSize;
};

struct FFortItemCard_S_PersonnelAndSchematics_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_PowerRatingBlock_PersonnelAndSchematics_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_NameplateBorder_Configuration Nameplate;
	struct Vector2D ClassIconSize;
	struct Vector2D LeadSurvivorTypeIconSize;
	struct Vector2D FirstIconSlotSize;
	float PaddingBetweenIconSlots;
	struct Vector2D SecondIconSlotSize;
	struct FortItemCard_DefenderWeaponTypeIcon_Configuration DefenderWeaponTypeIcon;
	struct FortItemCard_DetailAreaBorder_Configuration DetailAreaBorder;
	struct FortItemCard_LevelMeter_Configuration LevelMeter;
	float TierMeterLeftPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
};

struct FFortItemCard_S_ItemInstance_Configuration
{
	struct Margin BackgroundPadding;
	struct Margin PowerRatingBlockPadding;
	struct FortItemCard_PowerRatingBlock_ItemInstance_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Margin TraitBoxPadding;
	struct Vector2D FirstTraitSize;
	struct Margin TierMeterPadding;
	struct FortItemCard_TierMeter_Configuration TierMeter;
	struct FortItemCard_DurabilityMeter_Configuration DurabilityMeter;
};

struct FFortItemCard_S_TransformKey_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Vector2D TransformKeyIconSize;
};

struct FFortItemCard_XS_PersonnelAndSchematics_Configuration
{
	struct Margin BackgroundPadding;
	struct Vector2D BookmarkImageSize;
	struct FortItemCard_DetailAreaBorder_Configuration DetailAreaBorder;
	struct Vector2D AvailableUpgradeIconSize;
};

struct FFortItemCard_XS_ItemInstance_Configuration
{
	struct Margin BackgroundPadding;
	struct Margin PowerRatingBlockPadding;
	struct FortItemCard_PowerRatingBlock_ItemInstance_Configuration PowerRatingBlock;
	struct Vector2D BookmarkImageSize;
	struct Margin TraitBoxPadding;
	struct Vector2D FirstTraitSize;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct FortItemCard_DurabilityMeter_Configuration DurabilityMeter;
};

struct FFortItemCard_XS_TransformKey_Configuration
{
	struct Margin BackgroundPadding;
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
	struct Vector2D TransformKeyIconSize;
};

struct FFortItemCard_XXS_ItemInstance_Configuration
{
	struct FortItemCard_StackCountBlock_Configuration StackCountBlock;
};

struct FFortUIPickerTrapSortScores
{
	float UniqueTrapBonus;
	float SlottedBonus;
	float FavoriteBonus;
	float MaxTrackedTrapBonusTime;
	float TrackedTrapBonusMultiplier;
};

struct FFortUIStatStyle
{
	struct FText Name;
	struct FText HighestText;
	struct SlateBrush Icon;
	ECommonNumericType NumericType;
	float MinimalNotableValue;
};

struct FFortRootViewportLayoutInfo
{
	class UClass* LocalPlayer;
	class UClass* RootLayout;
};

struct FUINavigationData
{
	struct FText DisplayName;
	class UClass* ObjectData;
	struct FName IdData;
	int IntData;
};

struct FUINavigationEntry
{
	struct UINavigationData Data;
	struct FDelegate NavigateToDelegate;
	struct FDelegate NavigateFromDelegate;
};

struct FFortUINavigationOperation
{
	EFortUINavigationOp Operation;
	struct FName SquadId;
	int SquadSlotIndex;
	struct FName PageId;
	struct FName NodeId;
	class UClass* Item;
	EFortUIFeature Feature;
	EFortFrontendInventoryFilter ItemManagementFilter;
};

struct FFortUINavigationRequest
{
	TArray<struct FortUINavigationOperation> Operations;
};

struct FFortUIStyleWindowButtons
{
	struct ButtonStyle Close;
	struct ButtonStyle Minimize;
	struct ButtonStyle Maximize;
	struct ButtonStyle Restore;
};

struct FFortUIStyleDefinition
{
	struct FortUIStyleWindowButtons WindowButtons;
};

struct FFortVariantFilterContext
{
};

struct FFortVideoInfo
{
	struct FName ID;
	class UClass* PreviewImage;
	class UClass* VideoSource;
	class UClass* SubtitleOverlays;
	struct FName QuestObjectiveName;
	struct PrimaryAssetId RequiredActiveQuest;
};

struct FFortWeaponBoardDisplaySlot
{
	struct GameplayTag RequiredTag;
	struct Transform MeshRelativeTransform;
	struct Vector LocalOffsetMultiplier;
};

struct FFortWeaponBoardDisplaySlotItem
{
	struct FortWeaponBoardDisplaySlot DisplaySlot;
};

struct FHUDLayoutToolV2_ButtonStateColor
{
	struct SlateColor TextColor;
	struct LinearColor IconColor;
	struct LinearColor BackgroundColor;
	float IconAngle;
};

struct FLinkAcrossNumericalIndicatorActionSymbol
{
	ENumericalIndicatorActionType ActionType;
	struct FText ActionSymbol;
	struct LinearColor ActionColor;
};

struct FSimpleActionMessages
{
	ELinkAcrossSimpleAction SimpleActionType;
	struct FText SimpleActionTitle;
	struct FText SimpleActionBody;
};

struct FHUDLayoutToolV2_TextInputPopupContent
{
	EHUDLayoutToolTextInputPopupType PopupType;
	struct FText TitleText;
	struct FText DescriptionText;
	struct FText ButtonConfirmText;
	struct FText ButtonCancelText;
};

struct FQuickbarProxySlotIconContainer
{
	TArray<class UClass*> SlotIcons;
};

struct FPerkPipData
{
	class UClass* AccoladeItemDef;
	int NumOfPips;
};

struct FFortCountdownSounds
{
	class UClass* Sound;
	int StartTime;
	int StopTime;
	float FadeOutTime;
	bool bPlayed;
	class UClass* PlayInstance;
};

struct FPickupStreamEntry
{
	class UClass* PickupWidget;
};

struct FFortFeedbackDropdownCategoryHotfix_Add
{
	struct FName ParentInternalName;
	struct FName InternalName;
};

struct FFortFeedbackDropdownCategoryHotfix_Modify
{
	struct FName InternalName;
	bool bAddFlag;
	EFortPlayerFeedbackFlags Flag;
};

struct FSoundThreshold
{
	class UClass* SoundCue;
	float Threshold;
};

struct FAthenaSpatialCustomizationCategoryGroup
{
	TArray<class UClass*> CategoryEntryList;
};

struct FSpatialCustomizationCategoryStateData
{
	ESpatialCustomizationCategoryState State;
	struct LinearColor Color;
	struct FText CategorySelectionHeaderText;
};

struct FAthenaStyleMissionData
{
	class UClass* StyleDefinition;
	TArray<class UClass*> OptionalAdditionalStyleDefinitions;
	class UClass* Mission;
	class UClass* OverrideIdleAnimation;
};

struct FAthenaSpatialStyleCharacterData
{
	class UClass* Character;
	TArray<struct AthenaStyleMissionData> Styles;
	TArray<ESpatialStyleCharacterUnlockPrerequisite> CharacterPrerequisites;
	class UClass* CharacterIdleAnimation;
};

struct FWaxPlacementData
{
	int Placement;
	struct FString PlayerName;
	int CoinCount;
	bool bIsPlayer;
};

struct FFortBackendVersion
{
};

struct FFortFocusedBuildingInfo
{
	bool bIsInteractable;
	bool bCanBePlayerEdited;
	struct Vector IndicatorRelativeLocation;
	EFortBuildingHealthDisplayRule HealthDisplayRule;
	float MaxHealth;
	bool bIsAnyTrapAttached;
	bool bIsTrapAttachedFacingPlayer;
	bool bIsPreviewTrapAttached;
	float AttachedTrapMaxDurability;
	EFortBuildingInteraction InteractionType;
	int InteractionCost;
	EFortResourceType BuildingMaterial;
	struct FName QuickEditClass;
	bool bIsPlayerBuilt;
};

struct FFortHUDState
{
	bool bInBuildMode;
	bool bInCombatMode;
	bool bInEditMode;
	bool bInCreativeMode;
	bool bInBattleLabMode;
	bool bIsParachuteOpen;
	bool bIsFreeFalling;
	bool bInLockedBus;
	bool bInUnlockedBus;
	bool bOnTargeting;
	bool bOnUsingScopeTargeting;
	bool bOnCanTarget;
	bool bOnCanUseScopeTargeting;
	bool bOnCanUseSecondaryAbility;
	bool bCanOpenChute;
	bool bCrouching;
	bool bDBNO;
	bool bIsControllingRCPawn;
	bool bIsInVehicle;
	bool bIsDrivingVehicle;
	bool bCanSwapSeats;
	bool bIsCarryingDBNO;
	bool bIsCarryingHeldObject;
	bool bIsZiplining;
	bool bIsCreativeFlying;
};

struct FFortItemListFilter
{
	struct FString SearchText;
	EFortInventoryFilter FilterType;
	bool bInStorageVault;
	bool bIncludeVaultOverflow;
};

struct FFortAttributeModifierDisplayData
{
	struct GameplayAttribute Attribute;
	float Value;
	EGameplayModOp ModifierType;
	struct GameplayTagRequirements SourceTags;
	struct GameplayTagRequirements TargetTags;
};

struct FGridSortKey
{
	float Number;
	struct FString String;
};

struct FAthenaRewardTracker
{
};

struct FGlyphAllUpdateTransitions
{
};

struct FGlyphUpdateTransition
{
	int Count;
	float MinTimeOffset;
	float MaxTimeOffset;
};

struct FStatBound
{
	float Minimum;
	float Maximum;
};

struct FAthenaItemShopSectionData
{
};

struct FAthenaPlaylistLeaderboardData
{
	struct FName StatId;
	struct FText StatDisplayName;
	struct FText TabDisplayName;
	struct FText RowDisplayName;
	struct FString BaseGameplayTag;
	bool bIsGlobal;
};

struct FSelectedChallengesData
{
	bool bIsCompleted;
	bool bIsLocked;
	bool bHasIconOverride;
	float Progress;
	int NumObjectivesCompleted;
	int NumObjectives;
};

struct FAthenaEmergencyNotice
{
	struct FString Title;
	struct FString Body;
};

struct FAthenaMessageEntry
{
	struct FString Title;
	struct FString Body;
	struct FString Image;
	struct FString Type;
	struct FString AdSpace;
};

struct FImageRef
{
	int Width;
	int Height;
	struct FString URL;
};

struct FAthenaMOTDBase
{
	EAthenaNewsEntryType EntryType;
	struct FString Title;
	struct FString Body;
	TArray<struct ImageRef> Images;
	TArray<struct ImageRef> TileImages;
	struct Color CustomDarkColor;
	struct Color CustomLightColor;
	struct FString AdSpace;
	struct FString TabTitleOverride;
	struct FString ButtonTextOverride;
	bool bHasCustomColor;
};

struct FFortResurrectionUIData
{
	bool bResurrectionChipAvailable;
	bool bResurrectionChipPickedUp;
	bool bResurrectionChipRebooting;
};

struct FStatGroupData
{
	struct FText GroupName;
	struct SlateBrush Group;
	float Value;
	float ChartValue;
	float ChartOrigionalValue;
};

struct FAthenaReplayBrowserRowData
{
	struct FString Name;
	struct DateTime Date;
	float Length;
	int Rank;
	int NumPlayers;
	int Eliminations;
};

struct FAthenaSocialWhosGoingData
{
	Unknown WeakUser;
	TArray<struct RiftTourInfo> AttendingDates;
};

struct FAthenaTeamCountSlotData
{
	struct FText TeamNameText;
	struct FText TeamCountText;
	bool bIsMyTeam;
};

struct FWinConditionPlacementData
{
	struct Margin Padding;
	EHorizontalAlignment HorizontalAlignment;
	EVerticalAlignment VerticalAlignment;
};

struct FWinConditionPlacementDataMapWrapper
{
	Unknown Data;
};

struct FBattleLabDisplayData
{
	struct FText TitleText;
	struct FText DescriptionText;
	EBattleLabAlertType AlertType;
	int RewardCount;
	int RewardCountTotal;
	struct TSoftClassPtr<UObject> MainIconTexture;
};

struct FRewardPreviewParams
{
};

struct FAthenaBossHealthData
{
	bool bVisible;
	bool bShowShields;
	struct FText Name;
	int Health;
	int HealthMax;
	int Shields;
	int ShieldsMax;
};

struct FSpecialOfferVideo
{
	struct VideoWidgetConfig SpecialOfferVideo;
};

struct FAthenaCrewPackItem
{
	struct FString TemplateId;
	struct FString OfferTag;
	int Quantity;
};

struct FAthenaSubscriptionMonthlyRewards
{
	struct FString SkinImageURL;
	struct FText SkinTitle;
	bool bSkinImageTakeOver;
	struct FText MtxTitle;
	struct FString BattlepassImageURL;
	struct FString BattlepassTileImageURL;
	struct FText BattlepassTitle;
	struct FText BattlepassDescription;
	struct FString ColorA;
	struct FString ColorB;
	struct FString ColorC;
	struct FString ItemShopTileImageURL;
	struct FText ItemShopTileViolatorText;
	EViolatorIntensity ItemShopTileViolatorIntensity;
	struct DateTime CrewPackDateOverride;
	TArray<struct AthenaCrewPackItem> CrewPackItems;
};

struct FAthenaSubscriptionProgressiveReward
{
	struct AthenaCrewPackItem InitialCrewBling;
};

struct FSubscriptionBulletPoint
{
	struct FText Title;
	struct FText Description;
	struct FString IncludedCountries;
	TArray<struct FString> IncludedPlatforms;
};

struct FSubscriptionBulletPoints
{
	TArray<struct SubscriptionBulletPoint> BulletPoints;
};

struct FAthenaSubscriptionPurchaseDetails
{
	struct FText SkinTitle;
	struct FText SkinDescription;
	struct FText MtxTitle;
	struct FText MtxDescription;
	struct FText BattlepassTitle;
	struct FText BattlepassDescription;
	int BattlepassRefund;
	struct SubscriptionBulletPoints AdditionalBulletPoints;
};

struct FSubscriptionModalInfo
{
	struct FString ModalId;
	EAppStore PlatformId;
	TArray<struct FText> Entries;
	struct FString QrCodeImage;
};

struct FSubscriptionModals
{
	TArray<struct SubscriptionModalInfo> Modals;
};

struct FAthenaSubscriptionInfo
{
	struct AthenaSubscriptionMonthlyRewards CurrentRewards;
	struct AthenaSubscriptionMonthlyRewards NextRewards;
	struct AthenaSubscriptionProgressiveReward ProgressiveReward;
	struct AthenaSubscriptionPurchaseDetails PurchaseSubscriptionDetails;
	struct FText SubscriptionDisclaimer;
	struct FText BlockedBenefitsNotice;
	struct SubscriptionModals SubModals;
};

struct FDynamicBackground
{
	struct FString _title;
	struct FString __locale;
	EDynamicBackgroudKey Key;
	struct FString BackgroundImage;
	struct FString Stage;
};

struct FAthenaShopSection
{
	struct FName SectionId;
	struct FText SectionDisplayName;
	bool bSortOffersByOwnership;
	bool bShowIneligibleOffers;
	bool bShowIneligibleOffersIfGiftable;
	bool bShowTimer;
	bool bEnableToastNotification;
	int LandingPriority;
	struct DynamicBackground SectionBackground;
	bool bHidden;
};

struct FAthenaShopCarouselItem
{
	struct FText PreviewTitle;
	struct FText FullTitle;
	struct FString TileImage;
	struct FString VideoString;
	struct FString OfferId;
	int LandingPriority;
	EItemShopNavigationAction Action;
	bool bHidden;
};

struct FDynamicBackgrounds
{
	TArray<struct DynamicBackground> Backgrounds;
};

struct FDynamicBackgroundsSource
{
	struct FString _title;
	struct FString __locale;
	struct FString LastModified;
	struct DynamicBackgrounds Backgrounds;
};

struct FSurvivalObjectiveIconData
{
	int IconIndex;
	ESurvivalObjectiveIconState IconState;
	ESurvivalObjectiveIconState PrevIconState;
	struct FName SpecialActorID;
};

struct FDiscoTeamScoreData
{
	struct FText CurrScoreText;
	float CurrScorePercent;
	int CurrScore;
};

struct FProgressiveStageItemInfo
{
	class UClass* VariantToken;
	bool bIsLocked;
	bool bIsNextToUnlock;
	int StageIndex;
};

struct FVideoConfig_Mono
{
	class UClass* StreamingMediaSource;
	struct FName VideoString;
	struct FName StreamingVideoID;
	struct FName FallbackVideoID;
	struct FString ProdLinkID;
	struct FString GamedevLinkID;
	bool bIsAutoPlay;
	bool bForceAutoPlay;
	bool bStreamingEnabled;
	struct FString VideoUID;
	bool bShouldBeModal;
	bool bUseSkipBoundaries;
	bool bKairosPlayer;
	bool bHoldToSkip;
};

struct FFortAthenaStandaloneTutorialStepInfo
{
	TArray<struct FortAthenaTutorialScreenInfo> StepScreenInfo;
};

struct FVaultVariantOverrideOption
{
	struct GameplayTag VariantChannelOverride;
	struct GameplayTag VariantTagOverride;
	struct FString CustomDataPayload;
	class UClass* OptionalItemVariantIsUsedFor;
};

struct FItemPreviewSettings
{
	class UClass* ItemToView;
	int SubslotIndex;
	struct GameplayTagContainer PreviewExcludedTags;
	int ItemVariantPreviewIndex;
	bool bHasEtherealBackground;
	TArray<struct VaultVariantOverrideOption> VariantOverrides;
};

struct FFortWeightedObject
{
	class UClass* Object;
	int Weight;
};

struct FOptionsReleaseInfo
{
	ESettingType SettingType;
	int ReleaseVersion;
};

struct FFortDailyRewardsData
{
	int CurrentLoginDays;
	bool bCanClaim;
	TArray<struct FortDailyRewardsScheduleData> Schedules;
};

struct FFortPickerCategory
{
};

struct FConsumedCriteriaData
{
	float PowerMod;
	TArray<struct FName> CriteriaNames;
};

struct FViewVaultItemsParams
{
	TArray<class UClass*> ItemsToView;
};

struct FFortMissionRewardInfo
{
	struct SlateBrush Icon;
	struct FText DisplayName;
	class UClass* Item;
	bool bIsMissionAlertReward;
};

struct FFortMissionDetails
{
	EFortTheaterMapTileType TileType;
	struct FText TheaterDisplayName;
	struct FText MissionName;
	struct FText MissionDescription;
	bool bIsGroupContent;
	class UClass* MissionDefinition;
	struct FText ZoneName;
	struct FText ZoneDescription;
	struct FText RegionThemeName;
	struct TSoftClassPtr<UObject> RegionThemeIcon;
	struct FText DifficultyName;
	TArray<struct FortMissionRewardInfo> MissionRewards;
	struct Timespan AvailableTime;
	ERatingsEnforcementType RatingsEnforcement;
	int RequiredBaseRating;
	int RecommendedBaseRating;
	int MaxBaseRating;
	int ContentDifficultyLevel;
	bool bIsOnboarding;
	int LootLevel;
	bool bOverrideConningText;
	struct FText ConningOverrideText;
	struct FString TheaterUniqueId;
	class UClass* AssociatedCloudSaveItemDefinition;
	TArray<class UClass*> GameplayModifiers;
	TArray<struct FText> Objectives;
};

struct FFortItemSorterDefinition
{
};

struct FFortHeroLoadoutHeroPickerTabConfiguration
{
	struct FortItemFilterDefinition Filter;
	struct FortItemSorterDefinition Sorter;
};

struct FFortItemDelta
{
	class UClass* ItemDefinition;
	int BaseAmount;
	int DeltaAmount;
};

struct FFortBasicMissionInfo
{
	struct FText MissionName;
	struct FortMultiSizeBrush MissionIcons;
	struct FText TheaterName;
	struct FText DifficultyName;
	class UClass* EndOfMissionMediaSource;
	class UClass* EndOfMissionMediaSourceStreamed;
	bool bSkipEndOfMissionVideo;
	bool bIsGroupContent;
};

struct FFortActionBeingUnbound
{
	struct FName ActionName;
	int InputIndex;
};

struct FItemDefinitionChangedStruct
{
	struct FScriptMulticastDelegate ChangeDelegate;
};

struct FFortItemListViewConfig
{
};

struct FInterpolatedTransitionCamera
{
	struct Transform CameraTransform;
	float FieldOfView;
};

struct FSceneTransitionOptions
{
	ESceneTransitionType TRANSITION;
	bool bKeepInputRotation;
};

struct FFortItemEntryPreviewData
{
	int Quantity;
	EFortItemInspectionMode InspectMode;
};

struct FUserAction
{
	struct FString ActionName;
};

struct FFortMobileHUDWidgetLayout
{
	struct AnchorData LayoutData;
	int ZOrder;
};

struct FHUDWidgetSchemaInitializer
{
};

struct FFortPhoenixLevelProgressionRewards
{
	int NextRewardLevel;
	struct FortItemQuantityPair NextReward;
	int NextMajorRewardLevel;
	struct FortItemQuantityPair NextMajorReward;
};

struct FFortPlayerSurveyButtonListMultipleSelectionAnswerWidgetFocusInfo
{
	EFortPlayerSurveyButtonListMultipleSelectionAnswerWidgetFocusType FocusType;
	int Index;
};

struct FFortPlayerSurveyCMSCondition
{
	struct FString Type;
	struct FString Operation;
	struct FString ComparisonValue;
	TArray<struct FString> ConditionInfo;
	TArray<struct JsonObjectWrapper> ChildConditions;
};

struct FFortPlayerSurveyCMSConditionGroup
{
	struct FString ConditionGroupID;
	TArray<struct FortPlayerSurveyCMSCondition> Conditions;
};

struct FFortPlayerSurveyLocalizableText
{
	struct FString TextID;
	struct FString LocalizedText;
};

struct FFortPlayerSurveyCMSQuestion
{
	struct FortPlayerSurveyLocalizableText QuestionText;
	struct FString Type;
	uint32_t NumberOfOptions;
	TArray<struct FortPlayerSurveyLocalizableText> Responses;
	bool bRandomizeResponseOrder;
};

struct FFortPlayerSurveyCMSFrequentlyAskedQuestion
{
	struct FString FrequentlyAskedQuestionID;
	struct FortPlayerSurveyCMSQuestion Question;
};

struct FFortPlayerSurveyCMSCustomTextReplacement
{
	struct FString TextReplacementTag;
	TArray<struct FortPlayerSurveyLocalizableText> TextReplacementValues;
};

struct FFortPlayerSurveyCMSFrequentlyAskedQuestionID
{
	struct FString ID;
};

struct FFortPlayerSurveyCMSQuestionContainer
{
	struct FortPlayerSurveyCMSFrequentlyAskedQuestionID FrequentlyAskedQuestionID;
	struct FortPlayerSurveyCMSQuestion Question;
};

struct FFortPlayerSurveyCMSSurvey
{
	struct FString SurveyID;
	TArray<struct FString> SurveyTags;
	struct FString Title;
	TArray<struct FString> ConditionGroupIDs;
	TArray<struct FortPlayerSurveyCMSCondition> Conditions;
	TArray<struct FortPlayerSurveyCMSQuestionContainer> Questions;
	TArray<struct FortPlayerSurveyCMSCustomTextReplacement> TextReplacementOverrides;
	bool bRandomizeQuestionOrder;
};

struct FFortPlayerSurveyCMSData
{
	TArray<struct FortPlayerSurveyCMSConditionGroup> ConditionGroups;
	TArray<struct FortPlayerSurveyCMSFrequentlyAskedQuestion> FrequentlyAskedQuestions;
	TArray<struct FortPlayerSurveyCMSCustomTextReplacement> CustomTextReplacements;
	TArray<struct FortPlayerSurveyCMSSurvey> Surveys;
	bool bSurveysEnabled;
};

struct FFortPlayerSurveyCMSDataCondition
{
};

struct FFortPlayerSurveyCMSDataConditionBase
{
};

struct FFortPlayerSurveyCMSDataGameplayTagQuery
{
	EFortPlayerSurveyCMSDataGameplayTagQueryExprType T;
	TArray<struct FName> N;
};

struct FFortPlayerSurveyCMSDataConditionGroup
{
	struct FName ID;
	TArray<struct JsonObjectWrapper> C;
};

struct FFortPlayerSurveyCMSDataRelativeSurveyKey
{
	EFortPlayerSurveyCMSDataRelativeSurveyKeyType T;
	struct FString ID;
};

struct FFortPlayerSurveyCMSDataQuestion
{
};

struct FFortPlayerSurveyCMSDataQuestionBase
{
};

struct FFortPlayerSurveyCMSDataQuestionChoice
{
	struct FText T;
};

struct FFortPlayerSurveyCMSDataSurveyDescriptionMessage
{
	struct FText T;
	struct FText M;
};

struct FFortPlayerSurveyCMSDataSurvey
{
	struct FString ID;
	struct FText T;
	bool rt;
	TArray<struct JsonObjectWrapper> C;
	struct FortPlayerSurveyCMSDataSurveyDescriptionMessage cm;
	EFortPlayerSurveyCMSDataTrigger R;
	TArray<EFortPlayerSurveyCMSDataGameMode> sg;
	TArray<struct JsonObjectWrapper> Q;
};

struct FFortPlayerSurveyCMSDataRoot
{
	bool E;
	TArray<struct FortPlayerSurveyCMSDataConditionGroup> cg;
	TArray<struct FortPlayerSurveyCMSDataSurvey> S;
};

struct FFortShowdownEventBestResultsSummary
{
	int TotalScore;
	int MatchesPlayed;
	int NumVictoryRoyales;
	int PlacementPoints;
	int EliminationPoints;
	int EntryFeePoints;
};

struct FFortLandingPageDefenderSummaryInfo
{
	struct FName SquadId;
	struct FText TheaterDisplayName;
	struct FString TheaterUniqueId;
};

struct FFortAttributeModifierAccumulation
{
	struct GameplayTag GameplayTag;
	struct GameplayAttribute Attribute;
	EGameplayModOp ModifierOp;
	float Magnitude;
};

struct FFortToastDisplayInfo
{
	struct FText Header;
	struct FText Body;
	struct TSoftClassPtr<UObject> Image;
	EFortToastType Type;
};

struct FStoreCallout
{
	struct FortToastDisplayInfo ToastDisplayInfo;
	struct FText ItemName;
	struct FText ItemSet;
	struct FString OfferDisplayAssetPath;
};

struct FBundledItemInfo
{
	struct FString TemplateId;
	int Quantity;
	bool bOwned;
};

struct FFortSurvivorSquadSlottingFeedbackData
{
	bool HadLeaderMatch;
	bool HasLeaderMatch;
	Unknown PreviousSetBonusCounts;
	Unknown CurrentSetBonusCounts;
	int PreviousPersonalityMatchCount;
	int CurrentPersonalityMatchCount;
};

struct FFortSurvivorSquadSelectorButtonPersonalityMatches
{
	int NumPersonalityMatches;
	int TotalNonLeaderSquadMembers;
	bool HavePersonalityIcons;
	struct FortMultiSizeBrush PersonalityIcons;
};

struct FFortSurvivorSquadSelectorButtonSummaryStats
{
	struct FName SquadId;
	struct GameplayAttribute FortAttribute;
	float FortAttributeValue;
	struct GameplayAttribute FortTeamAttribute;
	float TeamFortAttributeValue;
	float SquadPowerValue;
	struct FText FortAttributeName;
};

struct FTouchMove
{
};

struct FFortItemTransformFilterTabLabelInfo
{
	struct FName FilterTabNameId;
	struct FortTabButtonLabelInfo TabButtonLabelInfo;
	TArray<EFortInventoryFilter> ItemFilters;
	EFortItemType ItemType;
};

struct FFortUIXpInfo
{
	int InitialLevel;
	int InitialDisplayXp;
	struct FortExperienceDelta ChangeInXp;
};

struct FMatchmakingOptionsOverride
{
	struct TSoftClassPtr<UObject> SoftMatchmakingOptionsClass;
};

struct FFortDisplayAttribute
{
	struct GameplayAttribute Attribute;
	struct FText Label;
	struct FText Value;
	struct FText HoverText;
	float NumericValue;
	EFortStatValueDisplayType DisplayType;
	EFortBuffState BuffState;
	EFortClampState ClampState;
	EFortComparisonType ComparisonType;
};

struct FFortDisplayModifier
{
	struct FText Label;
	struct FText Value;
	EFortStatValueDisplayType DisplayType;
	EFortBuffState BuffState;
};

struct FFortErrorInfo
{
	struct FText Operation;
	struct FText ErrorMessage;
	struct FString ErrorCode;
	EFortErrorSeverity ErrorSeverity;
	struct FText ContinueButtonText;
};

struct FHeistExitCraftIconData
{
	int IconIndex;
	EHeistExitCraftIconState IconState;
	EHeistExitCraftIconState PrevIconState;
	int SecondsUntilIncoming;
	bool bTeamHasBling;
};

struct FHeistBlingIconData
{
	int IconIndex;
	EHeistBlingIconState IconState;
	EHeistBlingIconState PrevIconState;
};

struct FTDMTeamScoreData
{
	struct FText CurrScoreText;
	float CurrScorePercent;
	int CurrScore;
};

struct FGliderThrustData
{
	struct FName ParameterName;
	struct Vector2D PitchOutput;
	struct Vector2D VolumeOutput;
	class UClass* Sound;
};

struct FPhysicsImpactData
{
	struct Vector2D VolumeOutput;
	struct Vector2D PitchOutput;
	TArray<class UClass*> Variations;
};

struct FPhysicsStateData
{
	struct FName ParameterName;
	struct Vector2D VolumeOutput;
	struct Vector2D PitchOutput;
	struct DistanceDatum CrossfadeInputSlow;
	class UClass* SlowLoop;
	struct DistanceDatum CrossfadeInputFast;
	class UClass* FastLoop;
};

struct FFriendChestRoundPacing
{
	TArray<int> NumTargetsPerRound;
};

struct FFriendChestPlayerActivationPair
{
	class UClass* Player;
	struct TimerHandle ActivationTimer;
};

struct FFriendChestAnalyticsLootRoll
{
};

struct FFriendChestAnalyticsParticipant
{
	class UClass* PlayerState;
	struct FString AccountId;
	struct Vector SpawnCoordinates;
	TArray<struct FriendChestAnalyticsLootRoll> InstancedLoot;
	TArray<struct FriendChestAnalyticsLootRoll> FailedLoot;
};

struct FGameFeatureComponentEntry
{
	struct TSoftClassPtr<UObject> ActorClass;
	struct TSoftClassPtr<UObject> ComponentClass;
	bool bClientComponent;
	bool bServerComponent;
};

struct FDataRegistrySourceToAdd
{
	struct FName RegistryToAddTo;
	int AssetPriority;
	bool bClientSource;
	bool bServerSource;
	struct TSoftClassPtr<UObject> DataTableToAdd;
	struct TSoftClassPtr<UObject> CurveTableToAdd;
};

struct FGameFeaturePluginStateMachineProperties
{
	class UClass* GameFeatureData;
};

struct FLogSubmitOptions
{
	struct FString EventName;
	bool bSubmitLogs;
	bool bSubmitLogsInOptionState;
	int LogTailKb;
	float LogSubmitChance;
	TArray<struct FString> DoNotUploadReasons;
};

struct FAttributeDefaults
{
	class UClass* Attributes;
	class UClass* DefaultStartingTable;
};

struct FGameplayAbilitySpecHandle
{
	int Handle;
};

struct FPredictionKey
{
	class UClass* PredictiveConnection;
	int16_t Current;
	int16_t Base;
	bool bIsStale;
	bool bIsServerInitiated;
};

struct FGameplayAbilityActivationInfo
{
	EGameplayAbilityActivationMode ActivationMode;
	bool bCanBeEndedByOtherInstance;
	struct PredictionKey PredictionKeyWhenActivated;
};

struct FActiveGameplayEffectHandle
{
	int Handle;
	bool bPassedFiltersAndWasExecuted;
};

struct FGameplayAbilityRepAnimMontage
{
	class UClass* AnimMontage;
	float PlayRate;
	float Position;
	float BlendTime;
	byte NextSectionID;
	bool bRepPosition;
	bool IsStopped;
	bool ForcePlayBit;
	bool SkipPositionCorrection;
	bool bSkipPlayRate;
	struct PredictionKey PredictionKey;
	byte SectionIdToPlay;
};

struct FGameplayAbilityLocalAnimMontage
{
	class UClass* AnimMontage;
	bool PlayBit;
	struct PredictionKey PredictionKey;
	class UClass* AnimatingAbility;
};

struct FGameplayAttribute
{
	struct FString AttributeName;
	Unknown Attribute;
	class UClass* AttributeOwner;
};

struct FGameplayEffectModifiedAttribute
{
	struct GameplayAttribute Attribute;
	float TotalMagnitude;
};

struct FGameplayEffectAttributeCaptureDefinition
{
	struct GameplayAttribute AttributeToCapture;
	EGameplayEffectAttributeCaptureSource AttributeSource;
	bool bSnapshot;
};

struct FGameplayEffectAttributeCaptureSpec
{
	struct GameplayEffectAttributeCaptureDefinition BackingDefinition;
};

struct FGameplayEffectAttributeCaptureSpecContainer
{
	TArray<struct GameplayEffectAttributeCaptureSpec> SourceAttributes;
	TArray<struct GameplayEffectAttributeCaptureSpec> TargetAttributes;
	bool bHasNonSnapshottedAttributes;
};

struct FTagContainerAggregator
{
	struct GameplayTagContainer CapturedActorTags;
	struct GameplayTagContainer CapturedSpecTags;
	struct GameplayTagContainer ScopedTags;
};

struct FModifierSpec
{
	float EvaluatedMagnitude;
};

struct FScalableFloat
{
	float Value;
	struct CurveTableRowHandle Curve;
	struct DataRegistryType RegistryType;
};

struct FGameplayAbilitySpecDef
{
	class UClass* Ability;
	struct ScalableFloat LevelScalableFloat;
	int InputID;
	EGameplayEffectGrantedAbilityRemovePolicy RemovalPolicy;
	class UClass* SourceObject;
	struct GameplayAbilitySpecHandle AssignedHandle;
};

struct FGameplayEffectContextHandle
{
};

struct FGameplayEffectSpec
{
	class UClass* Def;
	TArray<struct GameplayEffectModifiedAttribute> ModifiedAttributes;
	struct GameplayEffectAttributeCaptureSpecContainer CapturedRelevantAttributes;
	float Duration;
	float Period;
	float ChanceToApplyToTarget;
	struct TagContainerAggregator CapturedSourceTags;
	struct TagContainerAggregator CapturedTargetTags;
	struct GameplayTagContainer DynamicGrantedTags;
	struct GameplayTagContainer DynamicAssetTags;
	TArray<struct ModifierSpec> Modifiers;
	int StackCount;
	bool bCompletedSourceAttributeCapture;
	bool bCompletedTargetAttributeCapture;
	bool bDurationLocked;
	TArray<struct GameplayAbilitySpecDef> GrantedAbilitySpecs;
	struct GameplayEffectContextHandle EffectContext;
	float Level;
};

struct FGameplayCueParameters
{
	float NormalizedMagnitude;
	float RawMagnitude;
	struct GameplayEffectContextHandle EffectContext;
	struct GameplayTag MatchedTagName;
	struct GameplayTag OriginalTag;
	struct GameplayTagContainer AggregatedSourceTags;
	struct GameplayTagContainer AggregatedTargetTags;
	struct Vector_NetQuantize10 Location;
	struct Vector_NetQuantizeNormal Normal;
	Unknown Instigator;
	Unknown EffectCauser;
	Unknown SourceObject;
	Unknown PhysicalMaterial;
	int GameplayEffectLevel;
	int AbilityLevel;
	Unknown TargetAttachComponent;
	bool bReplicateLocationWhenUsingMinimalRepProxy;
};

struct FMinimalReplicationTagCountMap
{
	class UClass* Owner;
};

struct FNetSerializeScriptStructCache
{
	TArray<class UClass*> ScriptStructs;
};

struct FGameplayAbilityTargetDataHandle
{
};

struct FGameplayEventData
{
	struct GameplayTag EventTag;
	class UClass* Instigator;
	class UClass* Target;
	class UClass* OptionalObject;
	class UClass* OptionalObject2;
	struct GameplayEffectContextHandle ContextHandle;
	struct GameplayTagContainer InstigatorTags;
	struct GameplayTagContainer TargetTags;
	float EventMagnitude;
	struct GameplayAbilityTargetDataHandle TargetData;
};

struct FAbilityTriggerData
{
	struct GameplayTag TriggerTag;
	EGameplayAbilityTriggerSource TriggerSource;
};

struct FGameplayAbilityBindInfo
{
	EGameplayAbilityInputBinds Command;
	class UClass* GameplayAbilityClass;
};

struct FGameplayAbilityTargetingLocationInfo
{
	EGameplayAbilityTargetingLocationType LocationType;
	struct Transform LiteralTransform;
	class UClass* SourceActor;
	class UClass* SourceComponent;
	class UClass* SourceAbility;
	struct FName SourceSocketName;
};

struct FWorldReticleParameters
{
	struct Vector AOEScale;
};

struct FGameplayTargetDataFilterHandle
{
};

struct FGameplayCueObjectLibrary
{
	TArray<struct FString> Paths;
	class UClass* ActorObjectLibrary;
	class UClass* StaticObjectLibrary;
	class UClass* CueSet;
	bool bShouldSyncScan;
	bool bShouldAsyncLoad;
	bool bShouldSyncLoad;
	bool bHasBeenInitialized;
};

struct FGameplayEffectSpecForRPC
{
	class UClass* Def;
	TArray<struct GameplayEffectModifiedAttribute> ModifiedAttributes;
	struct GameplayEffectContextHandle EffectContext;
	struct GameplayTagContainer AggregatedSourceTags;
	struct GameplayTagContainer AggregatedTargetTags;
	float Level;
	float AbilityLevel;
};

struct FGameplayCuePendingExecute
{
	struct PredictionKey PredictionKey;
	EGameplayCuePayloadType PayloadType;
	class UClass* OwningComponent;
	struct GameplayEffectSpecForRPC FromSpec;
	struct GameplayCueParameters CueParameters;
};

struct FPreallocationInfo
{
	TArray<class UClass*> ClassesNeedingPreallocation;
};

struct FGameplayCueNotifyData
{
	struct GameplayTag GameplayCueTag;
	struct SoftObjectPath GameplayCueNotifyObj;
	class UClass* LoadedGameplayCueClass;
};

struct FAttributeBasedFloat
{
	struct ScalableFloat Coefficient;
	struct ScalableFloat PreMultiplyAdditiveValue;
	struct ScalableFloat PostMultiplyAdditiveValue;
	struct GameplayEffectAttributeCaptureDefinition BackingAttribute;
	struct CurveTableRowHandle AttributeCurve;
	EAttributeBasedFloatCalculationType AttributeCalculationType;
	EGameplayModEvaluationChannel FinalChannel;
	struct GameplayTagContainer SourceTagFilter;
	struct GameplayTagContainer TargetTagFilter;
};

struct FCustomCalculationBasedFloat
{
	class UClass* CalculationClassMagnitude;
	struct ScalableFloat Coefficient;
	struct ScalableFloat PreMultiplyAdditiveValue;
	struct ScalableFloat PostMultiplyAdditiveValue;
	struct CurveTableRowHandle FinalLookupCurve;
};

struct FSetByCallerFloat
{
	struct FName DataName;
	struct GameplayTag DataTag;
};

struct FGameplayEffectModifierMagnitude
{
	EGameplayEffectMagnitudeCalculation MagnitudeCalculationType;
	struct ScalableFloat ScalableFloatMagnitude;
	struct AttributeBasedFloat AttributeBasedMagnitude;
	struct CustomCalculationBasedFloat CustomMagnitude;
	struct SetByCallerFloat SetByCallerMagnitude;
};

struct FGameplayModEvaluationChannelSettings
{
	EGameplayModEvaluationChannel Channel;
};

struct FGameplayTagRequirements
{
	struct GameplayTagContainer RequireTags;
	struct GameplayTagContainer IgnoreTags;
};

struct FGameplayModifierInfo
{
	struct GameplayAttribute Attribute;
	EGameplayModOp ModifierOp;
	struct ScalableFloat Magnitude;
	struct GameplayEffectModifierMagnitude ModifierMagnitude;
	struct GameplayModEvaluationChannelSettings EvaluationChannelSettings;
	struct GameplayTagRequirements SourceTags;
	struct GameplayTagRequirements TargetTags;
};

struct FGameplayEffectExecutionScopedModifierInfo
{
	struct GameplayEffectAttributeCaptureDefinition CapturedAttribute;
	struct GameplayTag TransientAggregatorIdentifier;
	EGameplayEffectScopedModifierAggregatorType AggregatorType;
	EGameplayModOp ModifierOp;
	struct GameplayEffectModifierMagnitude ModifierMagnitude;
	struct GameplayModEvaluationChannelSettings EvaluationChannelSettings;
	struct GameplayTagRequirements SourceTags;
	struct GameplayTagRequirements TargetTags;
};

struct FConditionalGameplayEffect
{
	class UClass* EffectClass;
	struct GameplayTagContainer RequiredSourceTags;
};

struct FGameplayEffectExecutionDefinition
{
	class UClass* CalculationClass;
	struct GameplayTagContainer PassedInTags;
	TArray<struct GameplayEffectExecutionScopedModifierInfo> CalculationModifiers;
	TArray<class UClass*> ConditionalGameplayEffectClasses;
	TArray<struct ConditionalGameplayEffect> ConditionalGameplayEffects;
};

struct FGameplayEffectCue
{
	struct GameplayAttribute MagnitudeAttribute;
	float MinLevel;
	float MaxLevel;
	struct GameplayTagContainer GameplayCueTags;
};

struct FInheritedTagContainer
{
	struct GameplayTagContainer CombinedTags;
	struct GameplayTagContainer Added;
	struct GameplayTagContainer Removed;
};

struct FGameplayEffectQuery
{
	struct FDelegate CustomMatchDelegate_BP;
	struct GameplayTagQuery OwningTagQuery;
	struct GameplayTagQuery EffectTagQuery;
	struct GameplayTagQuery SourceTagQuery;
	struct GameplayAttribute ModifyingAttribute;
	class UClass* EffectSource;
	class UClass* EffectDefinition;
};

struct FGameplayTagReponsePair
{
	struct GameplayTag Tag;
	class UClass* ResponseGameplayEffect;
	TArray<class UClass*> ResponseGameplayEffects;
	int SoftCountCap;
};

struct FGameplayTagResponseTableEntry
{
	struct GameplayTagReponsePair Positive;
	struct GameplayTagReponsePair Negative;
};

struct FGameplayCueTag
{
	struct GameplayTag GameplayCueTag;
};

struct FMovieSceneGameplayCueKey
{
	struct GameplayCueTag Cue;
	struct Vector Location;
	struct Vector Normal;
	struct FName AttachSocketName;
	float NormalizedMagnitude;
	struct MovieSceneObjectBindingID Instigator;
	struct MovieSceneObjectBindingID EffectCauser;
	class UClass* PhysicalMaterial;
	int GameplayEffectLevel;
	int AbilityLevel;
	bool bAttachToBinding;
};

struct FGameplayEffectSpecHandle
{
};

struct FGameplayEffectRemovalInfo
{
	bool bPrematureRemoval;
	int StackCount;
	struct GameplayEffectContextHandle EffectContext;
};

struct FGameplayAbilities_FServerAbilityRPCBatch
{
	struct GameplayAbilitySpecHandle AbilitySpecHandle;
	struct PredictionKey PredictionKey;
	struct GameplayAbilityTargetDataHandle TargetData;
	bool InputPressed;
	bool Ended;
	bool Started;
};

struct FGameplayAttributeData
{
	float BaseValue;
	float CurrentValue;
};

struct FGameplayTargetDataFilter
{
	class UClass* SelfActor;
	class UClass* RequiredActorClass;
	ETargetDataFilterSelf SelfFilter;
	bool bReverseFilter;
};

struct FGameplayAbilityTargetData
{
};

struct FGameplayAbilitySpecHandleAndPredictionKey
{
	struct GameplayAbilitySpecHandle AbilityHandle;
	int PredictionKeyAtCreation;
};

struct FAbilityTaskDebugMessage
{
	class UClass* FromTask;
	struct FString Message;
};

struct FAbilityEndedData
{
	class UClass* AbilityThatEnded;
	struct GameplayAbilitySpecHandle AbilitySpecHandle;
	bool bReplicateEndAbility;
	bool bWasCancelled;
};

struct FGameplayAbilityActorInfo
{
	Unknown OwnerActor;
	Unknown AvatarActor;
	Unknown PlayerController;
	Unknown AbilitySystemComponent;
	Unknown SkeletalMeshComponent;
	Unknown AnimInstance;
	Unknown MovementComponent;
	struct FName AffectedAnimInstanceTag;
};

struct FMinimalGameplayCueReplicationProxy
{
	class UClass* Owner;
};

struct FGameplayCueTranslationLink
{
	class UClass* RulesCDO;
};

struct FGameplayCueTranslatorNodeIndex
{
	int Index;
};

struct FGameplayCueTranslatorNode
{
	TArray<struct GameplayCueTranslationLink> Links;
	struct GameplayCueTranslatorNodeIndex CachedIndex;
	struct GameplayTag CachedGameplayTag;
	struct FName CachedGameplayTagName;
};

struct FGameplayCueTranslationManager
{
	TArray<struct GameplayCueTranslatorNode> TranslationLUT;
	Unknown TranslationNameToIndexMap;
	class UClass* TagManager;
};

struct FActiveGameplayEffectQuery
{
};

struct FGameplayModifierEvaluatedData
{
	struct GameplayAttribute Attribute;
	EGameplayModOp ModifierOp;
	float Magnitude;
	struct ActiveGameplayEffectHandle Handle;
	bool IsValid;
};

struct FGameplayEffectCustomExecutionOutput
{
	TArray<struct GameplayModifierEvaluatedData> OutputModifiers;
	bool bTriggerConditionalGameplayEffects;
	bool bHandledStackCountManually;
	bool bHandledGameplayCuesManually;
};

struct FGameplayEffectCustomExecutionParameters
{
};

struct FGameplayTagBlueprintPropertyMapping
{
	struct GameplayTag TagToMap;
	Unknown PropertyToEdit;
	struct FName PropertyName;
	struct Guid PropertyGuid;
};

struct FGameplayTagBlueprintPropertyMap
{
	TArray<struct GameplayTagBlueprintPropertyMapping> PropertyMappings;
};

struct FGameplayEffectContext
{
	Unknown Instigator;
	Unknown EffectCauser;
	Unknown AbilityCDO;
	Unknown AbilityInstanceNotReplicated;
	int AbilityLevel;
	Unknown SourceObject;
	Unknown InstigatorAbilitySystemComponent;
	TArray<Unknown> Actors;
	struct Vector WorldOrigin;
	bool bHasWorldOrigin;
	bool bReplicateSourceObject;
};

struct FMontagePlaybackData
{
	class UClass* Avatar;
	class UClass* AnimMontage;
	class UClass* AbilityComponent;
};

struct FAgentGameplayBehaviors
{
	TArray<class UClass*> Behaviors;
};

struct FFOscillator
{
	float Amplitude;
	float Frequency;
	EInitialOscillatorOffset InitialOffset;
	EOscillatorWaveform Waveform;
};

struct FROscillator
{
	struct FOscillator Pitch;
	struct FOscillator Yaw;
	struct FOscillator Roll;
};

struct FVOscillator
{
	struct FOscillator X;
	struct FOscillator Y;
	struct FOscillator Z;
};

struct FPerlinNoiseShaker
{
	float Amplitude;
	float Frequency;
};

struct FWaveOscillator
{
	float Amplitude;
	float Frequency;
	EInitialWaveOscillatorOffsetType InitialOffsetType;
};

struct FReplicatedMessage
{
};

struct FReplicatedMessageData
{
	class UClass* StructType;
};

struct FGameplayMessageReceiverData
{
};

struct FGameplayMessageReceiverHandle
{
	Unknown Subsystem;
	struct EventMessageTag Channel;
	int ID;
};

struct FActiveGameplayStateData
{
	class UClass* GameplayState;
	struct GameplayTag PreviousStateId;
	float BeginStateDelay;
	float InitializationTime;
};

struct FGameplayStateTransition
{
	struct GameplayTagContainer TransitionConditionTags;
	struct GameplayTag TransitionStateTag;
	float BeginStateDelay;
};

struct FGameplayStateSettings
{
	class UClass* StateClass;
	struct GameplayTag StateId;
	TArray<struct GameplayStateTransition> StateTransitions;
};

struct FGameplayStateMachineData
{
	class UClass* StateMachine;
};

struct FGameplayTag
{
	struct FName TagName;
};

struct FGameplayTagQuery
{
	int TokenStreamVersion;
	TArray<struct GameplayTag> TagDictionary;
	TArray<byte> QueryTokenStream;
	struct FString UserDescription;
	struct FString AutoDescription;
};

struct FGameplayTagContainer
{
	TArray<struct GameplayTag> GameplayTags;
	TArray<struct GameplayTag> ParentTags;
};

struct FGameplayTagSource
{
	struct FName SourceName;
	EGameplayTagSourceType SourceType;
	class UClass* SourceTagList;
	class UClass* SourceRestrictedTagList;
};

struct FGameplayTagCategoryRemap
{
	struct FString BaseCategory;
	TArray<struct FString> RemapCategories;
};

struct FGameplayTagRedirect
{
	struct FName OldTagName;
	struct FName NewTagName;
};

struct FRestrictedConfigInfo
{
	struct FString RestrictedConfigName;
	TArray<struct FString> Owners;
};

struct FGameplayTagCreationWidgetHelper
{
};

struct FGameplayTagReferenceHelper
{
};

struct FGameplayTagNode
{
};

struct FGameplayResourceSet
{
};

struct FCatalogPurchaseNotification
{
	struct McpLootResult LootResult;
};

struct FCatalogKeyValue
{
	struct FString Key;
	struct FString Value;
};

struct FCatalogItemPrice
{
	EStoreCurrencyType CurrencyType;
	struct FString CurrencySubType;
	int RegularPrice;
	int FinalPrice;
	struct FText PriceTextOverride;
	ECatalogSaleType SaleType;
	struct DateTime SaleExpiration;
	EAppStore AppStoreId;
};

struct FItemQuantity
{
	struct FString TemplateId;
	int Quantity;
	struct JsonObjectWrapper Attributes;
};

struct FCatalogDynamicBundleItem
{
	struct ItemQuantity Item;
	bool bCanOwnMultiple;
	int RegularPrice;
	int DiscountedPrice;
	int AlreadyOwnedPriceReduction;
	struct FText Title;
	struct FText Description;
};

struct FCatalogDynamicBundle
{
	int DiscountedBasePrice;
	int RegularBasePrice;
	int FloorPrice;
	EStoreCurrencyType CurrencyType;
	struct FString CurrencySubType;
	ECatalogSaleType DisplayType;
	TArray<struct CatalogDynamicBundleItem> BundleItems;
};

struct FCatalogMetaAssetInfo
{
	struct FString StructName;
	struct JsonObjectWrapper Payload;
};

struct FCatalogOfferRequirement
{
	ECatalogRequirementType RequirementType;
	int MinQuantity;
	struct FString RequiredId;
};

struct FCatalogGiftInfo
{
	bool bIsEnabled;
	struct FString ForcedGiftBoxTemplateId;
	TArray<struct CatalogOfferRequirement> PurchaseRequirements;
};

struct FCatalogOffer
{
	struct FString OfferId;
	struct FString DevName;
	TArray<struct CatalogKeyValue> MetaInfo;
	ECatalogOfferType OfferType;
	TArray<struct CatalogItemPrice> Prices;
	struct CatalogDynamicBundle DynamicBundleInfo;
	int DailyLimit;
	int WeeklyLimit;
	int MonthlyLimit;
	TArray<struct FString> Categories;
	struct FString CatalogGroup;
	int CatalogGroupPriority;
	int SortPriority;
	struct FText Title;
	struct FText ShortDescription;
	struct FText Description;
	struct FString AppStoreId;
	struct CatalogMetaAssetInfo MetaAssetInfo;
	struct FString DisplayAssetPath;
	TArray<struct ItemQuantity> ItemGrants;
	TArray<struct CatalogOfferRequirement> Requirements;
	struct CatalogGiftInfo GiftInfo;
	bool Refundable;
	TArray<struct FString> DenyItemTemplateIds;
};

struct FStorefront
{
	struct FString Name;
	TArray<struct CatalogOffer> CatalogEntries;
};

struct FCatalogDownload
{
	int RefreshIntervalHrs;
	int DailyPurchaseHrs;
	struct DateTime Expiration;
	TArray<struct Storefront> Storefronts;
};

struct FCatalogItemSalePrice
{
	int SalePrice;
	ECatalogSaleType SaleType;
	struct DateTime StartTime;
	struct DateTime EndTime;
};

struct FCatalogPurchaseInfoGift
{
	struct FString OfferId;
	EStoreCurrencyType Currency;
	struct FString CurrencySubType;
	int ExpectedTotalPrice;
	struct FString GameContext;
	TArray<struct FString> ReceiverAccountIds;
	struct FString GiftWrapTemplateId;
	struct FString PersonalMessage;
};

struct FCatalogPurchaseInfo
{
	struct FString OfferId;
	int PurchaseQuantity;
	EStoreCurrencyType Currency;
	struct FString CurrencySubType;
	int ExpectedTotalPrice;
	struct FString GameContext;
};

struct FCatalogReceiptInfo
{
	EAppStore AppStore;
	struct FString AppStoreId;
	struct FString ReceiptId;
	struct FString ReceiptInfo;
	struct FString PurchaseCorrelationId;
};

struct FKeychainDownload
{
};

struct FChaosCollisionEventRequestSettings
{
	int MaxNumberResults;
	float MinMass;
	float MinSpeed;
	float MinImpulse;
	float MaxDistance;
	EChaosCollisionSortMethod SortMethod;
};

struct FChaosBreakingEventRequestSettings
{
	int MaxNumberOfResults;
	float MinRadius;
	float MinSpeed;
	float MinMass;
	float MaxDistance;
	EChaosBreakingSortMethod SortMethod;
};

struct FChaosTrailingEventRequestSettings
{
	int MaxNumberOfResults;
	float MinMass;
	float MinSpeed;
	float MinAngularSpeed;
	float MaxDistance;
	EChaosTrailingSortMethod SortMethod;
};

struct FGeomComponentCacheParameters
{
	EGeometryCollectionCacheType CacheMode;
	class UClass* TargetCache;
	float ReverseCacheBeginTime;
	bool SaveCollisionData;
	bool DoGenerateCollisionData;
	int CollisionDataSizeMax;
	bool DoCollisionDataSpatialHash;
	float CollisionDataSpatialHashRadius;
	int MaxCollisionPerCell;
	bool SaveBreakingData;
	bool DoGenerateBreakingData;
	int BreakingDataSizeMax;
	bool DoBreakingDataSpatialHash;
	float BreakingDataSpatialHashRadius;
	int MaxBreakingPerCell;
	bool SaveTrailingData;
	bool DoGenerateTrailingData;
	int TrailingDataSizeMax;
	float TrailingMinSpeedThreshold;
	float TrailingMinVolumeThreshold;
};

struct FGeometryCollectionRepData
{
};

struct FGeometryCollectionDebugDrawWarningMessage
{
};

struct FGeometryCollectionDebugDrawActorSelectedRigidBody
{
	int ID;
	class UClass* Solver;
	class UClass* GeometryCollection;
};

struct FGeometryCollectionSource
{
	struct SoftObjectPath SourceGeometryObject;
	struct Transform LocalTransform;
	TArray<class UClass*> SourceMaterial;
};

struct FGeometryCollectionSizeSpecificData
{
	float MaxSize;
	ECollisionTypeEnum CollisionType;
	EImplicitTypeEnum ImplicitType;
	int MinLevelSetResolution;
	int MaxLevelSetResolution;
	int MinClusterLevelSetResolution;
	int MaxClusterLevelSetResolution;
	int CollisionObjectReductionPercentage;
	float CollisionParticlesFraction;
	int MaximumCollisionParticles;
};

struct FChaosCollisionEventData
{
	struct Vector Location;
	struct Vector Normal;
	struct Vector Velocity1;
	struct Vector Velocity2;
	float Mass1;
	float Mass2;
	struct Vector Impulse;
};

struct FChaosBreakingEventData
{
	struct Vector Location;
	struct Vector Velocity;
	float Mass;
};

struct FChaosTrailingEventData
{
	struct Vector Location;
	struct Vector Velocity;
	struct Vector AngularVelocity;
	float Mass;
	int ParticleIndex;
};

struct FMovieSceneGeometryCollectionParams
{
	struct SoftObjectPath GeometryCollectionCache;
	struct FrameNumber StartFrameOffset;
	struct FrameNumber EndFrameOffset;
	float PlayRate;
};

struct FFortGiftingRecipientState
{
	struct CatalogItemPrice Price;
	TArray<struct ItemQuantity> Items;
};

struct FXRMotionControllerData
{
	bool bValid;
	struct FName DeviceName;
	struct Guid ApplicationInstanceID;
	EXRVisualType DeviceVisualType;
	EControllerHand HandIndex;
	ETrackingStatus TrackingStatus;
	struct Vector GripPosition;
	struct Quat GripRotation;
	struct Vector AimPosition;
	struct Quat AimRotation;
	TArray<struct Vector> HandKeyPositions;
	TArray<struct Quat> HandKeyRotations;
	TArray<float> HandKeyRadii;
	bool bIsGrasped;
};

struct FXRHMDData
{
	bool bValid;
	struct FName DeviceName;
	struct Guid ApplicationInstanceID;
	ETrackingStatus TrackingStatus;
	struct Vector Position;
	struct Quat Rotation;
};

struct FXRDeviceId
{
	struct FName SystemName;
	int DeviceID;
};

struct FXRGestureConfig
{
	bool bTap;
	bool bHold;
	ESpatialInputGestureAxis AxisGesture;
	bool bNavigationAxisX;
	bool bNavigationAxisY;
	bool bNavigationAxisZ;
};

struct FFortHeliFlightModel
{
	class UClass* Configs;
};

struct FRotorHit
{
	Unknown HitActor;
	float LastHitTimer;
};

struct FCachedSeatCollision
{
	int SeatIndex;
	bool bOccupied;
};

struct FReplicatedHeliControlState
{
	struct Vector_NetQuantizeNormal Up;
	struct Vector_NetQuantizeNormal Forward;
};

struct FHoagieCmd
{
};

struct FImagePlateParameters
{
	class UClass* Material;
	struct FName TextureParameterName;
	bool bFillScreen;
	struct Vector2D FillScreenAmount;
	struct Vector2D FixedSize;
	class UClass* RenderTexture;
	class UClass* DynamicMaterial;
};

struct FMovieSceneImagePlateSectionParams
{
	struct FrameNumber SectionStartTime;
	class UClass* FileSequence;
	bool bReuseExistingTexture;
};

struct FImageWriteOptions
{
	EDesiredImageFormat Format;
	struct FDelegate OnComplete;
	int CompressionQuality;
	bool bOverwriteFile;
	bool bAsync;
};

struct FKey
{
	struct FName KeyName;
};

struct FBrushStampData
{
};

struct FBehaviorInfo
{
	class UClass* Behavior;
};

struct FActiveGizmo
{
};

struct FGizmoFloatParameterChange
{
	float InitialValue;
	float CurrentValue;
};

struct FGizmoVec2ParameterChange
{
	struct Vector2D InitialValue;
	struct Vector2D CurrentValue;
};

struct FInputRayHit
{
};

struct FJsonObjectWrapper
{
	struct FString JsonString;
};

struct FKairosAvatarAttributesMcpData
{
	TArray<struct McpVariantReader> Variants;
};

struct FKairosAvatarItemMcpData
{
	struct FString TemplateId;
	struct KairosAvatarAttributesMcpData Attributes;
};

struct FKairosAvatarMcpData
{
	struct FString ID;
	struct FString DisplayName;
	bool CurrentUser;
	bool ShouldPlayFacialAnimation;
	struct KairosAvatarItemMcpData Character;
	struct KairosAvatarItemMcpData Backpack;
	struct KairosAvatarItemMcpData Dance;
	TArray<struct FString> BackgroundColors;
};

struct FKairosCaptureParams
{
	int FrameRate;
	int OverrideNumFrames;
	struct Vector2D Size;
	struct Color BackgroundColor;
	struct FString FullPathAndName;
};

struct FKairosAvatarDisplayInfo
{
	EKairosAvatarCaptureState CurrentState;
	EKairosAvatarCaptureState TargetState;
	struct Transform SpawnTransform;
	struct KairosAvatarMcpData AvatarData;
	struct FortAthenaLoadout Loadout;
	class UClass* AnimToPlay;
	struct FString SnippetPathToPlay;
	class UClass* HeroItem;
	struct KairosCaptureParams CaptureParams;
	TArray<struct LinearColor> BackgroundColors;
	class UClass* KairosHero;
	class UClass* Pawn;
	bool bIsCurrentUser;
	struct FString DisplayName;
	struct Transform HeadBoneTransform;
	bool bSpawnTransformIsAbsolute;
	bool bIsNewCharacter;
	class UClass* HeroPawnClass;
	bool bUseDefaultFrontendAnimClass;
};

struct FKairosSceneInfo
{
	EKairosAppMode SceneAppMode;
	TArray<struct KairosAvatarDisplayInfo> SpawnedAvatars;
	struct Color BackgroundColor;
};

struct FKairosSnippetPlayRecord
{
	struct FString PlayerID;
	bool bPause;
};

struct FKairosSnippetParams
{
	struct FString PlayerID;
	bool bPause;
};

struct FKairosCaptureAvatarParams
{
	struct FString Background;
	struct FString Format;
	int Width;
	int Height;
	int FrameRate;
	int NumFrames;
	TArray<struct KairosAvatarMcpData> Characters;
};

struct FKairosBobbleTimeParams
{
	struct FString Background;
	TArray<struct KairosAvatarMcpData> Characters;
	struct FString SnippetPath;
	struct FString ContainerType;
	struct FString ContainerID;
	struct FString MessageID;
};

struct FKairosAnimationFramingInfo
{
	struct FName Name;
	class UClass* IdleMontage;
	EKairosHeroAnimationType AnimationType;
	EKairosHeroSkeletonType SkeletonType;
	struct Transform CameraTransform;
};

struct FFortAthenaLivingWorldEncounterStage
{
	struct TSoftClassPtr<UObject> EventTable;
	struct ScalableFloat MaximumConcurrentNumberOfAI;
};

struct FLivingWorldCalendarEventCondition
{
	struct FName CalendarEventName;
	ELivingWorldCalendarEventConditionBehavior Behavior;
	bool ShouldEventBeActive;
	ELivingWorldCalendarEventConditionRatioBehavior RatioBehavior;
	float RatioValue;
	float RatioMaxValue;
};

struct FLivingWorldCalendarEventConditions
{
	TArray<struct LivingWorldCalendarEventCondition> Conditions;
	bool IsActiveWithoutSeasonalManager;
};

struct FPointProviderFilterEntry
{
	struct GameplayTagQuery ProviderFiltersTagQuery;
	struct LivingWorldCalendarEventConditions CalendarEventConditions;
	struct ScalableFloat Weight;
};

struct FFortAthenaLivingWorldEventDataActorSpawnDescription
{
	struct TSoftClassPtr<UObject> ActorClass;
	struct TSoftClassPtr<UObject> SpawnerData;
	struct GameplayTagQuery SpawnerDataTagQuery;
	bool bUseProviderTagOverrides;
	struct GameplayTagQuery ProviderFiltersTagQueryOverride;
	bool bSpawnAroundDefaultPoint;
	class UClass* SpawnAroundEnvironmentQuery;
};

struct FFortLivingWorldConfigOverride
{
	struct TSoftClassPtr<UObject> SourceWorld;
	struct GameplayTagContainer PlaylistTag;
	struct TSoftClassPtr<UObject> LagerConfig;
	struct FortReleaseVersion StartVersion;
	struct FortReleaseVersion EndVersion;
};

struct FFortAthenaLivingWorldPointProviderSpawnLimiter
{
	ELivingWorldPointProviderSpawnLimiterBehavior Behavior;
	struct ScalableFloat MaxNumberOfSpawn;
	bool bResetLimitWhenEnabling;
};

struct FFortAthenaLivingWorldPointProviderFilterRules
{
};

struct FBrushEffectBlurring
{
	bool bBlurShape;
	int Radius;
};

struct FBrushEffectCurlNoise
{
	float Curl1Amount;
	float Curl2Amount;
	float Curl1Tiling;
	float Curl2Tiling;
};

struct FBrushEffectDisplacement
{
	float DisplacementHeight;
	float DisplacementTiling;
	class UClass* Texture;
	float Midpoint;
	struct LinearColor Channel;
	float WeightmapInfluence;
};

struct FBrushEffectSmoothBlending
{
	float InnerSmoothDistance;
	float OuterSmoothDistance;
};

struct FBrushEffectTerracing
{
	float TerraceAlpha;
	float TerraceSpacing;
	float TerraceSmoothness;
	float MaskLength;
	float MaskStartOffset;
};

struct FLandmassBrushEffectsList
{
	struct BrushEffectBlurring Blurring;
	struct BrushEffectCurlNoise CurlNoise;
	struct BrushEffectDisplacement Displacement;
	struct BrushEffectSmoothBlending SmoothBlending;
	struct BrushEffectTerracing Terracing;
};

struct FBrushEffectCurves
{
	bool bUseCurveChannel;
	class UClass* ElevationCurveAsset;
	float ChannelEdgeOffset;
	float ChannelDepth;
	float CurveRampWidth;
};

struct FLandmassFalloffSettings
{
	EBrushFalloffMode FalloffMode;
	float FalloffAngle;
	float FalloffWidth;
	float EdgeOffset;
	float ZOffset;
};

struct FLandmassTerrainCarvingSettings
{
	EBrushBlendType BlendMode;
	bool bInvertShape;
	struct LandmassFalloffSettings FalloffSettings;
	struct LandmassBrushEffectsList Effects;
	int Priority;
};

struct FLandscapeProxyMaterialOverride
{
	struct PerPlatformInt LODIndex;
	class UClass* Material;
};

struct FLandscapeComponentMaterialOverride
{
	struct PerPlatformInt LODIndex;
	class UClass* Material;
};

struct FWeightmapLayerAllocationInfo
{
	class UClass* LayerInfo;
	byte WeightmapTextureIndex;
	byte WeightmapTextureChannel;
};

struct FGrassVariety
{
	class UClass* GrassMesh;
	TArray<class UClass*> OverrideMaterials;
	struct PerPlatformFloat GrassDensity;
	bool bUseGrid;
	float PlacementJitter;
	struct PerPlatformInt StartCullDistance;
	struct PerPlatformInt EndCullDistance;
	int MinLOD;
	EGrassScaling Scaling;
	struct FloatInterval ScaleX;
	struct FloatInterval ScaleY;
	struct FloatInterval ScaleZ;
	bool RandomRotation;
	bool AlignToSurface;
	bool bUseLandscapeLightmap;
	struct LightingChannels LightingChannels;
	bool bReceivesDecals;
	bool bCastDynamicShadow;
	bool bKeepInstanceBufferCPUCopy;
};

struct FLandscapeMaterialTextureStreamingInfo
{
	struct FName TextureName;
	float TexelFactor;
};

struct FLandscapeSplineConnection
{
	class UClass* Segment;
	bool End;
};

struct FLandscapeSplineInterpPoint
{
	struct Vector Center;
	struct Vector Left;
	struct Vector Right;
	struct Vector FalloffLeft;
	struct Vector FalloffRight;
	struct Vector LayerLeft;
	struct Vector LayerRight;
	struct Vector LayerFalloffLeft;
	struct Vector LayerFalloffRight;
	float StartEndFalloff;
};

struct FLandscapeSplineSegmentConnection
{
	class UClass* ControlPoint;
	float TangentLen;
	struct FName SocketName;
};

struct FGrassInput
{
	struct FName Name;
	class UClass* GrassType;
	struct ExpressionInput Input;
};

struct FLayerBlendInput
{
	struct FName LayerName;
	ELandscapeLayerBlendType BlendType;
	struct ExpressionInput LayerInput;
	struct ExpressionInput HeightInput;
	float PreviewWeight;
	struct Vector ConstLayerInput;
	float ConstHeightInput;
};

struct FPhysicalMaterialInput
{
	class UClass* PhysicalMaterial;
	struct ExpressionInput Input;
};

struct FLandscapeLayerBrush
{
};

struct FLandscapeLayer
{
	struct Guid Guid;
	struct FName Name;
	bool bVisible;
	bool bLocked;
	float HeightmapAlpha;
	float WeightmapAlpha;
	ELandscapeBlendMode BlendMode;
	TArray<struct LandscapeLayerBrush> Brushes;
	Unknown WeightmapLayerAllocationBlend;
};

struct FHeightmapData
{
	class UClass* Texture;
};

struct FWeightmapData
{
	TArray<class UClass*> Textures;
	TArray<struct WeightmapLayerAllocationInfo> LayerAllocations;
	TArray<class UClass*> TextureUsages;
};

struct FLandscapeLayerComponentData
{
	struct HeightmapData HeightmapData;
	struct WeightmapData WeightmapData;
};

struct FLandscapeEditToolRenderData
{
	class UClass* ToolMaterial;
	class UClass* GizmoMaterial;
	int SelectedType;
	int DebugChannelR;
	int DebugChannelG;
	int DebugChannelB;
	class UClass* DataTexture;
	class UClass* LayerContributionTexture;
	class UClass* DirtyTexture;
};

struct FGizmoSelectData
{
};

struct FLandscapeInfoLayerSettings
{
	class UClass* LayerInfoObj;
	struct FName LayerName;
};

struct FLandscapeImportLayerInfo
{
};

struct FLandscapeLayerStruct
{
	class UClass* LayerInfoObj;
};

struct FLandscapeEditorLayerSettings
{
};

struct FForeignWorldSplineData
{
};

struct FForeignSplineSegmentData
{
};

struct FForeignControlPointData
{
};

struct FLandscapeSplineMeshEntry
{
	class UClass* Mesh;
	TArray<class UClass*> MaterialOverrides;
	bool bCenterH;
	struct Vector2D CenterAdjust;
	bool bScaleToWidth;
	struct Vector Scale;
	ELandscapeSplineMeshOrientation Orientation;
	ESplineMeshAxis ForwardAxis;
	ESplineMeshAxis UpAxis;
};

struct FChatChromeColorScheme
{
	struct LinearColor ChatEntryBackgroundColor;
	struct LinearColor NoneActiveTabColor;
	struct LinearColor TabFontColor;
	struct LinearColor TabFontColorInverted;
	struct LinearColor ChatBackgroundColor;
};

struct FChatChromeMargins
{
	float TabWidth;
	struct Margin TabPadding;
	struct Margin ChatWindowPadding;
	struct Margin ChatWindowToEntryMargin;
	struct Margin ChatChannelPadding;
	struct Margin UserListButtonPadding;
	struct Margin UserListIconPadding;
};

struct FChatChromeStyle
{
	struct SlateBrush UserListBrush;
	struct SlateBrush ChatBackgroundBrush;
	struct SlateBrush ChatEntryBackgroundBrush;
	struct SlateBrush ChannelBackgroundBrush;
	struct SlateBrush TabBackgroundBrush;
	struct ButtonStyle TabSelectorButtonStyle;
	struct Margin TabOptionPadding;
	struct Margin TabContentPadding;
	struct Margin TabClosePadding;
	struct ButtonStyle UserListButtonStyle;
};

struct FChatColorScheme
{
	struct LinearColor TimeStampColor;
	struct LinearColor DefaultChatColor;
	struct LinearColor WhisperChatColor;
	struct LinearColor GlobalChatColor;
	struct LinearColor FounderChatColor;
	struct LinearColor GameChatColor;
	struct LinearColor TeamChatColor;
	struct LinearColor PartyChatColor;
	struct LinearColor AdminChatColor;
	struct LinearColor GameAdminChatColor;
	struct LinearColor WhisperHyperlinkChatColor;
	struct LinearColor GlobalHyperlinkChatColor;
	struct LinearColor FounderHyperlinkChatColor;
	struct LinearColor GameHyperlinkChatColor;
	struct LinearColor TeamHyperlinkChatColor;
	struct LinearColor PartyHyperlinkChatColor;
	struct LinearColor EnemyColor;
	struct LinearColor FriendlyColor;
};

struct FChatMarkupStyle
{
	struct ButtonStyle MarkupButtonStyle;
	struct TextBlockStyle MarkupTextStyle;
	struct SlateBrush MarkupBackground;
	struct SlateColor ButtonColor;
	struct SlateColor ButtonHoverColor;
	struct SlateColor TipColor;
	struct SlateBrush SeperatorBrush;
	float SeperatorThickness;
	struct Margin MarkupPadding;
	struct Margin ButtonPadding;
};

struct FChatStyle
{
	struct EditableTextBoxStyle ChatEntryTextStyle;
	struct EditableTextBoxStyle ChatDisplayTextStyle;
	struct ScrollBoxStyle ScrollBorderStyle;
	struct SlateBrush MessageNotificationBrush;
	struct Margin ChatEntryPadding;
	float ChatEntryHeight;
	struct SlateBrush ChatMenuBackgroundBrush;
	struct Margin FriendActionPadding;
	struct Margin FriendActionHeaderPadding;
	struct Margin FriendActionStatusMargin;
};

struct FSocialFontStyle
{
	struct SlateFontInfo FontSmall;
	struct SlateFontInfo FontSmallBold;
	struct SlateFontInfo FontNormal;
	struct SlateFontInfo FontNormalBold;
	struct SlateFontInfo FontLarge;
	struct SlateFontInfo FontLargeBold;
	struct LinearColor DefaultFontColor;
	struct LinearColor InvertedFontColor;
	struct LinearColor DefaultDullFontColor;
};

struct FSocialListMargins
{
	struct Vector2D UserPresenceImageSize;
	struct Margin HeaderButtonMargin;
	struct Margin FriendsListMargin;
	struct Margin FriendsListNoFriendsMargin;
	struct Margin FriendsListHeaderMargin;
	struct Margin FriendsListHeaderCountMargin;
	struct Margin HeaderButtonContentMargin;
	struct Margin FriendItemMargin;
	struct Margin FriendItemStatusMargin;
	struct Margin FriendTipStatusMargin;
	struct Margin FriendItemPresenceMargin;
	struct Margin FriendItemPlatformMargin;
	struct Margin FriendItemTextScrollerMargin;
	struct Margin ConfirmationBorderMargin;
	struct Margin ConfirmationButtonMargin;
	struct Margin ConfirmationButtonContentMargin;
	struct Margin NoneFriendContentMargin;
	float NoneFriendContentHeight;
	float NoneFriendIconWidth;
	struct Margin SubMenuBackIconMargin;
	struct Margin SubMenuPageIconMargin;
	struct Margin RadioSettingTitleMargin;
	struct Margin SubMenuSearchIconMargin;
	struct Margin SubMenuSearchTextMargin;
	struct Margin SubMenuBackButtonMargin;
	struct Margin SubMenuSettingButtonMargin;
	struct Margin SubMenuListMargin;
	float SubMenuSeperatorThickness;
	float PresenceSeperatorThickness;
	struct Margin FriendTipMargin;
	struct Margin FriendTipPresenceMargin;
	struct Margin FriendTipSeperatorMargin;
	struct Margin ToolTipMargin;
	struct Margin TipStatusMargin;
	struct Margin AddButtonMargin;
	struct Vector2D AddButtonSpacing;
};

struct FSocialListStyle
{
	struct ButtonStyle GlobalChatButtonStyle;
	struct SlateBrush GlobalChatIcon;
	struct ButtonStyle FriendItemButtonStyle;
	struct ButtonStyle ConfirmButtonStyle;
	struct ButtonStyle CancelButtonStyle;
	struct SlateColor ButtonContentColor;
	struct SlateColor ButtonHoverContentColor;
	struct SlateBrush ActionMenuArrowBrush;
	struct SlateBrush ActionMenuArrowRightBrush;
	struct SlateColor ActionMenuBackgroundColor;
	struct SlateBrush ToolTipArrowBrush;
	struct ButtonStyle BackButtonStyle;
	struct ButtonStyle HeaderButtonStyle;
	struct ButtonStyle FriendListActionButtonStyle;
	struct SlateBrush AddFriendButtonContentBrush;
	struct SlateBrush StatusIconBrush;
	struct SlateBrush PCIconBrush;
	struct SlateBrush ConsoleIconBrush;
	struct SlateBrush MobileIconBrush;
	struct SlateBrush FacebookIconBrush;
	struct SlateBrush EpicIconBrush;
	struct SlateBrush FriendImageBrush;
	struct SlateBrush OfflineBrush;
	struct SlateBrush OnlineBrush;
	struct SlateBrush AwayBrush;
	struct SlateBrush SpectateBrush;
	struct SlateBrush FriendsContainerBackground;
	struct SlateBrush FriendsListBackground;
	struct EditableTextBoxStyle AddFriendEditableTextStyle;
	struct SlateBrush BackBrush;
	struct SlateBrush SelectedOptionBrush;
	struct SlateBrush SettingsBrush;
	struct SlateBrush SeperatorBrush;
	struct SlateBrush PresenceSeperatorBrush;
	struct SlateBrush FontSizeBrush;
	struct SlateBrush SearchBrush;
};

struct FProfanityData
{
	struct FString CountryCode;
	struct FString ProfanityList;
	struct FString WhiteList;
	bool bAutoAdd;
};

struct FSocialSoundSchema
{
	struct SlateSound MessageReceivedSound;
	struct SlateSound PartyInviteReceivedSound;
	struct SlateSound FriendInviteReceivedSound;
};

struct FSocialStyle
{
	struct ScrollBarStyle ScrollBarStyle;
	struct ButtonStyle ActionButtonStyle;
	struct SocialFontStyle SmallFontStyle;
	struct SocialFontStyle NormalFontStyle;
	struct SocialFontStyle LargeFontStyle;
	struct SocialFontStyle ChatFontStyle;
	struct CheckBoxStyle CheckBoxStyle;
	struct CheckBoxStyle RadioBoxStyle;
	struct SocialListStyle SocialListStyle;
	struct SocialListMargins SocialListMargins;
	struct ChatStyle ChatStyle;
	struct ChatColorScheme ChatColorScheme;
	struct ChatChromeStyle ChatChromeStyle;
	struct ChatChromeMargins ChatChromeMargins;
	struct ChatChromeColorScheme ChatChromeColorScheme;
	struct ChatMarkupStyle ChatMarkupStyle;
	struct SocialSoundSchema SoundSchema;
};

struct FLevelSequenceObjectReferenceMap
{
};

struct FLevelSequenceBindingReference
{
	struct FString PackageName;
	struct SoftObjectPath ExternalObjectPath;
	struct FString ObjectPath;
};

struct FLevelSequenceBindingReferenceArray
{
	TArray<struct LevelSequenceBindingReference> References;
};

struct FLevelSequenceBindingReferences
{
	Unknown BindingIdToReferences;
	Unknown AnimSequenceInstances;
};

struct FLevelSequenceObject
{
	Unknown ObjectOrOwner;
	struct FString ComponentName;
	Unknown CachedComponent;
};

struct FLevelSequenceCameraSettings
{
	bool bOverrideAspectRatioAxisConstraint;
	EAspectRatioAxisConstraint AspectRatioAxisConstraint;
};

struct FLevelSequenceAnimSequenceLinkItem
{
	struct Guid SkelTrackGuid;
	struct SoftObjectPath PathToAnimSequence;
	bool bExportTransforms;
	bool bExportCurves;
	bool bRecordInWorldSpace;
};

struct FLevelSequenceSnapshotSettings
{
	byte ZeroPadAmount;
	struct FrameRate FrameRate;
};

struct FLevelSequencePlayerSnapshot
{
	struct FString MasterName;
	struct QualifiedFrameTime MasterTime;
	struct QualifiedFrameTime SourceTime;
	struct FString CurrentShotName;
	struct QualifiedFrameTime CurrentShotLocalTime;
	struct QualifiedFrameTime CurrentShotSourceTime;
	struct FString SourceTimecode;
	struct TSoftClassPtr<UObject> CameraComponent;
	struct LevelSequenceSnapshotSettings Settings;
	class UClass* ActiveShot;
	struct MovieSceneSequenceID ShotID;
};

struct FBoundActorProxy
{
};

struct FLevelSequenceLegacyObjectReference
{
};

struct FLiveLinkTransformControllerData
{
	bool bWorldTransform;
	bool bUseScale;
	bool bSweep;
	bool bTeleport;
};

struct FLiveLinkSubjectName
{
	struct FName Name;
};

struct FLiveLinkSourceBufferManagementSettings
{
	bool bValidEngineTimeEnabled;
	float ValidEngineTime;
	float EngineTimeOffset;
	double EngineTimeClockOffset;
	bool bGenerateSubFrame;
	struct FrameRate DetectedFrameRate;
	bool bUseTimecodeSmoothLatest;
	struct FrameRate SourceTimecodeFrameRate;
	bool bValidTimecodeFrameEnabled;
	int ValidTimecodeFrame;
	float TimecodeFrameOffset;
	double TimecodeClockOffset;
	int LatestOffset;
	int MaxNumberOfFrameToBuffered;
	bool bKeepAtLeastOneFrame;
};

struct FLiveLinkCurveConversionSettings
{
	Unknown CurveConversionAssetMap;
};

struct FLiveLinkSubjectKey
{
	struct Guid Source;
	struct LiveLinkSubjectName SubjectName;
};

struct FLiveLinkSubjectPreset
{
	struct LiveLinkSubjectKey Key;
	class UClass* Role;
	class UClass* Settings;
	class UClass* VirtualSubject;
	bool bEnabled;
};

struct FLiveLinkWorldTime
{
	double Time;
	double Offset;
};

struct FLiveLinkMetaData
{
	Unknown StringMetaData;
	struct QualifiedFrameTime SceneTime;
};

struct FLiveLinkBaseFrameData
{
	struct LiveLinkWorldTime WorldTime;
	struct LiveLinkMetaData MetaData;
	TArray<float> PropertyValues;
};

struct FLiveLinkBaseStaticData
{
	TArray<struct FName> PropertyNames;
};

struct FLiveLinkSourceHandle
{
};

struct FLiveLinkBaseBlueprintData
{
};

struct FLiveLinkTransform
{
};

struct FCachedSubjectFrame
{
};

struct FSubjectMetadata
{
	Unknown StringMetaData;
	struct Timecode SceneTimecode;
	struct FrameRate SceneFramerate;
};

struct FLiveLinkSourcePreset
{
	struct Guid Guid;
	class UClass* Settings;
	struct FText SourceType;
};

struct FLiveLinkRefSkeleton
{
	TArray<struct FName> BoneNames;
	TArray<int> BoneParents;
};

struct FLiveLinkSubjectRepresentation
{
	struct LiveLinkSubjectName Subject;
	class UClass* Role;
};

struct FLiveLinkInterpolationSettings
{
	bool bUseInterpolation;
	float InterpolationOffset;
};

struct FLiveLinkTimeSynchronizationSettings
{
	struct FrameRate FrameRate;
	struct FrameNumber FrameOffset;
};

struct FLiveLinkSourceDebugInfo
{
	struct LiveLinkSubjectName SubjectName;
	int SnapshotIndex;
	int NumberOfBufferAtSnapshot;
};

struct FLiveLinkCurveElement
{
	struct FName CurveName;
	float CurveValue;
};

struct FLiveLinkFrameData
{
	TArray<struct Transform> Transforms;
	TArray<struct LiveLinkCurveElement> CurveElements;
	struct LiveLinkWorldTime WorldTime;
	struct LiveLinkMetaData MetaData;
};

struct FLiveLinkTimeCode_Base_DEPRECATED
{
	int Seconds;
	int Frames;
	struct LiveLinkFrameRate FrameRate;
};

struct FLiveLinkTime
{
	double WorldTime;
	struct QualifiedFrameTime SceneTime;
};

struct FLiveLinkSubjectFrameMessage
{
	struct FName SubjectName;
	TArray<struct Transform> Transforms;
	TArray<struct LiveLinkCurveElement> Curves;
	struct LiveLinkMetaData MetaData;
	double Time;
};

struct FLiveLinkSubjectDataMessage
{
	struct LiveLinkRefSkeleton RefSkeleton;
	struct FName SubjectName;
};

struct FLiveLinkClearSubject
{
	struct FName SubjectName;
};

struct FLiveLinkHeartbeatMessage
{
};

struct FLiveLinkConnectMessage
{
	int LiveLinkVersion;
};

struct FLiveLinkPongMessage
{
	struct FString ProviderName;
	struct FString MachineName;
	struct Guid PollRequest;
	int LiveLinkVersion;
	double CreationPlatformTime;
};

struct FLiveLinkPingMessage
{
	struct Guid PollRequest;
	int LiveLinkVersion;
};

struct FLiveLinkPropertyData
{
	struct FName PropertyName;
	TArray<struct MovieSceneFloatChannel> FloatChannel;
	TArray<struct MovieSceneStringChannel> StringChannel;
	TArray<struct MovieSceneIntegerChannel> IntegerChannel;
	TArray<struct MovieSceneBoolChannel> BoolChannel;
	TArray<struct MovieSceneByteChannel> ByteChannel;
};

struct FLiveLinkSubSectionData
{
	TArray<struct LiveLinkPropertyData> Properties;
};

struct FLiveLinkRoleProjectSetting
{
	class UClass* Role;
	class UClass* SettingClass;
	class UClass* FrameInterpolationProcessor;
	TArray<class UClass*> FramePreProcessors;
};

struct FProviderPollResult
{
	struct FString Name;
	struct FString MachineName;
	double MachineTimeOffset;
};

struct FLiveLinkRetargetAssetReference
{
};

struct FLiveStreamAnimationHandleWrapper
{
	struct FName Handle;
};

struct FLocalNotificationData
{
	bool Enable;
	ELocalNotificationType Type;
	ELocalNotificationEventType EventType;
	Unknown ParamsByName;
	bool LocalTime;
	struct FString TitleKey;
	struct FString BodyKey;
	struct FString ActivationEvent;
};

struct FLSALiveLinkTranslationProfile
{
	struct TSoftClassPtr<UObject> Skeleton;
	Unknown BoneRemappings;
	TArray<struct FName> BonesToUse;
};

struct FLSALiveLinkSourceOptions
{
	bool bWithSceneTime;
	bool bWithStringMetaData;
	bool bWithPropertyValues;
	bool bWithTransformTranslation;
	bool bWithTransformRotation;
	bool bWithTransformScale;
};

struct FFortMantisRootMotionWarpInfo
{
	class UClass* WarpTarget;
};

struct FFortMantisTechniqueData
{
	struct FName Name;
	bool bStartsSequence;
	bool bEndsSequence;
	float InputWindowDelay;
	class UClass* Montage;
	bool bUseRootMotion;
	struct GameplayTagQuery ActivationTagQuery;
	struct GameplayTagQuery OngoingTagQuery;
	struct GameplayTag ApplicationTag;
};

struct FFortMantisTechniqueBranch
{
	struct FName FromTechnique;
	struct FName ToTechnique;
	EFortMantisBranchPath BranchPath;
};

struct FFortMantisMontageData
{
	class UClass* Montage;
};

struct FMaterialQualityOverrides
{
	bool bDiscardQualityDuringCook;
	bool bEnableOverride;
	bool bForceFullyRough;
	bool bForceNonMetal;
	bool bForceDisableLMDirectionality;
	bool bForceLQReflections;
	bool bForceDisablePreintegratedGF;
	bool bDisableMaterialNormalCalculation;
	EMobileShadowQuality MobileShadowQuality;
};

struct FProfileEntry
{
	struct FString ProfileId;
	class UClass* ProfileObject;
	bool bWaitingForRefreshAllProfilesResponse;
	bool bForwardUpdatesToClient;
};

struct FProfileGroupEntry
{
	class UClass* ProfileGroup;
};

struct FMcpLootEntry
{
	struct FString ItemType;
	struct FString ItemGuid;
	int Quantity;
	struct JsonObjectWrapper Attributes;
	struct FString ItemProfile;
};

struct FProfileUpdateSingle
{
	int64_t ProfileRevision;
	struct FString ProfileId;
	int64_t ProfileChangesBaseRevision;
	TArray<struct JsonObjectWrapper> ProfileChanges;
	struct DateTime LockExpiration;
	TArray<struct JsonObjectWrapper> Notifications;
	int ProfileCommandRevision;
};

struct FBaseUrlContext
{
};

struct FMcpAddItemRequest
{
	struct FString ItemId;
	struct FString TemplateId;
	int Quantity;
	struct JsonObjectWrapper Attributes;
};

struct FMcpRemoveItemRequest
{
	struct FString ItemId;
};

struct FMcpChangeQuantityRequest
{
	struct FString ItemId;
	int DeltaQuantity;
};

struct FMcpChangeAttributesRequest
{
	struct FString ItemId;
	struct JsonObjectWrapper Attributes;
};

struct FMcpProfileChangeRequest
{
	int BaseCommandRevision;
	TArray<struct McpAddItemRequest> AddRequests;
	TArray<struct McpRemoveItemRequest> RemoveRequests;
	TArray<struct McpChangeQuantityRequest> ChangeQuantityRequests;
	TArray<struct McpChangeAttributesRequest> ChangeAttributesRequests;
	TArray<struct JsonObjectWrapper> ChangeStatRequests;
};

struct FProfileUpdateNotification
{
	TArray<struct JsonObjectWrapper> Changes;
	struct DateTime LockExpiration;
	int CommandRevision;
	int64_t Revision;
};

struct FAccountIdAndProfileResponse
{
	struct FString AccountId;
	struct ProfileUpdate Response;
};

struct FGiftBoxInfo
{
	struct DateTime GiftedOn;
	struct FString FromAccountId;
	TArray<struct McpLootEntry> LootList;
	struct JsonObjectWrapper Params;
};

struct FMcpLootResult
{
	struct FString TierGroupName;
	TArray<struct McpLootEntry> Items;
};

struct FMediaCaptureDevice
{
	struct FText DisplayName;
	struct FString URL;
};

struct FMediaSoundComponentSpectralData
{
	float FrequencyHz;
	float Magnitude;
};

struct FMediaPlayerTrackOptions
{
	int Audio;
	int Caption;
	int MetaData;
	int Script;
	int Subtitle;
	int Text;
	int Video;
};

struct FMediaPlayerOptions
{
	struct MediaPlayerTrackOptions Tracks;
	struct Timespan SeekTime;
	EMediaPlayerOptionBooleanOverride PlayOnOpen;
	EMediaPlayerOptionBooleanOverride Loop;
};

struct FElementID
{
	int IDValue;
};

struct FNoAggregationParameters
{
	Unknown Parameters;
};

struct FAggregatedFunction
{
	class UClass* Function;
};

struct FMeshMetaDataStruct
{
};

struct FMIDIDeviceInfo
{
	int DeviceID;
	struct FString DeviceName;
	bool bIsAlreadyInUse;
	bool bIsDefaultDevice;
};

struct FFoundMIDIDevice
{
	int DeviceID;
	struct FString DeviceName;
	bool bCanReceiveFrom;
	bool bCanSendTo;
	bool bIsAlreadyInUse;
	bool bIsDefaultInputDevice;
	bool bIsDefaultOutputDevice;
};

struct FPaddleGingerPropActivatedData
{
	bool bActivated;
	class UClass* Sender;
};

struct FPaddleGingerPropData
{
	struct FText CategoryLabelText;
	int PropIndex;
	int MaxPropCount;
	int CategoryIndex;
	int MaxCategoryCount;
	class UClass* Sender;
};

struct FMotoSynthRuntimeSettings
{
	bool bSynthToneEnabled;
	struct Vector2D SynthToneVolumeRange;
	struct Vector2D SynthToneFilterFrequencyRange;
	bool bSynthToneEnvelopeEnabled;
	struct Vector2D SynthToneAttackTimeMsecRange;
	struct Vector2D SynthToneAttackCurveRange;
	struct Vector2D SynthToneDecayTimeMsecRange;
	struct Vector2D SynthToneDecayCurveRange;
	int SynthOctaveShift;
	bool bNoiseEnabled;
	struct Vector2D NoiseVolumeRange;
	bool bNoiseEnvelopeEnabled;
	struct Vector2D NoiseLPFRange;
	struct Vector2D NoiseAttackTimeMsecRange;
	struct Vector2D NoiseAttackCurveRange;
	struct Vector2D NoiseDecayTimeMsecRange;
	struct Vector2D NoiseDecayCurveRange;
	bool bGranularEngineEnabled;
	float GranularEngineVolume;
	float GranularEnginePitchScale;
	int NumSamplesToCrossfadeBetweenGrains;
	int NumGrainTableEntriesPerGrain;
	int GrainTableRandomOffsetForConstantRPMs;
	int GrainCrossfadeSamplesForConstantRPMs;
	class UClass* AccelerationSource;
	class UClass* DecelerationSource;
	bool bStereoWidenerEnabled;
	float StereoDelayMsec;
	float StereoFeedback;
	float StereoWidenerWetlevel;
	float StereoWidenerDryLevel;
	float StereoWidenerDelayRatio;
	bool bStereoWidenerFilterEnabled;
	float StereoWidenerFilterFrequency;
	float StereoWidenerFilterQ;
};

struct FGrainTableEntry
{
	int SampleIndex;
	float RPM;
};

struct FCompositionGraphCapturePasses
{
	TArray<struct FString> Value;
};

struct FCaptureResolution
{
	int ResX;
	int ResY;
};

struct FMovieSceneCaptureSettings
{
	struct DirectoryPath OutputDirectory;
	class UClass* GameModeOverride;
	struct FString OutputFormat;
	bool bOverwriteExisting;
	bool bUseRelativeFrameNumbers;
	int HandleFrames;
	struct FString MovieExtension;
	byte ZeroPadFrameNumbers;
	struct FrameRate FrameRate;
	bool bUseCustomFrameRate;
	struct FrameRate CustomFrameRate;
	struct CaptureResolution Resolution;
	bool bEnableTextureStreaming;
	bool bCinematicEngineScalability;
	bool bCinematicMode;
	bool bAllowMovement;
	bool bAllowTurning;
	bool bShowPlayer;
	bool bShowHUD;
	bool bUsePathTracer;
	int PathTracerSamplePerPixel;
};

struct FFrameMetrics
{
	float TotalElapsedTime;
	float FrameDelta;
	int FrameNumber;
	int NumDroppedFrames;
};

struct FCapturedPixels
{
};

struct FCapturedPixelsID
{
	Unknown Identifiers;
};

struct FBoolParameterNameAndCurve
{
	struct FName ParameterName;
	struct MovieSceneBoolChannel ParameterCurve;
};

struct FScalarParameterNameAndCurve
{
	struct FName ParameterName;
	struct MovieSceneFloatChannel ParameterCurve;
};

struct FVector2DParameterNameAndCurves
{
	struct FName ParameterName;
	struct MovieSceneFloatChannel XCurve;
	struct MovieSceneFloatChannel YCurve;
};

struct FVectorParameterNameAndCurves
{
	struct FName ParameterName;
	struct MovieSceneFloatChannel XCurve;
	struct MovieSceneFloatChannel YCurve;
	struct MovieSceneFloatChannel ZCurve;
};

struct FColorParameterNameAndCurves
{
	struct FName ParameterName;
	struct MovieSceneFloatChannel RedCurve;
	struct MovieSceneFloatChannel GreenCurve;
	struct MovieSceneFloatChannel BlueCurve;
	struct MovieSceneFloatChannel AlphaCurve;
};

struct FTransformParameterNameAndCurves
{
	struct FName ParameterName;
	struct MovieSceneFloatChannel Translation;
	struct MovieSceneFloatChannel Rotation;
	struct MovieSceneFloatChannel Scale;
};

struct FMovieSceneTransformMask
{
	uint32_t Mask;
};

struct FMovieSceneActorReferenceKey
{
	struct MovieSceneObjectBindingID Object;
	struct FName ComponentName;
	struct FName SocketName;
};

struct FMovieSceneCameraAnimSectionData
{
	class UClass* CameraAnim;
	float PlayRate;
	float PlayScale;
	float BlendInTime;
	float BlendOutTime;
	bool bLooping;
};

struct FMovieSceneCameraShakeSectionData
{
	class UClass* ShakeClass;
	float PlayScale;
	ECameraShakePlaySpace Playspace;
	struct Rotator UserDefinedPlaySpace;
};

struct FMovieSceneCameraShakeSourceTrigger
{
	class UClass* ShakeClass;
	float PlayScale;
	ECameraShakePlaySpace Playspace;
	struct Rotator UserDefinedPlaySpace;
};

struct FMovieSceneEventPtrs
{
	class UClass* Function;
	Unknown BoundObjectProperty;
};

struct FMovieSceneEvent
{
	struct MovieSceneEventPtrs Ptrs;
};

struct FMovieSceneEventParameters
{
};

struct FEventPayload
{
	struct FName EventName;
	struct MovieSceneEventParameters Parameters;
};

struct FMovieSceneSkeletalAnimationParams
{
	class UClass* Animation;
	struct FrameNumber FirstLoopStartFrameOffset;
	struct FrameNumber StartFrameOffset;
	struct FrameNumber EndFrameOffset;
	float PlayRate;
	bool bReverse;
	struct FName SlotName;
	struct MovieSceneFloatChannel Weight;
	bool bSkipAnimNotifiers;
	bool bForceCustomMode;
	float StartOffset;
	float EndOffset;
};

struct FMovieSceneSkeletalAnimRootMotionTrackParams
{
};

struct FMovieSceneEventPayloadVariable
{
	struct FString Value;
};

struct FMovieSceneEventTriggerData
{
	struct MovieSceneEventPtrs Ptrs;
	struct Guid ObjectBindingID;
};

struct FLevelVisibilityComponentData
{
	class UClass* Section;
};

struct FMovieSceneSectionEvalOptions
{
	bool bCanEditCompletionMode;
	EMovieSceneCompletionMode CompletionMode;
};

struct FMovieSceneEasingSettings
{
	int AutoEaseInDuration;
	int AutoEaseOutDuration;
	Unknown EaseIn;
	bool bManualEaseIn;
	int ManualEaseInDuration;
	Unknown EaseOut;
	bool bManualEaseOut;
	int ManualEaseOutDuration;
};

struct FMovieSceneFrameRange
{
};

struct FOptionalMovieSceneBlendType
{
	EMovieSceneBlendType BlendType;
	bool bIsValid;
};

struct FMovieSceneTrackEvalOptions
{
	bool bCanEvaluateNearestSection;
	bool bEvalNearestSection;
	bool bEvaluateInPreroll;
	bool bEvaluateInPostroll;
	bool bEvaluateNearestSection;
};

struct FMovieSceneTrackEvaluationFieldEntry
{
	class UClass* Section;
	struct FrameNumberRange Range;
	struct FrameNumber ForcedTime;
	ESectionEvaluationFlags Flags;
	int16_t LegacySortOrder;
};

struct FMovieSceneTrackEvaluationField
{
	TArray<struct MovieSceneTrackEvaluationFieldEntry> Entries;
};

struct FMovieSceneSequenceLoopCount
{
	int Value;
};

struct FMovieSceneSequencePlaybackSettings
{
	bool bAutoPlay;
	struct MovieSceneSequenceLoopCount LoopCount;
	float PlayRate;
	float StartTime;
	bool bRandomStartTime;
	bool bRestoreState;
	bool bDisableMovementInput;
	bool bDisableLookAtInput;
	bool bHidePlayer;
	bool bHideHUD;
	bool bDisableCameraCuts;
	bool bPauseAtEnd;
};

struct FMovieSceneSequenceID
{
	uint32_t Value;
};

struct FMovieSceneRootEvaluationTemplateInstance
{
	Unknown WeakRootSequence;
	class UClass* CompiledDataManager;
	class UClass* EntitySystemLinker;
	Unknown DirectorInstances;
};

struct FMovieSceneSequenceReplProperties
{
	struct FrameTime LastKnownPosition;
	EMovieScenePlayerStatus LastKnownStatus;
	int LastKnownNumLoops;
};

struct FMovieSceneSectionParameters
{
	struct FrameNumber StartFrameOffset;
	bool bCanLoop;
	struct FrameNumber EndFrameOffset;
	struct FrameNumber FirstLoopStartFrameOffset;
	float TimeScale;
	int HierarchicalBias;
	float StartOffset;
	float PrerollTime;
	float PostrollTime;
};

struct FMovieSceneSpawnable
{
	struct Transform SpawnTransform;
	TArray<struct FName> Tags;
	bool bContinuouslyRespawn;
	bool bNetAddressableName;
	bool bEvaluateTracksWhenNotSpawned;
	struct Guid Guid;
	struct FString Name;
	class UClass* ObjectTemplate;
	TArray<struct Guid> ChildPossessables;
	ESpawnOwnership Ownership;
	struct FName LevelName;
};

struct FMovieScenePossessable
{
	TArray<struct FName> Tags;
	struct Guid Guid;
	struct FString Name;
	class UClass* PossessedObjectClass;
	struct Guid ParentGuid;
};

struct FMovieSceneBinding
{
	struct Guid ObjectGuid;
	struct FString BindingName;
	TArray<class UClass*> Tracks;
};

struct FMovieSceneObjectBindingID
{
	struct Guid Guid;
	int SequenceID;
	int ResolveParentIndex;
};

struct FMovieSceneObjectBindingIDs
{
	TArray<struct MovieSceneObjectBindingID> IDs;
};

struct FMovieSceneMarkedFrame
{
	struct FrameNumber FrameNumber;
	struct FString Label;
	bool bIsDeterminismFence;
};

struct FMovieSceneBindingOverrideData
{
	struct MovieSceneObjectBindingID ObjectBindingID;
	Unknown Object;
	bool bOverridesDefault;
};

struct FMovieSceneChannel
{
};

struct FMovieSceneTrackIdentifier
{
	uint32_t Value;
};

struct FMovieSceneEvalTemplatePtr
{
};

struct FMovieSceneTrackImplementationPtr
{
};

struct FMovieSceneEvaluationTrack
{
	struct Guid ObjectBindingID;
	uint16_t EvaluationPriority;
	EEvaluationMethod EvaluationMethod;
	Unknown SourceTrack;
	TArray<struct MovieSceneEvalTemplatePtr> ChildTemplates;
	struct MovieSceneTrackImplementationPtr TrackTemplate;
	struct FName EvaluationGroup;
	bool bEvaluateInPreroll;
	bool bEvaluateInPostroll;
	bool bTearDownPriority;
};

struct FMovieSceneEvaluationTemplateSerialNumber
{
	uint32_t Value;
};

struct FMovieSceneTemplateGenerationLedger
{
	struct MovieSceneTrackIdentifier LastTrackIdentifier;
	Unknown TrackSignatureToTrackIdentifier;
	Unknown SubSectionRanges;
};

struct FMovieSceneEvaluationTemplate
{
	Unknown Tracks;
	struct Guid SequenceSignature;
	struct MovieSceneEvaluationTemplateSerialNumber TemplateSerialNumber;
	struct MovieSceneTemplateGenerationLedger TemplateLedger;
};

struct FMovieSceneSequenceHierarchyNode
{
	struct MovieSceneSequenceID ParentID;
	TArray<struct MovieSceneSequenceID> Children;
};

struct FMovieSceneSubSequenceTree
{
};

struct FMovieSceneTimeTransform
{
	float TimeScale;
	struct FrameTime Offset;
};

struct FMovieSceneTimeWarping
{
	struct FrameNumber Start;
	struct FrameNumber End;
};

struct FMovieSceneNestedSequenceTransform
{
	struct MovieSceneTimeTransform LinearTransform;
	struct MovieSceneTimeWarping Warping;
};

struct FMovieSceneSequenceTransform
{
	struct MovieSceneTimeTransform LinearTransform;
	TArray<struct MovieSceneNestedSequenceTransform> NestedTransforms;
};

struct FMovieSceneSequenceInstanceDataPtr
{
};

struct FMovieSceneSubSequenceData
{
	struct SoftObjectPath Sequence;
	struct MovieSceneSequenceTransform OuterToInnerTransform;
	struct MovieSceneSequenceTransform RootToSequenceTransform;
	struct FrameRate TickResolution;
	struct MovieSceneSequenceID DeterministicSequenceID;
	struct MovieSceneFrameRange ParentPlayRange;
	struct FrameNumber ParentStartFrameOffset;
	struct FrameNumber ParentEndFrameOffset;
	struct FrameNumber ParentFirstLoopStartFrameOffset;
	bool bCanLoop;
	struct MovieSceneFrameRange PlayRange;
	struct MovieSceneFrameRange FullPlayRange;
	struct MovieSceneFrameRange UnwarpedPlayRange;
	struct MovieSceneFrameRange PreRollRange;
	struct MovieSceneFrameRange PostRollRange;
	int16_t HierarchicalBias;
	bool bHasHierarchicalEasing;
	struct MovieSceneSequenceInstanceDataPtr InstanceData;
	struct Guid SubSectionSignature;
};

struct FMovieSceneSequenceHierarchy
{
	struct MovieSceneSequenceHierarchyNode RootNode;
	struct MovieSceneSubSequenceTree Tree;
	Unknown SubSequences;
	Unknown Hierarchy;
};

struct FMovieSceneEvaluationFieldEntityTree
{
};

struct FMovieSceneEvaluationFieldEntityKey
{
	Unknown EntityOwner;
	uint32_t EntityID;
};

struct FMovieSceneEvaluationFieldEntity
{
	struct MovieSceneEvaluationFieldEntityKey Key;
	int SharedMetaDataIndex;
};

struct FMovieSceneEvaluationFieldEntityMetaData
{
	struct FString OverrideBoundPropertyPath;
	struct FrameNumber ForcedTime;
	ESectionEvaluationFlags Flags;
	bool bEvaluateInSequencePreRoll;
	bool bEvaluateInSequencePostRoll;
};

struct FMovieSceneEvaluationFieldSharedEntityMetaData
{
	struct Guid ObjectBindingID;
};

struct FMovieSceneEntityComponentField
{
	struct MovieSceneEvaluationFieldEntityTree PersistentEntityTree;
	struct MovieSceneEvaluationFieldEntityTree OneShotEntityTree;
	TArray<struct MovieSceneEvaluationFieldEntity> Entities;
	TArray<struct MovieSceneEvaluationFieldEntityMetaData> EntityMetaData;
	TArray<struct MovieSceneEvaluationFieldSharedEntityMetaData> SharedMetaData;
};

struct FMovieSceneEvaluationGroupLUTIndex
{
	int NumInitPtrs;
	int NumEvalPtrs;
};

struct FMovieSceneEvaluationFieldTrackPtr
{
	struct MovieSceneSequenceID SequenceID;
	struct MovieSceneTrackIdentifier TrackIdentifier;
};

struct FMovieSceneFieldEntry_EvaluationTrack
{
	struct MovieSceneEvaluationFieldTrackPtr TrackPtr;
	uint16_t NumChildren;
};

struct FMovieSceneFieldEntry_ChildTemplate
{
	uint16_t ChildIndex;
	ESectionEvaluationFlags Flags;
	struct FrameNumber ForcedTime;
};

struct FMovieSceneEvaluationGroup
{
	TArray<struct MovieSceneEvaluationGroupLUTIndex> LUTIndices;
	TArray<struct MovieSceneFieldEntry_EvaluationTrack> TrackLUT;
	TArray<struct MovieSceneFieldEntry_ChildTemplate> SectionLUT;
};

struct FMovieSceneEvaluationKey
{
	struct MovieSceneSequenceID SequenceID;
	struct MovieSceneTrackIdentifier TrackIdentifier;
	uint32_t SectionIndex;
};

struct FMovieSceneOrderedEvaluationKey
{
	struct MovieSceneEvaluationKey Key;
	uint16_t SetupIndex;
	uint16_t TearDownIndex;
};

struct FMovieSceneEvaluationMetaData
{
	TArray<struct MovieSceneSequenceID> ActiveSequences;
	TArray<struct MovieSceneOrderedEvaluationKey> ActiveEntities;
};

struct FMovieSceneEvaluationField
{
	TArray<struct MovieSceneFrameRange> Ranges;
	TArray<struct MovieSceneEvaluationGroup> Groups;
	TArray<struct MovieSceneEvaluationMetaData> MetaData;
};

struct FMovieSceneSequenceCompilerMaskStruct
{
	bool bHierarchy;
	bool bEvaluationTemplate;
	bool bEvaluationTemplateField;
	bool bEntityComponentField;
};

struct FMovieSceneEntitySystemGraphNodes
{
};

struct FMovieSceneEntitySystemGraph
{
	struct MovieSceneEntitySystemGraphNodes Nodes;
};

struct FMovieSceneEvaluationInstanceKey
{
};

struct FMovieSceneEvaluationHookComponent
{
	Unknown Interface;
};

struct FMovieSceneEvaluationHookEvent
{
	struct MovieSceneEvaluationHookComponent Hook;
};

struct FMovieSceneEvaluationHookEventContainer
{
	TArray<struct MovieSceneEvaluationHookEvent> Events;
};

struct FMovieSceneSequenceActorPointers
{
	class UClass* SequenceActor;
	Unknown SequenceActorInterface;
};

struct FMovieSceneTrackInstanceInput
{
	class UClass* Section;
};

struct FMovieSceneEvalTemplateBase
{
};

struct FMovieSceneTangentData
{
	float ArriveTangent;
	float LeaveTangent;
	float ArriveTangentWeight;
	float LeaveTangentWeight;
	ERichCurveTangentWeightMode TangentWeightMode;
};

struct FMovieSceneFloatValue
{
	float Value;
	struct MovieSceneTangentData Tangent;
	ERichCurveInterpMode InterpMode;
	ERichCurveTangentMode TangentMode;
	byte PaddingByte;
};

struct FMovieScenePropertyBinding
{
	struct FName PropertyName;
	struct FName PropertyPath;
	bool bCanUseClassLookup;
};

struct FMovieScenePropertySectionData
{
	struct FName PropertyName;
	struct FString PropertyPath;
};

struct FMovieSceneSequenceInstanceData
{
};

struct FMovieSceneEvaluationOperand
{
	struct Guid ObjectBindingID;
	struct MovieSceneSequenceID SequenceID;
};

struct FTrackInstanceInputComponent
{
	class UClass* Section;
	int OutputIndex;
};

struct FMovieSceneTrackInstanceComponent
{
	class UClass* Owner;
	class UClass* TrackInstanceClass;
};

struct FEasingComponentData
{
	class UClass* Section;
};

struct FMovieSceneDeterminismData
{
	TArray<struct FrameTime> Fences;
	bool bParentSequenceRequiresLowerFence;
	bool bParentSequenceRequiresUpperFence;
};

struct FMovieSceneSectionGroup
{
	TArray<Unknown> Sections;
};

struct FMovieSceneTrackLabels
{
	TArray<struct FString> Strings;
};

struct FMovieSceneExpansionState
{
	bool bExpanded;
};

struct FMovieSceneEditorData
{
	Unknown ExpansionStates;
	TArray<struct FString> PinnedNodes;
	double ViewStart;
	double ViewEnd;
	double WorkStart;
	double WorkEnd;
	Unknown MarkedFrames;
	struct FloatRange WorkingRange;
	struct FloatRange ViewRange;
};

struct FMovieSceneTimecodeSource
{
	struct Timecode Timecode;
	struct FrameNumber DeltaFrame;
};

struct FMovieSceneCompiledSequenceFlagStruct
{
	bool bParentSequenceRequiresLowerFence;
	bool bParentSequenceRequiresUpperFence;
};

struct FMovieSceneEntitySystemGraphNode
{
	class UClass* System;
};

struct FMovieSceneEmptyStruct
{
};

struct FMovieSceneSegmentIdentifier
{
	int IdentifierIndex;
};

struct FMovieSceneSubSectionData
{
	Unknown Section;
	struct Guid ObjectBindingID;
	ESectionEvaluationFlags Flags;
};

struct FMovieSceneKeyStruct
{
};

struct FGeneratedMovieSceneKeyStruct
{
};

struct FMovieSceneObjectPathChannelKeyValue
{
	struct TSoftClassPtr<UObject> SoftPtr;
	class UClass* HardPtr;
};

struct FMovieSceneSegment
{
};

struct FSectionEvaluationData
{
	int ImplIndex;
	struct FrameNumber ForcedTime;
	ESectionEvaluationFlags Flags;
};

struct FMovieSceneSubSequenceTreeEntry
{
};

struct FMovieSceneSequencePlaybackParams
{
	struct FrameTime Frame;
	float Time;
	struct FString MarkedFrame;
	EMovieScenePositionType PositionType;
	EUpdatePositionMethod UpdateMethod;
};

struct FMovieSceneWarpCounter
{
	TArray<uint32_t> WarpCounts;
};

struct FMovieSceneTrackDisplayOptions
{
	bool bShowVerticalFrames;
};

struct FMovieSceneTrackInstanceEntry
{
	class UClass* BoundObject;
	class UClass* TrackInstance;
};

struct FMRMeshConfiguration
{
};

struct FSupportedAreaData
{
	struct FString AreaClassName;
	int AreaID;
	class UClass* AreaClass;
};

struct FNavigationFilterArea
{
	class UClass* AreaClass;
	float TravelCostOverride;
	float EnteringCostOverride;
	bool bIsExcluded;
	bool bOverrideTravelCost;
	bool bOverrideEnteringCost;
};

struct FNavigationFilterFlags
{
	bool bNavFlag0;
	bool bNavFlag1;
	bool bNavFlag2;
	bool bNavFlag3;
	bool bNavFlag4;
	bool bNavFlag5;
	bool bNavFlag6;
	bool bNavFlag7;
	bool bNavFlag8;
	bool bNavFlag9;
	bool bNavFlag10;
	bool bNavFlag11;
	bool bNavFlag12;
	bool bNavFlag13;
	bool bNavFlag14;
	bool bNavFlag15;
};

struct FNavCollisionCylinder
{
	struct Vector Offset;
	float Radius;
	float Height;
};

struct FNavCollisionBox
{
	struct Vector Offset;
	struct Vector Extent;
};

struct FNavGraphNode
{
	class UClass* Owner;
};

struct FNavGraphEdge
{
};

struct FRecastNavMeshGenerationProperties
{
	int TilePoolSize;
	float TileSizeUU;
	float CellSize;
	float CellHeight;
	float AgentRadius;
	float AgentHeight;
	float AgentMaxSlope;
	float AgentMaxStepHeight;
	float MinRegionArea;
	float MergeRegionSize;
	float MaxSimplificationError;
	int TileNumberHardLimit;
	ERecastPartitioning RegionPartitioning;
	ERecastPartitioning LayerPartitioning;
	int RegionChunkSplits;
	int LayerChunkSplits;
	bool bSortNavigationAreasByCost;
	bool bPerformVoxelFiltering;
	bool bMarkLowHeightAreas;
	bool bUseExtraTopCellWhenMarkingAreas;
	bool bFilterLowSpanSequences;
	bool bFilterLowSpanFromTileCache;
	bool bFixedTilePoolSize;
};

struct FNetAnalyticsDataConfig
{
	struct FName DataName;
	bool bEnabled;
};

struct FFastArraySerializerItem
{
	int ReplicationID;
	int ReplicationKey;
	int MostRecentArrayReplicationKey;
};

struct FFastArraySerializer
{
	int ArrayReplicationKey;
	EFastArraySerializerDeltaFlags DeltaFlags;
};

struct FTempMockInputCmd
{
	struct Vector Force;
	float Turn;
	bool bJumpedPressed;
	bool bBrakesPressed;
};

struct FNetworkPhysicsState
{
};

struct FTempMockObject
{
	int JumpCooldownMS;
	int JumpCount;
	float ForceMultiplier;
	int RandValue;
};

struct FNetworkPredictionProxyAsync
{
};

struct FNetworkPredictionProxy
{
	class UClass* WorldManager;
};

struct FReplicationProxy
{
};

struct FSharedPackageMapItem
{
	struct TSoftClassPtr<UObject> SoftPtr;
};

struct FSharedPackageMap
{
	TArray<struct SharedPackageMapItem> Items;
};

struct FNetworkPredictionSettings
{
	ENetworkPredictionTickingPolicy PreferredTickingPolicy;
	class UClass* ReplicatedManagerClassOverride;
	int FixedTickFrameRate;
	bool bForceEngineFixTickForcePhysics;
	ENetworkLOD SimulatedProxyNetworkLOD;
	int FixedTickInterpolationBufferedMS;
	int IndependentTickInterpolationBufferedMS;
	int IndependentTickInterpolationMaxBufferedMS;
};

struct FNetworkPredictionDevHUDItem
{
	struct FString DisplayName;
	struct FString ExecCommand;
	bool bAutoBack;
	bool bRequirePIE;
	bool bRequireNotPIE;
};

struct FNetworkPredictionDevHUD
{
	struct FString HUDName;
	TArray<struct NetworkPredictionDevHUDItem> Items;
	bool bRequirePIE;
	bool bRequireNotPIE;
};

struct FServerReplicationRPCParameter
{
};

struct FNevadaState
{
	ENevadaFlightStates CurrentStatus;
	int LivesRemaining;
	bool bHijackingActive;
	bool bPilotBubbleCollisionEnabled;
	float CurrentBatteryCharge;
	bool bCannonEnabled;
	bool bTractorBeamEnabled;
};

struct FNevadaMoveModeConfig
{
	float ThrustForce;
	float LiftForce;
	float MaxHorizontalSpeed;
	float MaxVerticalSpeed;
	float DragForceMultiplier;
};

struct FNevadaAudioUpdateContext
{
};

struct FNiagaraCompileHash
{
	TArray<byte> DataHash;
};

struct FSimulationStageMetaData
{
	struct FName SimulationStageName;
	struct FName IterationSource;
	bool bSpawnOnly;
	bool bWritesParticles;
	bool bPartialParticleUpdate;
	TArray<struct FName> OutputDestinations;
	int MinStage;
	int MaxStage;
};

struct FNiagaraDataInterfaceGeneratedFunction
{
};

struct FNiagaraDataInterfaceGPUParamInfo
{
	struct FString DataInterfaceHLSLSymbol;
	struct FString DIClassName;
	TArray<struct NiagaraDataInterfaceGeneratedFunction> GeneratedFunctions;
};

struct FNiagaraCompileEvent
{
	EFNiagaraCompileEventSeverity Severity;
	struct FString Message;
	struct FString ShortDescription;
	bool bDismissable;
	struct Guid NodeGUID;
	struct Guid PinGuid;
	TArray<struct Guid> StackGuids;
};

struct FNiagaraTypeDefinitionHandle
{
	int RegisteredTypeIndex;
};

struct FNiagaraVariableBase
{
	struct FName Name;
	struct NiagaraTypeDefinitionHandle TypeDefHandle;
};

struct FNiagaraBakerTextureSource
{
	struct FName SourceName;
};

struct FNiagaraBakerTextureSettings
{
	struct FName OutputName;
	struct NiagaraBakerTextureSource SourceBinding;
	bool bUseFrameSize;
	struct IntPoint FrameSize;
	struct IntPoint TextureSize;
	class UClass* GeneratedTexture;
};

struct FNiagaraParameterStore
{
	class UClass* Owner;
	TArray<struct NiagaraVariableWithOffset> SortedParameterOffsets;
	TArray<byte> ParameterData;
	TArray<class UClass*> DataInterfaces;
	TArray<class UClass*> UObjects;
};

struct FNCPoolElement
{
	class UClass* Component;
};

struct FNCPool
{
	TArray<struct NCPoolElement> FreeElements;
};

struct FNiagaraDeviceProfileStateEntry
{
	struct FName ProfileName;
	uint32_t QualityLevelMask;
	uint32_t SetQualityLevelMask;
};

struct FNiagaraPlatformSetCVarCondition
{
	struct FName CVarName;
	bool Value;
	int MinInt;
	int MaxInt;
	float MinFloat;
	float MaxFloat;
	bool bUseMinInt;
	bool bUseMaxInt;
	bool bUseMinFloat;
	bool bUseMaxFloat;
};

struct FNiagaraPlatformSet
{
	int QualityLevelMask;
	TArray<struct NiagaraDeviceProfileStateEntry> DeviceProfileStates;
	TArray<struct NiagaraPlatformSetCVarCondition> CVarConditions;
};

struct FNiagaraVariableAttributeBinding
{
	struct NiagaraVariableBase ParamMapVariable;
	struct NiagaraVariable DataSetVariable;
	struct NiagaraVariable RootVariable;
	ENiagaraBindingSource BindingSourceMode;
	bool bBindingExistsOnSource;
	bool bIsCachedParticleValue;
};

struct FNiagaraTypeDefinition
{
	class UClass* ClassStructOrEnum;
	uint16_t UnderlyingType;
};

struct FNiagaraComponentPropertyBinding
{
	struct NiagaraVariableAttributeBinding AttributeBinding;
	struct FName PropertyName;
	struct NiagaraTypeDefinition PropertyType;
	struct FName MetadataSetterName;
	Unknown PropertySetterParameterDefaults;
	struct NiagaraVariable WritableValue;
};

struct FNiagaraEmitterNameSettingsRef
{
	struct FName SystemName;
	struct FString EmitterName;
};

struct FNiagaraCulledComponentInfo
{
};

struct FNiagaraUserParameterBinding
{
	struct NiagaraVariable Parameter;
};

struct FNDIStaticMeshSectionFilter
{
	TArray<int> AllowedMaterialSlots;
};

struct FNiagaraDebugHUDVariable
{
	bool bEnabled;
	struct FString Name;
};

struct FNiagaraDebugHudTextOptions
{
	ENiagaraDebugHudFont Font;
	ENiagaraDebugHudHAlign HorizontalAlignment;
	ENiagaraDebugHudVAlign VerticalAlignment;
	struct Vector2D ScreenOffset;
};

struct FNiagaraDebugHUDSettingsData
{
	bool bEnabled;
	bool bValidateSystemSimulationDataBuffers;
	bool bValidateParticleDataBuffers;
	bool bOverviewEnabled;
	ENiagaraDebugHudFont OverviewFont;
	struct Vector2D OverviewLocation;
	struct FString ActorFilter;
	bool bComponentFilterEnabled;
	struct FString ComponentFilter;
	bool bSystemFilterEnabled;
	struct FString SystemFilter;
	bool bEmitterFilterEnabled;
	struct FString EmitterFilter;
	bool bActorFilterEnabled;
	ENiagaraDebugHudVerbosity SystemDebugVerbosity;
	ENiagaraDebugHudVerbosity SystemEmitterVerbosity;
	bool bSystemShowBounds;
	bool bSystemShowActiveOnlyInWorld;
	bool bShowSystemVariables;
	TArray<struct NiagaraDebugHUDVariable> SystemVariables;
	struct NiagaraDebugHudTextOptions SystemTextOptions;
	bool bShowParticleVariables;
	bool bEnableGpuParticleReadback;
	TArray<struct NiagaraDebugHUDVariable> ParticlesVariables;
	struct NiagaraDebugHudTextOptions ParticleTextOptions;
	bool bShowParticlesVariablesWithSystem;
	bool bUseMaxParticlesToDisplay;
	int MaxParticlesToDisplay;
	ENiagaraDebugPlaybackMode PlaybackMode;
	bool bPlaybackRateEnabled;
	float PlaybackRate;
	bool bLoopTimeEnabled;
	float LoopTime;
	bool bShowGlobalBudgetInfo;
};

struct FNiagaraLinearRamp
{
	float StartX;
	float StartY;
	float EndX;
	float EndY;
};

struct FNiagaraGlobalBudgetScaling
{
	bool bCullByGlobalBudget;
	bool bScaleMaxDistanceByGlobalBudgetUse;
	bool bScaleMaxInstanceCountByGlobalBudgetUse;
	bool bScaleSystemInstanceCountByGlobalBudgetUse;
	float MaxGlobalBudgetUsage;
	struct NiagaraLinearRamp MaxDistanceScaleByGlobalBudgetUse;
	struct NiagaraLinearRamp MaxInstanceCountScaleByGlobalBudgetUse;
	struct NiagaraLinearRamp MaxSystemInstanceCountScaleByGlobalBudgetUse;
};

struct FNiagaraSystemScalabilitySettings
{
	struct NiagaraPlatformSet Platforms;
	bool bCullByDistance;
	bool bCullMaxInstanceCount;
	bool bCullPerSystemMaxInstanceCount;
	bool bCullByMaxTimeWithoutRender;
	float MaxDistance;
	int MaxInstances;
	int MaxSystemInstances;
	float MaxTimeWithoutRender;
	ENiagaraCullProxyMode CullProxyMode;
	int MaxSystemProxies;
	struct NiagaraGlobalBudgetScaling BudgetScaling;
};

struct FNiagaraSystemScalabilitySettingsArray
{
	TArray<struct NiagaraSystemScalabilitySettings> Settings;
};

struct FNiagaraEmitterScalabilitySettings
{
	struct NiagaraPlatformSet Platforms;
	bool bScaleSpawnCount;
	float SpawnCountScale;
};

struct FNiagaraEmitterScalabilitySettingsArray
{
	TArray<struct NiagaraEmitterScalabilitySettings> Settings;
};

struct FNiagaraPerfBaselineStats
{
	float PerInstanceAvg_GT;
	float PerInstanceAvg_RT;
	float PerInstanceMax_GT;
	float PerInstanceMax_RT;
};

struct FNiagaraEventReceiverProperties
{
	struct FName Name;
	struct FName SourceEventGenerator;
	struct FName SourceEmitter;
};

struct FNiagaraTypeLayoutInfo
{
	TArray<uint32_t> FloatComponentByteOffsets;
	TArray<uint32_t> FloatComponentRegisterOffsets;
	TArray<uint32_t> Int32ComponentByteOffsets;
	TArray<uint32_t> Int32ComponentRegisterOffsets;
	TArray<uint32_t> HalfComponentByteOffsets;
	TArray<uint32_t> HalfComponentRegisterOffsets;
};

struct FNiagaraVariableLayoutInfo
{
	uint32_t FloatComponentStart;
	uint32_t Int32ComponentStart;
	uint32_t HalfComponentStart;
	struct NiagaraTypeLayoutInfo LayoutInfo;
};

struct FNiagaraDataSetID
{
	struct FName Name;
	ENiagaraDataSetType Type;
};

struct FNiagaraDataSetCompiledData
{
	TArray<struct NiagaraVariable> Variables;
	TArray<struct NiagaraVariableLayoutInfo> VariableLayouts;
	struct NiagaraDataSetID ID;
	uint32_t TotalFloatComponents;
	uint32_t TotalInt32Components;
	uint32_t TotalHalfComponents;
	bool bRequiresPersistentIDs;
	ENiagaraSimTarget SimTarget;
};

struct FNiagaraEventGeneratorProperties
{
	int MaxEventsPerFrame;
	struct FName ID;
	struct NiagaraDataSetCompiledData DataSetCompiledData;
};

struct FNiagaraEmitterScriptProperties
{
	class UClass* Script;
	TArray<struct NiagaraEventReceiverProperties> EventReceivers;
	TArray<struct NiagaraEventGeneratorProperties> EventGenerators;
};

struct FNiagaraDetailsLevelScaleOverrides
{
	float Low;
	float Medium;
	float High;
	float Epic;
	float Cine;
};

struct FNiagaraEmitterScalabilityOverrides
{
	TArray<struct NiagaraEmitterScalabilityOverride> Overrides;
};

struct FNiagaraMeshRendererMeshProperties
{
	class UClass* Mesh;
	struct NiagaraUserParameterBinding UserParamBinding;
	struct Vector Scale;
	struct Vector PivotOffset;
	ENiagaraMeshPivotOffsetSpace PivotOffsetSpace;
};

struct FNiagaraMeshMaterialOverride
{
	class UClass* ExplicitMat;
	struct NiagaraUserParameterBinding UserParamBinding;
};

struct FNiagaraMaterialAttributeBinding
{
	struct FName MaterialParameterName;
	struct NiagaraVariableBase NiagaraVariable;
	struct NiagaraVariableBase ResolvedNiagaraVariable;
	struct NiagaraVariableBase NiagaraChildVariable;
};

struct FNiagaraRibbonUVSettings
{
	ENiagaraRibbonUVDistributionMode DistributionMode;
	ENiagaraRibbonUVEdgeMode LeadingEdgeMode;
	ENiagaraRibbonUVEdgeMode TrailingEdgeMode;
	float TilingLength;
	struct Vector2D Offset;
	struct Vector2D Scale;
	bool bEnablePerParticleUOverride;
	bool bEnablePerParticleVRangeOverride;
};

struct FNiagaraRibbonShapeCustomVertex
{
	struct Vector2D Position;
	struct Vector2D Normal;
	float TextureV;
};

struct FNiagaraScriptExecutionPaddingInfo
{
	uint16_t SrcOffset;
	uint16_t DestOffset;
	uint16_t SrcSize;
	uint16_t DestSize;
};

struct FNiagaraBoundParameter
{
	struct NiagaraVariable Parameter;
	int SrcOffset;
	int DestOffset;
};

struct FNiagaraVMExecutableDataId
{
	struct Guid CompilerVersionID;
	ENiagaraScriptUsage ScriptUsageType;
	struct Guid ScriptUsageTypeID;
	bool bUsesRapidIterationParams;
	bool bInterpolatedSpawn;
	bool bRequiresPersistentIDs;
	struct Guid BaseScriptID;
	struct NiagaraCompileHash BaseScriptCompileHash;
	struct Guid ScriptVersionID;
};

struct FNiagaraCompilerTag
{
	struct NiagaraVariable Variable;
	struct FString StringValue;
};

struct FNiagaraScriptDataUsageInfo
{
	bool bReadsAttributeData;
};

struct FNiagaraScriptDataInterfaceCompileInfo
{
	struct FName Name;
	int UserPtrIdx;
	struct NiagaraTypeDefinition Type;
	struct FName RegisteredParameterMapRead;
	struct FName RegisteredParameterMapWrite;
	bool bIsPlaceholder;
};

struct FVMFunctionSpecifier
{
	struct FName Key;
	struct FName Value;
};

struct FVMExternalFunctionBindingInfo
{
	struct FName Name;
	struct FName OwnerName;
	TArray<bool> InputParamLocations;
	int NumOutputs;
	TArray<struct VMFunctionSpecifier> FunctionSpecifiers;
};

struct FNiagaraDataSetProperties
{
	struct NiagaraDataSetID ID;
	TArray<struct NiagaraVariable> Variables;
};

struct FNiagaraStatScope
{
	struct FName FullName;
	struct FName FriendlyName;
};

struct FNiagaraVMExecutableData
{
	TArray<byte> ByteCode;
	TArray<byte> OptimizedByteCode;
	int NumTempRegisters;
	int NumUserPtrs;
	TArray<struct NiagaraCompilerTag> CompileTags;
	TArray<byte> ScriptLiterals;
	TArray<struct NiagaraVariable> Attributes;
	struct NiagaraScriptDataUsageInfo DataUsage;
	TArray<struct NiagaraScriptDataInterfaceCompileInfo> DataInterfaceInfo;
	TArray<struct VMExternalFunctionBindingInfo> CalledVMExternalFunctions;
	TArray<struct NiagaraDataSetID> ReadDataSets;
	TArray<struct NiagaraDataSetProperties> WriteDataSets;
	TArray<struct NiagaraStatScope> StatScopes;
	TArray<struct NiagaraDataInterfaceGPUParamInfo> DIParamInfo;
	ENiagaraScriptCompileStatus LastCompileStatus;
	TArray<struct SimulationStageMetaData> SimulationStageMetaData;
	bool bReadsSignificanceIndex;
	bool bNeedsGPUContextInit;
};

struct FNiagaraScriptDataInterfaceInfo
{
	class UClass* DataInterface;
	struct FName Name;
	int UserPtrIdx;
	struct NiagaraTypeDefinition Type;
	struct FName RegisteredParameterMapRead;
	struct FName RegisteredParameterMapWrite;
};

struct FNiagaraVariableDataInterfaceBinding
{
	struct NiagaraVariable BoundVariable;
};

struct FNiagaraSystemScalabilityOverrides
{
	TArray<struct NiagaraSystemScalabilityOverride> Overrides;
};

struct FNiagaraEmitterHandle
{
	struct Guid ID;
	struct FName IdName;
	bool bIsEnabled;
	struct FName Name;
	class UClass* Instance;
};

struct FNiagaraParameterDataSetBinding
{
	int ParameterOffset;
	int DataSetComponentOffset;
};

struct FNiagaraParameterDataSetBindingCollection
{
	TArray<struct NiagaraParameterDataSetBinding> FloatOffsets;
	TArray<struct NiagaraParameterDataSetBinding> Int32Offsets;
};

struct FNiagaraSystemCompiledData
{
	struct NiagaraParameterStore InstanceParamStore;
	struct NiagaraDataSetCompiledData DataSetCompiledData;
	struct NiagaraDataSetCompiledData SpawnInstanceParamsDataSetCompiledData;
	struct NiagaraDataSetCompiledData UpdateInstanceParamsDataSetCompiledData;
	struct NiagaraParameterDataSetBindingCollection SpawnInstanceGlobalBinding;
	struct NiagaraParameterDataSetBindingCollection SpawnInstanceSystemBinding;
	struct NiagaraParameterDataSetBindingCollection SpawnInstanceOwnerBinding;
	TArray<struct NiagaraParameterDataSetBindingCollection> SpawnInstanceEmitterBindings;
	struct NiagaraParameterDataSetBindingCollection UpdateInstanceGlobalBinding;
	struct NiagaraParameterDataSetBindingCollection UpdateInstanceSystemBinding;
	struct NiagaraParameterDataSetBindingCollection UpdateInstanceOwnerBinding;
	TArray<struct NiagaraParameterDataSetBindingCollection> UpdateInstanceEmitterBindings;
};

struct FNiagaraScalabilityState
{
	float Significance;
	bool bCulled;
	bool bPreviousCulled;
	bool bCulledByDistance;
	bool bCulledByInstanceCount;
	bool bCulledByVisibility;
	bool bCulledByGlobalBudget;
};

struct FNiagaraCompileDependency
{
	struct FString LinkerErrorMessage;
	struct Guid NodeGUID;
	struct Guid PinGuid;
	TArray<struct Guid> StackGuids;
	struct NiagaraVariableBase DependentVariable;
};

struct FNiagaraRandInfo
{
	int Seed1;
	int Seed2;
	int Seed3;
};

struct FNiagaraScriptVariableBinding
{
	struct FName Name;
};

struct FNiagaraVariableInfo
{
	struct NiagaraVariable Variable;
	struct FText Definition;
	class UClass* DataInterface;
};

struct FNiagaraSystemUpdateContext
{
	TArray<class UClass*> ComponentsToReset;
	TArray<class UClass*> ComponentsToReInit;
	TArray<class UClass*> ComponentsToNotifySimDestroy;
	TArray<class UClass*> SystemSimsToDestroy;
};

struct FNiagaraFunctionSignature
{
	struct FName Name;
	TArray<struct NiagaraVariable> Inputs;
	TArray<struct NiagaraVariable> Outputs;
	struct FName OwnerName;
	bool bRequiresContext;
	bool bRequiresExecPin;
	bool bMemberFunction;
	bool bExperimental;
	bool bSupportsCPU;
	bool bSupportsGPU;
	bool bWriteFunction;
	bool bSoftDeprecatedFunction;
	bool bIsCompileTagGenerator;
	bool bHidden;
	int ModuleUsageBitmask;
	int ContextStageMinIndex;
	int ContextStageMaxIndex;
	Unknown FunctionSpecifiers;
};

struct FBasicParticleData
{
	struct Vector Position;
	float Size;
	struct Vector Velocity;
};

struct FMeshTriCoordinate
{
	int Tri;
	struct Vector BaryCoord;
};

struct FNiagaraSimpleClientInfo
{
	TArray<struct FString> Systems;
	TArray<struct FString> Actors;
	TArray<struct FString> Components;
	TArray<struct FString> Emitters;
};

struct FNiagaraOutlinerCaptureSettings
{
	bool bTriggerCapture;
	uint32_t CaptureDelayFrames;
	bool bGatherPerfData;
};

struct FNiagaraRequestSimpleClientInfoMessage
{
};

struct FNiagaraOutlinerEmitterInstanceData
{
	struct FString EmitterName;
	ENiagaraSimTarget SimTarget;
	ENiagaraExecutionState ExecState;
	int NumParticles;
};

struct FNiagaraOutlinerTimingData
{
	float GameThread;
	float RenderThread;
};

struct FNiagaraOutlinerSystemInstanceData
{
	struct FString ComponentName;
	TArray<struct NiagaraOutlinerEmitterInstanceData> Emitters;
	ENiagaraExecutionState ActualExecutionState;
	ENiagaraExecutionState RequestedExecutionState;
	struct NiagaraScalabilityState ScalabilityState;
	bool bPendingKill;
	bool bUsingCullProxy;
	ENCPoolMethod PoolMethod;
	struct NiagaraOutlinerTimingData AverageTime;
	struct NiagaraOutlinerTimingData MaxTime;
};

struct FNiagaraOutlinerSystemData
{
	TArray<struct NiagaraOutlinerSystemInstanceData> SystemInstances;
	struct NiagaraOutlinerTimingData AveragePerFrameTime;
	struct NiagaraOutlinerTimingData MaxPerFrameTime;
	struct NiagaraOutlinerTimingData AveragePerInstanceTime;
	struct NiagaraOutlinerTimingData MaxPerInstanceTime;
};

struct FNiagaraOutlinerWorldData
{
	Unknown Systems;
	bool bHasBegunPlay;
	byte WorldType;
	byte NetMode;
	struct NiagaraOutlinerTimingData AveragePerFrameTime;
	struct NiagaraOutlinerTimingData MaxPerFrameTime;
};

struct FNiagaraOutlinerData
{
	Unknown WorldData;
};

struct FNiagaraDebuggerOutlinerUpdate
{
	struct NiagaraOutlinerData OutlinerData;
};

struct FNiagaraDebuggerExecuteConsoleCommand
{
	struct FString Command;
	bool bRequiresWorld;
};

struct FNiagaraDebuggerConnectionClosed
{
	struct Guid SessionId;
	struct Guid InstanceID;
};

struct FNiagaraDebuggerAcceptConnection
{
	struct Guid SessionId;
	struct Guid InstanceID;
};

struct FNiagaraDebuggerRequestConnection
{
	struct Guid SessionId;
	struct Guid InstanceID;
};

struct FNiagaraGraphViewSettings
{
	struct Vector2D Location;
	float Zoom;
	bool bIsValid;
};

struct FNiagaraCollisionEventPayload
{
	struct Vector CollisionPos;
	struct Vector CollisionNormal;
	struct Vector CollisionVelocity;
	int ParticleIndex;
	int PhysicalMaterialIndex;
};

struct FParameterDefinitionsSubscription
{
};

struct FNiagaraParameters
{
	TArray<struct NiagaraVariable> Parameters;
};

struct FNiagaraPlatformSetConflictEntry
{
	struct FName ProfileName;
	int QualityLevelMask;
};

struct FNiagaraPlatformSetConflictInfo
{
	int SetAIndex;
	int SetBIndex;
	TArray<struct NiagaraPlatformSetConflictEntry> Conflicts;
};

struct FNiagaraScalabilityManager
{
	class UClass* EffectType;
	TArray<class UClass*> ManagedComponents;
};

struct FVersionedNiagaraScriptData
{
};

struct FNiagaraModuleDependency
{
	struct FName ID;
	ENiagaraModuleDependencyType Type;
	ENiagaraModuleDependencyScriptConstraint ScriptConstraint;
	struct FText Description;
};

struct FNiagaraScriptHighlight
{
	struct LinearColor Color;
	struct FText DisplayName;
};

struct FNiagaraSystemCompileRequest
{
	TArray<class UClass*> RootObjects;
};

struct FEmitterCompiledScriptPair
{
};

struct FNiagaraEmitterCompiledData
{
	TArray<struct FName> SpawnAttributes;
	struct NiagaraVariable EmitterSpawnIntervalVar;
	struct NiagaraVariable EmitterInterpSpawnStartDTVar;
	struct NiagaraVariable EmitterSpawnGroupVar;
	struct NiagaraVariable EmitterAgeVar;
	struct NiagaraVariable EmitterRandomSeedVar;
	struct NiagaraVariable EmitterInstanceSeedVar;
	struct NiagaraVariable EmitterTotalSpawnedParticlesVar;
	struct NiagaraDataSetCompiledData DataSetCompiledData;
};

struct FNiagaraInputConditionMetadata
{
	struct FName InputName;
	TArray<struct FString> TargetValues;
};

struct FNiagaraVariableMetaData
{
	struct FText Description;
	struct FText CategoryName;
	bool bAdvancedDisplay;
	int EditorSortPriority;
	bool bInlineEditConditionToggle;
	struct NiagaraInputConditionMetadata EditCondition;
	struct NiagaraInputConditionMetadata VisibleCondition;
	Unknown PropertyMetaData;
	struct FName ParentAttribute;
	struct Guid VariableGuid;
	bool bIsStaticSwitch;
	int StaticSwitchDefaultValue;
};

struct FNiagaraCompileHashVisitorDebugInfo
{
	struct FString Object;
	TArray<struct FString> PropertyKeys;
	TArray<struct FString> PropertyValues;
};

struct FNiagaraID
{
	int Index;
	int AcquireTag;
};

struct FNiagaraSpawnInfo
{
	int Count;
	float InterpStartDt;
	float IntervalDt;
	int SpawnGroup;
};

struct FNiagaraAssetVersion
{
	int MajorVersion;
	int MinorVersion;
	struct Guid VersionGuid;
	bool bIsVisibleInVersionSelector;
};

struct FNiagaraMatrix
{
	struct Vector4 Row0;
	struct Vector4 Row1;
	struct Vector4 Row2;
	struct Vector4 Row3;
};

struct FNiagaraParameterMap
{
};

struct FNiagaraNumeric
{
};

struct FNiagaraHalfVector4
{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint16_t W;
};

struct FNiagaraHalfVector3
{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
};

struct FNiagaraHalfVector2
{
	uint16_t X;
	uint16_t Y;
};

struct FNiagaraHalf
{
	uint16_t Value;
};

struct FNiagaraBool
{
	int Value;
};

struct FNiagaraInt32
{
	int Value;
};

struct FNiagaraFloat
{
	float Value;
};

struct FNiagaraWildcard
{
};

struct FNiagaraVariant
{
	class UClass* Object;
	class UClass* DataInterface;
	TArray<byte> Bytes;
	ENiagaraVariantMode CurrentMode;
};

struct FGfeSDKHighlightGroupView
{
	struct FString GroupId;
	EGfeSDKHighlightType TagsFilter;
	EGfeSDKHighlightSignificance SignificanceFilter;
};

struct FGfeSDKHighlightSummaryParams
{
	TArray<struct GfeSDKHighlightGroupView> GroupViews;
};

struct FGfeSDKHighlightVideoParams
{
	struct FString GroupId;
	struct FString HighlightId;
	int StartDelta;
	int EndDelta;
};

struct FGfeSDKHighlightScreenshotParams
{
	struct FString GroupId;
	struct FString HighlightId;
};

struct FGfeSDKHighlightCloseGroupParams
{
	struct FString GroupId;
	bool DestroyHighlights;
};

struct FGfeSDKHighlightOpenGroupParams
{
	struct FString GroupId;
	Unknown GroupDescriptionTranslationTable;
};

struct FGfeSDKPermissionsChangedData
{
	Unknown ScopePermissions;
};

struct FGfeSDKRequestPermissionsParams
{
	TArray<EGfeSDKScope> Scopes;
};

struct FGfeSDKHighlightDefinition
{
	struct FString ID;
	bool UserDefaultInterest;
	EGfeSDKHighlightType HighlightTags;
	EGfeSDKHighlightSignificance Significance;
	Unknown NameTranslationTable;
};

struct FGfeSDKHighlightConfigParams
{
	TArray<struct GfeSDKHighlightDefinition> HighlightDefinitions;
	struct FString DefaultLocale;
};

struct FGfeSDKCreateResponse
{
	uint16_t VersionMajor;
	uint16_t VersionMinor;
	struct FString NVIDIAGfeVersion;
	Unknown ScopePermissions;
};

struct FGfeSDKCreateInputParams
{
	struct FString AppName;
	TArray<EGfeSDKScope> RequiredScopes;
	bool PollForCallbacks;
};

struct FJsonToxicityBulkEvaluationResponse
{
	TArray<bool> isToxic;
};

struct FJsonToxicityEvaluationResponse
{
	bool toxic;
};

struct FJsonToxicityBulkEvaluationRequest
{
	TArray<struct FString> texts;
};

struct FJsonToxicityEvaluationRequest
{
	struct FString Text;
};

struct FPIELoginSettingsInternal
{
	struct FString ID;
	struct FString Token;
	struct FString Type;
	TArray<byte> TokenBytes;
};

struct FPlayerReservation
{
	struct UniqueNetIdRepl UniqueId;
	struct FString ValidationStr;
	struct FString Platform;
	bool bAllowCrossplay;
	float ElapsedTime;
};

struct FPartyReservation
{
	int TeamNum;
	struct UniqueNetIdRepl PartyLeader;
	TArray<struct PlayerReservation> PartyMembers;
	TArray<struct PlayerReservation> RemovedPartyMembers;
};

struct FPartyBeaconCrossplayPlatformMapping
{
	struct FString PlatformName;
	struct FString PlatformType;
};

struct FSpectatorReservation
{
	struct UniqueNetIdRepl SpectatorId;
	struct PlayerReservation Spectator;
};

struct FBlueprintSessionResult
{
};

struct FInAppPurchaseReceiptInfo2
{
	struct FString ItemName;
	struct FString ItemId;
	struct FString ValidationInfo;
};

struct FOnlineProxyStoreOffer
{
	struct FString OfferId;
	struct FText Title;
	struct FText Description;
	struct FText LongDescription;
	struct FText RegularPriceText;
	int RegularPrice;
	struct FText PriceText;
	int NumericPrice;
	struct FString CurrencyCode;
	struct DateTime ReleaseDate;
	struct DateTime ExpirationDate;
	EOnlineProxyStoreOfferDiscountType DiscountType;
	Unknown DynamicFields;
};

struct FInAppPurchaseRestoreInfo2
{
	struct FString ItemName;
	struct FString ItemId;
	struct FString ValidationInfo;
};

struct FInAppPurchaseReceiptInfo
{
	struct FString ItemName;
	struct FString ItemId;
	struct FString ValidationInfo;
};

struct FInAppPurchaseProductInfo2
{
	struct FString Identifier;
	struct FString TransactionIdentifier;
	struct FString DisplayName;
	struct FString DisplayDescription;
	struct FString DisplayPrice;
	float RawPrice;
	struct FString CurrencyCode;
	struct FString CurrencySymbol;
	struct FString DecimalSeparator;
	struct FString GroupingSeparator;
	struct FString ReceiptData;
	Unknown DynamicFields;
};

struct FInAppPurchaseProductRequest2
{
	struct FString ProductIdentifier;
	bool bIsConsumable;
};

struct FNamedInterface
{
	struct FName InterfaceName;
	class UClass* InterfaceObject;
};

struct FNamedInterfaceDef
{
	struct FName InterfaceName;
	struct FString InterfaceClassName;
};

struct FInAppPurchaseProductInfo
{
	struct FString Identifier;
	struct FString TransactionIdentifier;
	struct FString DisplayName;
	struct FString DisplayDescription;
	struct FString DisplayPrice;
	float RawPrice;
	struct FString CurrencyCode;
	struct FString CurrencySymbol;
	struct FString DecimalSeparator;
	struct FString GroupingSeparator;
	struct FString ReceiptData;
};

struct FInAppPurchaseRestoreInfo
{
	struct FString Identifier;
	struct FString ReceiptData;
	struct FString TransactionIdentifier;
};

struct FInAppPurchaseProductRequest
{
	struct FString ProductIdentifier;
	bool bIsConsumable;
};

struct FOverlayItem
{
	struct Timespan StartTime;
	struct Timespan EndTime;
	struct FString Text;
	struct Vector2D Position;
};

struct FPaperFlipbookKeyFrame
{
	class UClass* Sprite;
	int FrameRun;
};

struct FSpriteInstanceData
{
	struct Matrix Transform;
	class UClass* SourceSprite;
	struct Color VertexColor;
	int MaterialIndex;
};

struct FPaperSpriteSocket
{
	struct Transform LocalTransform;
	struct FName SocketName;
};

struct FPaperTerrainMaterialRule
{
	class UClass* StartCap;
	TArray<class UClass*> Body;
	class UClass* EndCap;
	float MinimumAngle;
	float MaximumAngle;
	bool bEnableCollision;
	float CollisionOffset;
	int DrawOrder;
};

struct FPaperTileInfo
{
	class UClass* TileSet;
	int PackedTileIndex;
};

struct FIntMargin
{
	int Left;
	int Top;
	int Right;
	int Bottom;
};

struct FSpriteGeometryShape
{
	ESpriteShapeType ShapeType;
	TArray<struct Vector2D> Vertices;
	struct Vector2D BoxSize;
	struct Vector2D BoxPosition;
	float Rotation;
	bool bNegativeWinding;
};

struct FSpriteGeometryCollection
{
	TArray<struct SpriteGeometryShape> Shapes;
	ESpritePolygonMode GeometryType;
	int PixelsPerSubdivisionX;
	int PixelsPerSubdivisionY;
	bool bAvoidVertexMerging;
	float AlphaThreshold;
	float DetailAmount;
	float SimplifyEpsilon;
};

struct FPaperTileMetadata
{
	struct FName UserDataName;
	struct SpriteGeometryCollection CollisionData;
	byte TerrainMembership;
};

struct FPaperTileSetTerrain
{
	struct FString TerrainName;
	int CenterTileIndex;
};

struct FPaperSpriteAtlasSlot
{
	struct TSoftClassPtr<UObject> SpriteRef;
	int AtlasIndex;
	int X;
	int Y;
	int Width;
	int Height;
};

struct FSpriteDrawCallRecord
{
	struct Vector Destination;
	class UClass* BaseTexture;
	struct Color Color;
};

struct FSpriteAssetInitParameters
{
};

struct FSocialPlatformDescription
{
	struct FString Name;
	struct FString PlatformType;
	struct FName OnlineSubsystem;
	struct FString SessionType;
	struct FString ExternalAccountType;
	struct FString CrossplayPool;
};

struct FOnlinePartyRepDataBase
{
};

struct FUserPlatform
{
	struct SocialPlatformDescription PlatformDescription;
};

struct FPartyMemberPlatformData
{
	struct UserPlatform Platform;
	struct UniqueNetIdRepl UniqueId;
	struct FString SessionId;
};

struct FPartyPrivacySettings
{
	EPartyType PartyType;
	EPartyInviteRestriction PartyInviteRestriction;
	bool bOnlyLeaderFriendsCanJoin;
};

struct FPartyPlatformSessionInfo
{
	struct FString SessionType;
	struct FString SessionId;
	struct UniqueNetIdRepl OwnerPrimaryId;
};

struct FSocialChatChannelConfig
{
	class UClass* SocialUser;
	TArray<class UClass*> ListenChannels;
};

struct FBodyInstanceCore
{
	bool bSimulatePhysics;
	bool bOverrideMass;
	bool bEnableGravity;
	bool bAutoWeld;
	bool bStartAwake;
	bool bGenerateWakeEvents;
	bool bUpdateMassWhenScaleChanges;
};

struct FWheelSetup
{
	class UClass* WheelClass;
	struct FName BoneName;
	struct Vector AdditionalOffset;
	bool bDisableSteering;
};

struct FReplicatedVehicleState
{
	float SteeringInput;
	float ThrottleInput;
	float BrakeInput;
	float HandbrakeInput;
	int CurrentGear;
};

struct FVehicleInputRate
{
	float RiseRate;
	float FallRate;
};

struct FTireConfigMaterialFriction
{
	class UClass* PhysicalMaterial;
	float FrictionScale;
};

struct FVehicleEngineData
{
	struct RuntimeFloatCurve TorqueCurve;
	float MaxRPM;
	float MOI;
	float DampingRateFullThrottle;
	float DampingRateZeroThrottleClutchEngaged;
	float DampingRateZeroThrottleClutchDisengaged;
};

struct FVehicleDifferential4WData
{
	EVehicleDifferential4W DifferentialType;
	float FrontRearSplit;
	float FrontLeftRightSplit;
	float RearLeftRightSplit;
	float CentreBias;
	float FrontBias;
	float RearBias;
};

struct FVehicleGearData
{
	float Ratio;
	float DownRatio;
	float UpRatio;
};

struct FVehicleTransmissionData
{
	bool bUseGearAutoBox;
	float GearSwitchTime;
	float GearAutoBoxLatency;
	float FinalRatio;
	TArray<struct VehicleGearData> ForwardGears;
	float ReverseGearRatio;
	float NeutralGearUpRatio;
	float ClutchStrength;
};

struct FReplicatedSpawnInfo
{
	struct Vector SpawnLocation;
	struct Rotator SpawnRotation;
	float ServerSpawnTime;
	struct Vector LastLocation;
	float MinTimeForCameraFadeTransition;
};

struct FPlayspaceSpawningInfo
{
	struct UniqueNetIdRepl UserId;
	Unknown RequestingPlayspace;
	Unknown SpawnLocationActor;
	struct Vector SpawnLocation;
	struct Rotator SpawnRotation;
	float SpawnTimeServer;
	struct Vector LastLocation;
	float MinTimeForCameraFadeTransition;
};

struct FActorOverlapEvent
{
};

struct FProcMeshTangent
{
	struct Vector TangentX;
	bool bFlipTangentY;
};

struct FProcMeshVertex
{
	struct Vector Position;
	struct Vector Normal;
	struct ProcMeshTangent Tangent;
	struct Color Color;
	struct Vector2D UV0;
	struct Vector2D UV1;
	struct Vector2D UV2;
	struct Vector2D UV3;
};

struct FProcMeshSection
{
	TArray<struct ProcMeshVertex> ProcVertexBuffer;
	TArray<uint32_t> ProcIndexBuffer;
	struct Box SectionLocalBox;
	bool bEnableCollision;
	bool bSectionVisible;
};

struct FProceduralContentVariationMap
{
	int VariationIndex;
	class UClass* Map;
	float Weight;
};

struct FProceduralDebugSettings
{
	bool bShowDebugPointCloud;
	struct Color DebugPointColor;
	float DebugCullingDistance;
};

struct FProceduralRemapFloatCurve
{
	struct RuntimeFloatCurve RemapCurve;
};

struct FProceduralRemapVectorCurve
{
	struct RuntimeCurveLinearColor RemapCurve;
};

struct FProceduralGenerationStackElement
{
	bool bEnabled;
	class UClass* Generator;
};

struct FProceduralPointCloudPoint
{
	struct Transform Transform;
	struct Vector SurfaceNormal;
	class UClass* SurfaceObject;
	class UClass* Object;
	int ObjectVariation;
	TArray<struct FName> Tags;
	float CollisionRadius;
	float SourcePointGeneratorIndex;
};

struct FProceduralScatterTileSettings
{
	int NumUniqueTiles;
	float TileSize;
	float TileOverlapPercentage;
	float MinimumQuadTreeSize;
	EProceduralScatterTileRandomGenerator RandomGenerator;
};

struct FProceduralScatterSettingsElement
{
	bool bEnabled;
	class UClass* ScatterSettings;
	int MaxPoints;
};

struct FProceduralScatterDebugSettings
{
	bool bShowDebugPlane;
	bool bShowDebugTrace;
	bool bShowDebugFootprint;
	bool bShowDebugHitNormal;
	bool bShowDebugPoints;
	bool bShowDebugPivots;
	float DebugMaxCullingDistance;
};

struct FProceduralHitPoint
{
	bool bIsTraceDone;
	struct Vector Position;
	struct Vector Normal;
	Unknown Component;
};

struct FProceduralScatterPoint
{
	struct Vector Location;
	struct Rotator Rotation;
	float Scale;
	class UClass* ScatterSettings;
	int VariationIdx;
	struct Vector GenerationLocation;
	struct ProceduralHitPoint HitPoint;
};

struct FScatteredPointCloud
{
	TArray<struct ProceduralScatterPoint> Points;
};

struct FSourcePointGenerator
{
};

struct FProceduralScatterProjectionPoint
{
	struct Vector GenerationLocation;
	struct Rotator Rotation;
	float Scale;
	class UClass* ScatterSettings;
	int VariationIdx;
	struct Vector StartTrace;
	struct Vector EndTrace;
	struct ProceduralHitPoint HitPoint;
};

struct FProceduralScatter2DPoint
{
	struct Vector Location;
	float Scale;
	class UClass* ScatterSettings;
	struct Box2D MaxAABB;
	float CollisionRadius;
	bool bBlocker;
	int RandomNumber;
};

struct FProceduralContentVariationsModifiers
{
	TArray<class UClass*> Modifiers;
};

struct FProceduralDensityModifiers
{
	TArray<class UClass*> Modifiers;
};

struct FProceduralScaleModifiers
{
	bool bRandomScaleEnabled;
	struct FloatInterval RandomScale;
	TArray<class UClass*> Modifiers;
};

struct FProceduralRotationModifiers
{
	TArray<class UClass*> Modifiers;
};

struct FProceduralScatterTargetSurface
{
	bool bAllowLandscape;
	bool bAllowBSP;
	bool bAllowStaticMesh;
	bool bAllowTranslucent;
	TArray<struct FName> AllowedActorTags;
	TArray<struct FName> DisallowedActorTags;
	TArray<struct FName> AllowedComponentTags;
	TArray<struct FName> DisallowedComponentTags;
};

struct FProceduralTextureSource
{
	bool bUseRenderTarget;
	class UClass* Texture;
	class UClass* RenderTarget;
	class UClass* RenderTargetMaterial;
	bool bStretchToFit;
	float TexelSize;
	float XOffset;
	float YOffset;
	float Rotation;
};

struct FPropertyAccessSegment
{
	struct FName Name;
	class UClass* Struct;
	Unknown Property;
	class UClass* Function;
	int ArrayIndex;
	uint16_t Flags;
};

struct FPropertyAccessPath
{
	int PathSegmentStartIndex;
	int PathSegmentCount;
	bool bHasEvents;
};

struct FPropertyAccessCopy
{
	int AccessIndex;
	int DestAccessStartIndex;
	int DestAccessEndIndex;
	EPropertyAccessCopyType Type;
};

struct FPropertyAccessCopyBatch
{
	TArray<struct PropertyAccessCopy> Copies;
};

struct FPropertyAccessIndirectionChain
{
	Unknown Property;
	int IndirectionStartIndex;
	int IndirectionEndIndex;
	int EventId;
};

struct FPropertyAccessIndirection
{
	Unknown ArrayProperty;
	class UClass* Function;
	int ReturnBufferSize;
	int ReturnBufferAlignment;
	int ArrayIndex;
	uint32_t Offset;
	EPropertyAccessObjectType ObjectType;
	EPropertyAccessIndirectionType Type;
};

struct FPropertyAccessLibrary
{
	TArray<struct PropertyAccessSegment> PathSegments;
	TArray<struct PropertyAccessPath> SrcPaths;
	TArray<struct PropertyAccessPath> DestPaths;
	struct PropertyAccessCopyBatch CopyBatches;
	TArray<struct PropertyAccessIndirectionChain> SrcAccesses;
	TArray<struct PropertyAccessIndirectionChain> DestAccesses;
	TArray<struct PropertyAccessIndirection> Indirections;
	TArray<int> EventAccessIndices;
};

struct FPropertyPathSegment
{
	struct FName Name;
	int ArrayIndex;
	class UClass* Struct;
};

struct FCachedPropertyPath
{
	TArray<struct PropertyPathSegment> Segments;
	class UClass* CachedFunction;
};

struct FPurchaseFlowItem
{
	struct FString ItemId;
	struct FString EntitlementId;
	struct FString ValidationInfo;
};

struct FPurchaseFlowOffer
{
	struct FString OfferNamespace;
	struct FString OfferId;
	int Quantity;
	TArray<struct PurchaseFlowItem> Items;
};

struct FPurchaseFlowReceiptParam
{
	struct FString TransactionId;
	struct FString TransactionState;
	TArray<struct PurchaseFlowOffer> Offers;
};

struct FQosPingServerInfo
{
	struct FString Address;
	int Port;
};

struct FQosDatacenterInfo
{
	struct FString ID;
	struct FString RegionId;
	bool bEnabled;
	TArray<struct QosPingServerInfo> Servers;
};

struct FDatacenterQosInstance
{
	struct QosDatacenterInfo Definition;
	EQosDatacenterResult Result;
	int AvgPingMs;
	TArray<int> PingResults;
	struct DateTime LastCheckTimestamp;
	bool bUsable;
};

struct FQosRegionInfo
{
	struct FText DisplayName;
	struct FString RegionId;
	bool bEnabled;
	bool bVisible;
	bool bAutoAssignable;
};

struct FRegionQosInstance
{
	struct QosRegionInfo Definition;
	TArray<struct DatacenterQosInstance> DatacenterOptions;
};

struct FLightPropagationVolumeSettings
{
	bool bOverride_LPVIntensity;
	bool bOverride_LPVDirectionalOcclusionIntensity;
	bool bOverride_LPVDirectionalOcclusionRadius;
	bool bOverride_LPVDiffuseOcclusionExponent;
	bool bOverride_LPVSpecularOcclusionExponent;
	bool bOverride_LPVDiffuseOcclusionIntensity;
	bool bOverride_LPVSpecularOcclusionIntensity;
	bool bOverride_LPVSize;
	bool bOverride_LPVSecondaryOcclusionIntensity;
	bool bOverride_LPVSecondaryBounceIntensity;
	bool bOverride_LPVGeometryVolumeBias;
	bool bOverride_LPVVplInjectionBias;
	bool bOverride_LPVEmissiveInjectionIntensity;
	float LPVIntensity;
	float LPVVplInjectionBias;
	float LPVSize;
	float LPVSecondaryOcclusionIntensity;
	float LPVSecondaryBounceIntensity;
	float LPVGeometryVolumeBias;
	float LPVEmissiveInjectionIntensity;
	float LPVDirectionalOcclusionIntensity;
	float LPVDirectionalOcclusionRadius;
	float LPVDiffuseOcclusionExponent;
	float LPVSpecularOcclusionExponent;
	float LPVDiffuseOcclusionIntensity;
	float LPVSpecularOcclusionIntensity;
	float LPVFadeRange;
	float LPVDirectionalOcclusionFadeRange;
};

struct FLastLocationGatherInfo
{
	class UClass* Connection;
	struct Vector LastLocation;
	struct Vector LastOutOfRangeLocationCheck;
};

struct FConnectionAlwaysRelevantNodePair
{
	class UClass* NetConnection;
	class UClass* Node;
};

struct FAlwaysRelevantActorInfo
{
	class UClass* Connection;
	class UClass* LastViewer;
	class UClass* LastViewTarget;
};

struct FTearOffActorInfo
{
	class UClass* Actor;
};

struct FClassReplicationInfo
{
	float DistancePriorityScale;
	float StarvationPriorityScale;
	float AccumulatedNetPriorityBias;
	uint16_t ReplicationPeriodFrame;
	uint16_t FastPath_ReplicationPeriodFrame;
	uint16_t ActorChannelFrameTimeout;
	float CullDistance;
	float CullDistanceSquared;
};

struct FResonanceAudioReverbPluginSettings
{
	bool bEnableRoomEffects;
	bool bGetTransformFromAudioVolume;
	struct Vector RoomPosition;
	struct Quat RoomRotation;
	struct Vector RoomDimensions;
	ERaMaterialName LeftWallMaterial;
	ERaMaterialName RightWallMaterial;
	ERaMaterialName FloorMaterial;
	ERaMaterialName CeilingMaterial;
	ERaMaterialName FrontWallMaterial;
	ERaMaterialName BackWallMaterial;
	float ReflectionScalar;
	float ReverbGain;
	float ReverbTimeModifier;
	float ReverbBrightness;
};

struct FRigVMRegister
{
	ERigVMRegisterType Type;
	uint32_t ByteIndex;
	uint16_t ElementSize;
	uint16_t ElementCount;
	uint16_t SliceCount;
	byte AlignmentBytes;
	uint16_t TrailingBytes;
	struct FName Name;
	int ScriptStructIndex;
	bool bIsArray;
	bool bIsDynamic;
};

struct FRigVMRegisterOffset
{
	TArray<int> Segments;
	ERigVMRegisterType Type;
	struct FName CPPType;
	class UClass* ScriptStruct;
	class UClass* ParentScriptStruct;
	int ArrayIndex;
	uint16_t ElementSize;
	struct FString CachedSegmentPath;
};

struct FRigVMMemoryContainer
{
	bool bUseNameMap;
	ERigVMMemoryType MemoryType;
	TArray<struct RigVMRegister> Registers;
	TArray<struct RigVMRegisterOffset> RegisterOffsets;
	TArray<byte> Data;
	TArray<class UClass*> ScriptStructs;
	Unknown NameMap;
	bool bEncounteredErrorDuringLoad;
};

struct FRigVMByteCodeEntry
{
	struct FName Name;
	int InstructionIndex;
};

struct FRigVMByteCode
{
	TArray<byte> ByteCode;
	int NumInstructions;
	TArray<struct RigVMByteCodeEntry> Entries;
};

struct FRigVMInstruction
{
	uint64_t ByteCodeIndex;
	ERigVMOpCode OpCode;
	byte OperandAlignment;
};

struct FRigVMInstructionArray
{
	TArray<struct RigVMInstruction> Instructions;
};

struct FRigVMExecuteContext
{
};

struct FRigVMParameter
{
	ERigVMParameterType Type;
	struct FName Name;
	int RegisterIndex;
	struct FString CPPType;
	class UClass* ScriptStruct;
	struct FName ScriptStructPath;
};

struct FRigVMStruct
{
};

struct FRigVMBaseOp
{
};

struct FRigVMSlice
{
};

struct FRigVMOperand
{
	ERigVMMemoryType MemoryType;
	uint16_t RegisterIndex;
	uint16_t RegisterOffset;
};

struct FRigVMMemoryStatistics
{
	uint32_t RegisterCount;
	uint32_t DataBytes;
	uint32_t TotalBytes;
};

struct FRigVMByteCodeStatistics
{
	uint32_t InstructionCount;
	uint32_t DataBytes;
};

struct FRigVMStatistics
{
	uint32_t BytesForCDO;
	uint32_t BytesPerInstance;
	struct RigVMMemoryStatistics LiteralMemory;
	struct RigVMMemoryStatistics WorkMemory;
	uint32_t BytesForCaching;
	struct RigVMByteCodeStatistics ByteCode;
};

struct FEventRecord
{
	struct FString EventType;
	struct DateTime ActiveUntil;
	struct DateTime ActiveSince;
};

struct FEventChannelState
{
	struct DateTime ValidFrom;
	TArray<struct EventRecord> ActiveEvents;
	struct JsonObjectWrapper State;
};

struct FEventsTimeline
{
	struct DateTime CacheExpire;
	TArray<struct EventChannelState> States;
};

struct FCalendarDownload
{
	struct DateTime CurrentTime;
	double CacheIntervalMins;
	Unknown Channels;
};

struct FSequencerBindingProxy
{
	struct Guid BindingID;
	class UClass* Sequence;
};

struct FSequencerScriptingRange
{
	bool bHasStart;
	bool bHasEnd;
	int InclusiveStart;
	int ExclusiveEnd;
	struct FrameRate InternalRate;
};

struct FStructSerializerNumericTestStruct
{
	int8_t Int8;
	int16_t Int16;
	int Int32;
	int64_t Int64;
	byte UInt8;
	uint16_t UInt16;
	uint32_t UInt32;
	uint64_t UInt64;
	float Float;
	double Double;
};

struct FStructSerializerBooleanTestStruct
{
	bool BoolFalse;
	bool BoolTrue;
	bool Bitfield0;
	bool Bitfield1;
	bool Bitfield2Set;
	bool Bitfield3;
	bool Bitfield4Set;
	bool Bitfield5Set;
	bool Bitfield6;
	bool Bitfield7Set;
};

struct FStructSerializerObjectTestStruct
{
	class UClass* Class;
	class UClass* SubClass;
	struct TSoftClassPtr<UObject> SoftClass;
	class UClass* Object;
	Unknown WeakObject;
	struct TSoftClassPtr<UObject> SoftObject;
	struct SoftClassPath ClassPath;
	struct SoftObjectPath ObjectPath;
};

struct FStructSerializerBuiltinTestStruct
{
	struct Guid Guid;
	struct FName Name;
	struct FString String;
	struct FText Text;
	struct Vector Vector;
	struct Vector4 Vector4;
	struct Rotator Rotator;
	struct Quat Quat;
	struct Color Color;
};

struct FStructSerializerArrayTestStruct
{
	TArray<int> Int32Array;
	TArray<byte> ByteArray;
	int StaticSingleElement;
	int StaticInt32Array;
	float StaticFloatArray;
	TArray<struct Vector> VectorArray;
	TArray<struct StructSerializerBuiltinTestStruct> StructArray;
};

struct FStructSerializerMapTestStruct
{
	Unknown IntToStr;
	Unknown StrToStr;
	Unknown StrToVec;
	Unknown StrToStruct;
};

struct FStructSerializerSetTestStruct
{
	Unknown StrSet;
	Unknown IntSet;
	Unknown NameSet;
	Unknown StructSet;
};

struct FStructSerializerTestStruct
{
	struct StructSerializerNumericTestStruct Numerics;
	struct StructSerializerBooleanTestStruct Booleans;
	struct StructSerializerObjectTestStruct Objects;
	struct StructSerializerBuiltinTestStruct Builtins;
	struct StructSerializerArrayTestStruct Arrays;
	struct StructSerializerMapTestStruct Maps;
	struct StructSerializerSetTestStruct Sets;
};

struct FStructSerializerByteArray
{
	int Dummy1;
	TArray<byte> ByteArray;
	int Dummy2;
	TArray<int8_t> Int8Array;
	int Dummy3;
};

struct FSessionServiceLogUnsubscribe
{
};

struct FSessionServiceLogSubscribe
{
};

struct FSessionServiceLog
{
	struct FName Category;
	struct FString Data;
	struct Guid InstanceID;
	double TimeSeconds;
	byte Verbosity;
};

struct FSessionServicePong
{
	bool Authorized;
	struct FString BuildDate;
	struct FString DeviceName;
	struct Guid InstanceID;
	struct FString InstanceName;
	struct FString PlatformName;
	struct Guid SessionId;
	struct FString SessionName;
	struct FString SessionOwner;
	bool Standalone;
};

struct FSessionServicePing
{
	struct FString UserName;
};

struct FSidecarFileInfo
{
	Unknown Meta;
	bool bIsCheckedOut;
	bool bOperationPending;
	struct FString CheckoutGuid;
};

struct FSkyfireItemsAndAbilitySet
{
	TArray<struct TSoftClassPtr<UObject>> Items;
	struct TSoftClassPtr<UObject> AbilitySet;
};

struct FSkyfireTeleportData
{
	struct ScalableFloat PreTeleportTime;
	struct GameplayTag PreTeleportGameplayEventTag;
	struct GameplayTag PostTeleportGameplayEventTag;
};

struct FSkyfirePlayerData
{
};

struct FSkyfireSquadDataEntry
{
	byte SquadId;
	EFortRarity EarnedLootRarity;
	float TeleportToLootRoomTime;
	bool bHasTeleportedToLootRoom;
	class UClass* PlayspaceTeleportSquadPoint;
	class UClass* PrisonTeleportSquadPoint;
	Unknown SquadMembersInMothershipData;
};

struct FSkyfireLootRoomTeleportPointsData
{
	TArray<class UClass*> TeleportPoints;
};

struct FSkyfirePlayerAnalyticData
{
};

struct FSkyfireSquadMemberStatusData
{
	class UClass* PlayerState;
	bool bIsOnMothership;
};

struct FOrbSpawnerConfigByCategory
{
	Unknown ValuesByCategory;
};

struct FOrbCountLimit
{
	struct ScalableFloat LimitValue;
	struct ScalableFloat MaximumOrbCount;
};

struct FOrbSpawnPoint
{
	struct Vector Location;
	class UClass* ActorAtSpawnPoint;
};

struct FSkyfireBoundGameplayEvents
{
};

struct FCachedMarkerData
{
};

struct FGeometry
{
};

struct FMargin
{
	float Left;
	float Top;
	float Right;
	float Bottom;
};

struct FSlateColor
{
	struct LinearColor SpecifiedColor;
	ESlateColorStylingMode ColorUseRule;
};

struct FSlateBrush
{
	struct Vector2D ImageSize;
	struct Margin Margin;
	struct SlateColor TintColor;
	class UClass* ResourceObject;
	struct FName ResourceName;
	struct Box2D UVRegion;
	ESlateBrushDrawType DrawAs;
	ESlateBrushTileType Tiling;
	ESlateBrushMirrorType Mirroring;
	ESlateBrushImageType ImageType;
	bool bIsDynamicallyLoaded;
	bool bHasUObject;
};

struct FInputEvent
{
};

struct FFontOutlineSettings
{
	int OutlineSize;
	bool bSeparateFillAlpha;
	bool bApplyOutlineToDropShadows;
	class UClass* OutlineMaterial;
	struct LinearColor OutlineColor;
};

struct FSlateFontInfo
{
	class UClass* FontObject;
	class UClass* FontMaterial;
	struct FontOutlineSettings OutlineSettings;
	struct FName TypefaceFontName;
	int Size;
	int LetterSpacing;
};

struct FSlateWidgetStyle
{
};

struct FSlateSound
{
	class UClass* ResourceObject;
};

struct FFocusEvent
{
};

struct FFontData
{
	struct FString FontFilename;
	EFontHinting Hinting;
	EFontLoadingPolicy LoadingPolicy;
	int SubFaceIndex;
	class UClass* FontFaceAsset;
};

struct FTypefaceEntry
{
	struct FName Name;
	struct FontData Font;
};

struct FTypeface
{
	TArray<struct TypefaceEntry> Fonts;
};

struct FCompositeFallbackFont
{
	struct Typeface Typeface;
	float ScalingFactor;
};

struct FCompositeFont
{
	struct Typeface DefaultTypeface;
	struct CompositeFallbackFont FallbackTypeface;
	TArray<struct CompositeSubFont> SubTypefaces;
};

struct FCaptureLostEvent
{
};

struct FVirtualKeyboardOptions
{
	bool bEnableAutocorrect;
};

struct FInputChord
{
	struct Key Key;
	bool bShift;
	bool bCtrl;
	bool bAlt;
	bool bCmd;
};

struct FAnchors
{
	struct Vector2D Minimum;
	struct Vector2D Maximum;
};

struct FCustomizedToolMenuEntry
{
	ECustomizedToolMenuVisibility Visibility;
};

struct FCustomizedToolMenuSection
{
	ECustomizedToolMenuVisibility Visibility;
};

struct FCustomizedToolMenuNameArray
{
	TArray<struct FName> Names;
};

struct FCustomizedToolMenu
{
	struct FName Name;
	Unknown Entries;
	Unknown Sections;
	Unknown EntryOrder;
	TArray<struct FName> SectionOrder;
};

struct FSmartObjectID
{
};

struct FSmartObjectSlot
{
	struct Vector Offset;
	struct Vector Direction;
	class UClass* BehaviorConfig;
};

struct FSmartObjectUseConfig
{
	struct GameplayTagQuery AvatarTagFilter;
	struct GameplayTagQuery ObjectTagFilter;
	struct GameplayTagContainer ActivityTags;
	TArray<struct SmartObjectSlot> Slots;
	class UClass* DefaultBehavior;
	uint32_t MaxConcurrentUsers;
	uint32_t DefaultExecutionPriority;
};

struct FSmartObjectExecutionSlot
{
	ESmartObjectSlotState State;
	class UClass* UserAvatar;
	class UClass* AssignedBehavior;
	int SlotIndex;
};

struct FSmartObjectUse
{
	struct SmartObjectExecutionSlot Slot;
};

struct FSmartObjectRuntime
{
	class UClass* SOComponent;
	Unknown AsRichSO;
	TArray<struct SmartObjectUse> ActiveUses;
};

struct FSocialChatMessageEntryTextStyle
{
	struct SlateFontInfo FontInfo;
	struct LinearColor ColorAndOpacity;
};

struct FSocialChatMessageEntryStyle
{
	struct SocialChatMessageEntryTextStyle SenderNameStyle;
	struct SocialChatMessageEntryTextStyle ChannelNameStyle;
	struct SocialChatMessageEntryTextStyle MessageTextStyle;
};

struct FSoundCueCrossfadeInfo
{
	struct DistanceDatum DistanceInfo;
	class UClass* Sound;
};

struct FSoundCueTemplateQualitySettings
{
	struct FText DisplayName;
	int MaxConcatenatedVariations;
	int MaxRandomizedVariations;
	int MaxMixVariations;
};

struct FSoundLibraryAnimContextSettings
{
	struct FloatInterval AnimRateThreshold;
	struct GameplayTag EventName;
	bool bAttachToActor;
	bool bFadeOutOnEnd;
	float FadeTime;
	float NotifyTriggerChance;
};

struct FSoundLibraryPlaySoundResult
{
	struct GameplayTag EventName;
	bool bWasBlocked;
	bool bSuccess;
	TArray<class UClass*> AudioComponents;
};

struct FSimpleSoundLibraryContextSettings
{
	struct GameplayTag EventName;
	bool bIs1P;
	bool bForceComponentCreation;
	float VolumeMultiplier;
	float PitchMultiplier;
	float StartTime;
	class UClass* ConcurrencySettings;
	class UClass* AttenuationSettings;
};

struct FAttachedSoundLibraryContextSettings
{
	struct GameplayTag EventName;
	struct FName AttachName;
	EAttachLocation LocationType;
	struct Vector Location;
	float VolumeMultiplier;
	float PitchMultiplier;
	float StartTime;
	class UClass* ConcurrencySettings;
	class UClass* AttenuationSettings;
	bool bStopWhenAttachedDestroyed;
	bool bAutoDestroy;
};

struct FSoundLibraryActorData
{
};

struct FFortDelayRTMMData
{
	bool bDelayRTTM;
	float MinRTTMDelay;
	float MaxRTTMDelay;
	float Timestamp;
};

struct FFortSpecialEventGEData
{
	class UClass* GameplayEffect;
	int Level;
};

struct FFortSpecialEventOverrideParts
{
	TArray<class UClass*> OverrideParts;
	struct FortSpecialEventGEData GameplayEffectToApplyOnSwap;
};

struct FPhaseLevelEntry
{
	struct TSoftClassPtr<UObject> LevelToLoad;
	struct FString LevelName;
	bool bLoadLevelOnScriptStart;
	bool bLevelStartsVisible;
	bool bUnloadLevelOnPhaseEnd;
	struct GameplayTagContainer MakeVisibleAtPhaseTags;
	struct GameplayTagContainer UnloadLevelAtPhaseTags;
	struct Vector Location;
	struct Rotator Rotation;
	class UClass* LevelInstance;
};

struct FPhaseInfo
{
	class UClass* PhaseActorClass;
	struct GameplayTag PhaseTag;
	float SequenceTime;
	ESpecialRelevancyMode RelevancyMode;
	int NumOfSquadsInRelevancyGroup;
	struct GameplayTagContainer RequireLevelsFromPhaseTags;
	bool bStartNextPhaseOnPhaseFinished;
	TArray<struct PhaseLevelEntry> Levels;
	class UClass* PhaseActor;
};

struct FSpecialClientEvent
{
	class UClass* PlayerController;
	float TimeSeconds;
	struct GameplayTag EventTag;
	int Count;
};

struct FScriptStartUpStateMessage
{
	EScriptStartupState StartupState;
};

struct FSrirachaInputMappingData
{
	struct FName MainActionName;
	struct FName GamepadActionName;
	struct FText DisplayName;
};

struct FAthenaRadioStation
{
	struct FText Title;
	struct FString StationImage;
	struct FString ResourceID;
};

struct FStreamingRadioSourceData
{
	EStreamingRadioSourceState State;
	int PlayingIndex;
	float FadeoutSeconds;
	struct AthenaRadioStation SourceOverride;
};

struct FSrirachaPerClassSpecialSeats
{
	TArray<int> OutsideSeatIndices;
};

struct FUVMapSettings
{
	struct Vector Size;
	struct Vector2D UVTile;
	struct Vector Position;
	struct Rotator Rotation;
	struct Vector Scale;
};

struct FItemData
{
	struct TSoftClassPtr<UObject> Icon;
	struct FText Name;
};

struct FSubtitleFormat
{
	ESubtitleDisplayTextSize SubtitleTextSize;
	ESubtitleDisplayTextColor SubtitleTextColor;
	ESubtitleDisplayTextBorder SubtitleTextBorder;
	ESubtitleDisplayBackgroundOpacity SubtitleBackgroundOpacity;
};

struct FSynth1PatchCable
{
	float Depth;
	ESynth1PatchDestination Destination;
};

struct FEpicSynth1Patch
{
	ESynth1PatchSource PatchSource;
	TArray<struct Synth1PatchCable> PatchCables;
};

struct FModularSynthPresetBankEntry
{
	struct FString PresetName;
	struct ModularSynthPreset Preset;
};

struct FSourceEffectBitCrusherSettings
{
	float CrushedSampleRate;
	struct SoundModulationDestinationSettings SampleRateModulation;
	float CrushedBits;
	struct SoundModulationDestinationSettings BitModulation;
};

struct FSourceEffectChorusSettings
{
	float Depth;
	float Frequency;
	float Feedback;
	float WetLevel;
	float DryLevel;
	float Spread;
	struct SoundModulationDestinationSettings DepthModulation;
	struct SoundModulationDestinationSettings FrequencyModulation;
	struct SoundModulationDestinationSettings FeedbackModulation;
	struct SoundModulationDestinationSettings WetModulation;
	struct SoundModulationDestinationSettings DryModulation;
	struct SoundModulationDestinationSettings SpreadModulation;
};

struct FSourceEffectDynamicsProcessorSettings
{
	ESourceEffectDynamicsProcessorType DynamicsProcessorType;
	ESourceEffectDynamicsPeakMode PeakMode;
	float LookAheadMsec;
	float AttackTimeMsec;
	float ReleaseTimeMsec;
	float ThresholdDb;
	float Ratio;
	float KneeBandwidthDb;
	float InputGainDb;
	float OutputGainDb;
	bool bStereoLinked;
	bool bAnalogMode;
};

struct FSourceEffectEnvelopeFollowerSettings
{
	float AttackTime;
	float ReleaseTime;
	EEnvelopeFollowerPeakMode PeakMode;
	bool bIsAnalogMode;
};

struct FSourceEffectEQBand
{
	float Frequency;
	float Bandwidth;
	float GainDb;
	bool bEnabled;
};

struct FSourceEffectEQSettings
{
	TArray<struct SourceEffectEQBand> EQBands;
};

struct FSourceEffectFilterAudioBusModulationSettings
{
	class UClass* AudioBus;
	int EnvelopeFollowerAttackTimeMsec;
	int EnvelopeFollowerReleaseTimeMsec;
	float EnvelopeGainMultiplier;
	ESourceEffectFilterParam FilterParam;
	float MinFrequencyModulation;
	float MaxFrequencyModulation;
	float MinResonanceModulation;
	float MaxResonanceModulation;
};

struct FSourceEffectFilterSettings
{
	ESourceEffectFilterCircuit FilterCircuit;
	ESourceEffectFilterType FilterType;
	float CutoffFrequency;
	float FilterQ;
	TArray<struct SourceEffectFilterAudioBusModulationSettings> AudioBusModulation;
};

struct FSourceEffectFoldbackDistortionSettings
{
	float InputGainDb;
	float ThresholdDb;
	float OutputGainDb;
};

struct FSourceEffectMidSideSpreaderSettings
{
	float SpreadAmount;
	EStereoChannelMode InputMode;
	EStereoChannelMode OutputMode;
	bool bEqualPower;
};

struct FSourceEffectPannerSettings
{
	float Spread;
	float Pan;
};

struct FSourceEffectPhaserSettings
{
	float WetLevel;
	float Frequency;
	float Feedback;
	EPhaserLFOType LFOType;
	bool UseQuadraturePhase;
};

struct FSourceEffectRingModulationSettings
{
	ERingModulatorTypeSourceEffect ModulatorType;
	float Frequency;
	float Depth;
	float DryLevel;
	float WetLevel;
	class UClass* AudioBusModulator;
};

struct FSourceEffectSimpleDelaySettings
{
	float SpeedOfSound;
	float DelayAmount;
	float DryAmount;
	float WetAmount;
	float Feedback;
	bool bDelayBasedOnDistance;
};

struct FSourceEffectStereoDelaySettings
{
	EStereoDelaySourceEffect DelayMode;
	float DelayTimeMsec;
	float Feedback;
	float DelayRatio;
	float WetLevel;
	float DryLevel;
	bool bFilterEnabled;
	EStereoDelayFiltertype FilterType;
	float FilterFrequency;
	float FilterQ;
};

struct FSourceEffectWaveShaperSettings
{
	float Amount;
	float OutputGainDb;
};

struct FSubmixEffectConvolutionReverbSettings
{
	float NormalizationVolumeDb;
	bool bBypass;
	bool bMixInputChannelFormatToImpulseResponseFormat;
	bool bMixReverbOutputToOutputChannelFormat;
	float SurroundRearChannelBleedDb;
	bool bInvertRearChannelBleedPhase;
	bool bSurroundRearChannelFlip;
	float SurroundRearChannelBleedAmount;
	class UClass* ImpulseResponse;
	bool AllowHArdwareAcceleration;
};

struct FSubmixEffectDelaySettings
{
	float MaximumDelayLength;
	float InterpolationTime;
	float DelayLength;
};

struct FSubmixEffectFilterSettings
{
	ESubmixFilterType FilterType;
	ESubmixFilterAlgorithm FilterAlgorithm;
	float FilterFrequency;
	float FilterQ;
};

struct FSubmixEffectFlexiverbSettings
{
	float PreDelay;
	float DecayTime;
	float RoomDampening;
	int Complexity;
};

struct FDynamicsBandSettings
{
	float CrossoverTopFrequency;
	float AttackTimeMsec;
	float ReleaseTimeMsec;
	float ThresholdDb;
	float Ratio;
	float KneeBandwidthDb;
	float InputGainDb;
	float OutputGainDb;
};

struct FSubmixEffectMultibandCompressorSettings
{
	ESubmixEffectDynamicsProcessorType DynamicsProcessorType;
	ESubmixEffectDynamicsPeakMode PeakMode;
	float LookAheadMsec;
	bool bLinkChannels;
	bool bAnalogMode;
	bool bFourPole;
	TArray<struct DynamicsBandSettings> Bands;
};

struct FSubmixEffectStereoDelaySettings
{
	EStereoDelaySourceEffect DelayMode;
	float DelayTimeMsec;
	float Feedback;
	float DelayRatio;
	float WetLevel;
	float DryLevel;
	bool bFilterEnabled;
	EStereoDelayFiltertype FilterType;
	float FilterFrequency;
	float FilterQ;
};

struct FTapDelayInfo
{
	ETapLineMode TapLineMode;
	float DelayLength;
	float Gain;
	int OutputChannel;
	float PanInDegrees;
	int TapId;
};

struct FSubmixEffectTapDelaySettings
{
	float MaximumDelayLength;
	float InterpolationTime;
	TArray<struct TapDelayInfo> Taps;
};

struct FPatchId
{
	int ID;
};

struct FSourceEffectBitCrusherBaseSettings
{
	float SampleRate;
	float BitDepth;
};

struct FSourceEffectChorusBaseSettings
{
	float Depth;
	float Frequency;
	float Feedback;
	float WetLevel;
	float DryLevel;
	float Spread;
};

struct FTargetingTaskSet
{
	TArray<class UClass*> Tasks;
};

struct FTargetingRequestHandle
{
};

struct FTargetingDebugData
{
};

struct FTargetingAsyncTaskData
{
};

struct FTargetingRequestData
{
	struct FDelegate TargetingRequestDynamicDelegate;
};

struct FTargetingSourceContext
{
	class UClass* SourceActor;
	class UClass* InstigatorActor;
	struct Vector SourceLocation;
	struct FName SourceSocketName;
};

struct FTargetingDefaultResultData
{
	struct HitResult HitResult;
};

struct FTargetingDefaultResultsSet
{
	TArray<struct TargetingDefaultResultData> TargetResults;
};

struct FTemplateSequenceBindingOverrideData
{
	Unknown Object;
	bool bOverridesDefault;
};

struct FTemplateSectionPropertyScale
{
	struct Guid ObjectBinding;
	struct MovieScenePropertyBinding PropertyBinding;
	ETemplateSectionPropertyScaleType PropertyScaleType;
	struct MovieSceneFloatChannel FloatChannel;
};

struct FTimedDataInputEvaluationData
{
	float DistanceToNewestSampleSeconds;
	float DistanceToOldestSampleSeconds;
};

struct FTimedDataChannelSampleTime
{
};

struct FTimeSynthClipSound
{
	class UClass* SoundWave;
	float RandomWeight;
	struct Vector2D DistanceRange;
};

struct FTimeSynthTimeDef
{
	int NumBars;
	int NumBeats;
};

struct FTimeSynthQuantizationSettings
{
	float BeatsPerMinute;
	int BeatsPerBar;
	ETimeSynthBeatDivision BeatDivision;
	float EventDelaySeconds;
	ETimeSynthEventQuantization GlobalQuantization;
};

struct FTimeSynthFilterSettings
{
	ETimeSynthFilterType FilterType;
	float CutoffFrequency;
	float FilterQ;
};

struct FTimeSynthEnvelopeFollowerSettings
{
	float AttackTime;
	float ReleaseTime;
	ETimeSynthEnvelopeFollowerPeakMode PeakMode;
	bool bIsAnalogMode;
};

struct FTimeSynthClipHandle
{
	struct FName ClipName;
	int ClipId;
};

struct FTimeSynthSpectralData
{
	float FrequencyHz;
	float Magnitude;
};

struct FUdpMockMessage
{
	TArray<byte> Data;
};

struct FEvasiveManeuver_Step
{
	struct ScalableFloat RightMultiplier;
	struct ScalableFloat UpMultiplier;
};

struct FHotfixableUmami3DAgentConfiguration
{
	struct ScalableFloat ProjectionExtentsX;
	struct ScalableFloat ProjectionExtentsY;
	struct ScalableFloat ProjectionExtentsZ;
};

struct FDestroyBlockerOffsetConfiguration
{
	struct ScalableFloat XOffset;
	struct ScalableFloat YOffset;
	struct ScalableFloat ZOffset;
};

struct FKidnapCustomTargetWeightConfiguration
{
	bool bCheckOnVehicle;
	struct GameplayTag TargetTag;
	float WeightDelta;
};

struct FUmamiMaximumStuckTimePerBuildingTypeDigested
{
	struct GameplayTag RequiredTag;
};

struct FUmamiMaximumStuckTimePerBuildingType
{
	struct GameplayTag RequiredTag;
	struct ScalableFloat MaximumStuckTime;
};

struct FEvasiveManeuver_Step_Digested
{
	float RightMultiplier;
	float UpMultiplier;
};

struct FUmami3DNavigationGraphWrapper
{
};

struct FWidgetTransform
{
	struct Vector2D Translation;
	struct Vector2D Scale;
	struct Vector2D Shear;
	float Angle;
};

struct FNamedSlotBinding
{
	struct FName Name;
	class UClass* Content;
};

struct FAnimationEventBinding
{
	class UClass* Animation;
	struct FDelegate Delegate;
	EWidgetAnimationEvent AnimationEvent;
	struct FName UserTag;
};

struct FSlateChildSize
{
	float Value;
	ESlateSizeRule SizeRule;
};

struct FRadialBoxSettings
{
	float StartingAngle;
	bool bDistributeItemsEvenly;
	float AngleBetweenItems;
	float SectorCentralAngle;
};

struct FUserWidgetPool
{
	TArray<class UClass*> ActiveWidgets;
	TArray<class UClass*> InactiveWidgets;
};

struct FShapedTextOptions
{
	bool bOverride_TextShapingMethod;
	bool bOverride_TextFlowDirection;
	ETextShapingMethod TextShapingMethod;
	ETextFlowDirection TextFlowDirection;
};

struct FAnchorData
{
	struct Margin Offsets;
	struct Anchors Anchors;
	struct Vector2D Alignment;
};

struct FMovieScene2DTransformMask
{
	uint32_t Mask;
};

struct FSlateMeshVertex
{
	struct Vector2D Position;
	struct Color Color;
	struct Vector2D UV0;
	struct Vector2D UV1;
	struct Vector2D UV2;
	struct Vector2D UV3;
	struct Vector2D UV4;
	struct Vector2D UV5;
};

struct FWidgetAnimationBinding
{
	struct FName WidgetName;
	struct FName SlotWidgetName;
	struct Guid AnimationGuid;
	bool bIsRootWidget;
};

struct FBlueprintWidgetAnimationDelegateBinding
{
	EWidgetAnimationEvent Action;
	struct FName AnimationToBind;
	struct FName FunctionNameToBind;
	struct FName UserTag;
};

struct FDelegateRuntimeBinding
{
	struct FString ObjectName;
	struct FName PropertyName;
	struct FName FunctionName;
	struct DynamicPropertyPath SourcePath;
	EBindingKind Kind;
};

struct FWidgetNavigationData
{
	EUINavigationRule Rule;
	struct FName WidgetToFocus;
	Unknown Widget;
	struct FDelegate CustomDelegate;
};

struct FEventReply
{
};

struct FPaintContext
{
};

struct FModInteractCategory
{
	struct GameplayTag CategoryTag;
	TArray<struct FName> InteractSockets;
};

struct FFortDagwoodCmd
{
	float ForwardAlpha;
	float RightAlpha;
	float AccelerationAlpha;
	struct Vector_NetQuantize100 MovementDir;
	bool bBoost;
	bool bHandbrake;
};

struct FRuntimeBoostInfo
{
	bool bCanBoost;
	bool bUsesRechargeableBoost;
	float BoostPushForce;
	float BoostTopSpeedForceMultiplier;
	float BoostTopSpeedMultiplier;
	float RechargeableBoostRateOfRegen;
	float RechargeableBoostRateOfUse;
};

struct FRuntimeFuelInfo
{
	float MaxFuel;
	float FuelPerSecondDriving;
	float FuelPerSecondBoosting;
};

struct FRuntimeSpringsInfo
{
	float SpringStiffMultiplier;
	float SpringDampMultiplier;
	float SpringLengthMultiplier;
	float RearSpringLengthMultiplier;
	float TireZOffset;
	float TireZOffset_B;
};

struct FRuntimeGearInfo
{
	int GearIndex;
	float TopSpeed;
	float MinSpeed;
	float PushForce;
};

struct FDagwoodRuntimeModifiers
{
	struct RuntimeBoostInfo BoostInfo;
	struct RuntimeFuelInfo FuelInfo;
	struct RuntimeSpringsInfo SpringsInfo;
	bool bDamageFriendlyFire;
	bool bDamageOtherVehicles;
	bool bDamageOwnVehicle;
	float GravityMultiplier;
	float MaxInclineAngle;
	float MaxTiltAngle;
	TArray<struct RuntimeGearInfo> GearInfos;
	struct GameplayTag TireModTag;
	byte TireModVersion;
	byte DataVersion;
};

struct FDagwoodState_PT
{
	struct DagwoodInternal internal;
	struct DagwoodOutPersistent OutPersistent;
	struct DagwoodOutContinuous OutContinuous;
	bool bOutValid;
};

struct FDagwoodFutureClientInput
{
	int Frame;
	struct FortDagwoodCmd InputCmd;
};

struct FDagwoodManagedState
{
	int Frame;
	struct FortDagwoodCmd InputCmd;
	struct DagwoodInPersistent GT_State;
	struct DagwoodState_PT PT_State;
	class UClass* PC;
	TArray<struct DagwoodFutureClientInput> FutureInputs;
};

struct FMountedGun
{
	struct WeaponSeatDefinition SeatDefinition;
	class UClass* SkeletalMesh;
	class UClass* AnimInstanceWeapon;
	struct FName SocketNameMuzzle;
	struct FName SocketNameWeaponPawnAttach;
	struct FName SocketNameVehicle;
	float AttachmentAngleOffset;
	class UClass* AnimInstancePawn;
	bool bHasApplied;
};

struct FRuntimeTerrainHandlingInfo
{
	float PushForceMultiplier;
	float TopSpeedMultiplier;
	float FrontTireFriction;
	float RearTireFriction;
};

struct FTireInfo
{
	class UClass* TireStaticMesh;
	class UClass* UndercarriageCollisionStaticMesh;
	struct RuntimeSpringsInfo SpringTuning;
	struct RuntimeTerrainHandlingInfo RoadBehavior;
	struct RuntimeTerrainHandlingInfo DirtBehavior;
	struct RuntimeTerrainHandlingInfo GrassBehavior;
	float PoppedTireSpringLength;
	float PoppedTireSpringStiff;
	float PoppedTireSpringDamp;
	float TireMeshYaw;
	float TireMeshLateralAdjustment;
	float TireMeshScaleFront;
	float TireMeshScaleRear;
	struct ScalableFloat MaxDriveInclineAngle;
	float UprightRollStiff;
	float UprightRollDamp;
	float UprightPitchStiff;
	float UprightPitchDamp;
	float UprightYawStiff;
	float UprightYawDamp;
	float MidAxleForwardAdjust;
	struct ScalableFloat TireHealth;
	struct ScalableFloat WaterTopSpeedMultiplier;
	struct ScalableFloat PontoonScaleRadiusPerTick;
	TArray<struct FName> CollisionShapeNames;
	TArray<struct FName> DamageablePartShapeNames;
	struct FName PoppedTireBoneName;
	struct FName CollisionBoneName;
	class UClass* TireDirtLoop;
	class UClass* TireGrassLoop;
	class UClass* TireRoadLoop;
	class UClass* SuspensionLight;
	class UClass* SuspensionMedium;
	class UClass* SuspensionHeavy;
	class UClass* AppliedGameplayEffect;
};

struct FAttachedWheel
{
	class UClass* StaticMeshComponent;
	class UClass* MatInstance;
	float CurrentHealth;
};

struct FDagwoodActionDefForUI
{
	TArray<struct ActionDefForUI> ActionDefForUI;
};

struct FVehicleAttachment
{
	struct FName ModName;
	struct FName SocketName;
	class UClass* SkeletalMesh;
};

struct FVehiclePropReplacementData
{
	struct TSoftClassPtr<UObject> FortVehicleItemDefinition;
	struct Vector TransOffset;
	struct Rotator RotOffset;
};

struct FTireSimulationRuntimeData
{
	struct RuntimeTerrainHandlingInfo HandlingInfo;
	float PoppedTireSpringLength;
	float PoppedTireSpringStiff;
	float PoppedTireSpringDamp;
};

struct FValetUpdateContext
{
};

struct FVkContentPackageRequest
{
	struct FString ProjectID;
	struct VkModuleVersionRef Root;
	Unknown Resolutions;
};

struct FVkModuleBinaries
{
	struct FString BaseUrl;
	struct FString Manifest;
	TArray<struct FString> Files;
	double TotalSizeKb;
	double ManifestSizeKb;
	double ManifestDiskSizeKb;
	double ManifestDownloadSizeKb;
};

struct FVkResolvedModule
{
	struct FString ModuleId;
	struct VkModuleBinaries Binaries;
};

struct FVkModuleVersionRef
{
	struct FString ModuleId;
	struct FString Version;
};

struct FVkPublishRequest
{
	struct VkModuleVersionRef Root;
	Unknown Resolutions;
	struct FString MatchmakingScheme;
	struct JsonObjectWrapper Meta;
};

struct FVkCreateModuleVersionRequest
{
	Unknown RawFiles;
	TArray<struct VkModuleVersionRef> Dependencies;
	struct JsonObjectWrapper Meta;
	struct FString Checksum;
};

struct FVkCreateModuleRequest
{
	struct FString ModuleName;
	struct JsonObjectWrapper Meta;
	struct FString ContentType;
	Unknown Relevance;
	struct FString DesiredModuleId;
};

struct FVkNamedId
{
	struct FString Name;
	struct FString ID;
};

struct FVkCreateProjectRequest
{
	struct FString DesiredProjectId;
	struct FString OwnerId;
	struct JsonObjectWrapper Meta;
	Unknown Access;
};

struct FVkPublishedLink
{
	struct FString LinkCode;
	int LinkVersion;
	struct DateTime LastPublished;
	struct FString RootModuleId;
	int ModuleVersion;
	struct FString MatchmakingScheme;
};

struct FVkProjectDoc
{
	struct FString ProjectID;
	struct DateTime Created;
	struct DateTime Updated;
	struct VkNamedId Owner;
	struct VkNamedId Author;
	struct JsonObjectWrapper Meta;
	Unknown Access;
	Unknown Links;
};

struct FVkJobOutput
{
	struct FString BaseUrl;
	int TotalSizeKb;
	TArray<struct FString> Files;
	struct FString Manifest;
};

struct FVkBuildVersion
{
	struct FString Major;
	int Minor;
	int Patch;
};

struct FVkContentFilter
{
	struct FString Platform;
};

struct FVkArtifactOption
{
	struct FString ArtifactId;
	EConsumerRole Role;
	struct VkContentFilter Filter;
};

struct FVkModuleVersionDoc
{
	struct FString ModuleId;
	int Version;
	struct FString ProjectID;
	struct DateTime Created;
	struct VkNamedId Author;
	struct JsonObjectWrapper Meta;
	TArray<struct VkModuleVersionRef> Dependencies;
	struct VkJobOutput StagedFiles;
	struct VkBuildVersion SourceVersion;
	struct FString ContentType;
	Unknown Relevance;
	struct FString ArtifactKey;
	struct FString Checksum;
	TArray<struct VkArtifactOption> Artifacts;
};

struct FVkContentManifest
{
	TArray<struct VkResolvedModule> Content;
};

struct FVkModuleVersion
{
	struct FString ModuleId;
	int Version;
};

struct FVkContentPackage
{
	struct FString ProjectID;
	Unknown ProjectFlags;
	TArray<struct VkModuleVersionWithArtifacts> Content;
};

struct FVkModuleDoc
{
	struct FString ModuleId;
	struct DateTime Created;
	struct DateTime Updated;
	struct VkNamedId Author;
	struct FString ProjectID;
	struct FString ModuleName;
	struct FString ContentType;
	Unknown Relevance;
	struct FString AliasForModuleId;
	struct JsonObjectWrapper Meta;
	Unknown Labels;
	int LatestVersion;
};

struct FVkMetaDataFlags
{
	Unknown _validation_flags;
};

struct FVkPersistenceVersion
{
	int Version;
	struct FString Name;
};

struct FVoteSelectionSettings
{
	struct GameplayTag VoteSelectionId;
	struct FText VoteSelectionText;
};

struct FVoteSettings
{
	struct GameplayTag VoteId;
	bool bLocksSelection;
	struct FText VoteText;
	TArray<struct VoteSelectionSettings> VoteSelections;
};

struct FVoteSessionSettings
{
	struct GameplayTag VoteSessionId;
	struct FText VoteSessionText;
	TArray<struct VoteSettings> VoteSettingsList;
	EVoteSessionNetworkType NetworkType;
};

struct FCastedVote
{
	struct FName VoteSessionId;
	struct FName VoteId;
	struct FName VoteSelectionId;
};

struct FVoteSelection
{
	struct GameplayTag VoteSessionId;
	struct GameplayTag VoteId;
	struct GameplayTag VoteSelectionId;
};

struct FSphericalPontoon
{
	struct FName CenterSocket;
	struct Vector RelativeLocation;
	float Radius;
	struct Vector LocalForce;
	struct Vector CenterLocation;
	struct Quat SocketRotation;
	struct Vector Offset;
	float WaterHeight;
	float WaterDepth;
	float ImmersionDepth;
	struct Vector WaterPlaneLocation;
	struct Vector WaterPlaneNormal;
	struct Vector WaterSurfacePosition;
	struct Vector WaterVelocity;
	int WaterBodyIndex;
	class UClass* CurrentWaterBody;
};

struct FBuoyancyData
{
	TArray<struct SphericalPontoon> Pontoons;
	float BuoyancyCoefficient;
	float BuoyancyDamp;
	float BuoyancyDamp2;
	float BuoyancyRampMinVelocity;
	float BuoyancyRampMaxVelocity;
	float BuoyancyRampMax;
	float MaxBuoyantForce;
	float WaterShorePushFactor;
	float WaterVelocityStrength;
	float MaxWaterForce;
	float DragCoefficient;
	float DragCoefficient2;
	float AngularDragCoefficient;
	float MaxDragSpeed;
	bool bApplyDragForcesInWater;
};

struct FGerstnerWaveOctave
{
	int NumWaves;
	float AmplitudeScale;
	float MainDirection;
	float SpreadAngle;
	bool bUniformSpread;
};

struct FGerstnerWave
{
	float WaveLength;
	float Amplitude;
	float Steepness;
	struct Vector Direction;
	struct Vector2D WaveVector;
	float WaveSpeed;
	float WKA;
	float Q;
	float PhaseOffset;
};

struct FUnderwaterPostProcessSettings
{
	bool bEnabled;
	float Priority;
	float BlendRadius;
	float BlendWeight;
	struct PostProcessSettings PostProcessSettings;
	class UClass* UnderwaterPostProcessMaterial;
};

struct FWaterCurveSettings
{
	bool bUseCurveChannel;
	class UClass* ElevationCurveAsset;
	float ChannelEdgeOffset;
	float ChannelDepth;
	float CurveRampWidth;
};

struct FWaterSplineCurveDefaults
{
	float DefaultDepth;
	float DefaultWidth;
	float DefaultVelocity;
	float DefaultAudioIntensity;
};

struct FWaterFalloffSettings
{
	EWaterBrushFalloffMode FalloffMode;
	float FalloffAngle;
	float FalloffWidth;
	float EdgeOffset;
	float ZOffset;
};

struct FWaterBrushEffectBlurring
{
	bool bBlurShape;
	int Radius;
};

struct FWaterBrushEffectCurlNoise
{
	float Curl1Amount;
	float Curl2Amount;
	float Curl1Tiling;
	float Curl2Tiling;
};

struct FWaterBrushEffectDisplacement
{
	float DisplacementHeight;
	float DisplacementTiling;
	class UClass* Texture;
	float Midpoint;
	struct LinearColor Channel;
	float WeightmapInfluence;
};

struct FWaterBrushEffectSmoothBlending
{
	float InnerSmoothDistance;
	float OuterSmoothDistance;
};

struct FWaterBrushEffectTerracing
{
	float TerraceAlpha;
	float TerraceSpacing;
	float TerraceSmoothness;
	float MaskLength;
	float MaskStartOffset;
};

struct FWaterBrushEffects
{
	struct WaterBrushEffectBlurring Blurring;
	struct WaterBrushEffectCurlNoise CurlNoise;
	struct WaterBrushEffectDisplacement Displacement;
	struct WaterBrushEffectSmoothBlending SmoothBlending;
	struct WaterBrushEffectTerracing Terracing;
};

struct FWaterBodyHeightmapSettings
{
	EWaterBrushBlendType BlendMode;
	bool bInvertShape;
	struct WaterFalloffSettings FalloffSettings;
	struct WaterBrushEffects Effects;
	int Priority;
};

struct FWaterBodyWeightmapSettings
{
	float FalloffWidth;
	float EdgeOffset;
	class UClass* ModulationTexture;
	float TextureTiling;
	float TextureInfluence;
	float Midpoint;
	float FinalOpacity;
};

struct FWaterBrushEffectCurves
{
	bool bUseCurveChannel;
	class UClass* ElevationCurveAsset;
	float ChannelEdgeOffset;
	float ChannelDepth;
	float CurveRampWidth;
};

struct FWebJSCallbackBase
{
};

