enum EExternalAccountType
{
	None,
	Facebook,
	Google,
	Epic_PSN,
	Epic_XBL,
	Epic_Erebus,
	Epic_Facebook,
	Epic_Google,
	Epic_Portal,
	Epic_Portal_Kairos,
	EExternalAccountType_MAX
};

enum ECreateAccountResult
{
	NotStarted,
	Pending,
	Success,
	Console_LoginFailed,
	Console_DuplicateAuthAssociation,
	DuplicateAccount,
	GenericError,
	ECreateAccountResult_MAX
};

enum ELoginResult
{
	NotStarted,
	Pending,
	Success,
	Console_LoginFailed,
	Console_AuthFailed,
	Console_MissingAuthAssociation,
	Console_DuplicateAuthAssociation,
	Console_AddedAuthAssociation,
	Console_AuthAssociationFailure,
	Console_NotEntitled,
	Console_EntitlementCheckFailed,
	Console_PrivilegeCheck,
	Console_PatchOrUpdateRequired,
	AuthFailed,
	AuthFailed_RefreshInvalid,
	AuthFailed_InvalidMFA,
	AuthFailed_RequiresMFA,
	AuthFailed_AgeVerificationIncomplete,
	AuthFailed_LoginLockout,
	AuthFailed_InvalidCredentials,
	AuthParentalLock,
	PlatformNotAllowed,
	NotEntitled,
	Banned,
	EULACheckFailed,
	WaitingRoomFailed,
	ServiceUnavailable,
	GenericError,
	RegisterSecondaryPlayerInPrimaryPartyFailed,
	RejoinCheckFailure,
	ConnectionFailed,
	NetworkConnectionUnavailable,
	AlreadyLoggingIn,
	ExternalAuth_AddedAuthAssociation,
	ExternalAuth_ConnectionTimeout,
	ExternalAuth_AuthFailure,
	ExternalAuth_AssociationFailure,
	ExternalAuth_MissingAuthAssociation,
	ExternalAuth_Canceled,
	FailedToCreateParty,
	ProfileQueryFailed,
	QueryKeychainFailed,
	ClientSettingsDownloadFailed,
	PinGrantFailure,
	PinGrantTimeout,
	PinGrantCanceled,
	ELoginResult_MAX
};

enum EConsoleAuthLinkState
{
	NotOnConsole,
	ConsoleNotLoggedIn,
	EpicNotLoggedIn,
	ThisEpicAccountLinked,
	OtherEpicAccountLinked,
	NoEpicAccountLinked,
	PrimaryIdNotLinked,
	SecondaryIdNotLinked,
	EConsoleAuthLinkState_MAX
};

enum EACLCompressionLevel
{
	ACLCL_Lowest,
	ACLCL_Low,
	ACLCL_Medium,
	ACLCL_High,
	ACLCL_Highest,
	ACLCL_MAX
};

enum EACLVectorFormat
{
	ACLVF_Vector3_96,
	ACLVF_Vector3_Variable,
	ACLVF_Vector3_MAX
};

enum EACLRotationFormat
{
	ACLRF_Quat_128,
	ACLRF_QuatDropW_96,
	ACLRF_QuatDropW_Variable,
	ACLRF_MAX
};

enum EACLVisualFidelityChangeResult
{
	Dispatched,
	Completed,
	Failed,
	ACLVisualFidelityChangeResult_MAX
};

enum EACLVisualFidelity
{
	Highest,
	Medium,
	Lowest,
	ACLVisualFidelity_MAX
};

enum EPathFollowingResult
{
	Success,
	Blocked,
	OffPath,
	Aborted,
	Skipped_DEPRECATED,
	Invalid,
	EPathFollowingResult_MAX
};

enum EEnvQueryStatus
{
	Processing,
	Success,
	Failed,
	Aborted,
	OwnerLost,
	MissingParam,
	EEnvQueryStatus_MAX
};

enum EAIParamType
{
	Float,
	Int,
	Bool,
	MAX
};

enum EAISenseNotifyType
{
	OnEveryPerception,
	OnPerceptionChange,
	EAISenseNotifyType_MAX
};

enum EAITaskPriority
{
	Lowest,
	Low,
	AutonomousAI,
	High,
	Ultimate,
	EAITaskPriority_MAX
};

enum EGenericAICheck
{
	Less,
	LessOrEqual,
	Equal,
	NotEqual,
	GreaterOrEqual,
	Greater,
	IsTrue,
	MAX
};

enum EAILockSource
{
	Animation,
	Logic,
	Script,
	Gameplay,
	MAX
};

enum EAIRequestPriority
{
	SoftScript,
	Logic,
	HardScript,
	Reaction,
	Ultimate,
	MAX
};

enum EPawnActionEventType
{
	Invalid,
	FailedToStart,
	InstantAbort,
	FinishedAborting,
	FinishedExecution,
	Push,
	EPawnActionEventType_MAX
};

enum EPawnActionResult
{
	NotStarted,
	InProgress,
	Success,
	Failed,
	Aborted,
	EPawnActionResult_MAX
};

enum EPawnActionAbortState
{
	NeverStarted,
	NotBeingAborted,
	MarkPendingAbort,
	LatentAbortInProgress,
	AbortDone,
	MAX
};

enum EFAIDistanceType
{
	Distance3D,
	Distance2D,
	DistanceZ,
	MAX
};

enum EAIOptionFlag
{
	Default,
	Enable,
	Disable,
	MAX
};

enum EBTFlowAbortMode
{
	None,
	LowerPriority,
	Self,
	Both,
	EBTFlowAbortMode_MAX
};

enum EBTNodeResult
{
	Succeeded,
	Failed,
	Aborted,
	InProgress,
	EBTNodeResult_MAX
};

enum ETextKeyOperation
{
	Equal,
	NotEqual,
	Contain,
	NotContain,
	ETextKeyOperation_MAX
};

enum EArithmeticKeyOperation
{
	Equal,
	NotEqual,
	Less,
	LessOrEqual,
	Greater,
	GreaterOrEqual,
	EArithmeticKeyOperation_MAX
};

enum EBasicKeyOperation
{
	Set,
	NotSet,
	EBasicKeyOperation_MAX
};

enum EBTParallelMode
{
	AbortBackground,
	WaitForBackground,
	EBTParallelMode_MAX
};

enum EBTDecoratorLogic
{
	Invalid,
	Test,
	And,
	Or,
	Not,
	EBTDecoratorLogic_MAX
};

enum EBTChildIndex
{
	FirstNode,
	TaskNode,
	EBTChildIndex_MAX
};

enum EBTBlackboardRestart
{
	ValueChange,
	ResultChange,
	EBTBlackboardRestart_MAX
};

enum EBlackBoardEntryComparison
{
	Equal,
	NotEqual,
	EBlackBoardEntryComparison_MAX
};

enum EPathExistanceQueryType
{
	NavmeshRaycast2D,
	HierarchicalQuery,
	RegularPathFinding,
	EPathExistanceQueryType_MAX
};

enum EPointOnCircleSpacingMethod
{
	BySpaceBetween,
	ByNumberOfPoints,
	EPointOnCircleSpacingMethod_MAX
};

enum EEQSNormalizationType
{
	Absolute,
	RelativeToScores,
	EEQSNormalizationType_MAX
};

enum EEnvTestDistance
{
	Distance3D,
	Distance2D,
	DistanceZ,
	DistanceAbsoluteZ,
	EEnvTestDistance_MAX
};

enum EEnvTestDot
{
	Dot3D,
	Dot2D,
	EEnvTestDot_MAX
};

enum EEnvTestPathfinding
{
	PathExist,
	PathCost,
	PathLength,
	EEnvTestPathfinding_MAX
};

enum EEnvQueryTestClamping
{
	None,
	SpecifiedValue,
	FilterThreshold,
	EEnvQueryTestClamping_MAX
};

enum EEnvDirection
{
	TwoPoints,
	Rotation,
	EEnvDirection_MAX
};

enum EEnvOverlapShape
{
	Box,
	Sphere,
	Capsule,
	EEnvOverlapShape_MAX
};

enum EEnvTraceShape
{
	Line,
	Box,
	Sphere,
	Capsule,
	EEnvTraceShape_MAX
};

enum EEnvQueryTrace
{
	None,
	Navigation,
	GeometryByChannel,
	GeometryByProfile,
	NavigationOverLedges,
	EEnvQueryTrace_MAX
};

enum EEnvQueryParam
{
	Float,
	Int,
	Bool,
	EEnvQueryParam_MAX
};

enum EEnvQueryRunMode
{
	SingleResult,
	RandomBest5Pct,
	RandomBest25Pct,
	AllMatching,
	EEnvQueryRunMode_MAX
};

enum EEnvTestScoreOperator
{
	AverageScore,
	MinScore,
	MaxScore,
	Multiply,
	EEnvTestScoreOperator_MAX
};

enum EEnvTestFilterOperator
{
	AllPass,
	AnyPass,
	EEnvTestFilterOperator_MAX
};

enum EEnvTestCost
{
	Low,
	Medium,
	High,
	EEnvTestCost_MAX
};

enum EEnvTestWeight
{
	None,
	Square,
	Inverse,
	Unused,
	Constant,
	Skip,
	EEnvTestWeight_MAX
};

enum EEnvTestScoreEquation
{
	Linear,
	Square,
	InverseLinear,
	SquareRoot,
	Constant,
	EEnvTestScoreEquation_MAX
};

enum EEnvTestFilterType
{
	Minimum,
	Maximum,
	Range,
	Match,
	EEnvTestFilterType_MAX
};

enum EEnvTestPurpose
{
	Filter,
	Score,
	FilterAndScore,
	EEnvTestPurpose_MAX
};

enum EEnvQueryHightlightMode
{
	All,
	Best5Pct,
	Best25Pct,
	EEnvQueryHightlightMode_MAX
};

enum ETeamAttitude
{
	Friendly,
	Neutral,
	Hostile,
	ETeamAttitude_MAX
};

enum EPathFollowingRequestResult
{
	Failed,
	AlreadyAtGoal,
	RequestSuccessful,
	EPathFollowingRequestResult_MAX
};

enum EPathFollowingAction
{
	Error,
	NoMove,
	DirectMove,
	PartialPath,
	PathToGoal,
	EPathFollowingAction_MAX
};

enum EPathFollowingStatus
{
	Idle,
	Waiting,
	Paused,
	Moving,
	EPathFollowingStatus_MAX
};

enum EPawnActionFailHandling
{
	RequireSuccess,
	IgnoreFailure,
	EPawnActionFailHandling_MAX
};

enum EPawnSubActionTriggeringPolicy
{
	CopyBeforeTriggering,
	ReuseInstances,
	EPawnSubActionTriggeringPolicy_MAX
};

enum EPawnActionMoveMode
{
	UsePathfinding,
	StraightLine,
	EPawnActionMoveMode_MAX
};

enum EAmbientAudioTagActionType
{
	Added,
	Removed,
	Count,
	EAmbientAudioTagActionType_MAX
};

enum EAmbientAudioEntryActionType
{
	Added,
	Updated,
	Removed,
	Count,
	EAmbientAudioEntryActionType_MAX
};

enum ETransformConstraintType
{
	Translation,
	Rotation,
	Scale,
	Parent,
	ETransformConstraintType_MAX
};

enum EConstraintType
{
	Transform,
	Aim,
	MAX
};

enum ESphericalLimitType
{
	Inner,
	Outer,
	ESphericalLimitType_MAX
};

enum EAnimPhysSimSpaceType
{
	Component,
	Actor,
	World,
	RootRelative,
	BoneRelative,
	AnimPhysSimSpaceType_MAX
};

enum EAnimPhysLinearConstraintType
{
	Free,
	Limited,
	AnimPhysLinearConstraintType_MAX
};

enum EAnimPhysAngularConstraintType
{
	Angular,
	Cone,
	AnimPhysAngularConstraintType_MAX
};

enum EBlendListTransitionType
{
	StandardBlend,
	Inertialization,
	EBlendListTransitionType_MAX
};

enum EDrivenDestinationMode
{
	Bone,
	MorphTarget,
	MaterialParameter,
	EDrivenDestinationMode_MAX
};

enum EDrivenBoneModificationMode
{
	AddToInput,
	ReplaceComponent,
	AddToRefPose,
	EDrivenBoneModificationMode_MAX
};

enum EConstraintOffsetOption
{
	None,
	Offset_RefPose,
	EConstraintOffsetOption_MAX
};

enum ECopyBoneDeltaMode
{
	Accumulate,
	Copy,
	CopyBoneDeltaMode_MAX
};

enum EInterpolationBlend
{
	Linear,
	Cubic,
	Sinusoidal,
	EaseInOutExponent2,
	EaseInOutExponent3,
	EaseInOutExponent4,
	EaseInOutExponent5,
	MAX
};

enum EBoneModificationMode
{
	BMM_Ignore,
	BMM_Replace,
	BMM_Additive,
	BMM_MAX
};

enum EModifyCurveApplyMode
{
	Add,
	Scale,
	Blend,
	WeightedMovingAverage,
	RemapCurve,
	EModifyCurveApplyMode_MAX
};

enum EPoseDriverOutput
{
	DrivePoses,
	DriveCurves,
	EPoseDriverOutput_MAX
};

enum EPoseDriverSource
{
	Rotation,
	Translation,
	EPoseDriverSource_MAX
};

enum EPoseDriverType
{
	SwingAndTwist,
	SwingOnly,
	Translation,
	EPoseDriverType_MAX
};

enum ESnapshotSourceMode
{
	NamedSnapshot,
	SnapshotPin,
	ESnapshotSourceMode_MAX
};

enum ERefPoseType
{
	EIT_LocalSpace,
	EIT_Additive,
	EIT_MAX
};

enum ESimulationSpace
{
	ComponentSpace,
	WorldSpace,
	BaseBoneSpace,
	ESimulationSpace_MAX
};

enum EScaleChainInitialLength
{
	FixedDefaultLengthValue,
	Distance,
	ChainLength,
	EScaleChainInitialLength_MAX
};

enum ESequenceEvalReinit
{
	NoReset,
	StartPosition,
	ExplicitTime,
	ESequenceEvalReinit_MAX
};

enum ESplineBoneAxis
{
	None,
	X,
	Y,
	Z,
	ESplineBoneAxis_MAX
};

enum ERotationComponent
{
	EulerX,
	EulerY,
	EulerZ,
	QuaternionAngle,
	SwingAngle,
	TwistAngle,
	ERotationComponent_MAX
};

enum EEasingFuncType
{
	Linear,
	Sinusoidal,
	Cubic,
	QuadraticInOut,
	CubicInOut,
	HermiteCubic,
	QuarticInOut,
	QuinticInOut,
	CircularIn,
	CircularOut,
	CircularInOut,
	ExpIn,
	ExpOut,
	ExpInOut,
	CustomCurve,
	EEasingFuncType_MAX
};

enum ERBFNormalizeMethod
{
	OnlyNormalizeAboveOne,
	AlwaysNormalize,
	NormalizeWithinMedian,
	NoNormalization,
	ERBFNormalizeMethod_MAX
};

enum ERBFDistanceMethod
{
	Euclidean,
	Quaternion,
	SwingAngle,
	TwistAngle,
	DefaultMethod,
	ERBFDistanceMethod_MAX
};

enum ERBFFunctionType
{
	Gaussian,
	Exponential,
	Linear,
	Cubic,
	Quintic,
	DefaultFunction,
	ERBFFunctionType_MAX
};

enum ERBFSolverType
{
	Additive,
	Interpolative,
	ERBFSolverType_MAX
};

enum ECollectionScriptingShareType
{
	Local,
	Private,
	Shared,
	ECollectionScriptingShareType_MAX
};

enum EAudioDeviceChangedRole
{
	Invalid,
	Console,
	Multimedia,
	Communications,
	Count,
	EAudioDeviceChangedRole_MAX
};

enum EAudioDeviceChangedState
{
	Invalid,
	Active,
	Disabled,
	NotPresent,
	Unplugged,
	Count,
	EAudioDeviceChangedState_MAX
};

enum EAudioMixerChannelType
{
	FrontLeft,
	FrontRight,
	FrontCenter,
	LowFrequency,
	BackLeft,
	BackRight,
	FrontLeftOfCenter,
	FrontRightOfCenter,
	BackCenter,
	SideLeft,
	SideRight,
	TopCenter,
	TopFrontLeft,
	TopFrontCenter,
	TopFrontRight,
	TopBackLeft,
	TopBackCenter,
	TopBackRight,
	Unknown,
	ChannelTypeCount,
	DefaultChannel,
	EAudioMixerChannelType_MAX
};

enum EAudioMixerStreamDataFormatType
{
	Unknown,
	Float,
	Int16,
	Unsupported,
	EAudioMixerStreamDataFormatType_MAX
};

enum ESwapAudioOutputDeviceResultState
{
	Failure,
	Success,
	None,
	ESwapAudioOutputDeviceResultState_MAX
};

enum EMusicalNoteName
{
	C,
	Db,
	D,
	Eb,
	E,
	F,
	Gb,
	G,
	Ab,
	A,
	Bb,
	B,
	EMusicalNoteName_MAX
};

enum ESubmixEffectDynamicsKeySource
{
	Default,
	AudioBus,
	Submix,
	Count,
	ESubmixEffectDynamicsKeySource_MAX
};

enum ESubmixEffectDynamicsChannelLinkMode
{
	Disabled,
	Average,
	Peak,
	Count,
	ESubmixEffectDynamicsChannelLinkMode_MAX
};

enum ESubmixEffectDynamicsPeakMode
{
	MeanSquared,
	RootMeanSquared,
	Peak,
	Count,
	ESubmixEffectDynamicsPeakMode_MAX
};

enum ESubmixEffectDynamicsProcessorType
{
	Compressor,
	Limiter,
	Expander,
	Gate,
	Count,
	ESubmixEffectDynamicsProcessorType_MAX
};

enum EQuarztClockManagerType
{
	AudioEngine,
	QuartzSubsystem,
	Count,
	EQuarztClockManagerType_MAX
};

enum ESoundwaveSampleRateSettings
{
	Max,
	High,
	Medium,
	Low,
	Min,
	MatchDevice
};

enum EConstantQFFTSizeEnum
{
	Min,
	XXSmall,
	XSmall,
	Small,
	Medium,
	Large,
	XLarge,
	XXLarge,
	Max
};

enum EConstantQNormalizationEnum
{
	EqualEuclideanNorm,
	EqualEnergy,
	EqualAmplitude,
	EConstantQNormalizationEnum_MAX
};

enum ELoudnessNRTCurveTypeEnum
{
	A,
	B,
	C,
	D,
	None,
	ELoudnessNRTCurveTypeEnum_MAX
};

enum EARTrackingState
{
	Unknown,
	Tracking,
	NotTracking,
	StoppedTracking,
	EARTrackingState_MAX
};

enum EGeoAnchorComponentDebugMode
{
	None,
	ShowGeoData,
	EGeoAnchorComponentDebugMode_MAX
};

enum EPoseComponentDebugMode
{
	None,
	ShowSkeleton,
	EPoseComponentDebugMode_MAX
};

enum EQRCodeComponentDebugMode
{
	None,
	ShowQRCode,
	EQRCodeComponentDebugMode_MAX
};

enum EImageComponentDebugMode
{
	None,
	ShowDetectedImage,
	EImageComponentDebugMode_MAX
};

enum EARFaceTransformMixing
{
	ComponentOnly,
	ComponentLocationTrackedRotation,
	ComponentWithTracked,
	TrackingOnly,
	EARFaceTransformMixing_MAX
};

enum EFaceComponentDebugMode
{
	None,
	ShowEyeVectors,
	ShowFaceMesh,
	EFaceComponentDebugMode_MAX
};

enum EPlaneComponentDebugMode
{
	None,
	ShowNetworkRole,
	ShowClassification,
	EPlaneComponentDebugMode_MAX
};

enum EARSessionConfigFlags
{
	None,
	GenerateMeshData,
	RenderMeshDataInWireframe,
	GenerateCollisionForMeshData,
	GenerateNavMeshForMeshData,
	UseMeshDataForOcclusion,
	EARSessionConfigFlags_MAX
};

enum EARServicePermissionRequestResult
{
	Granted,
	Denied,
	EARServicePermissionRequestResult_MAX
};

enum EARServiceInstallRequestResult
{
	Installed,
	DeviceNotCompatible,
	UserDeclinedInstallation,
	FatalError,
	EARServiceInstallRequestResult_MAX
};

enum EARServiceAvailability
{
	UnknownError,
	UnknownChecking,
	UnknownTimedOut,
	UnsupportedDeviceNotCapable,
	SupportedNotInstalled,
	SupportedVersionTooOld,
	SupportedInstalled,
	EARServiceAvailability_MAX
};

enum EARGeoTrackingAccuracy
{
	Undetermined,
	Low,
	Medium,
	High,
	EARGeoTrackingAccuracy_MAX
};

enum EARGeoTrackingStateReason
{
	None,
	NotAvailableAtLocation,
	NeedLocationPermissions,
	DevicePointedTooLow,
	WorldTrackingUnstable,
	WaitingForLocation,
	GeoDataNotLoaded,
	VisualLocalizationFailed,
	WaitingForAvailabilityCheck,
	EARGeoTrackingStateReason_MAX
};

enum EARGeoTrackingState
{
	Initializing,
	Localized,
	Localizing,
	NotAvailable,
	EARGeoTrackingState_MAX
};

enum EARSceneReconstruction
{
	None,
	MeshOnly,
	MeshWithClassification,
	EARSceneReconstruction_MAX
};

enum EARSessionTrackingFeature
{
	None,
	PoseDetection2D,
	PersonSegmentation,
	PersonSegmentationWithDepth,
	SceneDepth,
	SmoothedSceneDepth,
	EARSessionTrackingFeature_MAX
};

enum EARFaceTrackingUpdate
{
	CurvesAndGeo,
	CurvesOnly,
	EARFaceTrackingUpdate_MAX
};

enum EAREnvironmentCaptureProbeType
{
	None,
	Manual,
	Automatic,
	EAREnvironmentCaptureProbeType_MAX
};

enum EARFrameSyncMode
{
	SyncTickWithCameraImage,
	SyncTickWithoutCameraImage,
	EARFrameSyncMode_MAX
};

enum EARLightEstimationMode
{
	None,
	AmbientLightEstimate,
	DirectionalLightEstimate,
	EARLightEstimationMode_MAX
};

enum EARPlaneDetectionMode
{
	None,
	HorizontalPlaneDetection,
	VerticalPlaneDetection,
	EARPlaneDetectionMode_MAX
};

enum EARSessionType
{
	None,
	Orientation,
	World,
	Face,
	Image,
	ObjectScanning,
	PoseTracking,
	GeoTracking,
	EARSessionType_MAX
};

enum EARWorldAlignment
{
	Gravity,
	GravityAndHeading,
	Camera,
	EARWorldAlignment_MAX
};

enum EARDepthAccuracy
{
	Unkown,
	Approximate,
	Accurate,
	EARDepthAccuracy_MAX
};

enum EARDepthQuality
{
	Unkown,
	Low,
	High,
	EARDepthQuality_MAX
};

enum EARTextureType
{
	Unknown,
	CameraImage,
	CameraDepth,
	EnvironmentCapture,
	PersonSegmentationImage,
	PersonSegmentationDepth,
	SceneDepthMap,
	SceneDepthConfidenceMap,
	EARTextureType_MAX
};

enum EAREye
{
	LeftEye,
	RightEye,
	EAREye_MAX
};

enum EARFaceBlendShape
{
	EyeBlinkLeft,
	EyeLookDownLeft,
	EyeLookInLeft,
	EyeLookOutLeft,
	EyeLookUpLeft,
	EyeSquintLeft,
	EyeWideLeft,
	EyeBlinkRight,
	EyeLookDownRight,
	EyeLookInRight,
	EyeLookOutRight,
	EyeLookUpRight,
	EyeSquintRight,
	EyeWideRight,
	JawForward,
	JawLeft,
	JawRight,
	JawOpen,
	MouthClose,
	MouthFunnel,
	MouthPucker,
	MouthLeft,
	MouthRight,
	MouthSmileLeft,
	MouthSmileRight,
	MouthFrownLeft,
	MouthFrownRight,
	MouthDimpleLeft,
	MouthDimpleRight,
	MouthStretchLeft,
	MouthStretchRight,
	MouthRollLower,
	MouthRollUpper,
	MouthShrugLower,
	MouthShrugUpper,
	MouthPressLeft,
	MouthPressRight,
	MouthLowerDownLeft,
	MouthLowerDownRight,
	MouthUpperUpLeft,
	MouthUpperUpRight,
	BrowDownLeft,
	BrowDownRight,
	BrowInnerUp,
	BrowOuterUpLeft,
	BrowOuterUpRight,
	CheekPuff,
	CheekSquintLeft,
	CheekSquintRight,
	NoseSneerLeft,
	NoseSneerRight,
	TongueOut,
	HeadYaw,
	HeadPitch,
	HeadRoll,
	LeftEyeYaw,
	LeftEyePitch,
	LeftEyeRoll,
	RightEyeYaw,
	RightEyePitch,
	RightEyeRoll,
	MAX
};

enum EARFaceTrackingDirection
{
	FaceRelative,
	FaceMirrored,
	EARFaceTrackingDirection_MAX
};

enum EARCandidateImageOrientation
{
	Landscape,
	Portrait,
	EARCandidateImageOrientation_MAX
};

enum EARAltitudeSource
{
	Precise,
	Coarse,
	UserDefined,
	Unknown,
	EARAltitudeSource_MAX
};

enum EARJointTransformSpace
{
	Model,
	ParentJoint,
	EARJointTransformSpace_MAX
};

enum EARObjectClassification
{
	NotApplicable,
	Unknown,
	Wall,
	Ceiling,
	Floor,
	Table,
	Seat,
	Face,
	Image,
	World,
	SceneObject,
	HandMesh,
	Door,
	Window,
	EARObjectClassification_MAX
};

enum EARPlaneOrientation
{
	Horizontal,
	Vertical,
	Diagonal,
	EARPlaneOrientation_MAX
};

enum EARWorldMappingState
{
	NotAvailable,
	StillMappingNotRelocalizable,
	StillMappingRelocalizable,
	Mapped,
	EARWorldMappingState_MAX
};

enum EARSessionStatus
{
	NotStarted,
	Running,
	NotSupported,
	FatalError,
	PermissionNotGranted,
	UnsupportedConfiguration,
	Other,
	EARSessionStatus_MAX
};

enum EARTrackingQualityReason
{
	None,
	Initializing,
	Relocalizing,
	ExcessiveMotion,
	InsufficientFeatures,
	InsufficientLight,
	BadState,
	EARTrackingQualityReason_MAX
};

enum EARTrackingQuality
{
	NotTracking,
	OrientationOnly,
	OrientationAndPosition,
	EARTrackingQuality_MAX
};

enum EARLineTraceChannels
{
	None,
	FeaturePoint,
	GroundPlane,
	PlaneUsingExtent,
	PlaneUsingBoundaryPolygon,
	EARLineTraceChannels_MAX
};

enum EARCaptureType
{
	Camera,
	QRCode,
	SpatialMapping,
	SceneUnderstanding,
	EARCaptureType_MAX
};

enum EBattlePassLandingPageSpecialEntryType
{
	None,
	Subscription,
	CharacterCustomizer,
	SpecialCharacter,
	COUNT,
	EBattlePassLandingPageSpecialEntryType_MAX
};

enum EBattlePassTileAvailabilityStates
{
	Invalid,
	Available,
	Owned,
	Locked,
	BattlePassTileAvailabilityStates_MAX
};

enum EChaosClothTetherMode
{
	FastTetherFastLength,
	AccurateTetherFastLength,
	AccurateTetherAccurateLength,
	MaxChaosClothTetherMode,
	EChaosClothTetherMode_MAX
};

enum EChaosWeightMapTarget
{
	None,
	MaxDistance,
	BackstopDistance,
	BackstopRadius,
	AnimDriveStiffness,
	AnimDriveDamping,
	TetherStiffness,
	TetherScale,
	Drag,
	Lift,
	EdgeStiffness,
	BendingStiffness,
	AreaStiffness,
	EChaosWeightMapTarget_MAX
};

enum ELocationZToSpawnEnum
{
	ChaosNiagara_LocationZToSpawn_None,
	ChaosNiagara_LocationZToSpawn_Min,
	ChaosNiagara_LocationZToSpawn_Max,
	ChaosNiagara_LocationZToSpawn_MinMax,
	ChaosNiagara_Max
};

enum ELocationYToSpawnEnum
{
	ChaosNiagara_LocationYToSpawn_None,
	ChaosNiagara_LocationYToSpawn_Min,
	ChaosNiagara_LocationYToSpawn_Max,
	ChaosNiagara_LocationYToSpawn_MinMax,
	ChaosNiagara_Max
};

enum ELocationXToSpawnEnum
{
	ChaosNiagara_LocationXToSpawn_None,
	ChaosNiagara_LocationXToSpawn_Min,
	ChaosNiagara_LocationXToSpawn_Max,
	ChaosNiagara_LocationXToSpawn_MinMax,
	ChaosNiagara_Max
};

enum ELocationFilteringModeEnum
{
	ChaosNiagara_LocationFilteringMode_Inclusive,
	ChaosNiagara_LocationFilteringMode_Exclusive,
	ChaosNiagara_Max
};

enum EDataSourceTypeEnum
{
	ChaosNiagara_DataSourceType_Collision,
	ChaosNiagara_DataSourceType_Breaking,
	ChaosNiagara_DataSourceType_Trailing,
	ChaosNiagara_Max
};

enum EDebugTypeEnum
{
	ChaosNiagara_DebugType_NoDebug,
	ChaosNiagara_DebugType_ColorBySolver,
	ChaosNiagara_DebugType_ColorByParticleIndex,
	ChaosNiagara_Max
};

enum ERandomVelocityGenerationTypeEnum
{
	ChaosNiagara_RandomVelocityGenerationType_RandomDistribution,
	ChaosNiagara_RandomVelocityGenerationType_RandomDistributionWithStreamers,
	ChaosNiagara_RandomVelocityGenerationType_CollisionNormalBased,
	ChaosNiagara_Max
};

enum EDataSortTypeEnum
{
	ChaosNiagara_DataSortType_NoSorting,
	ChaosNiagara_DataSortType_RandomShuffle,
	ChaosNiagara_DataSortType_SortByMassMaxToMin,
	ChaosNiagara_DataSortType_SortByMassMinToMax,
	ChaosNiagara_Max
};

enum EClusterConnectionTypeEnum
{
	Chaos_PointImplicit,
	Chaos_DelaunayTriangulation,
	Chaos_MinimalSpanningSubsetDelaunayTriangulation,
	Chaos_PointImplicitAugmentedWithMinimalDelaunay,
	Chaos_None,
	Chaos_EClsuterCreationParameters_Max,
	Chaos_MAX
};

enum EClusterUnionMethod
{
	PointImplicit,
	DelaunayTriangulation,
	MinimalSpanningSubsetDelaunayTriangulation,
	PointImplicitAugmentedWithMinimalDelaunay,
	None,
	EClusterUnionMethod_MAX
};

enum EFieldPhysicsDefaultFields
{
	Field_RadialIntMask,
	Field_RadialFalloff,
	Field_UniformVector,
	Field_RadialVector,
	Field_RadialVectorFalloff,
	Field_EFieldPhysicsDefaultFields_Max,
	Field_MAX
};

enum EFieldOutputType
{
	Field_Output_Vector,
	Field_Output_Scalar,
	Field_Output_Integer,
	Field_Output_Max
};

enum EFieldIntegerType
{
	Integer_DynamicState,
	Integer_ActivateDisabled,
	Integer_CollisionGroup,
	Integer_PositionAnimated,
	Integer_PositionStatic,
	Integer_TargetMax,
	Integer_MAX
};

enum EFieldScalarType
{
	Scalar_ExternalClusterStrain,
	Scalar_Kill,
	Scalar_DisableThreshold,
	Scalar_SleepingThreshold,
	Scalar_InternalClusterStrain,
	Scalar_DynamicConstraint,
	Scalar_TargetMax,
	Scalar_MAX
};

enum EFieldVectorType
{
	Vector_LinearForce,
	Vector_LinearVelocity,
	Vector_AngularVelocity,
	Vector_AngularTorque,
	Vector_PositionTarget,
	Vector_InitialLinearVelocity,
	Vector_InitialAngularVelocity,
	Vector_TargetMax,
	Vector_MAX
};

enum EFieldPhysicsType
{
	Field_None,
	Field_DynamicState,
	Field_LinearForce,
	Field_ExternalClusterStrain,
	Field_Kill,
	Field_LinearVelocity,
	Field_AngularVelociy,
	Field_AngularTorque,
	Field_InternalClusterStrain,
	Field_DisableThreshold,
	Field_SleepingThreshold,
	Field_PositionStatic,
	Field_PositionAnimated,
	Field_PositionTarget,
	Field_DynamicConstraint,
	Field_CollisionGroup,
	Field_ActivateDisabled,
	Field_InitialLinearVelocity,
	Field_InitialAngularVelocity,
	Field_PhysicsType_Max
};

enum EFieldFalloffType
{
	Field_FallOff_None,
	Field_Falloff_Linear,
	Field_Falloff_Inverse,
	Field_Falloff_Squared,
	Field_Falloff_Logarithmic,
	Field_Falloff_Max
};

enum EFieldPositionType
{
	Field_Position_CenterOfMass,
	Field_Position_PivotPoint,
	Field_Position_Max
};

enum EFieldObjectType
{
	Field_Object_Rigid,
	Field_Object_Cloth,
	Field_Object_Destruction,
	Field_Object_Character,
	Field_Object_All,
	Field_Object_Max
};

enum EFieldFilterType
{
	Field_Filter_Dynamic,
	Field_Filter_Kinematic,
	Field_Filter_Static,
	Field_Filter_All,
	Field_Filter_Sleeping,
	Field_Filter_Disabled,
	Field_Filter_Max
};

enum EFieldResolutionType
{
	Field_Resolution_Minimal,
	Field_Resolution_DisabledParents,
	Field_Resolution_Maximum,
	Field_Resolution_Max
};

enum EFieldCullingOperationType
{
	Field_Culling_Inside,
	Field_Culling_Outside,
	Field_Culling_Operation_Max,
	Field_Culling_MAX
};

enum EFieldOperationType
{
	Field_Multiply,
	Field_Divide,
	Field_Add,
	Field_Substract,
	Field_Operation_Max
};

enum EWaveFunctionType
{
	Field_Wave_Cosine,
	Field_Wave_Gaussian,
	Field_Wave_Falloff,
	Field_Wave_Decay,
	Field_Wave_Max
};

enum ESetMaskConditionType
{
	Field_Set_Always,
	Field_Set_IFF_NOT_Interior,
	Field_Set_IFF_NOT_Exterior,
	Field_MaskCondition_Max
};

enum EEmissionPatternTypeEnum
{
	Chaos_Emission_Pattern_First_Frame,
	Chaos_Emission_Pattern_On_Demand,
	Chaos_Max
};

enum EInitialVelocityTypeEnum
{
	Chaos_Initial_Velocity_User_Defined,
	Chaos_Initial_Velocity_None,
	Chaos_Max
};

enum EGeometryCollectionPhysicsTypeEnum
{
	Chaos_AngularVelocity,
	Chaos_DynamicState,
	Chaos_LinearVelocity,
	Chaos_InitialAngularVelocity,
	Chaos_InitialLinearVelocity,
	Chaos_CollisionGroup,
	Chaos_LinearForce,
	Chaos_AngularTorque,
	Chaos_Max
};

enum EObjectStateTypeEnum
{
	Chaos_NONE,
	Chaos_Object_Sleeping,
	Chaos_Object_Kinematic,
	Chaos_Object_Static,
	Chaos_Object_Dynamic,
	Chaos_Object_UserDefined,
	Chaos_Max
};

enum EImplicitTypeEnum
{
	Chaos_Implicit_Box,
	Chaos_Implicit_Sphere,
	Chaos_Implicit_Capsule,
	Chaos_Implicit_LevelSet,
	Chaos_Implicit_None,
	Chaos_Max
};

enum ECollisionTypeEnum
{
	Chaos_Volumetric,
	Chaos_Surface_Volumetric,
	Chaos_Max
};

enum EChaosBufferMode
{
	Double,
	Triple,
	Num,
	Invalid,
	EChaosBufferMode_MAX
};

enum EChaosThreadingMode
{
	DedicatedThread,
	TaskGraph,
	SingleThread,
	Num,
	Invalid,
	EChaosThreadingMode_MAX
};

enum EChaosSolverTickMode
{
	Fixed,
	Variable,
	VariableCapped,
	VariableCappedWithTarget,
	EChaosSolverTickMode_MAX
};

enum EGeometryCollectionCacheType
{
	None,
	Record,
	Play,
	RecordAndPlay,
	EGeometryCollectionCacheType_MAX
};

enum ECameraFocusMethod
{
	DoNotOverride,
	Manual,
	Tracking,
	Disable,
	MAX
};

enum EClothMassMode
{
	UniformMass,
	TotalMass,
	Density,
	MaxClothMassMode,
	EClothMassMode_MAX
};

enum EClothingWindMethod_Legacy
{
	Legacy,
	Accurate,
	EClothingWindMethod_MAX
};

enum EWeightMapTargetCommon
{
	None,
	MaxDistance,
	BackstopDistance,
	BackstopRadius,
	AnimDriveStiffness,
	AnimDriveDamping_DEPRECATED,
	EWeightMapTargetCommon_MAX
};

enum EClothingWindMethodNv
{
	Legacy,
	Accurate,
	EClothingWindMethodNv_MAX
};

enum EConversationTaskResultType
{
	Invalid,
	AbortConversation,
	AdvanceConversation,
	AdvanceConversationWithChoice,
	PauseConversationAndSendClientChoices,
	ReturnToLastClientChoice,
	ReturnToCurrentClientChoice,
	ReturnToConversationStart,
	EConversationTaskResultType_MAX
};

enum EConversationRequirementResult
{
	Passed,
	FailedButVisible,
	FailedAndHidden,
	EConversationRequirementResult_MAX
};

enum EConversationChoiceType
{
	ServerOnly,
	UserChoiceAvailable,
	UserChoiceUnavailable,
	EConversationChoiceType_MAX
};

enum ECommonInputType
{
	MouseAndKeyboard,
	Gamepad,
	Touch,
	Count,
	ECommonInputType_MAX
};

enum EOperation
{
	Intro,
	Outro,
	Push,
	Pop,
	Invalid,
	EOperation_MAX
};

enum ECommonGamepadType
{
	XboxOneController,
	PS4Controller,
	SwitchController,
	GenericController,
	XboxSeriesXController,
	PS5Controller,
	Count,
	ECommonGamepadType_MAX
};

enum ECommonPlatformType
{
	PC,
	Mac,
	PS4,
	XBox,
	IOS,
	Android,
	Switch,
	XSX,
	PS5,
	Count,
	ECommonPlatformType_MAX
};

enum ECommonNumericType
{
	Number,
	Percentage,
	Seconds,
	Distance,
	ECommonNumericType_MAX
};

enum ECommonInputMode
{
	Menu,
	Game,
	All,
	MAX
};

enum ERichTextInlineIconDisplayMode
{
	IconOnly,
	TextOnly,
	IconAndText,
	MAX
};

enum EInputActionState
{
	Enabled,
	Disabled,
	Hidden,
	HiddenAndDisabled,
	EInputActionState_MAX
};

enum ETransitionCurve
{
	Linear,
	QuadIn,
	QuadOut,
	QuadInOut,
	CubicIn,
	CubicOut,
	CubicInOut,
	ETransitionCurve_MAX
};

enum ECommonSwitcherTransition
{
	FadeOnly,
	Horizontal,
	Vertical,
	Zoom,
	ECommonSwitcherTransition_MAX
};

enum EControlRigComponentMapDirection
{
	Input,
	Output,
	EControlRigComponentMapDirection_MAX
};

enum EControlRigComponentSpace
{
	WorldSpace,
	ActorSpace,
	ComponentSpace,
	RigSpace,
	LocalSpace,
	Max
};

enum ERigExecutionType
{
	Runtime,
	Editing,
	Max
};

enum EBoneGetterSetterMode
{
	LocalSpace,
	GlobalSpace,
	Max
};

enum ETransformGetterType
{
	Initial,
	Current,
	Max
};

enum EControlRigClampSpatialMode
{
	Plane,
	Cylinder,
	Sphere,
	EControlRigClampSpatialMode_MAX
};

enum ETransformSpaceMode
{
	LocalSpace,
	GlobalSpace,
	BaseSpace,
	BaseJoint,
	Max
};

enum EControlRigDrawSettings
{
	Points,
	Lines,
	LineStrip,
	DynamicMesh,
	EControlRigDrawSettings_MAX
};

enum EControlRigDrawHierarchyMode
{
	Axes,
	Max
};

enum EControlRigAnimEasingType
{
	Linear,
	QuadraticEaseIn,
	QuadraticEaseOut,
	QuadraticEaseInOut,
	CubicEaseIn,
	CubicEaseOut,
	CubicEaseInOut,
	QuarticEaseIn,
	QuarticEaseOut,
	QuarticEaseInOut,
	QuinticEaseIn,
	QuinticEaseOut,
	QuinticEaseInOut,
	SineEaseIn,
	SineEaseOut,
	SineEaseInOut,
	CircularEaseIn,
	CircularEaseOut,
	CircularEaseInOut,
	ExponentialEaseIn,
	ExponentialEaseOut,
	ExponentialEaseInOut,
	ElasticEaseIn,
	ElasticEaseOut,
	ElasticEaseInOut,
	BackEaseIn,
	BackEaseOut,
	BackEaseInOut,
	BounceEaseIn,
	BounceEaseOut,
	BounceEaseInOut,
	EControlRigAnimEasingType_MAX
};

enum EControlRigRotationOrder
{
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX,
	EControlRigRotationOrder_MAX
};

enum ECRSimPointIntegrateType
{
	Verlet,
	SemiExplicitEuler,
	ECRSimPointIntegrateType_MAX
};

enum ECRSimConstraintType
{
	Distance,
	DistanceFromA,
	DistanceFromB,
	Plane,
	ECRSimConstraintType_MAX
};

enum ECRSimPointForceType
{
	Direction,
	ECRSimPointForceType_MAX
};

enum ECRSimSoftCollisionType
{
	Plane,
	Sphere,
	Cone,
	ECRSimSoftCollisionType_MAX
};

enum EControlRigFKRigExecuteMode
{
	Replace,
	Additive,
	Max
};

enum ERigBoneType
{
	Imported,
	User,
	ERigBoneType_MAX
};

enum ERigControlAxis
{
	X,
	Y,
	Z,
	ERigControlAxis_MAX
};

enum ERigControlValueType
{
	Initial,
	Current,
	Minimum,
	Maximum,
	ERigControlValueType_MAX
};

enum ERigControlType
{
	Bool,
	Float,
	Integer,
	Vector2D,
	Position,
	Scale,
	Rotator,
	Transform,
	TransformNoScale,
	EulerTransform,
	ERigControlType_MAX
};

enum ERigHierarchyImportMode
{
	Append,
	Replace,
	ReplaceLocalTransform,
	ReplaceGlobalTransform,
	Max
};

enum EControlRigSetKey
{
	DoNotCare,
	Always,
	Never,
	EControlRigSetKey_MAX
};

enum ERigEvent
{
	None,
	RequestAutoKey,
	Max
};

enum ERigElementType
{
	None,
	Bone,
	Space,
	Control,
	Curve,
	All,
	ERigElementType_MAX
};

enum ERigSpaceType
{
	Global,
	Bone,
	Control,
	Space,
	ERigSpaceType_MAX
};

enum EAimMode
{
	AimAtTarget,
	OrientToTarget,
	MAX
};

enum EApplyTransformMode
{
	Override,
	Additive,
	Max
};

enum ERigUnitDebugPointMode
{
	Point,
	Vector,
	Max
};

enum ERigUnitDebugTransformMode
{
	Point,
	Axes,
	Box,
	Max
};

enum EControlRigCurveAlignment
{
	Front,
	Stretched,
	EControlRigCurveAlignment_MAX
};

enum EControlRigVectorKind
{
	Direction,
	Location,
	EControlRigVectorKind_MAX
};

enum ERBFVectorDistanceType
{
	Euclidean,
	Manhattan,
	ArcLength,
	ERBFVectorDistanceType_MAX
};

enum ERBFQuatDistanceType
{
	Euclidean,
	ArcLength,
	SwingAngle,
	TwistAngle,
	ERBFQuatDistanceType_MAX
};

enum ERBFKernelType
{
	Gaussian,
	Exponential,
	Linear,
	Cubic,
	Quintic,
	ERBFKernelType_MAX
};

enum EControlRigModifyBoneMode
{
	OverrideLocal,
	OverrideGlobal,
	AdditiveLocal,
	AdditiveGlobal,
	Max
};

enum ERigUnitVisualDebugPointMode
{
	Point,
	Vector,
	Max
};

enum EControlRigState
{
	Init,
	Update,
	Invalid,
	EControlRigState_MAX
};

enum ECoreOnlineDummy
{
	Dummy,
	ECoreOnlineDummy_MAX
};

enum EInterpCurveMode
{
	CIM_Linear,
	CIM_CurveAuto,
	CIM_Constant,
	CIM_CurveUser,
	CIM_CurveBreak,
	CIM_CurveAutoClamped,
	CIM_MAX
};

enum ERangeBoundTypes
{
	Exclusive,
	Inclusive,
	Open,
	ERangeBoundTypes_MAX
};

enum ELocalizedTextSourceCategory
{
	Game,
	Engine,
	Editor,
	ELocalizedTextSourceCategory_MAX
};

enum EAutomationEventType
{
	Info,
	Warning,
	Error,
	EAutomationEventType_MAX
};

enum EMouseCursor
{
	None,
	Default,
	TextEditBeam,
	ResizeLeftRight,
	ResizeUpDown,
	ResizeSouthEast,
	ResizeSouthWest,
	CardinalCross,
	Crosshairs,
	Hand,
	GrabHand,
	GrabHandClosed,
	SlashedCircle,
	EyeDropper,
	EMouseCursor_MAX
};

enum ELifetimeCondition
{
	COND_None,
	COND_InitialOnly,
	COND_OwnerOnly,
	COND_SkipOwner,
	COND_SimulatedOnly,
	COND_AutonomousOnly,
	COND_SimulatedOrPhysics,
	COND_InitialOrOwner,
	COND_Custom,
	COND_ReplayOrOwner,
	COND_ReplayOnly,
	COND_SimulatedOnlyNoReplay,
	COND_SimulatedOrPhysicsNoReplay,
	COND_SkipReplay,
	COND_Never,
	COND_Max
};

enum EDataValidationResult
{
	Invalid,
	Valid,
	NotValidated,
	EDataValidationResult_MAX
};

enum EAppMsgType
{
	Ok,
	YesNo,
	OkCancel,
	YesNoCancel,
	CancelRetryContinue,
	YesNoYesAllNoAll,
	YesNoYesAllNoAllCancel,
	YesNoYesAll,
	EAppMsgType_MAX
};

enum EAppReturnType
{
	No,
	Yes,
	YesAll,
	NoAll,
	Cancel,
	Ok,
	Retry,
	Continue,
	EAppReturnType_MAX
};

enum EPropertyAccessChangeNotifyMode
{
	Default,
	Never,
	Always,
	EPropertyAccessChangeNotifyMode_MAX
};

enum EUnit
{
	Micrometers,
	Millimeters,
	Centimeters,
	Meters,
	Kilometers,
	Inches,
	Feet,
	Yards,
	Miles,
	Lightyears,
	Degrees,
	Radians,
	MetersPerSecond,
	KilometersPerHour,
	MilesPerHour,
	Celsius,
	Farenheit,
	Kelvin,
	Micrograms,
	Milligrams,
	Grams,
	Kilograms,
	MetricTons,
	Ounces,
	Pounds,
	Stones,
	Newtons,
	PoundsForce,
	KilogramsForce,
	Hertz,
	Kilohertz,
	Megahertz,
	Gigahertz,
	RevolutionsPerMinute,
	Bytes,
	Kilobytes,
	Megabytes,
	Gigabytes,
	Terabytes,
	Lumens,
	Milliseconds,
	Seconds,
	Minutes,
	Hours,
	Days,
	Months,
	Years,
	Multiplier,
	Percentage,
	Unspecified,
	EUnit_MAX
};

enum EPixelFormat
{
	PF_Unknown,
	PF_A32B32G32R32F,
	PF_B8G8R8A8,
	PF_G8,
	PF_G16,
	PF_DXT1,
	PF_DXT3,
	PF_DXT5,
	PF_UYVY,
	PF_FloatRGB,
	PF_FloatRGBA,
	PF_DepthStencil,
	PF_ShadowDepth,
	PF_R32_FLOAT,
	PF_G16R16,
	PF_G16R16F,
	PF_G16R16F_FILTER,
	PF_G32R32F,
	PF_A2B10G10R10,
	PF_A16B16G16R16,
	PF_D24,
	PF_R16F,
	PF_R16F_FILTER,
	PF_BC5,
	PF_V8U8,
	PF_A1,
	PF_FloatR11G11B10,
	PF_A8,
	PF_R32_UINT,
	PF_R32_SINT,
	PF_PVRTC2,
	PF_PVRTC4,
	PF_R16_UINT,
	PF_R16_SINT,
	PF_R16G16B16A16_UINT,
	PF_R16G16B16A16_SINT,
	PF_R5G6B5_UNORM,
	PF_R8G8B8A8,
	PF_A8R8G8B8,
	PF_BC4,
	PF_R8G8,
	PF_ATC_RGB,
	PF_ATC_RGBA_E,
	PF_ATC_RGBA_I,
	PF_X24_G8,
	PF_ETC1,
	PF_ETC2_RGB,
	PF_ETC2_RGBA,
	PF_R32G32B32A32_UINT,
	PF_R16G16_UINT,
	PF_ASTC_4x4,
	PF_ASTC_6x6,
	PF_ASTC_8x8,
	PF_ASTC_10x10,
	PF_ASTC_12x12,
	PF_BC6H,
	PF_BC7,
	PF_R8_UINT,
	PF_L8,
	PF_XGXR8,
	PF_R8G8B8A8_UINT,
	PF_R8G8B8A8_SNORM,
	PF_R16G16B16A16_UNORM,
	PF_R16G16B16A16_SNORM,
	PF_PLATFORM_HDR_0,
	PF_PLATFORM_HDR_1,
	PF_PLATFORM_HDR_2,
	PF_NV12,
	PF_R32G32_UINT,
	PF_ETC2_R11_EAC,
	PF_ETC2_RG11_EAC,
	PF_MAX
};

enum EAxis
{
	None,
	X,
	Y,
	Z,
	EAxis_MAX
};

enum ELogTimes
{
	None,
	UTC,
	SinceGStartTime,
	Local,
	ELogTimes_MAX
};

enum ESearchDir
{
	FromStart,
	FromEnd,
	ESearchDir_MAX
};

enum ESearchCase
{
	CaseSensitive,
	IgnoreCase,
	ESearchCase_MAX
};

enum ECraftingObjectState
{
	Invalid,
	Idle,
	Crafting,
	Ready,
	OverCrafting,
	Resetting,
	TotalStates,
	ECraftingObjectState_MAX
};

enum ECraftingIngredientReqError
{
	None,
	NoItem,
	NotEnough,
	ECraftingIngredientReqError_MAX
};

enum ECurieHandlerPriority
{
	Priority_1,
	Priority_2,
	Priority_3,
	Priority_4,
	Priority_5,
	Priority_6,
	Priority_7,
	Priority_8,
	Priority_9,
	Priority_10,
	Priority_Default,
	Priority_MAX
};

enum ECurieHandlerBehavior
{
	Handler_Add,
	Handler_Replace,
	Handler_MAX
};

enum ECurieEntityType
{
	Invalid,
	Material,
	Element,
	ECurieEntityType_MAX
};

enum EDataAssetDirectoryUpdateStatus
{
	Failed,
	Success,
	SuccessNoChange,
	EDataAssetDirectoryUpdateStatus_MAX
};

enum EDataAssetDirectoryTestEnum
{
	A,
	B,
	C,
	D,
	EDataAssetDirectoryTestEnum_MAX
};

enum EDataRegistryAcquireStatus
{
	NotStarted,
	WaitingForInitialAcquire,
	InitialAcquireFinished,
	WaitingForResources,
	AcquireFinished,
	AcquireError,
	DoesNotExist,
	EDataRegistryAcquireStatus_MAX
};

enum EMetaDataRegistrySourceAssetUsage
{
	NoAssets,
	SearchAssets,
	RegisterAssets,
	SearchAndRegisterAssets,
	EMetaDataRegistrySourceAssetUsage_MAX
};

enum EDataRegistrySubsystemGetItemResult
{
	Found,
	NotFound,
	EDataRegistrySubsystemGetItemResult_MAX
};

enum EDataRegistryAvailability
{
	DoesNotExist,
	Unknown,
	Remote,
	OnDisk,
	LocalAsset,
	PreCached,
	EDataRegistryAvailability_MAX
};

enum EUDLSSMode
{
	Off,
	UltraPerformance,
	Performance,
	Balanced,
	Quality,
	UltraQuality,
	UDLSSMode_MAX
};

enum EUDLSSSupport
{
	Supported,
	NotSupported,
	NotSupportedIncompatibleHardware,
	NotSupportedDriverOutOfDate,
	NotSupportedOperatingSystemOutOfDate,
	UDLSSSupport_MAX
};

enum EDLSSSettingOverride
{
	Enabled,
	Disabled,
	UseProjectSettings,
	EDLSSSettingOverride_MAX
};

enum EDynamicHUDComparison
{
	Equal,
	LessOrEqual,
	GreaterOrEqual,
	EDynamicHUDComparison_MAX
};

enum EDynamicHUDOperator
{
	Addition,
	Substraction,
	EDynamicHUDOperator_MAX
};

enum EDynamicHUDSide
{
	Top,
	Bottom,
	Left,
	Right,
	EDynamicHUDSide_MAX
};

enum EDynamicHUDAnchor
{
	TopLeft,
	TopCenter,
	TopRight,
	CenterLeft,
	CenterCenter,
	CenterRight,
	BottomLeft,
	BottomCenter,
	BottomRight,
	EDynamicHUDAnchor_MAX
};

enum EDynamicHUDStrength
{
	Weak,
	Medium,
	Strong,
	Required,
	EDynamicHUDStrength_MAX
};

enum EDynamicHUDZOrder
{
	Back,
	Middle,
	Front,
	Custom,
	EDynamicHUDZOrder_MAX
};

enum ETriangleTessellationMode
{
	ThreeTriangles,
	FourTriangles,
	ETriangleTessellationMode_MAX
};

enum EInsetPolygonsMode
{
	All,
	CenterPolygonOnly,
	SidePolygonsOnly,
	EInsetPolygonsMode_MAX
};

enum EPolygonEdgeHardness
{
	NewEdgesSoft,
	NewEdgesHard,
	AllEdgesSoft,
	AllEdgesHard,
	EPolygonEdgeHardness_MAX
};

enum EMeshElementAttributeType
{
	None,
	FVector4,
	FVector,
	FVector2D,
	Float,
	Int,
	Bool,
	FName,
	EMeshElementAttributeType_MAX
};

enum EMeshTopologyChange
{
	NoTopologyChange,
	TopologyChange,
	EMeshTopologyChange_MAX
};

enum EMeshModificationType
{
	FirstInterim,
	Interim,
	Final,
	EMeshModificationType_MAX
};

enum ESubLevelStripMode
{
	ExactClass,
	IsChildOf,
	ESubLevelStripMode_MAX
};

enum EFourPlayerSplitScreenType
{
	Grid,
	Vertical,
	Horizontal,
	EFourPlayerSplitScreenType_MAX
};

enum EThreePlayerSplitScreenType
{
	FavorTop,
	FavorBottom,
	Vertical,
	Horizontal,
	EThreePlayerSplitScreenType_MAX
};

enum ETwoPlayerSplitScreenType
{
	Horizontal,
	Vertical,
	ETwoPlayerSplitScreenType_MAX
};

enum ETextGender
{
	Masculine,
	Feminine,
	Neuter,
	ETextGender_MAX
};

enum EFormatArgumentType
{
	Int,
	UInt,
	Float,
	Double,
	Text,
	Gender,
	EFormatArgumentType_MAX
};

enum EEndPlayReason
{
	Destroyed,
	LevelTransition,
	EndPlayInEditor,
	RemovedFromWorld,
	Quit,
	EEndPlayReason_MAX
};

enum ETickingGroup
{
	TG_PrePhysics,
	TG_StartPhysics,
	TG_DuringPhysics,
	TG_EndPhysics,
	TG_PostPhysics,
	TG_PostUpdateWork,
	TG_LastDemotable,
	TG_NewlySpawned,
	TG_MAX
};

enum EComponentCreationMethod
{
	Native,
	SimpleConstructionScript,
	UserConstructionScript,
	Instance,
	EComponentCreationMethod_MAX
};

enum ETemperatureSeverityType
{
	Unknown,
	Good,
	Bad,
	Serious,
	Critical,
	NumSeverities,
	ETemperatureSeverityType_MAX
};

enum EQuartzCommandQuantization
{
	Bar,
	Beat,
	ThirtySecondNote,
	SixteenthNote,
	EighthNote,
	QuarterNote,
	HalfNote,
	WholeNote,
	DottedSixteenthNote,
	DottedEighthNote,
	DottedQuarterNote,
	DottedHalfNote,
	DottedWholeNote,
	SixteenthNoteTriplet,
	EighthNoteTriplet,
	QuarterNoteTriplet,
	HalfNoteTriplet,
	Tick,
	Count,
	None,
	EQuartzCommandQuantization_MAX
};

enum EQuartzCommandDelegateSubType
{
	CommandOnFailedToQueue,
	CommandOnQueued,
	CommandOnCanceled,
	CommandOnAboutToStart,
	CommandOnStarted,
	Count,
	EQuartzCommandDelegateSubType_MAX
};

enum EAudioComponentPlayState
{
	Playing,
	Stopped,
	Paused,
	FadingIn,
	FadingOut,
	Count,
	EAudioComponentPlayState_MAX
};

enum EPlaneConstraintAxisSetting
{
	Custom,
	X,
	Y,
	Z,
	UseGlobalPhysicsSetting,
	EPlaneConstraintAxisSetting_MAX
};

enum EInterpToBehaviourType
{
	OneShot,
	OneShot_Reverse,
	Loop_Reset,
	PingPong,
	EInterpToBehaviourType_MAX
};

enum ETeleportType
{
	None,
	TeleportPhysics,
	ResetPhysics,
	ETeleportType_MAX
};

enum EPlatformInterfaceDataType
{
	PIDT_None,
	PIDT_Int,
	PIDT_Float,
	PIDT_String,
	PIDT_Object,
	PIDT_Custom,
	PIDT_MAX
};

enum EMovementMode
{
	MOVE_None,
	MOVE_Walking,
	MOVE_NavWalking,
	MOVE_Falling,
	MOVE_Swimming,
	MOVE_Flying,
	MOVE_Custom,
	MOVE_MAX
};

enum ENetworkFailure
{
	NetDriverAlreadyExists,
	NetDriverCreateFailure,
	NetDriverListenFailure,
	ConnectionLost,
	ConnectionTimeout,
	FailureReceived,
	OutdatedClient,
	OutdatedServer,
	PendingConnectionFailure,
	NetGuidMismatch,
	NetChecksumMismatch,
	ENetworkFailure_MAX
};

enum ETravelFailure
{
	NoLevel,
	LoadMapFailure,
	InvalidURL,
	PackageMissing,
	PackageVersion,
	NoDownload,
	TravelFailure,
	CheatCommands,
	PendingNetGameCreateFailure,
	CloudSaveFailure,
	ServerTravelFailure,
	ClientTravelFailure,
	ETravelFailure_MAX
};

enum EScreenOrientation
{
	Unknown,
	Portrait,
	PortraitUpsideDown,
	LandscapeLeft,
	LandscapeRight,
	FaceUp,
	FaceDown,
	EScreenOrientation_MAX
};

enum EApplicationState
{
	Unknown,
	Inactive,
	Background,
	Active,
	EApplicationState_MAX
};

enum EObjectTypeQuery
{
	ObjectTypeQuery1,
	ObjectTypeQuery2,
	ObjectTypeQuery3,
	ObjectTypeQuery4,
	ObjectTypeQuery5,
	ObjectTypeQuery6,
	ObjectTypeQuery7,
	ObjectTypeQuery8,
	ObjectTypeQuery9,
	ObjectTypeQuery10,
	ObjectTypeQuery11,
	ObjectTypeQuery12,
	ObjectTypeQuery13,
	ObjectTypeQuery14,
	ObjectTypeQuery15,
	ObjectTypeQuery16,
	ObjectTypeQuery17,
	ObjectTypeQuery18,
	ObjectTypeQuery19,
	ObjectTypeQuery20,
	ObjectTypeQuery21,
	ObjectTypeQuery22,
	ObjectTypeQuery23,
	ObjectTypeQuery24,
	ObjectTypeQuery25,
	ObjectTypeQuery26,
	ObjectTypeQuery27,
	ObjectTypeQuery28,
	ObjectTypeQuery29,
	ObjectTypeQuery30,
	ObjectTypeQuery31,
	ObjectTypeQuery32,
	ObjectTypeQuery_MAX,
	EObjectTypeQuery_MAX
};

enum EDrawDebugTrace
{
	None,
	ForOneFrame,
	ForDuration,
	Persistent,
	EDrawDebugTrace_MAX
};

enum ETraceTypeQuery
{
	TraceTypeQuery1,
	TraceTypeQuery2,
	TraceTypeQuery3,
	TraceTypeQuery4,
	TraceTypeQuery5,
	TraceTypeQuery6,
	TraceTypeQuery7,
	TraceTypeQuery8,
	TraceTypeQuery9,
	TraceTypeQuery10,
	TraceTypeQuery11,
	TraceTypeQuery12,
	TraceTypeQuery13,
	TraceTypeQuery14,
	TraceTypeQuery15,
	TraceTypeQuery16,
	TraceTypeQuery17,
	TraceTypeQuery18,
	TraceTypeQuery19,
	TraceTypeQuery20,
	TraceTypeQuery21,
	TraceTypeQuery22,
	TraceTypeQuery23,
	TraceTypeQuery24,
	TraceTypeQuery25,
	TraceTypeQuery26,
	TraceTypeQuery27,
	TraceTypeQuery28,
	TraceTypeQuery29,
	TraceTypeQuery30,
	TraceTypeQuery31,
	TraceTypeQuery32,
	TraceTypeQuery_MAX,
	ETraceTypeQuery_MAX
};

enum EMoveComponentAction
{
	Move,
	Stop,
	Return,
	EMoveComponentAction_MAX
};

enum EQuitPreference
{
	Quit,
	Background,
	EQuitPreference_MAX
};

enum EMouseLockMode
{
	DoNotLock,
	LockOnCapture,
	LockAlways,
	LockInFullscreen,
	EMouseLockMode_MAX
};

enum EWindowTitleBarMode
{
	Overlay,
	VerticalBox,
	EWindowTitleBarMode_MAX
};

enum ERoundingMode
{
	HalfToEven,
	HalfFromZero,
	HalfToZero,
	FromZero,
	ToZero,
	ToNegativeInfinity,
	ToPositiveInfinity,
	ERoundingMode_MAX
};

enum EInputEvent
{
	IE_Pressed,
	IE_Released,
	IE_Repeat,
	IE_DoubleClick,
	IE_Axis,
	IE_MAX
};

enum ENetRole
{
	ROLE_None,
	ROLE_SimulatedProxy,
	ROLE_AutonomousProxy,
	ROLE_Authority,
	ROLE_MAX
};

enum EAttachLocation
{
	KeepRelativeOffset,
	KeepWorldPosition,
	SnapToTarget,
	SnapToTargetIncludingScale,
	EAttachLocation_MAX
};

enum EAttachmentRule
{
	KeepRelative,
	KeepWorld,
	SnapToTarget,
	EAttachmentRule_MAX
};

enum EDetachmentRule
{
	KeepRelative,
	KeepWorld,
	EDetachmentRule_MAX
};

enum ENetDormancy
{
	DORM_Never,
	DORM_Awake,
	DORM_DormantAll,
	DORM_DormantPartial,
	DORM_Initial,
	DORM_MAX
};

enum EAutoReceiveInput
{
	Disabled,
	Player0,
	Player1,
	Player2,
	Player3,
	Player4,
	Player5,
	Player6,
	Player7,
	EAutoReceiveInput_MAX
};

enum ESpawnActorCollisionHandlingMethod
{
	Undefined,
	AlwaysSpawn,
	AdjustIfPossibleButAlwaysSpawn,
	AdjustIfPossibleButDontSpawnIfColliding,
	DontSpawnIfColliding,
	ESpawnActorCollisionHandlingMethod_MAX
};

enum ERotatorQuantization
{
	ByteComponents,
	ShortComponents,
	ERotatorQuantization_MAX
};

enum EVectorQuantization
{
	RoundWholeNumber,
	RoundOneDecimal,
	RoundTwoDecimals,
	EVectorQuantization_MAX
};

enum EActorUpdateOverlapsMethod
{
	UseConfigDefault,
	AlwaysUpdate,
	OnlyUpdateMovable,
	NeverUpdate,
	EActorUpdateOverlapsMethod_MAX
};

enum ECollisionEnabled
{
	NoCollision,
	QueryOnly,
	PhysicsOnly,
	QueryAndPhysics,
	ECollisionEnabled_MAX
};

enum ECameraShakePlaySpace
{
	CameraLocal,
	World,
	UserDefined,
	ECameraShakePlaySpace_MAX
};

enum EViewTargetBlendFunction
{
	VTBlend_Linear,
	VTBlend_Cubic,
	VTBlend_EaseIn,
	VTBlend_EaseOut,
	VTBlend_EaseInOut,
	VTBlend_PreBlended,
	VTBlend_MAX
};

enum ETravelType
{
	TRAVEL_Absolute,
	TRAVEL_Partial,
	TRAVEL_Relative,
	TRAVEL_MAX
};

enum ECollisionChannel
{
	ECC_WorldStatic,
	ECC_WorldDynamic,
	ECC_Pawn,
	ECC_Visibility,
	ECC_Camera,
	ECC_PhysicsBody,
	ECC_Vehicle,
	ECC_Destructible,
	ECC_EngineTraceChannel1,
	ECC_EngineTraceChannel2,
	ECC_EngineTraceChannel3,
	ECC_EngineTraceChannel4,
	ECC_EngineTraceChannel5,
	ECC_EngineTraceChannel6,
	ECC_GameTraceChannel1,
	ECC_GameTraceChannel2,
	ECC_GameTraceChannel3,
	ECC_GameTraceChannel4,
	ECC_GameTraceChannel5,
	ECC_GameTraceChannel6,
	ECC_GameTraceChannel7,
	ECC_GameTraceChannel8,
	ECC_GameTraceChannel9,
	ECC_GameTraceChannel10,
	ECC_GameTraceChannel11,
	ECC_GameTraceChannel12,
	ECC_GameTraceChannel13,
	ECC_GameTraceChannel14,
	ECC_GameTraceChannel15,
	ECC_GameTraceChannel16,
	ECC_GameTraceChannel17,
	ECC_GameTraceChannel18,
	ECC_OverlapAll_Deprecated,
	ECC_MAX
};

enum EControllerAnalogStick
{
	CAS_LeftStick,
	CAS_RightStick,
	CAS_MAX
};

enum EDynamicForceFeedbackAction
{
	Start,
	Update,
	Stop,
	EDynamicForceFeedbackAction_MAX
};

enum ERelativeTransformSpace
{
	RTS_World,
	RTS_Actor,
	RTS_Component,
	RTS_ParentBoneSpace,
	RTS_MAX
};

enum EComponentMobility
{
	Static,
	Stationary,
	Movable,
	EComponentMobility_MAX
};

enum EDetailMode
{
	DM_Low,
	DM_Medium,
	DM_High,
	DM_MAX
};

enum ECollisionResponse
{
	ECR_Ignore,
	ECR_Overlap,
	ECR_Block,
	ECR_MAX
};

enum EWalkableSlopeBehavior
{
	WalkableSlope_Default,
	WalkableSlope_Increase,
	WalkableSlope_Decrease,
	WalkableSlope_Unwalkable,
	WalkableSlope_Max
};

enum EDOFMode
{
	Default,
	SixDOF,
	YZPlane,
	XZPlane,
	XYPlane,
	CustomPlane,
	None,
	EDOFMode_MAX
};

enum ERendererStencilMask
{
	ERSM_Default,
	ERSM_255,
	ERSM_1,
	ERSM_2,
	ERSM_4,
	ERSM_8,
	ERSM_16,
	ERSM_32,
	ERSM_64,
	ERSM_128,
	ERSM_MAX
};

enum ERuntimeVirtualTextureMainPassType
{
	Never,
	Exclusive,
	Always,
	ERuntimeVirtualTextureMainPassType_MAX
};

enum ECanBeCharacterBase
{
	ECB_No,
	ECB_Yes,
	ECB_Owner,
	ECB_MAX
};

enum EHasCustomNavigableGeometry
{
	No,
	Yes,
	EvenIfNotCollidable,
	DontExport,
	EHasCustomNavigableGeometry_MAX
};

enum ELightmapType
{
	Default,
	ForceSurface,
	ForceVolumetric,
	ELightmapType_MAX
};

enum EIndirectLightingCacheQuality
{
	ILCQ_Off,
	ILCQ_Point,
	ILCQ_Volume,
	ILCQ_MAX
};

enum ESceneDepthPriorityGroup
{
	SDPG_World,
	SDPG_Foreground,
	SDPG_MAX
};

enum EBrushType
{
	Brush_Default,
	Brush_Add,
	Brush_Subtract,
	Brush_MAX
};

enum EAlphaBlendOption
{
	Linear,
	Cubic,
	HermiteCubic,
	Sinusoidal,
	QuadraticInOut,
	CubicInOut,
	QuarticInOut,
	QuinticInOut,
	CircularIn,
	CircularOut,
	CircularInOut,
	ExpIn,
	ExpOut,
	ExpInOut,
	Custom,
	EAlphaBlendOption_MAX
};

enum EAnimSyncGroupScope
{
	Local,
	Component,
	EAnimSyncGroupScope_MAX
};

enum EAnimGroupRole
{
	CanBeLeader,
	AlwaysFollower,
	AlwaysLeader,
	TransitionLeader,
	TransitionFollower,
	EAnimGroupRole_MAX
};

enum EPreviewAnimationBlueprintApplicationMethod
{
	LinkedLayers,
	LinkedAnimGraph,
	EPreviewAnimationBlueprintApplicationMethod_MAX
};

enum EAnimationKeyFormat
{
	AKF_ConstantKeyLerp,
	AKF_VariableKeyLerp,
	AKF_PerTrackCompression,
	AKF_MAX
};

enum ERawCurveTrackTypes
{
	RCT_Float,
	RCT_Vector,
	RCT_Transform,
	RCT_MAX
};

enum EAnimAssetCurveFlags
{
	AACF_NONE,
	AACF_DriveMorphTarget_DEPRECATED,
	AACF_DriveAttribute_DEPRECATED,
	AACF_Editable,
	AACF_DriveMaterial_DEPRECATED,
	AACF_Metadata,
	AACF_DriveTrack,
	AACF_Disabled,
	AACF_MAX
};

enum EAnimationCompressionFormat
{
	ACF_None,
	ACF_Float96NoW,
	ACF_Fixed48NoW,
	ACF_IntervalFixed32NoW,
	ACF_Fixed32NoW,
	ACF_Float32NoW,
	ACF_Identity,
	ACF_MAX
};

enum EAdditiveBasePoseType
{
	ABPT_None,
	ABPT_RefPose,
	ABPT_AnimScaled,
	ABPT_AnimFrame,
	ABPT_MAX
};

enum ERootMotionMode
{
	NoRootMotionExtraction,
	IgnoreRootMotion,
	RootMotionFromEverything,
	RootMotionFromMontagesOnly,
	ERootMotionMode_MAX
};

enum ERootMotionRootLock
{
	RefPose,
	AnimFirstFrame,
	Zero,
	ERootMotionRootLock_MAX
};

enum EMontagePlayReturnType
{
	MontageLength,
	Duration,
	EMontagePlayReturnType_MAX
};

enum EDrawDebugItemType
{
	DirectionalArrow,
	Sphere,
	Line,
	OnScreenMessage,
	CoordinateSystem,
	EDrawDebugItemType_MAX
};

enum EAnimLinkMethod
{
	Absolute,
	Relative,
	Proportional,
	EAnimLinkMethod_MAX
};

enum EMontageSubStepResult
{
	Moved,
	NotMoved,
	InvalidSection,
	InvalidMontage,
	EMontageSubStepResult_MAX
};

enum EAnimNotifyEventType
{
	Begin,
	End,
	EAnimNotifyEventType_MAX
};

enum EInertializationSpace
{
	Default,
	WorldSpace,
	WorldRotation,
	EInertializationSpace_MAX
};

enum EInertializationBoneState
{
	Invalid,
	Valid,
	Excluded,
	EInertializationBoneState_MAX
};

enum EInertializationState
{
	Inactive,
	Pending,
	Active,
	EInertializationState_MAX
};

enum EEvaluatorMode
{
	EM_Standard,
	EM_Freeze,
	EM_DelayedFreeze,
	EM_MAX
};

enum EEvaluatorDataSource
{
	EDS_SourcePose,
	EDS_DestinationPose,
	EDS_MAX
};

enum EPostCopyOperation
{
	None,
	LogicalNegateBool,
	EPostCopyOperation_MAX
};

enum EPinHidingMode
{
	NeverAsPin,
	PinHiddenByDefault,
	PinShownByDefault,
	AlwaysAsPin,
	EPinHidingMode_MAX
};

enum EAnimPhysCollisionType
{
	CoM,
	CustomSphere,
	InnerSphere,
	OuterSphere,
	AnimPhysCollisionType_MAX
};

enum EAnimPhysTwistAxis
{
	AxisX,
	AxisY,
	AxisZ,
	AnimPhysTwistAxis_MAX
};

enum ETypeAdvanceAnim
{
	ETAA_Default,
	ETAA_Finished,
	ETAA_Looped,
	ETAA_MAX
};

enum ETransitionLogicType
{
	TLT_StandardBlend,
	TLT_Inertialization,
	TLT_Custom,
	TLT_MAX
};

enum ETransitionBlendMode
{
	TBM_Linear,
	TBM_Cubic,
	TBM_MAX
};

enum EComponentType
{
	None,
	TranslationX,
	TranslationY,
	TranslationZ,
	RotationX,
	RotationY,
	RotationZ,
	Scale,
	ScaleX,
	ScaleY,
	ScaleZ,
	EComponentType_MAX
};

enum EAxisOption
{
	X,
	Y,
	Z,
	X_Neg,
	Y_Neg,
	Z_Neg,
	Custom,
	EAxisOption_MAX
};

enum EAnimInterpolationType
{
	Linear,
	Step,
	EAnimInterpolationType_MAX
};

enum ECurveBlendOption
{
	Override,
	DoNotOverride,
	NormalizeByWeight,
	BlendByWeight,
	UseBasePose,
	UseMaxValue,
	UseMinValue,
	ECurveBlendOption_MAX
};

enum EAdditiveAnimationType
{
	AAT_None,
	AAT_LocalSpaceBase,
	AAT_RotationOffsetMeshSpace,
	AAT_MAX
};

enum ENotifyFilterType
{
	NoFiltering,
	LOD,
	ENotifyFilterType_MAX
};

enum EMontageNotifyTickType
{
	Queued,
	BranchingPoint,
	EMontageNotifyTickType_MAX
};

enum EBoneRotationSource
{
	BRS_KeepComponentSpaceRotation,
	BRS_KeepLocalSpaceRotation,
	BRS_CopyFromTarget,
	BRS_MAX
};

enum EBoneControlSpace
{
	BCS_WorldSpace,
	BCS_ComponentSpace,
	BCS_ParentBoneSpace,
	BCS_BoneSpace,
	BCS_MAX
};

enum EBoneAxis
{
	BA_X,
	BA_Y,
	BA_Z,
	BA_MAX
};

enum EPrimaryAssetCookRule
{
	Unknown,
	NeverCook,
	DevelopmentCook,
	DevelopmentAlwaysCook,
	AlwaysCook,
	EPrimaryAssetCookRule_MAX
};

enum ENaturalSoundFalloffMode
{
	Continues,
	Silent,
	Hold,
	ENaturalSoundFalloffMode_MAX
};

enum EAttenuationShape
{
	Sphere,
	Capsule,
	Box,
	Cone,
	EAttenuationShape_MAX
};

enum EAttenuationDistanceModel
{
	Linear,
	Logarithmic,
	Inverse,
	LogReverse,
	NaturalSound,
	Custom,
	EAttenuationDistanceModel_MAX
};

enum EAudioBusChannels
{
	Mono,
	Stereo,
	EAudioBusChannels_MAX
};

enum EAudioFaderCurve
{
	Linear,
	Logarithmic,
	SCurve,
	Sin,
	Count,
	EAudioFaderCurve_MAX
};

enum EAudioOutputTarget
{
	Speaker,
	Controller,
	ControllerFallbackToSpeaker,
	EAudioOutputTarget_MAX
};

enum EMonoChannelUpmixMethod
{
	Linear,
	EqualPower,
	FullVolume,
	EMonoChannelUpmixMethod_MAX
};

enum EPanningMethod
{
	Linear,
	EqualPower,
	EPanningMethod_MAX
};

enum EVoiceSampleRate
{
	Low16000Hz,
	Normal24000Hz,
	EVoiceSampleRate_MAX
};

enum EAudioVolumeLocationState
{
	InsideTheVolume,
	OutsideTheVolume,
	EAudioVolumeLocationState_MAX
};

enum EBlendableLocation
{
	BL_AfterTonemapping,
	BL_BeforeTonemapping,
	BL_BeforeTranslucency,
	BL_ReplacingTonemapper,
	BL_SSRInput,
	BL_MAX
};

enum ENotifyTriggerMode
{
	AllAnimations,
	HighestWeightedAnimation,
	None,
	ENotifyTriggerMode_MAX
};

enum EBlendSpaceAxis
{
	BSA_None,
	BSA_X,
	BSA_Y,
	BSA_Max
};

enum EBlueprintNativizationFlag
{
	Disabled,
	Dependency,
	ExplicitlyEnabled,
	EBlueprintNativizationFlag_MAX
};

enum EBlueprintCompileMode
{
	Default,
	Development,
	FinalRelease,
	EBlueprintCompileMode_MAX
};

enum EBlueprintType
{
	BPTYPE_Normal,
	BPTYPE_Const,
	BPTYPE_MacroLibrary,
	BPTYPE_Interface,
	BPTYPE_LevelScript,
	BPTYPE_FunctionLibrary,
	BPTYPE_MAX
};

enum EBlueprintStatus
{
	BS_Unknown,
	BS_Dirty,
	BS_Error,
	BS_UpToDate,
	BS_BeingCreated,
	BS_UpToDateWithWarnings,
	BS_MAX
};

enum ECsgOper
{
	CSG_Active,
	CSG_Add,
	CSG_Subtract,
	CSG_Intersect,
	CSG_Deintersect,
	CSG_None,
	CSG_MAX
};

enum EInterfaceValidResult
{
	Valid,
	Invalid,
	EInterfaceValidResult_MAX
};

enum ECameraShakeDurationType
{
	Fixed,
	Infinite,
	Custom,
	ECameraShakeDurationType_MAX
};

enum ECameraShakeUpdateResultFlags
{
	ApplyAsAbsolute,
	SkipAutoScale,
	SkipAutoPlaySpace,
	Default,
	ECameraShakeUpdateResultFlags_MAX
};

enum ECameraShakeAttenuation
{
	Linear,
	Quadratic,
	ECameraShakeAttenuation_MAX
};

enum ECameraAlphaBlendMode
{
	CABM_Linear,
	CABM_Cubic,
	CABM_MAX
};

enum ECameraProjectionMode
{
	Perspective,
	Orthographic,
	ECameraProjectionMode_MAX
};

enum ECloudStorageDelegate
{
	CSD_KeyValueReadComplete,
	CSD_KeyValueWriteComplete,
	CSD_ValueChanged,
	CSD_DocumentQueryComplete,
	CSD_DocumentReadComplete,
	CSD_DocumentWriteComplete,
	CSD_DocumentConflictDetected,
	CSD_MAX
};

enum EAngularDriveMode
{
	SLERP,
	TwistAndSwing,
	EAngularDriveMode_MAX
};

enum ECurveTableMode
{
	Empty,
	SimpleCurves,
	RichCurves,
	ECurveTableMode_MAX
};

enum ECustomAttributeBlendType
{
	Override,
	Blend,
	ECustomAttributeBlendType_MAX
};

enum EFDataDrivenCVarType
{
	CVarFloat,
	CVarInt,
	CVarBool,
	FDataDrivenCVarType_MAX
};

enum EEvaluateCurveTableResult
{
	RowFound,
	RowNotFound,
	EEvaluateCurveTableResult_MAX
};

enum EGrammaticalNumber
{
	Singular,
	Plural,
	EGrammaticalNumber_MAX
};

enum EGrammaticalGender
{
	Neuter,
	Masculine,
	Feminine,
	Mixed,
	EGrammaticalGender_MAX
};

enum EDistributionParamMode
{
	DPM_Normal,
	DPM_Abs,
	DPM_Direct,
	DPM_MAX
};

enum EDistributionVectorMirrorFlags
{
	EDVMF_Same,
	EDVMF_Different,
	EDVMF_Mirror,
	EDVMF_MAX
};

enum EDistributionVectorLockFlags
{
	EDVLF_None,
	EDVLF_XY,
	EDVLF_XZ,
	EDVLF_YZ,
	EDVLF_XYZ,
	EDVLF_MAX
};

enum ENodeEnabledState
{
	Enabled,
	Disabled,
	DevelopmentOnly,
	ENodeEnabledState_MAX
};

enum ENodeAdvancedPins
{
	NoPins,
	Shown,
	Hidden,
	ENodeAdvancedPins_MAX
};

enum ENodeTitleType
{
	FullTitle,
	ListView,
	EditableTitle,
	MenuTitle,
	MAX_TitleTypes,
	ENodeTitleType_MAX
};

enum EPinContainerType
{
	None,
	Array,
	Set,
	Map,
	EPinContainerType_MAX
};

enum EEdGraphPinDirection
{
	EGPD_Input,
	EGPD_Output,
	EGPD_MAX
};

enum EBlueprintPinStyleType
{
	BPST_Original,
	BPST_VariantA,
	BPST_MAX
};

enum ECanCreateConnectionResponse
{
	CONNECT_RESPONSE_MAKE,
	CONNECT_RESPONSE_DISALLOW,
	CONNECT_RESPONSE_BREAK_OTHERS_A,
	CONNECT_RESPONSE_BREAK_OTHERS_B,
	CONNECT_RESPONSE_BREAK_OTHERS_AB,
	CONNECT_RESPONSE_MAKE_WITH_CONVERSION_NODE,
	CONNECT_RESPONSE_MAX
};

enum EGraphType
{
	GT_Function,
	GT_Ubergraph,
	GT_Macro,
	GT_Animation,
	GT_StateMachine,
	GT_MAX
};

enum ETransitionType
{
	None,
	Paused,
	Loading,
	Saving,
	Connecting,
	Precaching,
	WaitingToConnect,
	MAX
};

enum EFullyLoadPackageType
{
	FULLYLOAD_Map,
	FULLYLOAD_Game_PreLoadClass,
	FULLYLOAD_Game_PostLoadClass,
	FULLYLOAD_Always,
	FULLYLOAD_Mutator,
	FULLYLOAD_MAX
};

enum EViewModeIndex
{
	VMI_BrushWireframe,
	VMI_Wireframe,
	VMI_Unlit,
	VMI_Lit,
	VMI_Lit_DetailLighting,
	VMI_LightingOnly,
	VMI_LightComplexity,
	VMI_ShaderComplexity,
	VMI_LightmapDensity,
	VMI_LitLightmapDensity,
	VMI_ReflectionOverride,
	VMI_VisualizeBuffer,
	VMI_StationaryLightOverlap,
	VMI_CollisionPawn,
	VMI_CollisionVisibility,
	VMI_LODColoration,
	VMI_QuadOverdraw,
	VMI_PrimitiveDistanceAccuracy,
	VMI_MeshUVDensityAccuracy,
	VMI_ShaderComplexityWithQuadOverdraw,
	VMI_HLODColoration,
	VMI_GroupLODColoration,
	VMI_MaterialTextureScaleAccuracy,
	VMI_RequiredTextureResolution,
	VMI_PathTracing,
	VMI_RayTracingDebug,
	VMI_Max,
	VMI_Unknown
};

enum EDemoPlayFailure
{
	Generic,
	DemoNotFound,
	Corrupt,
	InvalidVersion,
	InitBase,
	GameSpecificHeader,
	ReplayStreamerInternal,
	LoadMap,
	Serialization,
	EDemoPlayFailure_MAX
};

enum ENetworkLagState
{
	NotLagging,
	Lagging,
	ENetworkLagState_MAX
};

enum EMouseCaptureMode
{
	NoCapture,
	CapturePermanently,
	CapturePermanently_IncludingInitialMouseDown,
	CaptureDuringMouseDown,
	CaptureDuringRightMouseDown,
	EMouseCaptureMode_MAX
};

enum ECustomTimeStepSynchronizationState
{
	Closed,
	Error,
	Synchronized,
	Synchronizing,
	ECustomTimeStepSynchronizationState_MAX
};

enum EMeshBufferAccess
{
	Default,
	ForceCPUAndGPU,
	EMeshBufferAccess_MAX
};

enum EComponentSocketType
{
	Invalid,
	Bone,
	Socket,
	EComponentSocketType_MAX
};

enum EPhysicalMaterialMaskColor
{
	Red,
	Green,
	Blue,
	Cyan,
	Magenta,
	Yellow,
	White,
	Black,
	MAX
};

enum EAutoPossessAI
{
	Disabled,
	PlacedInWorld,
	Spawned,
	PlacedInWorldOrSpawned,
	EAutoPossessAI_MAX
};

enum EUpdateRateShiftBucket
{
	ShiftBucket0,
	ShiftBucket1,
	ShiftBucket2,
	ShiftBucket3,
	ShiftBucket4,
	ShiftBucket5,
	ShiftBucketMax,
	EUpdateRateShiftBucket_MAX
};

enum EShadowMapFlags
{
	SMF_None,
	SMF_Streamed,
	SMF_MAX
};

enum ELightMapPaddingType
{
	LMPT_NormalPadding,
	LMPT_PrePadding,
	LMPT_NoPadding,
	LMPT_MAX
};

enum ETimelineSigType
{
	ETS_EventSignature,
	ETS_FloatSignature,
	ETS_VectorSignature,
	ETS_LinearColorSignature,
	ETS_InvalidSignature,
	ETS_MAX
};

enum EFilterInterpolationType
{
	BSIT_Average,
	BSIT_Linear,
	BSIT_Cubic,
	BSIT_MAX
};

enum EOverlapFilterOption
{
	OverlapFilter_All,
	OverlapFilter_DynamicOnly,
	OverlapFilter_StaticOnly,
	OverlapFilter_MAX
};

enum ENetworkSmoothingMode
{
	Disabled,
	Linear,
	Exponential,
	Replay,
	ENetworkSmoothingMode_MAX
};

enum ELightingBuildQuality
{
	Quality_Preview,
	Quality_Medium,
	Quality_High,
	Quality_Production,
	Quality_MAX
};

enum EMaterialShadingRate
{
	MSR_1x1,
	MSR_2x1,
	MSR_1x2,
	MSR_2x2,
	MSR_4x2,
	MSR_2x4,
	MSR_4x4,
	MSR_Count,
	MSR_MAX
};

enum EMaterialStencilCompare
{
	MSC_Less,
	MSC_LessEqual,
	MSC_Greater,
	MSC_GreaterEqual,
	MSC_Equal,
	MSC_NotEqual,
	MSC_Never,
	MSC_Always,
	MSC_Count,
	MSC_MAX
};

enum EMaterialSamplerType
{
	SAMPLERTYPE_Color,
	SAMPLERTYPE_Grayscale,
	SAMPLERTYPE_Alpha,
	SAMPLERTYPE_Normal,
	SAMPLERTYPE_Masks,
	SAMPLERTYPE_DistanceFieldFont,
	SAMPLERTYPE_LinearColor,
	SAMPLERTYPE_LinearGrayscale,
	SAMPLERTYPE_Data,
	SAMPLERTYPE_External,
	SAMPLERTYPE_VirtualColor,
	SAMPLERTYPE_VirtualGrayscale,
	SAMPLERTYPE_VirtualAlpha,
	SAMPLERTYPE_VirtualNormal,
	SAMPLERTYPE_VirtualMasks,
	SAMPLERTYPE_VirtualLinearColor,
	SAMPLERTYPE_VirtualLinearGrayscale,
	SAMPLERTYPE_MAX
};

enum EMaterialTessellationMode
{
	MTM_NoTessellation,
	MTM_FlatTessellation,
	MTM_PNTriangles,
	MTM_MAX
};

enum EMaterialShadingModel
{
	MSM_Unlit,
	MSM_DefaultLit,
	MSM_Subsurface,
	MSM_PreintegratedSkin,
	MSM_ClearCoat,
	MSM_SubsurfaceProfile,
	MSM_TwoSidedFoliage,
	MSM_Hair,
	MSM_Cloth,
	MSM_Eye,
	MSM_SingleLayerWater,
	MSM_ThinTranslucent,
	MSM_NUM,
	MSM_FromMaterialExpression,
	MSM_MAX
};

enum EParticleCollisionMode
{
	SceneDepth,
	DistanceField,
	EParticleCollisionMode_MAX
};

enum ETrailWidthMode
{
	ETrailWidthMode_FromCentre,
	ETrailWidthMode_FromFirst,
	ETrailWidthMode_FromSecond,
	ETrailWidthMode_MAX
};

enum EGBufferFormat
{
	Force8BitsPerChannel,
	Default,
	HighPrecisionNormals,
	Force16BitsPerChannel,
	EGBufferFormat_MAX
};

enum ESceneCaptureCompositeMode
{
	SCCM_Overwrite,
	SCCM_Additive,
	SCCM_Composite,
	SCCM_MAX
};

enum ESceneCaptureSource
{
	SCS_SceneColorHDR,
	SCS_SceneColorHDRNoAlpha,
	SCS_FinalColorLDR,
	SCS_SceneColorSceneDepth,
	SCS_SceneDepth,
	SCS_DeviceDepth,
	SCS_Normal,
	SCS_BaseColor,
	SCS_FinalColorHDR,
	SCS_FinalToneCurveHDR,
	SCS_MAX
};

enum ETranslucentSortPolicy
{
	SortByDistance,
	SortByProjectedZ,
	SortAlongAxis,
	ETranslucentSortPolicy_MAX
};

enum ERefractionMode
{
	RM_IndexOfRefraction,
	RM_PixelNormalOffset,
	RM_MAX
};

enum ETranslucencyLightingMode
{
	TLM_VolumetricNonDirectional,
	TLM_VolumetricDirectional,
	TLM_VolumetricPerVertexNonDirectional,
	TLM_VolumetricPerVertexDirectional,
	TLM_Surface,
	TLM_SurfacePerPixelLighting,
	TLM_MAX
};

enum ESamplerSourceMode
{
	SSM_FromTextureAsset,
	SSM_Wrap_WorldGroupSettings,
	SSM_Clamp_WorldGroupSettings,
	SSM_MAX
};

enum EBlendMode
{
	BLEND_Opaque,
	BLEND_Masked,
	BLEND_Translucent,
	BLEND_Additive,
	BLEND_Modulate,
	BLEND_AlphaComposite,
	BLEND_AlphaHoldout,
	BLEND_MAX
};

enum EOcclusionCombineMode
{
	OCM_Minimum,
	OCM_Multiply,
	OCM_MAX
};

enum EAspectRatioAxisConstraint
{
	AspectRatio_MaintainYFOV,
	AspectRatio_MaintainXFOV,
	AspectRatio_MajorAxisFOV,
	AspectRatio_MAX
};

enum EFontCacheType
{
	Offline,
	Runtime,
	EFontCacheType_MAX
};

enum EFontImportCharacterSet
{
	FontICS_Default,
	FontICS_Ansi,
	FontICS_Symbol,
	FontICS_MAX
};

enum EStandbyType
{
	STDBY_Rx,
	STDBY_Tx,
	STDBY_BadPing,
	STDBY_MAX
};

enum ESuggestProjVelocityTraceOption
{
	DoNotTrace,
	TraceFullPath,
	OnlyTraceWhileAscending,
	ESuggestProjVelocityTraceOption_MAX
};

enum EWindowMode
{
	Fullscreen,
	WindowedFullscreen,
	Windowed,
	EWindowMode_MAX
};

enum EHitProxyPriority
{
	HPP_World,
	HPP_Wireframe,
	HPP_Foreground,
	HPP_UI,
	HPP_MAX
};

enum EImportanceWeight
{
	Luminance,
	Red,
	Green,
	Blue,
	Alpha,
	EImportanceWeight_MAX
};

enum EAdManagerDelegate
{
	AMD_ClickedBanner,
	AMD_UserClosedAd,
	AMD_MAX
};

enum EAnimAlphaInputType
{
	Float,
	Bool,
	Curve,
	EAnimAlphaInputType_MAX
};

enum ETrackActiveCondition
{
	ETAC_Always,
	ETAC_GoreEnabled,
	ETAC_GoreDisabled,
	ETAC_MAX
};

enum EInterpTrackMoveRotMode
{
	IMR_Keyframed,
	IMR_LookAtGroup,
	IMR_Ignore,
	IMR_MAX
};

enum EInterpMoveAxis
{
	AXIS_TranslationX,
	AXIS_TranslationY,
	AXIS_TranslationZ,
	AXIS_RotationX,
	AXIS_RotationY,
	AXIS_RotationZ,
	AXIS_MAX
};

enum ETrackToggleAction
{
	ETTA_Off,
	ETTA_On,
	ETTA_Toggle,
	ETTA_Trigger,
	ETTA_MAX
};

enum EVisibilityTrackCondition
{
	EVTC_Always,
	EVTC_GoreEnabled,
	EVTC_GoreDisabled,
	EVTC_MAX
};

enum EVisibilityTrackAction
{
	EVTA_Hide,
	EVTA_Show,
	EVTA_Toggle,
	EVTA_MAX
};

enum ESlateGesture
{
	None,
	Scroll,
	Magnify,
	Swipe,
	Rotate,
	LongPress,
	ESlateGesture_MAX
};

enum EMIDCreationFlags
{
	None,
	Transient,
	EMIDCreationFlags_MAX
};

enum EMatrixColumns
{
	First,
	Second,
	Third,
	Fourth,
	EMatrixColumns_MAX
};

enum ELerpInterpolationMode
{
	QuatInterp,
	EulerInterp,
	DualQuatInterp,
	ELerpInterpolationMode_MAX
};

enum EEasingFunc
{
	Linear,
	Step,
	SinusoidalIn,
	SinusoidalOut,
	SinusoidalInOut,
	EaseIn,
	EaseOut,
	EaseInOut,
	ExpoIn,
	ExpoOut,
	ExpoInOut,
	CircularIn,
	CircularOut,
	CircularInOut,
	EEasingFunc_MAX
};

enum EStreamingVolumeUsage
{
	SVB_Loading,
	SVB_LoadingAndVisibility,
	SVB_VisibilityBlockingOnLoad,
	SVB_BlockingOnLoad,
	SVB_LoadingNotVisible,
	SVB_MAX
};

enum ESyncOption
{
	Drive,
	Passive,
	Disabled,
	ESyncOption_MAX
};

enum EMaterialDecalResponse
{
	MDR_None,
	MDR_ColorNormalRoughness,
	MDR_Color,
	MDR_ColorNormal,
	MDR_ColorRoughness,
	MDR_Normal,
	MDR_NormalRoughness,
	MDR_Roughness,
	MDR_MAX
};

enum EDecalBlendMode
{
	DBM_Translucent,
	DBM_Stain,
	DBM_Normal,
	DBM_Emissive,
	DBM_DBuffer_ColorNormalRoughness,
	DBM_DBuffer_Color,
	DBM_DBuffer_ColorNormal,
	DBM_DBuffer_ColorRoughness,
	DBM_DBuffer_Normal,
	DBM_DBuffer_NormalRoughness,
	DBM_DBuffer_Roughness,
	DBM_DBuffer_Emissive,
	DBM_DBuffer_AlphaComposite,
	DBM_DBuffer_EmissiveAlphaComposite,
	DBM_Volumetric_DistanceFunction,
	DBM_AlphaComposite,
	DBM_AmbientOcclusion,
	DBM_MAX
};

enum ETextureColorChannel
{
	TCC_Red,
	TCC_Green,
	TCC_Blue,
	TCC_Alpha,
	TCC_MAX
};

enum EMaterialAttributeBlend
{
	Blend,
	UseA,
	UseB,
	EMaterialAttributeBlend_MAX
};

enum EChannelMaskParameterColor
{
	Red,
	Green,
	Blue,
	Alpha,
	EChannelMaskParameterColor_MAX
};

enum EClampMode
{
	CMODE_Clamp,
	CMODE_ClampMin,
	CMODE_ClampMax,
	CMODE_MAX
};

enum ECustomMaterialOutputType
{
	CMOT_Float1,
	CMOT_Float2,
	CMOT_Float3,
	CMOT_Float4,
	CMOT_MaterialAttributes,
	CMOT_MAX
};

enum EDepthOfFieldFunctionValue
{
	TDOF_NearAndFarMask,
	TDOF_NearMask,
	TDOF_FarMask,
	TDOF_CircleOfConfusionRadius,
	TDOF_MAX
};

enum EFunctionInputType
{
	FunctionInput_Scalar,
	FunctionInput_Vector2,
	FunctionInput_Vector3,
	FunctionInput_Vector4,
	FunctionInput_Texture2D,
	FunctionInput_TextureCube,
	FunctionInput_Texture2DArray,
	FunctionInput_VolumeTexture,
	FunctionInput_StaticBool,
	FunctionInput_MaterialAttributes,
	FunctionInput_TextureExternal,
	FunctionInput_MAX
};

enum ENoiseFunction
{
	NOISEFUNCTION_SimplexTex,
	NOISEFUNCTION_GradientTex,
	NOISEFUNCTION_GradientTex3D,
	NOISEFUNCTION_GradientALU,
	NOISEFUNCTION_ValueALU,
	NOISEFUNCTION_VoronoiALU,
	NOISEFUNCTION_MAX
};

enum ERuntimeVirtualTextureTextureAddressMode
{
	RVTTA_Clamp,
	RVTTA_Wrap,
	RVTTA_MAX
};

enum ERuntimeVirtualTextureMipValueMode
{
	RVTMVM_None,
	RVTMVM_MipLevel,
	RVTMVM_MipBias,
	RVTMVM_MAX
};

enum EMaterialSceneAttributeInputMode
{
	Coordinates,
	OffsetFraction,
	EMaterialSceneAttributeInputMode_MAX
};

enum ESpeedTreeLODType
{
	STLOD_Pop,
	STLOD_Smooth,
	STLOD_MAX
};

enum ESpeedTreeWindType
{
	STW_None,
	STW_Fastest,
	STW_Fast,
	STW_Better,
	STW_Best,
	STW_Palm,
	STW_BestPlus,
	STW_MAX
};

enum ESpeedTreeGeometryType
{
	STG_Branch,
	STG_Frond,
	STG_Leaf,
	STG_FacingLeaf,
	STG_Billboard,
	STG_MAX
};

enum EMaterialExposedTextureProperty
{
	TMTM_TextureSize,
	TMTM_TexelSize,
	TMTM_MAX
};

enum ETextureMipValueMode
{
	TMVM_None,
	TMVM_MipLevel,
	TMVM_MipBias,
	TMVM_Derivative,
	TMVM_MAX
};

enum EMaterialVectorCoordTransform
{
	TRANSFORM_Tangent,
	TRANSFORM_Local,
	TRANSFORM_World,
	TRANSFORM_View,
	TRANSFORM_Camera,
	TRANSFORM_ParticleWorld,
	TRANSFORM_MAX
};

enum EMaterialVectorCoordTransformSource
{
	TRANSFORMSOURCE_Tangent,
	TRANSFORMSOURCE_Local,
	TRANSFORMSOURCE_World,
	TRANSFORMSOURCE_View,
	TRANSFORMSOURCE_Camera,
	TRANSFORMSOURCE_ParticleWorld,
	TRANSFORMSOURCE_MAX
};

enum EMaterialPositionTransformSource
{
	TRANSFORMPOSSOURCE_Local,
	TRANSFORMPOSSOURCE_World,
	TRANSFORMPOSSOURCE_TranslatedWorld,
	TRANSFORMPOSSOURCE_View,
	TRANSFORMPOSSOURCE_Camera,
	TRANSFORMPOSSOURCE_Particle,
	TRANSFORMPOSSOURCE_MAX
};

enum EVectorNoiseFunction
{
	VNF_CellnoiseALU,
	VNF_VectorALU,
	VNF_GradientALU,
	VNF_CurlALU,
	VNF_VoronoiALU,
	VNF_MAX
};

enum EMaterialExposedViewProperty
{
	MEVP_BufferSize,
	MEVP_FieldOfView,
	MEVP_TanHalfFieldOfView,
	MEVP_ViewSize,
	MEVP_WorldSpaceViewPosition,
	MEVP_WorldSpaceCameraPosition,
	MEVP_ViewportOffset,
	MEVP_TemporalSampleCount,
	MEVP_TemporalSampleIndex,
	MEVP_TemporalSampleOffset,
	MEVP_RuntimeVirtualTextureOutputLevel,
	MEVP_RuntimeVirtualTextureOutputDerivative,
	MEVP_PreExposure,
	MEVP_RuntimeVirtualTextureMaxLevel,
	MEVP_MAX
};

enum EWorldPositionIncludedOffsets
{
	WPT_Default,
	WPT_ExcludeAllShaderOffsets,
	WPT_CameraRelative,
	WPT_CameraRelativeNoOffsets,
	WPT_MAX
};

enum EMaterialFunctionUsage
{
	Default,
	MaterialLayer,
	MaterialLayerBlend,
	EMaterialFunctionUsage_MAX
};

enum EMaterialUsage
{
	MATUSAGE_SkeletalMesh,
	MATUSAGE_ParticleSprites,
	MATUSAGE_BeamTrails,
	MATUSAGE_MeshParticles,
	MATUSAGE_StaticLighting,
	MATUSAGE_MorphTargets,
	MATUSAGE_SplineMesh,
	MATUSAGE_InstancedStaticMeshes,
	MATUSAGE_GeometryCollections,
	MATUSAGE_Clothing,
	MATUSAGE_NiagaraSprites,
	MATUSAGE_NiagaraRibbons,
	MATUSAGE_NiagaraMeshParticles,
	MATUSAGE_GeometryCache,
	MATUSAGE_Water,
	MATUSAGE_HairStrands,
	MATUSAGE_LidarPointCloud,
	MATUSAGE_VirtualHeightfieldMesh,
	MATUSAGE_MAX
};

enum EMaterialLayerLinkState
{
	Uninitialized,
	LinkedToParent,
	UnlinkedFromParent,
	NotFromParent,
	EMaterialLayerLinkState_MAX
};

enum EMaterialParameterAssociation
{
	LayerParameter,
	BlendParameter,
	GlobalParameter,
	EMaterialParameterAssociation_MAX
};

enum EMaterialMergeType
{
	MaterialMergeType_Default,
	MaterialMergeType_Simplygon,
	MaterialMergeType_MAX
};

enum ETextureSizingType
{
	TextureSizingType_UseSingleTextureSize,
	TextureSizingType_UseAutomaticBiasedSizes,
	TextureSizingType_UseManualOverrideTextureSize,
	TextureSizingType_UseSimplygonAutomaticSizing,
	TextureSizingType_MAX
};

enum ESceneTextureId
{
	PPI_SceneColor,
	PPI_SceneDepth,
	PPI_DiffuseColor,
	PPI_SpecularColor,
	PPI_SubsurfaceColor,
	PPI_BaseColor,
	PPI_Specular,
	PPI_Metallic,
	PPI_WorldNormal,
	PPI_SeparateTranslucency,
	PPI_Opacity,
	PPI_Roughness,
	PPI_MaterialAO,
	PPI_CustomDepth,
	PPI_PostProcessInput0,
	PPI_PostProcessInput1,
	PPI_PostProcessInput2,
	PPI_PostProcessInput3,
	PPI_PostProcessInput4,
	PPI_PostProcessInput5,
	PPI_PostProcessInput6,
	PPI_DecalMask,
	PPI_ShadingModelColor,
	PPI_ShadingModelID,
	PPI_AmbientOcclusion,
	PPI_CustomStencil,
	PPI_StoredBaseColor,
	PPI_StoredSpecular,
	PPI_Velocity,
	PPI_WorldTangent,
	PPI_Anisotropy,
	PPI_MAX
};

enum EMaterialDomain
{
	MD_Surface,
	MD_DeferredDecal,
	MD_LightFunction,
	MD_Volume,
	MD_PostProcess,
	MD_UI,
	MD_RuntimeVirtualTexture,
	MD_MAX
};

enum EMeshInstancingReplacementMethod
{
	RemoveOriginalActors,
	KeepOriginalActorsAsEditorOnly,
	EMeshInstancingReplacementMethod_MAX
};

enum EUVOutput
{
	DoNotOutputChannel,
	OutputChannel,
	EUVOutput_MAX
};

enum EMeshMergeType
{
	MeshMergeType_Default,
	MeshMergeType_MergeActor,
	MeshMergeType_MAX
};

enum EMeshLODSelectionType
{
	AllLODs,
	SpecificLOD,
	CalculateLOD,
	LowestDetailLOD,
	EMeshLODSelectionType_MAX
};

enum EProxyNormalComputationMethod
{
	AngleWeighted,
	AreaWeighted,
	EqualWeighted,
	EProxyNormalComputationMethod_MAX
};

enum ELandscapeCullingPrecision
{
	High,
	Medium,
	Low,
	ELandscapeCullingPrecision_MAX
};

enum EStaticMeshReductionTerimationCriterion
{
	Triangles,
	Vertices,
	Any,
	EStaticMeshReductionTerimationCriterion_MAX
};

enum EMeshFeatureImportance
{
	Off,
	Lowest,
	Low,
	Normal,
	High,
	Highest,
	EMeshFeatureImportance_MAX
};

enum EVertexPaintAxis
{
	X,
	Y,
	Z,
	EVertexPaintAxis_MAX
};

enum EMicroTransactionResult
{
	MTR_Succeeded,
	MTR_Failed,
	MTR_Canceled,
	MTR_RestoredFromServer,
	MTR_MAX
};

enum EMicroTransactionDelegate
{
	MTD_PurchaseQueryComplete,
	MTD_PurchaseComplete,
	MTD_MAX
};

enum EFNavigationSystemRunMode
{
	InvalidMode,
	GameMode,
	EditorMode,
	SimulationMode,
	PIEMode,
	InferFromWorldMode,
	FNavigationSystemRunMode_MAX
};

enum ENavigationQueryResult
{
	Invalid,
	Error,
	Fail,
	Success,
	ENavigationQueryResult_MAX
};

enum ENavPathEvent
{
	Cleared,
	NewPath,
	UpdatedDueToGoalMoved,
	UpdatedDueToNavigationChanged,
	Invalidated,
	RePathFailed,
	MetaPathUpdate,
	Custom,
	ENavPathEvent_MAX
};

enum ENavDataGatheringModeConfig
{
	Invalid,
	Instant,
	Lazy,
	ENavDataGatheringModeConfig_MAX
};

enum ENavDataGatheringMode
{
	Default,
	Instant,
	Lazy,
	ENavDataGatheringMode_MAX
};

enum ENavigationOptionFlag
{
	Default,
	Enable,
	Disable,
	MAX
};

enum ENavLinkDirection
{
	BothWays,
	LeftToRight,
	RightToLeft,
	ENavLinkDirection_MAX
};

enum EEmitterRenderMode
{
	ERM_Normal,
	ERM_Point,
	ERM_Cross,
	ERM_LightsOnly,
	ERM_None,
	ERM_MAX
};

enum EParticleSubUVInterpMethod
{
	PSUVIM_None,
	PSUVIM_Linear,
	PSUVIM_Linear_Blend,
	PSUVIM_Random,
	PSUVIM_Random_Blend,
	PSUVIM_MAX
};

enum EParticleBurstMethod
{
	EPBM_Instant,
	EPBM_Interpolated,
	EPBM_MAX
};

enum EParticleSystemInsignificanceReaction
{
	Auto,
	Complete,
	DisableTick,
	DisableTickAndKill,
	Num,
	EParticleSystemInsignificanceReaction_MAX
};

enum EParticleSignificanceLevel
{
	Low,
	Medium,
	High,
	Critical,
	Num,
	EParticleSignificanceLevel_MAX
};

enum EParticleDetailMode
{
	PDM_Low,
	PDM_Medium,
	PDM_High,
	PDM_MAX
};

enum EParticleSourceSelectionMethod
{
	EPSSM_Random,
	EPSSM_Sequential,
	EPSSM_MAX
};

enum EModuleType
{
	EPMT_General,
	EPMT_TypeData,
	EPMT_Beam,
	EPMT_Trail,
	EPMT_Spawn,
	EPMT_Required,
	EPMT_Event,
	EPMT_Light,
	EPMT_SubUV,
	EPMT_MAX
};

enum EAttractorParticleSelectionMethod
{
	EAPSM_Random,
	EAPSM_Sequential,
	EAPSM_MAX
};

enum EBeam2SourceTargetTangentMethod
{
	PEB2STTM_Direct,
	PEB2STTM_UserSet,
	PEB2STTM_Distribution,
	PEB2STTM_Emitter,
	PEB2STTM_MAX
};

enum EBeam2SourceTargetMethod
{
	PEB2STM_Default,
	PEB2STM_UserSet,
	PEB2STM_Emitter,
	PEB2STM_Particle,
	PEB2STM_Actor,
	PEB2STM_MAX
};

enum EBeamModifierType
{
	PEB2MT_Source,
	PEB2MT_Target,
	PEB2MT_MAX
};

enum EParticleCameraOffsetUpdateMethod
{
	EPCOUM_DirectSet,
	EPCOUM_Additive,
	EPCOUM_Scalar,
	EPCOUM_MAX
};

enum EParticleCollisionComplete
{
	EPCC_Kill,
	EPCC_Freeze,
	EPCC_HaltCollisions,
	EPCC_FreezeTranslation,
	EPCC_FreezeRotation,
	EPCC_FreezeMovement,
	EPCC_MAX
};

enum EParticleCollisionResponse
{
	Bounce,
	Stop,
	Kill,
	EParticleCollisionResponse_MAX
};

enum ELocationBoneSocketSelectionMethod
{
	BONESOCKETSEL_Sequential,
	BONESOCKETSEL_Random,
	BONESOCKETSEL_MAX
};

enum ELocationBoneSocketSource
{
	BONESOCKETSOURCE_Bones,
	BONESOCKETSOURCE_Sockets,
	BONESOCKETSOURCE_MAX
};

enum ELocationEmitterSelectionMethod
{
	ELESM_Random,
	ELESM_Sequential,
	ELESM_MAX
};

enum ECylinderHeightAxis
{
	PMLPC_HEIGHTAXIS_X,
	PMLPC_HEIGHTAXIS_Y,
	PMLPC_HEIGHTAXIS_Z,
	PMLPC_HEIGHTAXIS_MAX
};

enum ELocationSkelVertSurfaceSource
{
	VERTSURFACESOURCE_Vert,
	VERTSURFACESOURCE_Surface,
	VERTSURFACESOURCE_MAX
};

enum EOrbitChainMode
{
	EOChainMode_Add,
	EOChainMode_Scale,
	EOChainMode_Link,
	EOChainMode_MAX
};

enum EParticleAxisLock
{
	EPAL_NONE,
	EPAL_X,
	EPAL_Y,
	EPAL_Z,
	EPAL_NEGATIVE_X,
	EPAL_NEGATIVE_Y,
	EPAL_NEGATIVE_Z,
	EPAL_ROTATE_X,
	EPAL_ROTATE_Y,
	EPAL_ROTATE_Z,
	EPAL_MAX
};

enum EEmitterDynamicParameterValue
{
	EDPV_UserSet,
	EDPV_AutoSet,
	EDPV_VelocityX,
	EDPV_VelocityY,
	EDPV_VelocityZ,
	EDPV_VelocityMag,
	EDPV_MAX
};

enum EEmitterNormalsMode
{
	ENM_CameraFacing,
	ENM_Spherical,
	ENM_Cylindrical,
	ENM_MAX
};

enum EParticleSortMode
{
	PSORTMODE_None,
	PSORTMODE_ViewProjDepth,
	PSORTMODE_DistanceToView,
	PSORTMODE_Age_OldestFirst,
	PSORTMODE_Age_NewestFirst,
	PSORTMODE_MAX
};

enum EParticleUVFlipMode
{
	None,
	FlipUV,
	FlipUOnly,
	FlipVOnly,
	RandomFlipUV,
	RandomFlipUOnly,
	RandomFlipVOnly,
	RandomFlipUVIndependent,
	EParticleUVFlipMode_MAX
};

enum ETrail2SourceMethod
{
	PET2SRCM_Default,
	PET2SRCM_Particle,
	PET2SRCM_Actor,
	PET2SRCM_MAX
};

enum EBeamTaperMethod
{
	PEBTM_None,
	PEBTM_Full,
	PEBTM_Partial,
	PEBTM_MAX
};

enum EBeam2Method
{
	PEB2M_Distance,
	PEB2M_Target,
	PEB2M_Branch,
	PEB2M_MAX
};

enum EMeshCameraFacingOptions
{
	XAxisFacing_NoUp,
	XAxisFacing_ZUp,
	XAxisFacing_NegativeZUp,
	XAxisFacing_YUp,
	XAxisFacing_NegativeYUp,
	LockedAxis_ZAxisFacing,
	LockedAxis_NegativeZAxisFacing,
	LockedAxis_YAxisFacing,
	LockedAxis_NegativeYAxisFacing,
	VelocityAligned_ZAxisFacing,
	VelocityAligned_NegativeZAxisFacing,
	VelocityAligned_YAxisFacing,
	VelocityAligned_NegativeYAxisFacing,
	EMeshCameraFacingOptions_MAX
};

enum EMeshCameraFacingUpAxis
{
	CameraFacing_NoneUP,
	CameraFacing_ZUp,
	CameraFacing_NegativeZUp,
	CameraFacing_YUp,
	CameraFacing_NegativeYUp,
	CameraFacing_MAX
};

enum EMeshScreenAlignment
{
	PSMA_MeshFaceCameraWithRoll,
	PSMA_MeshFaceCameraWithSpin,
	PSMA_MeshFaceCameraWithLockedAxis,
	PSMA_MAX
};

enum ETrailsRenderAxisOption
{
	Trails_CameraUp,
	Trails_SourceUp,
	Trails_WorldUp,
	Trails_MAX
};

enum EParticleScreenAlignment
{
	PSA_FacingCameraPosition,
	PSA_Square,
	PSA_Rectangle,
	PSA_Velocity,
	PSA_AwayFromCenter,
	PSA_TypeSpecific,
	PSA_FacingCameraDistanceBlend,
	PSA_MAX
};

enum EParticleSystemOcclusionBoundsMethod
{
	EPSOBM_None,
	EPSOBM_ParticleBounds,
	EPSOBM_CustomBounds,
	EPSOBM_MAX
};

enum EParticleSystemLODMethod
{
	PARTICLESYSTEMLODMETHOD_Automatic,
	PARTICLESYSTEMLODMETHOD_DirectSet,
	PARTICLESYSTEMLODMETHOD_ActivateAutomatic,
	PARTICLESYSTEMLODMETHOD_MAX
};

enum EParticleSystemUpdateMode
{
	EPSUM_RealTime,
	EPSUM_FixedTime,
	EPSUM_MAX
};

enum EParticleEventType
{
	EPET_Any,
	EPET_Spawn,
	EPET_Death,
	EPET_Collision,
	EPET_Burst,
	EPET_Blueprint,
	EPET_MAX
};

enum EParticleReplayState
{
	PRS_Disabled,
	PRS_Capturing,
	PRS_Replaying,
	PRS_MAX
};

enum EParticleSysParamType
{
	PSPT_None,
	PSPT_Scalar,
	PSPT_ScalarRand,
	PSPT_Vector,
	PSPT_VectorRand,
	PSPT_Color,
	PSPT_Actor,
	PSPT_Material,
	PSPT_VectorUnitRand,
	PSPT_MAX
};

enum EPhysicsAssetSolverType
{
	RBAN,
	World,
	EPhysicsAssetSolverType_MAX
};

enum ESettingsLockedAxis
{
	None,
	X,
	Y,
	Z,
	Invalid,
	ESettingsLockedAxis_MAX
};

enum ESettingsDOF
{
	Full3D,
	YZPlane,
	XZPlane,
	XYPlane,
	ESettingsDOF_MAX
};

enum EQuarztQuantizationReference
{
	BarRelative,
	TransportRelative,
	CurrentTimeRelative,
	Count,
	EQuarztQuantizationReference_MAX
};

enum EQuartzDelegateType
{
	MetronomeTick,
	CommandEvent,
	Count,
	EQuartzDelegateType_MAX
};

enum EQuartzTimeSignatureQuantization
{
	HalfNote,
	QuarterNote,
	EighthNote,
	SixteenthNote,
	ThirtySecondNote,
	Count,
	EQuartzTimeSignatureQuantization_MAX
};

enum ERichCurveExtrapolation
{
	RCCE_Cycle,
	RCCE_CycleWithOffset,
	RCCE_Oscillate,
	RCCE_Linear,
	RCCE_Constant,
	RCCE_None,
	RCCE_MAX
};

enum ERichCurveInterpMode
{
	RCIM_Linear,
	RCIM_Constant,
	RCIM_Cubic,
	RCIM_None,
	RCIM_MAX
};

enum EMobileReflectionCompression
{
	Default,
	On,
	Off,
	EMobileReflectionCompression_MAX
};

enum EReflectionSourceType
{
	CapturedScene,
	SpecifiedCubemap,
	EReflectionSourceType_MAX
};

enum EDefaultBackBufferPixelFormat
{
	DBBPF_B8G8R8A8,
	DBBPF_A16B16G16R16_DEPRECATED,
	DBBPF_FloatRGB_DEPRECATED,
	DBBPF_FloatRGBA,
	DBBPF_A2B10G10R10,
	DBBPF_MAX
};

enum EAutoExposureMethodUI
{
	AEM_Histogram,
	AEM_Basic,
	AEM_Manual,
	AEM_MAX
};

enum EAlphaChannelMode
{
	Disabled,
	LinearColorSpaceOnly,
	AllowThroughTonemapper,
	EAlphaChannelMode_MAX
};

enum EEarlyZPass
{
	None,
	OpaqueOnly,
	OpaqueAndMasked,
	Auto,
	EEarlyZPass_MAX
};

enum ECustomDepthStencil
{
	Disabled,
	Enabled,
	EnabledOnDemand,
	EnabledWithStencil,
	ECustomDepthStencil_MAX
};

enum EMobileMSAASampleCount
{
	One,
	Two,
	Four,
	Eight,
	EMobileMSAASampleCount_MAX
};

enum ECompositingSampleCount
{
	One,
	Two,
	Four,
	Eight,
	ECompositingSampleCount_MAX
};

enum EClearSceneOptions
{
	NoClear,
	HardwareClear,
	QuadAtMaxZ,
	EClearSceneOptions_MAX
};

enum EReporterLineStyle
{
	Line,
	Dash,
	EReporterLineStyle_MAX
};

enum ELegendPosition
{
	Outside,
	Inside,
	ELegendPosition_MAX
};

enum EGraphDataStyle
{
	Lines,
	Filled,
	EGraphDataStyle_MAX
};

enum EGraphAxisStyle
{
	Lines,
	Notches,
	Grid,
	EGraphAxisStyle_MAX
};

enum EReverbPreset
{
	REVERB_Default,
	REVERB_Bathroom,
	REVERB_StoneRoom,
	REVERB_Auditorium,
	REVERB_ConcertHall,
	REVERB_Cave,
	REVERB_Hallway,
	REVERB_StoneCorridor,
	REVERB_Alley,
	REVERB_Forest,
	REVERB_City,
	REVERB_Mountains,
	REVERB_Quarry,
	REVERB_Plain,
	REVERB_ParkingLot,
	REVERB_SewerPipe,
	REVERB_Underwater,
	REVERB_SmallRoom,
	REVERB_MediumRoom,
	REVERB_LargeRoom,
	REVERB_MediumHall,
	REVERB_LargeHall,
	REVERB_Plate,
	REVERB_MAX
};

enum ERichCurveKeyTimeCompressionFormat
{
	RCKTCF_uint16,
	RCKTCF_float32,
	RCKTCF_MAX
};

enum ERichCurveCompressionFormat
{
	RCCF_Empty,
	RCCF_Constant,
	RCCF_Linear,
	RCCF_Cubic,
	RCCF_Mixed,
	RCCF_Weighted,
	RCCF_MAX
};

enum ERichCurveTangentWeightMode
{
	RCTWM_WeightedNone,
	RCTWM_WeightedArrive,
	RCTWM_WeightedLeave,
	RCTWM_WeightedBoth,
	RCTWM_MAX
};

enum ERichCurveTangentMode
{
	RCTM_Auto,
	RCTM_User,
	RCTM_Break,
	RCTM_None,
	RCTM_MAX
};

enum EConstraintTransform
{
	Absolute,
	Relative,
	EConstraintTransform_MAX
};

enum EControlConstraint
{
	Orientation,
	Translation,
	MAX
};

enum ERootMotionFinishVelocityMode
{
	MaintainLastRootMotionVelocity,
	SetVelocity,
	ClampVelocity,
	ERootMotionFinishVelocityMode_MAX
};

enum ERootMotionSourceSettingsFlags
{
	UseSensitiveLiftoffCheck,
	DisablePartialEndTick,
	IgnoreZAccumulate,
	ERootMotionSourceSettingsFlags_MAX
};

enum ERootMotionSourceStatusFlags
{
	Prepared,
	Finished,
	MarkedForRemoval,
	ERootMotionSourceStatusFlags_MAX
};

enum ERootMotionAccumulateMode
{
	Override,
	Additive,
	ERootMotionAccumulateMode_MAX
};

enum ERuntimeVirtualTextureMaterialType
{
	BaseColor,
	BaseColor_Normal_DEPRECATED,
	BaseColor_Normal_Specular,
	BaseColor_Normal_Specular_YCoCg,
	BaseColor_Normal_Specular_Mask_YCoCg,
	WorldHeight,
	Count,
	ERuntimeVirtualTextureMaterialType_MAX
};

enum EMobilePixelProjectedReflectionQuality
{
	Disabled,
	BestPerformance,
	BetterQuality,
	BestQuality,
	EMobilePixelProjectedReflectionQuality_MAX
};

enum EMobilePlanarReflectionMode
{
	Usual,
	MobilePPRExclusive,
	MobilePPR,
	EMobilePlanarReflectionMode_MAX
};

enum EReflectedAndRefractedRayTracedShadows
{
	Disabled,
	Hard_shadows,
	Area_shadows,
	EReflectedAndRefractedRayTracedShadows_MAX
};

enum ERayTracingGlobalIlluminationType
{
	Disabled,
	BruteForce,
	FinalGather,
	ERayTracingGlobalIlluminationType_MAX
};

enum ETranslucencyType
{
	Raster,
	RayTracing,
	ETranslucencyType_MAX
};

enum EReflectionsType
{
	ScreenSpace,
	RayTracing,
	EReflectionsType_MAX
};

enum ELightUnits
{
	Unitless,
	Candelas,
	Lumens,
	ELightUnits_MAX
};

enum EBloomMethod
{
	BM_SOG,
	BM_FFT,
	BM_MAX
};

enum EAutoExposureMethod
{
	AEM_Histogram,
	AEM_Basic,
	AEM_Manual,
	AEM_MAX
};

enum EAntiAliasingMethod
{
	AAM_None,
	AAM_FXAA,
	AAM_TemporalAA,
	AAM_MSAA,
	AAM_MAX
};

enum EDepthOfFieldMethod
{
	DOFM_BokehDOF,
	DOFM_Gaussian,
	DOFM_CircleDOF,
	DOFM_MAX
};

enum ESceneCapturePrimitiveRenderMode
{
	PRM_LegacySceneCapture,
	PRM_RenderScenePrimitives,
	PRM_UseShowOnlyList,
	PRM_MAX
};

enum EMaterialProperty
{
	MP_EmissiveColor,
	MP_Opacity,
	MP_OpacityMask,
	MP_DiffuseColor,
	MP_SpecularColor,
	MP_BaseColor,
	MP_Metallic,
	MP_Specular,
	MP_Roughness,
	MP_Anisotropy,
	MP_Normal,
	MP_Tangent,
	MP_WorldPositionOffset,
	MP_WorldDisplacement,
	MP_TessellationMultiplier,
	MP_SubsurfaceColor,
	MP_CustomData0,
	MP_CustomData1,
	MP_AmbientOcclusion,
	MP_Refraction,
	MP_CustomizedUVs0,
	MP_CustomizedUVs1,
	MP_CustomizedUVs2,
	MP_CustomizedUVs3,
	MP_CustomizedUVs4,
	MP_CustomizedUVs5,
	MP_CustomizedUVs6,
	MP_CustomizedUVs7,
	MP_PixelDepthOffset,
	MP_ShadingModel,
	MP_MaterialAttributes,
	MP_CustomOutput,
	MP_MAX
};

enum ESkinCacheDefaultBehavior
{
	Exclusive,
	Inclusive,
	ESkinCacheDefaultBehavior_MAX
};

enum ESkinCacheUsage
{
	Auto,
	Disabled,
	Enabled,
	ESkinCacheUsage_MAX
};

enum EPhysicsTransformUpdateMode
{
	SimulationUpatesComponentTransform,
	ComponentTransformIsKinematic,
	EPhysicsTransformUpdateMode_MAX
};

enum EAnimationMode
{
	AnimationBlueprint,
	AnimationSingleNode,
	AnimationCustomMode,
	EAnimationMode_MAX
};

enum EKinematicBonesUpdateToPhysics
{
	SkipSimulatingBones,
	SkipAllBones,
	EKinematicBonesUpdateToPhysics_MAX
};

enum ECustomBoneAttributeLookup
{
	BoneOnly,
	ImmediateParent,
	ParentHierarchy,
	ECustomBoneAttributeLookup_MAX
};

enum EAnimCurveType
{
	AttributeCurve,
	MaterialCurve,
	MorphTargetCurve,
	MaxAnimCurveType,
	EAnimCurveType_MAX
};

enum ESkeletalMeshSkinningImportVersions
{
	Before_Versionning,
	SkeletalMeshBuildRefactor,
	VersionPlusOne,
	LatestVersion,
	ESkeletalMeshSkinningImportVersions_MAX
};

enum ESkeletalMeshGeoImportVersions
{
	Before_Versionning,
	SkeletalMeshBuildRefactor,
	VersionPlusOne,
	LatestVersion,
	ESkeletalMeshGeoImportVersions_MAX
};

enum EBoneFilterActionOption
{
	Remove,
	Keep,
	Invalid,
	EBoneFilterActionOption_MAX
};

enum ESkeletalMeshOptimizationImportance
{
	SMOI_Off,
	SMOI_Lowest,
	SMOI_Low,
	SMOI_Normal,
	SMOI_High,
	SMOI_Highest,
	SMOI_MAX
};

enum ESkeletalMeshOptimizationType
{
	SMOT_NumOfTriangles,
	SMOT_MaxDeviation,
	SMOT_TriangleOrDeviation,
	SMOT_MAX
};

enum ESkeletalMeshTerminationCriterion
{
	SMTC_NumOfTriangles,
	SMTC_NumOfVerts,
	SMTC_TriangleOrVert,
	SMTC_AbsNumOfTriangles,
	SMTC_AbsNumOfVerts,
	SMTC_AbsTriangleOrVert,
	SMTC_MAX
};

enum EBoneTranslationRetargetingMode
{
	Animation,
	Skeleton,
	AnimationScaled,
	AnimationRelative,
	OrientAndScale,
	EBoneTranslationRetargetingMode_MAX
};

enum EVertexOffsetUsageType
{
	None,
	PreSkinningOffset,
	PostSkinningOffset,
	EVertexOffsetUsageType_MAX
};

enum EBoneSpaces
{
	WorldSpace,
	ComponentSpace,
	EBoneSpaces_MAX
};

enum EVisibilityBasedAnimTickOption
{
	AlwaysTickPoseAndRefreshBones,
	AlwaysTickPose,
	OnlyTickMontagesWhenNotRendered,
	OnlyTickPoseWhenRendered,
	EVisibilityBasedAnimTickOption_MAX
};

enum EPhysBodyOp
{
	PBO_None,
	PBO_Term,
	PBO_MAX
};

enum EBoneVisibilityStatus
{
	BVS_HiddenByParent,
	BVS_Visible,
	BVS_ExplicitlyHidden,
	BVS_MAX
};

enum ESkyAtmosphereTransformMode
{
	PlanetTopAtAbsoluteWorldOrigin,
	PlanetTopAtComponentTransform,
	PlanetCenterAtComponentTransform,
	ESkyAtmosphereTransformMode_MAX
};

enum ESkyLightSourceType
{
	SLS_CapturedScene,
	SLS_SpecifiedCubemap,
	SLS_MAX
};

enum EPriorityAttenuationMethod
{
	Linear,
	CustomCurve,
	Manual,
	EPriorityAttenuationMethod_MAX
};

enum ESubmixSendMethod
{
	Linear,
	CustomCurve,
	Manual,
	ESubmixSendMethod_MAX
};

enum EReverbSendMethod
{
	Linear,
	CustomCurve,
	Manual,
	EReverbSendMethod_MAX
};

enum EAirAbsorptionMethod
{
	Linear,
	CustomCurve,
	EAirAbsorptionMethod_MAX
};

enum ESoundSpatializationAlgorithm
{
	SPATIALIZATION_Default,
	SPATIALIZATION_HRTF,
	SPATIALIZATION_MAX
};

enum ESoundDistanceCalc
{
	SOUNDDISTANCE_Normal,
	SOUNDDISTANCE_InfiniteXYPlane,
	SOUNDDISTANCE_InfiniteXZPlane,
	SOUNDDISTANCE_InfiniteYZPlane,
	SOUNDDISTANCE_MAX
};

enum EVirtualizationMode
{
	Disabled,
	PlayWhenSilent,
	Restart,
	EVirtualizationMode_MAX
};

enum EConcurrencyVolumeScaleMode
{
	Default,
	Distance,
	Priority,
	EConcurrencyVolumeScaleMode_MAX
};

enum EMaxConcurrentResolutionRule
{
	PreventNew,
	StopOldest,
	StopFarthestThenPreventNew,
	StopFarthestThenOldest,
	StopLowestPriority,
	StopQuietest,
	StopLowestPriorityThenPreventNew,
	Count,
	EMaxConcurrentResolutionRule_MAX
};

enum ESoundGroup
{
	SOUNDGROUP_Default,
	SOUNDGROUP_Effects,
	SOUNDGROUP_UI,
	SOUNDGROUP_Music,
	SOUNDGROUP_Voice,
	SOUNDGROUP_GameSoundGroup1,
	SOUNDGROUP_GameSoundGroup2,
	SOUNDGROUP_GameSoundGroup3,
	SOUNDGROUP_GameSoundGroup4,
	SOUNDGROUP_GameSoundGroup5,
	SOUNDGROUP_GameSoundGroup6,
	SOUNDGROUP_GameSoundGroup7,
	SOUNDGROUP_GameSoundGroup8,
	SOUNDGROUP_GameSoundGroup9,
	SOUNDGROUP_GameSoundGroup10,
	SOUNDGROUP_GameSoundGroup11,
	SOUNDGROUP_GameSoundGroup12,
	SOUNDGROUP_GameSoundGroup13,
	SOUNDGROUP_GameSoundGroup14,
	SOUNDGROUP_GameSoundGroup15,
	SOUNDGROUP_GameSoundGroup16,
	SOUNDGROUP_GameSoundGroup17,
	SOUNDGROUP_GameSoundGroup18,
	SOUNDGROUP_GameSoundGroup19,
	SOUNDGROUP_GameSoundGroup20,
	SOUNDGROUP_MAX
};

enum EModulationRouting
{
	Disable,
	Inherit,
	Override,
	EModulationRouting_MAX
};

enum EModulationParamMode
{
	MPM_Normal,
	MPM_Abs,
	MPM_Direct,
	MPM_MAX
};

enum ESourceBusChannels
{
	Mono,
	Stereo,
	ESourceBusChannels_MAX
};

enum ESourceBusSendLevelControlMethod
{
	Linear,
	CustomCurve,
	Manual,
	ESourceBusSendLevelControlMethod_MAX
};

enum EGainParamMode
{
	Linear,
	Decibels,
	EGainParamMode_MAX
};

enum EAudioSpectrumType
{
	MagnitudeSpectrum,
	PowerSpectrum,
	Decibel,
	EAudioSpectrumType_MAX
};

enum EFFTWindowType
{
	None,
	Hamming,
	Hann,
	Blackman,
	EFFTWindowType_MAX
};

enum EFFTPeakInterpolationMethod
{
	NearestNeighbor,
	Linear,
	Quadratic,
	ConstantQ,
	EFFTPeakInterpolationMethod_MAX
};

enum EFFTSize
{
	DefaultSize,
	Min,
	Small,
	Medium,
	Large,
	VeryLarge,
	Max
};

enum ESubmixSendStage
{
	PostDistanceAttenuation,
	PreDistanceAttenuation,
	ESubmixSendStage_MAX
};

enum ESendLevelControlMethod
{
	Linear,
	CustomCurve,
	Manual,
	ESendLevelControlMethod_MAX
};

enum EAudioRecordingExportType
{
	SoundWave,
	WavFile,
	EAudioRecordingExportType_MAX
};

enum EAudioSpectrumBandPresetType
{
	KickDrum,
	SnareDrum,
	Voice,
	Cymbals,
	EAudioSpectrumBandPresetType_MAX
};

enum ESoundWaveFFTSize
{
	VerySmall_64,
	Small_256,
	Medium_512,
	Large_1024,
	VeryLarge_2048,
	ESoundWaveFFTSize_MAX
};

enum EDecompressionType
{
	DTYPE_Setup,
	DTYPE_Invalid,
	DTYPE_Preview,
	DTYPE_Native,
	DTYPE_RealTime,
	DTYPE_Procedural,
	DTYPE_Xenon,
	DTYPE_Streaming,
	DTYPE_MAX
};

enum ESoundWaveLoadingBehavior
{
	Inherited,
	RetainOnLoad,
	PrimeOnLoad,
	LoadOnDemand,
	ForceInline,
	Uninitialized,
	ESoundWaveLoadingBehavior_MAX
};

enum ESplineCoordinateSpace
{
	Local,
	World,
	ESplineCoordinateSpace_MAX
};

enum ESplinePointType
{
	Linear,
	Curve,
	Constant,
	CurveClamped,
	CurveCustomTangent,
	ESplinePointType_MAX
};

enum ESplineMeshAxis
{
	X,
	Y,
	Z,
	ESplineMeshAxis_MAX
};

enum EOptimizationType
{
	OT_NumOfTriangles,
	OT_MaxDeviation,
	OT_MAX
};

enum EImportanceLevel
{
	IL_Off,
	IL_Lowest,
	IL_Low,
	IL_Normal,
	IL_High,
	IL_Highest,
	TEMP_BROKEN2,
	EImportanceLevel_MAX
};

enum ENormalMode
{
	NM_PreserveSmoothingGroups,
	NM_RecalculateNormals,
	NM_RecalculateNormalsSmooth,
	NM_RecalculateNormalsHard,
	TEMP_BROKEN,
	ENormalMode_MAX
};

enum EStereoLayerShape
{
	SLSH_QuadLayer,
	SLSH_CylinderLayer,
	SLSH_CubemapLayer,
	SLSH_EquirectLayer,
	SLSH_MAX
};

enum EStereoLayerType
{
	SLT_WorldLocked,
	SLT_TrackerLocked,
	SLT_FaceLocked,
	SLT_MAX
};

enum EOpacitySourceMode
{
	OSM_Alpha,
	OSM_ColorBrightness,
	OSM_RedChannel,
	OSM_GreenChannel,
	OSM_BlueChannel,
	OSM_MAX
};

enum ESubUVBoundingVertexCount
{
	BVC_FourVertices,
	BVC_EightVertices,
	BVC_MAX
};

enum EVerticalTextAligment
{
	EVRTA_TextTop,
	EVRTA_TextCenter,
	EVRTA_TextBottom,
	EVRTA_QuadTop,
	EVRTA_MAX
};

enum EHorizTextAligment
{
	EHTA_Left,
	EHTA_Center,
	EHTA_Right,
	EHTA_MAX
};

enum ETextureLossyCompressionAmount
{
	TLCA_Default,
	TLCA_None,
	TLCA_Lowest,
	TLCA_Low,
	TLCA_Medium,
	TLCA_High,
	TLCA_Highest,
	TLCA_MAX
};

enum ETextureCompressionQuality
{
	TCQ_Default,
	TCQ_Lowest,
	TCQ_Low,
	TCQ_Medium,
	TCQ_High,
	TCQ_Highest,
	TCQ_MAX
};

enum ETextureSourceFormat
{
	TSF_Invalid,
	TSF_G8,
	TSF_BGRA8,
	TSF_BGRE8,
	TSF_RGBA16,
	TSF_RGBA16F,
	TSF_RGBA8,
	TSF_RGBE8,
	TSF_G16,
	TSF_MAX
};

enum ETextureSourceArtType
{
	TSAT_Uncompressed,
	TSAT_PNGCompressed,
	TSAT_DDSFile,
	TSAT_MAX
};

enum ETextureMipCount
{
	TMC_ResidentMips,
	TMC_AllMips,
	TMC_AllMipsBiased,
	TMC_MAX
};

enum ECompositeTextureMode
{
	CTM_Disabled,
	CTM_NormalRoughnessToRed,
	CTM_NormalRoughnessToGreen,
	CTM_NormalRoughnessToBlue,
	CTM_NormalRoughnessToAlpha,
	CTM_MAX
};

enum ETextureAddress
{
	TA_Wrap,
	TA_Clamp,
	TA_Mirror,
	TA_MAX
};

enum ETextureFilter
{
	TF_Nearest,
	TF_Bilinear,
	TF_Trilinear,
	TF_Default,
	TF_MAX
};

enum ETextureCompressionSettings
{
	TC_Default,
	TC_Normalmap,
	TC_Masks,
	TC_Grayscale,
	TC_Displacementmap,
	TC_VectorDisplacementmap,
	TC_HDR,
	TC_EditorIcon,
	TC_Alpha,
	TC_DistanceFieldFont,
	TC_HDR_Compressed,
	TC_BC7,
	TC_HalfFloat,
	TC_ReflectionCapture,
	TC_MAX
};

enum ETextureDownscaleOptions
{
	Default,
	Unfiltered,
	SimpleAverage,
	Sharpen0,
	Sharpen1,
	Sharpen2,
	Sharpen3,
	Sharpen4,
	Sharpen5,
	Sharpen6,
	Sharpen7,
	Sharpen8,
	Sharpen9,
	Sharpen10,
	ETextureDownscaleOptions_MAX
};

enum ETextureMipLoadOptions
{
	Default,
	AllMips,
	OnlyFirstMip,
	ETextureMipLoadOptions_MAX
};

enum ETextureSamplerFilter
{
	Point,
	Bilinear,
	Trilinear,
	AnisotropicPoint,
	AnisotropicLinear,
	ETextureSamplerFilter_MAX
};

enum ETexturePowerOfTwoSetting
{
	None,
	PadToPowerOfTwo,
	PadToSquarePowerOfTwo,
	ETexturePowerOfTwoSetting_MAX
};

enum ETextureMipGenSettings
{
	TMGS_FromTextureGroup,
	TMGS_SimpleAverage,
	TMGS_Sharpen0,
	TMGS_Sharpen1,
	TMGS_Sharpen2,
	TMGS_Sharpen3,
	TMGS_Sharpen4,
	TMGS_Sharpen5,
	TMGS_Sharpen6,
	TMGS_Sharpen7,
	TMGS_Sharpen8,
	TMGS_Sharpen9,
	TMGS_Sharpen10,
	TMGS_NoMipmaps,
	TMGS_LeaveExistingMips,
	TMGS_Blur1,
	TMGS_Blur2,
	TMGS_Blur3,
	TMGS_Blur4,
	TMGS_Blur5,
	TMGS_Unfiltered,
	TMGS_MAX
};

enum ETextureGroup
{
	TEXTUREGROUP_World,
	TEXTUREGROUP_WorldNormalMap,
	TEXTUREGROUP_WorldSpecular,
	TEXTUREGROUP_Character,
	TEXTUREGROUP_CharacterNormalMap,
	TEXTUREGROUP_CharacterSpecular,
	TEXTUREGROUP_Weapon,
	TEXTUREGROUP_WeaponNormalMap,
	TEXTUREGROUP_WeaponSpecular,
	TEXTUREGROUP_Vehicle,
	TEXTUREGROUP_VehicleNormalMap,
	TEXTUREGROUP_VehicleSpecular,
	TEXTUREGROUP_Cinematic,
	TEXTUREGROUP_Effects,
	TEXTUREGROUP_EffectsNotFiltered,
	TEXTUREGROUP_Skybox,
	TEXTUREGROUP_UI,
	TEXTUREGROUP_Lightmap,
	TEXTUREGROUP_RenderTarget,
	TEXTUREGROUP_MobileFlattened,
	TEXTUREGROUP_ProcBuilding_Face,
	TEXTUREGROUP_ProcBuilding_LightMap,
	TEXTUREGROUP_Shadowmap,
	TEXTUREGROUP_ColorLookupTable,
	TEXTUREGROUP_Terrain_Heightmap,
	TEXTUREGROUP_Terrain_Weightmap,
	TEXTUREGROUP_Bokeh,
	TEXTUREGROUP_IESLightProfile,
	TEXTUREGROUP_Pixels2D,
	TEXTUREGROUP_HierarchicalLOD,
	TEXTUREGROUP_Impostor,
	TEXTUREGROUP_ImpostorNormalDepth,
	TEXTUREGROUP_8BitData,
	TEXTUREGROUP_16BitData,
	TEXTUREGROUP_Project01,
	TEXTUREGROUP_Project02,
	TEXTUREGROUP_Project03,
	TEXTUREGROUP_Project04,
	TEXTUREGROUP_Project05,
	TEXTUREGROUP_Project06,
	TEXTUREGROUP_Project07,
	TEXTUREGROUP_Project08,
	TEXTUREGROUP_Project09,
	TEXTUREGROUP_Project10,
	TEXTUREGROUP_Project11,
	TEXTUREGROUP_Project12,
	TEXTUREGROUP_Project13,
	TEXTUREGROUP_Project14,
	TEXTUREGROUP_Project15,
	TEXTUREGROUP_Project16,
	TEXTUREGROUP_MAX
};

enum ETextureRenderTargetFormat
{
	RTF_R8,
	RTF_RG8,
	RTF_RGBA8,
	RTF_RGBA8_SRGB,
	RTF_R16f,
	RTF_RG16f,
	RTF_RGBA16f,
	RTF_R32f,
	RTF_RG32f,
	RTF_RGBA32f,
	RTF_RGB10A2,
	RTF_MAX
};

enum ETimecodeProviderSynchronizationState
{
	Closed,
	Error,
	Synchronized,
	Synchronizing,
	ETimecodeProviderSynchronizationState_MAX
};

enum ETimelineDirection
{
	Forward,
	Backward,
	ETimelineDirection_MAX
};

enum ETimelineLengthMode
{
	TL_TimelineLength,
	TL_LastKeyFrame,
	TL_MAX
};

enum ETimeStretchCurveMapping
{
	T_Original,
	T_TargetMin,
	T_TargetMax,
	MAX
};

enum ETwitterIntegrationDelegate
{
	TID_AuthorizeComplete,
	TID_TweetUIComplete,
	TID_RequestComplete,
	TID_MAX
};

enum ETwitterRequestMethod
{
	TRM_Get,
	TRM_Post,
	TRM_Delete,
	TRM_MAX
};

enum EUserDefinedStructureStatus
{
	UDSS_UpToDate,
	UDSS_Dirty,
	UDSS_Error,
	UDSS_Duplicate,
	UDSS_MAX
};

enum EUIScalingRule
{
	ShortestSide,
	LongestSide,
	Horizontal,
	Vertical,
	ScaleToFit,
	Custom,
	EUIScalingRule_MAX
};

enum ERenderFocusRule
{
	Always,
	NonPointer,
	NavigationOnly,
	Never,
	ERenderFocusRule_MAX
};

enum EVectorFieldConstructionOp
{
	VFCO_Extrude,
	VFCO_Revolve,
	VFCO_MAX
};

enum EWindSourceType
{
	Directional,
	Point,
	EWindSourceType_MAX
};

enum EPSCPoolMethod
{
	None,
	AutoRelease,
	ManualRelease,
	ManualRelease_OnComplete,
	FreeInPool,
	EPSCPoolMethod_MAX
};

enum EVolumeLightingMethod
{
	VLM_VolumetricLightmap,
	VLM_SparseVolumeLightingSamples,
	VLM_MAX
};

enum EVisibilityAggressiveness
{
	VIS_LeastAggressive,
	VIS_ModeratelyAggressive,
	VIS_MostAggressive,
	VIS_Max
};

enum EEntityActorReplicationRelevancyBucketType
{
	UseVisualCullDistanceForReplicationRelevancy,
	SmallReplicationRelevancy,
	MediumReplicationRelevancy,
	LargeReplicationRelevancy,
	MaxTargetRangeReplicationRelevancy,
	ImportantReplicationRelevancy,
	CustomReplicationRelevancy,
	EEntityActorReplicationRelevancyBucketType_MAX
};

enum EEntityActorReplicationOverrideType
{
	AutoReplication,
	DoNotReplicate,
	ReplicateAlways,
	Static_Spatial,
	Dynamic_Spatial,
	Dormancy_Spatial,
	EEntityActorReplicationOverrideType_MAX
};

enum EDefaultAnimationMode
{
	UseAnimationBlueprint,
	UseAnimationAsset,
	UseCustomMode,
	EDefaultAnimationMode_MAX
};

enum EDefaultStaticMesh
{
	Cube,
	Sphere,
	Cylinder,
	Cone,
	Plane,
	EDefaultStaticMesh_MAX
};

enum ESimObjectRepNodeMapping
{
	NotReplicated,
	RelevantAllInsidePlayspace,
	Spatial_Static,
	Spatial_Dynamic,
	Spatial_Dormancy,
	ESimObjectRepNodeMapping_MAX
};

enum EMovementType
{
	SweepPhysics,
	TeleportPhysics,
	ResetPhysics,
	EMovementType_MAX
};

enum EScriptDiagnosticMessageType
{
	Debug,
	Verbose,
	Normal,
	Warning,
	Error,
	Fatal,
	EScriptDiagnosticMessageType_MAX
};

enum EDateType
{
	None,
	Coming,
	Ending,
	EDateType_MAX
};

enum EEpicLeaderboardUpdateFunction
{
	Min,
	Max,
	Sum,
	MostRecent
};

enum EEpicLeaderboardTimeWindow
{
	Daily,
	Weekly,
	Monthly,
	AllTime,
	EEpicLeaderboardTimeWindow_MAX
};

enum EEpicLeaderboardDataType
{
	Integer,
	Double,
	EEpicLeaderboardDataType_MAX
};

enum EFortEventModeEmoteToPlay
{
	Emote1,
	Emote2,
	Emote3,
	RandomEmote,
	QuickEmotes,
	EFortEventModeEmoteToPlay_MAX
};

enum EEyeTrackerStatus
{
	NotConnected,
	NotTracking,
	Tracking,
	EEyeTrackerStatus_MAX
};

enum EFoliageScaling
{
	Uniform,
	Free,
	LockXY,
	LockXZ,
	LockYZ,
	EFoliageScaling_MAX
};

enum EVertexColorMaskChannel
{
	Red,
	Green,
	Blue,
	Alpha,
	MAX_None,
	EVertexColorMaskChannel_MAX
};

enum EFoliageVertexColorMask
{
	FOLIAGEVERTEXCOLORMASK_Disabled,
	FOLIAGEVERTEXCOLORMASK_Red,
	FOLIAGEVERTEXCOLORMASK_Green,
	FOLIAGEVERTEXCOLORMASK_Blue,
	FOLIAGEVERTEXCOLORMASK_Alpha,
	FOLIAGEVERTEXCOLORMASK_MAX
};

enum ESimulationQuery
{
	None,
	CollisionOverlap,
	ShadeOverlap,
	AnyOverlap,
	ESimulationQuery_MAX
};

enum ESimulationOverlap
{
	CollisionOverlap,
	ShadeOverlap,
	None,
	ESimulationOverlap_MAX
};

enum EInteractionRange
{
	Preview,
	Interaction,
	EInteractionRange_MAX
};

enum ECannotBuyReason
{
	CannotAfford,
	OutOfStock,
	ECannotBuyReason_MAX
};

enum ELinkToDirection
{
	Up,
	Down,
	Right,
	Left,
	Forward,
	Backward,
	ELinkToDirection_MAX
};

enum EFortDayPhase
{
	Morning,
	Day,
	Evening,
	Night,
	NumPhases,
	EFortDayPhase_MAX
};

enum EBuildingReplacementType
{
	BRT_None,
	BRT_Edited,
	BRT_Upgrade,
	BRT_MAX
};

enum EFortCustomPartType
{
	Head,
	Body,
	Hat,
	Backpack,
	MiscOrTail,
	Face,
	Gameplay,
	NumTypes,
	EFortCustomPartType_MAX
};

enum EFortDBNOCarryEvent
{
	PickedUp,
	Interrogating,
	Dropped,
	Thrown,
	EFortDBNOCarryEvent_MAX
};

enum EInteractionBeingAttempted
{
	FirstInteraction,
	SecondInteraction,
	AllInteraction,
	EInteractionBeingAttempted_MAX
};

enum EFortAIDirectorEvent
{
	PlayerAIEnemies,
	PlayerTakeDamage,
	PlayerHealth,
	PlayerDeath,
	PlayerLookAtAIEnemy,
	PlayerDamageAIEnemy,
	PlayerKillAIEnemy,
	PlayerHealingPotential,
	PlayerAmmoLight,
	PlayerAmmoMedium,
	PlayerAmmoHeavy,
	PlayerAmmoShells,
	PlayerAmmoEnergy,
	PlayerAINear,
	PlayerMovement,
	ObjectiveTakeDamage,
	ObjectiveHealth,
	ObjectiveDestroyed,
	TrapFired,
	TrapDamagedAIEnemy,
	ObjectivePathCost,
	PlayerPathCost,
	ObjectiveNearbyBuildingDamaged,
	Max_None,
	EFortAIDirectorEvent_MAX
};

enum EFortWeaponReloadType
{
	ReloadWholeClip,
	ReloadIndividualBullets,
	ReloadBasedOnAmmoCostPerFire,
	ReloadBasedOnCartridgePerFire,
	EFortWeaponReloadType_MAX
};

enum EFortInventoryType
{
	World,
	Account,
	Outpost,
	MAX
};

enum EOfferPurchaseError
{
	NoError,
	PendingServerConfirmation,
	CannotAffordItem,
	InvalidOfferID,
	InvalidPriceIndex,
	NoneLeft,
	PurchaseAlreadyPending,
	NoConnection,
	AccountNotEligible,
	CannotGiftItem,
	MFANotEnabled,
	EOfferPurchaseError_MAX
};

enum EFortAlteration
{
	AttributeSlot,
	GameplaySlot,
	ComplexCosmeticSlot,
	UserPickedCosmeticSlot,
	ColorSlot,
	HeroSpecializationTier1Slot,
	HeroSpecializationTier2Slot,
	HeroSpecializationTier3Slot,
	HeroSpecializationTier4Slot,
	HeroSpecializationTier5Slot,
	EFortAlteration_MAX
};

enum EPlayerCompetitiveBanReasons
{
	None,
	Cheating,
	Collusion,
	Toxicity,
	Harassment,
	Ringing,
	Gambling,
	Exploiting,
	IntentionalDisconnect,
	IllegalRestart,
	Other,
	AccountSharing,
	CircumventingRegionLock,
	CircumventingBan,
	Smurfing,
	CircumventingTeamLock,
	EPlayerCompetitiveBanReasons_MAX
};

enum EPlayerBanReasons
{
	Teaming,
	Afk_Leeching,
	Harassment,
	TradeScamming,
	Exploiting,
	Competitive,
	Creative,
	TeamingWithCheater,
	EPlayerBanReasons_MAX
};

enum EFortItemType
{
	WorldItem,
	Ammo,
	Badge,
	BackpackPickup,
	BuildingPiece,
	CharacterPart,
	Consumable,
	Deco,
	EditTool,
	Ingredient,
	ItemCache,
	Food,
	Gadget,
	AthenaGadget,
	HomebaseGadget,
	BattleLabDevice,
	SpyTechPerk,
	HeroAbility,
	MissionItem,
	Trap,
	MultiItem,
	Weapon,
	WeaponMelee,
	WeaponRanged,
	WeaponHarvest,
	WeaponCreativePhone,
	WeaponMod,
	WorldResource,
	CreativeUserPrefab,
	CreativePlayset,
	Vehicle,
	Npc,
	AccountItem,
	AccountResource,
	CollectedResource,
	Alteration,
	CardPack,
	Currency,
	Hero,
	Schematic,
	Worker,
	TeamPerk,
	PlayerTech,
	Token,
	DailyRewardScheduleToken,
	CodeToken,
	Stat,
	Buff,
	BuffCredit,
	Quest,
	Accolades,
	FriendChest,
	MedalsPunchCard,
	RepeatableDailiesCard,
	ChallengeBundle,
	ChallengeBundleSchedule,
	ChallengeBundleCompletionToken,
	GameplayModifier,
	Outpost,
	HomebaseNode,
	Defender,
	ConversionControl,
	DeployableBaseCloudSave,
	ConsumableAccountItem,
	Quota,
	Expedition,
	HomebaseBannerIcon,
	HomebaseBannerColor,
	AthenaSkyDiveContrail,
	PersonalVehicle,
	AthenaGlider,
	AthenaPickaxe,
	AthenaHat,
	AthenaBackpack,
	AthenaCharacter,
	AthenaDance,
	AthenaConsumableEmote,
	AthenaLoadingScreen,
	AthenaBattleBus,
	AthenaVehicleCosmetic,
	AthenaItemWrap,
	AthenaCallingCard,
	AthenaMapMarker,
	AthenaMusicPack,
	AthenaPetCosmetic,
	AthenaCharmCosmetic,
	AthenaVictoryPose,
	AthenaSeasonTreasure,
	AthenaSeason,
	AthenaRewardGraph,
	AthenaExtResource,
	EventDescription,
	BattleLabDeviceAccount,
	MatchAward,
	AthenaEventToken,
	EventPurchaseTracker,
	CosmeticVariantToken,
	CampaignHeroLoadout,
	Playset,
	PrerollData,
	CreativePlot,
	PlayerSurveyToken,
	CosmeticLocker,
	BannerToken,
	RestedXpBoosterToken,
	RewardEventGraphPurchaseToken,
	HardcoreModifier,
	EventDependentItem,
	ItemAccessToken,
	SpecialItem,
	Emote,
	Stack,
	CollectionBookPage,
	BGAConsumableWrapper,
	GiftBox,
	GiftBoxUnlock,
	PlaysetProp,
	RegCosmeticDef,
	Profile,
	Max_None,
	EFortItemType_MAX
};

enum EAthenaTravelLogPlayerType
{
	Self,
	Ally,
	Enemy,
	Invalid,
	EAthenaTravelLogPlayerType_MAX
};

enum EAthenaTravelEventType
{
	GroundMove,
	AirMove,
	BattleBusJump,
	LaunchJump,
	Landed,
	OpenChest,
	OpenAmmo,
	GotAssist,
	GotKnockdown,
	GotKill,
	PlayerDowned,
	PlayerDied,
	Won,
	DealtDamage,
	HealthChange,
	GotItem,
	DroppedItem,
	ShieldChange,
	WeaponExecuted,
	EnteredVehicle,
	ExitedVehicle,
	TrapBuilt,
	UsedItem,
	ZoneUpdate,
	PlacedBuilding,
	EmoteUsed,
	UpgradedBuilding,
	EditedBuilding,
	Count,
	EAthenaTravelEventType_MAX
};

enum ERewardSource
{
	Invalid,
	MinutesPlayed,
	FirstKill,
	TeamKills,
	FirstRevive,
	AdditionalRevives,
	Placement,
	Medals,
	FirstWin,
	SeasonLevelUp,
	BookLevelUp,
	MatchXP,
	MAX_COUNT,
	ERewardSource_MAX
};

enum EAthenaMatchXpMultiplierSource
{
	Invalid,
	BattlePassSelf,
	BattlePassFriends,
	CosmeticSet,
	AntiAddictionPenalty,
	BonusXpEvent,
	MAX_COUNT,
	EAthenaMatchXpMultiplierSource_MAX
};

enum EFortAccoladeSubtype
{
	NotSet,
	Action,
	Discovery,
	XpToken,
	EFortAccoladeSubtype_MAX
};

enum EAlertLevel
{
	Unaware,
	Alerted,
	LKP,
	Threatened,
	Count,
	EAlertLevel_MAX
};

enum EFortBudgetCategory
{
	Memory,
	Simulation,
	Rendering,
	Network,
	Audio,
	Max
};

enum ESpatialLoadingState
{
	Uninitialized,
	ReadOnly,
	Initializing,
	Ready,
	ESpatialLoadingState_MAX
};

enum EAthenaGameMsgType
{
	None,
	DefaultIntro,
	DefaultMessage,
	DefaultCriticalMessage,
	CommIntro,
	CommMessage,
	CommCriticalMessage,
	CornerIntro,
	CornerMessage,
	CornerCriticalMessage,
	RespawnTurningOffWarning,
	RespawnOffWarning,
	CenterMessage,
	CenterImportantMessage,
	CenterCriticalMessage,
	EAthenaGameMsgType_MAX
};

enum EHUDMessagePlacement
{
	None,
	BottomCenter,
	TopCenter,
	CenterRight,
	EHUDMessagePlacement_MAX
};

enum EFortVolumeType
{
	None,
	Island,
	Published,
	Featured,
	Prefab,
	Hub,
	EFortVolumeType_MAX
};

enum EVolumeShape
{
	Sphere,
	Box,
	EVolumeShape_MAX
};

enum EFortBuildingType
{
	Wall,
	Floor,
	Corner,
	Deco,
	Prop,
	Stairs,
	Roof,
	Pillar,
	SpawnedItem,
	Container,
	Trap,
	GenericCenterCellActor,
	None,
	EFortBuildingType_MAX
};

enum EFortItemEntryState
{
	NoneState,
	NewItemCount,
	ShouldShowItemToast,
	DurabilityInitialized,
	DoNotShowSpawnParticles,
	FromRecoveredBackpack,
	FromGift,
	PendingUpgradeCriteriaProgress,
	OwnerBuildingHandle,
	FromDroppedPickup,
	JustCrafted,
	CraftAndSlotTarget,
	GenericAttributeValueSet,
	PickupInstigatorHandle,
	RechargingWeaponServerTime,
	DisallowSwapOnNextPickUpAttempt,
	DroppedFromQuestSource,
	Tossed,
	EFortItemEntryState_MAX
};

enum EFortQuickBars
{
	Primary,
	Secondary,
	Creative,
	Max_None,
	EFortQuickBars_MAX
};

enum EAthenaCustomizationCategory
{
	None,
	Glider,
	Pickaxe,
	Hat,
	Backpack,
	Character,
	LoadingScreen,
	BattleBus,
	VehicleDecoration,
	CallingCard,
	MapMarker,
	Dance,
	ConsumableEmote,
	VictoryPose,
	SkyDiveContrail,
	MusicPack,
	ItemWrap,
	PetSkin,
	Charm,
	RegCosmeticDef,
	Loadout,
	SaveLoadout,
	MAX
};

enum EFortObjectiveStatus
{
	Created,
	InProgress,
	Succeeded,
	Failed,
	NeutralCompletion,
	Max_None,
	EFortObjectiveStatus_MAX
};

enum EStatCategory
{
	Combat,
	Building,
	Utility,
	Max_None,
	EStatCategory_MAX
};

enum EFortResourceType
{
	Wood,
	Stone,
	Metal,
	Permanite,
	GoldCurrency,
	Ingredient,
	None,
	EFortResourceType_MAX
};

enum ERichPresenceStateChange
{
	AutoUpdate,
	Idle,
	Active,
	Busy,
	NotBusy,
	ERichPresenceStateChange_MAX
};

enum ESubGame
{
	Campaign,
	Athena,
	Invalid,
	Count,
	Creative,
	ESubGame_MAX
};

enum ELockOnState
{
	NoTarget,
	TargetAcquired,
	LockingOnToTarget,
	TargetLockedOn,
	Cooldown,
	COUNT,
	ELockOnState_MAX
};

enum EStatRecordingPeriod
{
	Frame,
	Minute,
	AthenaSafeZonePhase,
	Life,
	Map,
	Campaign,
	Persistent,
	Max
};

enum EStatMod
{
	Delta,
	Set,
	Maximum,
	EStatMod_MAX
};

enum EFortDamageZone
{
	Light,
	Normal,
	Critical,
	Vulnerability,
	Max
};

enum EFortEmotePlayMode
{
	CheckIfOwned,
	ForcePlay,
	EFortEmotePlayMode_MAX
};

enum EFortItemTier
{
	No_Tier,
	I,
	II,
	III,
	IV,
	V,
	VI,
	VII,
	VIII,
	IX,
	X,
	NumItemTierValues,
	EFortItemTier_MAX
};

enum EFortResourceLevel
{
	First,
	Second,
	Third,
	Fourth,
	Fifth,
	Sixth,
	NumLevels,
	Invalid,
	EFortResourceLevel_MAX
};

enum EFortRequestedGameplayAction
{
	ContinuePlaying,
	StartPlaying,
	StopPlaying,
	EnterZone,
	LeaveZone,
	ReturnToMainMenu,
	QuitGame,
	Invalid,
	EFortRequestedGameplayAction_MAX
};

enum EFortGameplayState
{
	NormalGameplay,
	WaitingToStart,
	EndOfZone,
	EnteringZone,
	LeavingZone,
	Invalid,
	EFortGameplayState_MAX
};

enum EBuildingHighlightType
{
	Primary,
	Interact,
	WillBeDestroyed,
	Quest,
	AuxiliaryInterestPoint,
	MAX_None,
	EBuildingHighlightType_MAX
};

enum EFortCollectedVariantClientUpdate
{
	NewVariant,
	CollectedCount,
	Improvement,
	HiddenImprovement,
	EFortCollectedVariantClientUpdate_MAX
};

enum EFortCollectedState
{
	Unknown,
	New,
	Known,
	NewlyCollected,
	Collected,
	NewBest,
	NewRecord,
	NewLocation,
	NewlyCompleted,
	Complete,
	EFortCollectedState_MAX
};

enum EFortPawnStasisMode
{
	None,
	NoMovement,
	NoMovementOrTurning,
	NoMovementOrFalling,
	NoMovement_EmotesEnabled,
	NoMovementOrTurning_EmotesEnabled,
	NoMovementOrFalling_EmotesEnabled,
	EFortPawnStasisMode_MAX
};

enum EFortJumpStaminaCost
{
	None,
	Trigger,
	SprintTrigger,
	SprintAir,
	EFortJumpStaminaCost_MAX
};

enum EFortPCTutorialCompletedState
{
	Unknown,
	Incomplete,
	Complete,
	EFortPCTutorialCompletedState_MAX
};

enum EFortCompletionResult
{
	Win,
	Loss,
	Draw,
	Undefined,
	EFortCompletionResult_MAX
};

enum EFortRewardActivityType
{
	General,
	MissionPrimary,
	MissionSecondary,
	AccountLevelUp,
	Max_None,
	EFortRewardActivityType_MAX
};

enum EFortDelayedQuickBarAction
{
	Add,
	Remove,
	Replace,
	Invalid,
	EFortDelayedQuickBarAction_MAX
};

enum EFortElementalDamageType
{
	None,
	Fire,
	Ice,
	Lightning,
	Energy,
	MAX
};

enum EFortDamageNumberType
{
	None,
	Pawn,
	Building,
	Player,
	Shield,
	Score,
	DBNO,
	Percent,
	EFortDamageNumberType_MAX
};

enum EFortCostInfoTypes
{
	Placement,
	Repair,
	Conversion,
	Ability,
	None,
	EFortCostInfoTypes_MAX
};

enum EFortBuildPreviewMarkerOptionalAdjustment
{
	None,
	FreeWallPieceOnTop,
	FreeWallPieceOnBottom,
	EFortBuildPreviewMarkerOptionalAdjustment_MAX
};

enum EFortVoteStatus
{
	Begin,
	Update,
	End,
	BeginLockout,
	EndLockout,
	EFortVoteStatus_MAX
};

enum EFortVoteType
{
	SurvivalVote,
	DifficultyIncrease,
	MissionActivation,
	ContinueOrExtract,
	EFortVoteType_MAX
};

enum EAthenaGamePhase
{
	None,
	Setup,
	Warmup,
	Aircraft,
	SafeZones,
	EndGame,
	Count,
	EAthenaGamePhase_MAX
};

enum EAthenaGamePhaseStep
{
	None,
	Setup,
	Warmup,
	GetReady,
	BusLocked,
	BusFlying,
	StormForming,
	StormHolding,
	StormShrinking,
	Countdown,
	FinalCountdown,
	EndGame,
	Count,
	EAthenaGamePhaseStep_MAX
};

enum EAthenaStormCapState
{
	None,
	Clear,
	Warning,
	Damaging,
	EAthenaStormCapState_MAX
};

enum EFortIsFinalXpUpdate
{
	Uninitialized,
	NotFinal,
	Final,
	EFortIsFinalXpUpdate_MAX
};

enum EFortTeam
{
	Spectator,
	HumanCampaign,
	Monster,
	HumanPvP_Team1,
	HumanPvP_Team2,
	MAX
};

enum EFortVoteArbitratorType
{
	Invalid,
	Majority,
	Unanimous,
	EFortVoteArbitratorType_MAX
};

enum EWaveRules
{
	KillAllEnemies,
	Timed,
	KillPoints,
	KillSpecificEnemy,
	Mission,
	EWaveRules_MAX
};

enum EServerStability
{
	Stable,
	LowUnstability,
	HighUnstability,
	Count,
	EServerStability_MAX
};

enum EDBNOMutatorType
{
	Default,
	On,
	Off,
	EDBNOMutatorType_MAX
};

enum EFortPlaylistType
{
	Default,
	Playground,
	Creative,
	Creative_LTM,
	BattleLab,
	EFortPlaylistType_MAX
};

enum EMapLocationStateType
{
	Normal,
	Golden,
	Undiscovered,
	Max
};

enum ESpawnMachineState
{
	Default,
	WaitingForUse,
	Active,
	Complete,
	OnCooldown,
	ESpawnMachineState_MAX
};

enum ESafeZoneStartUp
{
	UseDefaultGameBehavior,
	StartsWithWarmUp,
	StartsWithAirCraft,
	StartsWithNoAirCraft,
	ESafeZoneStartUp_MAX
};

enum EEventTournamentRound
{
	Open,
	Qualifiers,
	SemiFinals,
	Finals,
	Unknown,
	Arena,
	EEventTournamentRound_MAX
};

enum EFriendlyFireType
{
	Off,
	On,
	EFriendlyFireType_MAX
};

enum EAirCraftBehavior
{
	Default,
	OpposingAirCraftForEachTeam,
	FlyTowardFirstCircleCenter,
	NoAircraft,
	EAirCraftBehavior_MAX
};

enum EAthenaScoringEvent
{
	None,
	Elimination,
	ChestOpened,
	AmmoCanOpened,
	SupplyDropOpened,
	SupplyLlamaOpened,
	ForagedItemConsumed,
	SurvivalInMinutes,
	CollectedCoinBronze,
	CollectedCoinSilver,
	CollectedCoinGold,
	AIKilled,
	EAthenaScoringEvent_MAX
};

enum EMatchAbandonState
{
	None,
	Joining,
	CanAbandon,
	TeamLocked,
	EMatchAbandonState_MAX
};

enum ETrustedPlatformType
{
	Unknown,
	Untrusted,
	PS4,
	PS5,
	XboxOne,
	Switch,
	ETrustedPlatformType_MAX
};

enum EReadyCheckState
{
	CheckStarted,
	Ready,
	NotReady,
	EReadyCheckState_MAX
};

enum EFortAppliedSwapItemAndVariantState
{
	None,
	Active,
	Inactive,
	EFortAppliedSwapItemAndVariantState_MAX
};

enum EFortCustomBodyType
{
	NONE,
	Small,
	Medium,
	MediumAndSmall,
	Large,
	LargeAndSmall,
	LargeAndMedium,
	All,
	Deprecated,
	EFortCustomBodyType_MAX
};

enum EFortCustomGender
{
	Invalid,
	Male,
	Female,
	Both,
	EFortCustomGender_MAX
};

enum EFortPlayerRole
{
	Player,
	LiveSpectator,
	ReplaySpectator,
	InactivePlayer,
	EFortPlayerRole_MAX
};

enum EFortKickReason
{
	NotKicked,
	GenericKick,
	WasBanned,
	EncryptionRequired,
	CrossPlayRestriction,
	ClientIdRestriction,
	EFortKickReason_MAX
};

enum EMessageDispatcherErrorMessageType
{
	FailedToSetTrigger_TooManyTriggers,
	FailedToSetReceiver_TooManyReceivers,
	FailedToSetReceiver_TooManyReceiversOnOneChannel,
	FailedToSetTriggerReceiver_InvalidChannel,
	FailedToEnqueueMessage,
	BuildLimitReached,
	UnknownError,
	EMessageDispatcherErrorMessageType_MAX
};

enum EDeathCause
{
	OutsideSafeZone,
	FallDamage,
	Pistol,
	Shotgun,
	Rifle,
	SMG,
	Sniper,
	SniperNoScope,
	Melee,
	InfinityBlade,
	Grenade,
	C4,
	GrenadeLauncher,
	RocketLauncher,
	Minigun,
	Bow,
	Trap,
	DBNOTimeout,
	Banhammer,
	RemovedFromGame,
	MassiveMelee,
	MassiveDiveBomb,
	MassiveRanged,
	Vehicle,
	ShoppingCart,
	ATK,
	QuadCrasher,
	Biplane,
	BiplaneGun,
	LMG,
	GasGrenade,
	InstantEnvironmental,
	InstantEnvironmentalFellOutOfWorld,
	InstantEnvironmentalUnderLandscape,
	Turret,
	ShipCannon,
	Cube,
	Balloon,
	StormSurge,
	Lava,
	BasicFiend,
	EliteFiend,
	RangedFiend,
	BasicBrute,
	EliteBrute,
	MegaBrute,
	SilentSwitchingToSpectate,
	LoggedOut,
	TeamSwitchSuicide,
	WonMatch,
	Unspecified,
	EDeathCause_MAX
};

enum EKeepPlayingTogetherVotingStatus
{
	Undecided,
	OptedIn,
	OptedOut_Manual,
	OptedOut_Forced,
	OptedOut_TimedOut,
	EKeepPlayingTogetherVotingStatus_MAX
};

enum ETeamMemberState
{
	None,
	FIRST_CHAT_MESSAGE,
	NeedAmmoHeavy,
	NeedAmmoLight,
	NeedAmmoMedium,
	NeedAmmoShells,
	NeedAmmoRocket,
	ChatBubble,
	EnemySpotted,
	NeedBandages,
	NeedMaterials,
	NeedShields,
	NeedWeapon,
	LAST_CHAT_MESSAGE,
	MAX
};

enum EFortWeaponOverheatState
{
	None,
	Heating,
	Cooling,
	Overheated,
	EFortWeaponOverheatState_MAX
};

enum EDialogMarkerInteractionState
{
	Conversation,
	InteractionRange,
	IndicatorRange,
	None,
	EDialogMarkerInteractionState_MAX
};

enum EFortReportDayPhase
{
	Dawn,
	Dusk,
	ZoneFinished,
	PlayerLogout,
	EFortReportDayPhase_MAX
};

enum EXPEventPriorityType
{
	NearReticle,
	XPBarOnly,
	TopCenter,
	Feed,
	EXPEventPriorityType_MAX
};

enum EMatchmakingCancelReasonV2
{
	Explicit,
	JoinedParty,
	LeftParty,
	PartyMemberJoined,
	PartyMemberLeft,
	PartyMemberCanceled,
	PartyLeaderSwap,
	PlayReplay,
	UIDestroyed,
	PCDestroyed,
	AppBackgrounded,
	HotfixOutdated,
	TournamentOver,
	NotInParty,
	CrossplayBlocked,
	TournamentCrossplayBlocked,
	AccountLevelTooLow,
	FillNoCrossplay,
	CreativeCanceledByLeader,
	CreativeMemberLeftIsland,
	CreativeIslandStateChanged,
	JoinInProgressRejected,
	MatchmakingDisabled,
	NotLoggedIn,
	NoIdentityInterface,
	NoSessionInterface,
	TimedOut,
	InvalidPlaylist,
	AttemptedToQueueTooFrequently,
	TournamentBlackout,
	CellularDataRefusal,
	CancelledDownloadContent,
	Unknown,
	EMatchmakingCancelReasonV2_MAX
};

enum EPlayerControllerFollow
{
	NextTeammate,
	PreviousTeammate,
	NextPlayer,
	PreviousPlayer,
	SpecialActor,
	EPlayerControllerFollow_MAX
};

enum EFortQuickTimeEventResult
{
	None,
	Miss,
	Good,
	Great,
	Perfect,
	EFortQuickTimeEventResult_MAX
};

enum EVehicleSeats
{
	Driver,
	Passenger1,
	Passenger2,
	Passenger3,
	Passenger4,
	Passenger5,
	MaxCount,
	EVehicleSeats_MAX
};

enum ELeecherStatus
{
	NotReady,
	NonLeecher,
	Leecher,
	ELeecherStatus_MAX
};

enum EEndOfMatchReason
{
	LastManStanding,
	ScoreReached,
	TimeRanOut,
	WinEventOccurred,
	AllLoggedOut,
	AllEliminated,
	EEndOfMatchReason_MAX
};

enum EFortMarkedActorScreenClamping
{
	Default,
	Clamp,
	ClampWhileNew,
	DontClamp,
	EFortMarkedActorScreenClamping_MAX
};

enum EFortWorldMarkerType
{
	None,
	Location,
	Enemy,
	Item,
	Elimination,
	SpecialLocal,
	SpecialServer,
	MAX
};

enum EBuildingGameplayActorAircraftSpawnSide
{
	None,
	Side1,
	Side2,
	EBuildingGameplayActorAircraftSpawnSide_MAX
};

enum EForceKickAfterDeathMode
{
	Disabled,
	KickAll,
	KickPrivate,
	EForceKickAfterDeathMode_MAX
};

enum ECapturePointState
{
	Idle,
	Capturing,
	Contested,
	Resetting,
	Captured,
	Reset,
	ECapturePointState_MAX
};

enum EAdHocSquads_InviteStatus
{
	Unset,
	Inviting,
	InviteEnded_APlayerAcceptedTheInvite,
	InviteCancelled_SquadFull,
	InviteCancelled_ByInvitingPlayer,
	EAdHocSquads_MAX
};

enum EAdHocSquads_SquadUpResult
{
	Success,
	Failure_OneOrMorePlayersWereNull,
	Failure_BothPlayersAreInAdHocSquads,
	Failure_BothPlayersAreAlreadyInTheSameAdHocSquad,
	Failure_CouldNotCreateAdHocSquad,
	Failure_CalledOnClient,
	Failure_SquadIsAlreadyFull,
	Failure_TooManyPlayersToMergeSquads,
	Failure_MutatorIsDisabled,
	Failure_Unknown,
	Failure_Custom,
	EAdHocSquads_MAX
};

enum EAdHocSquads_LeaveSquadReason
{
	ManualLeave,
	MutatorDisabled,
	JoinedADifferentSquad,
	PlayerDiedAndCannotBeRevived,
	PlayerHasWonGame,
	EAdHocSquads_MAX
};

enum EMiniMapComponentDiscoverableVisibility
{
	Unset,
	NotVisible,
	Visible_NotDiscovered,
	Discovered,
	EMiniMapComponentDiscoverableVisibility_MAX
};

enum EPlayerBountyThreatLevel
{
	Low,
	Medium,
	High,
	MAX
};

enum EBackupSaveState
{
	Ready,
	InProgress,
	OnCooldown,
	EBackupSaveState_MAX
};

enum EPropertyOverrideTargetType
{
	None,
	Default,
	ImmutableTarget,
	EPropertyOverrideTargetType_MAX
};

enum ETInteractionType
{
	IT_NoInteraction,
	IT_Simple,
	IT_LongPress,
	IT_BuildingEdit,
	IT_BuildingImprovement,
	IT_TrapPlacement,
	IT_MAX
};

enum EFortBuildingPersistentState
{
	Default,
	New,
	Constructed,
	Destroyed,
	Searched,
	None,
	EFortBuildingPersistentState_MAX
};

enum EFortBuildingInitializationReason
{
	StaticallyPlaced,
	Spawned,
	Replaced,
	LoadedFromSave,
	DynamicBuilderPlaced,
	PlacementTool,
	TrapTool,
	None,
	EFortBuildingInitializationReason_MAX
};

enum ENavigationObstacleOverride
{
	UseMeshSettings,
	ForceEnabled,
	ForceDisabled,
	ENavigationObstacleOverride_MAX
};

enum EDynamicBuildingPlacementType
{
	CountsTowardsBounds,
	DestroyIfColliding,
	DestroyAnythingThatCollides,
	EDynamicBuildingPlacementType_MAX
};

enum EFortBaseWeaponDamage
{
	Combat,
	Environmental,
	EFortBaseWeaponDamage_MAX
};

enum EBuildingActorComponentCreationPolicy
{
	Never,
	Lazy,
	Always,
	EBuildingActorComponentCreationPolicy_MAX
};

enum EAttributeInitLevelSource
{
	WorldDifficulty,
	PlayerBuildingSkill,
	AthenaPlaylist,
	EAttributeInitLevelSource_MAX
};

enum EFortAbilityTargetingSource
{
	Camera,
	PawnForward,
	PawnTowardsFocus,
	WeaponForward,
	WeaponTowardsFocus,
	Custom,
	Max_None,
	EFortAbilityTargetingSource_MAX
};

enum EFortDeliveryInfoBuildingActorSpecification
{
	All,
	PlayerBuildable,
	NonPlayerBuildable,
	EFortDeliveryInfoBuildingActorSpecification_MAX
};

enum EFortTeamAffiliation
{
	Friendly,
	Neutral,
	Hostile,
	EFortTeamAffiliation_MAX
};

enum EFortProximityBasedGEApplicationType
{
	ApplyOnProximityPulse,
	ApplyOnProximityTouch,
	ApplyOnlyDuringProximityTouch,
	ApplyOnProximityExit,
	ApplyOnProximityPrePulse,
	EFortProximityBasedGEApplicationType_MAX
};

enum EHudVisibilityState
{
	FullyVisible,
	FullyHidden,
	GameOnly,
	ReplayOnly,
	MAX
};

enum EFortReplayEventType
{
	Elimination,
	Eliminated,
	DBNO,
	UserPlaced,
	MAX
};

enum ESpectatorCameraType
{
	ThirdPerson,
	DroneFree,
	Gameplay,
	ReverseShot,
	FollowShot,
	DroneFollow,
	DroneAttach,
	BattleMap,
	ARDrone,
	MAX
};

enum EIntelStateEnum
{
	None,
	OnGround,
	HeldByAttacker,
	HeldByDefender,
	Downloaded,
	Captured,
	Downloading,
	EIntelStateEnum_MAX
};

enum EEventResponderEventType
{
	OneShotEvent,
	PersistentEventJoinInProgress,
	PersistentEventStart,
	PersistentEventChange,
	PersistentEventEnd,
	EEventResponderEventType_MAX
};

enum EMatchConditionMutatorTimingType
{
	Round,
	MatchAtEndOfRound,
	MatchImmediate,
	COUNT,
	EMatchConditionMutatorTimingType_MAX
};

enum EAthenaRoundsMutatorPhase
{
	GamePhase_Setup,
	GamePhase_Warmup,
	FadeOutToNextRound,
	RoundSetup,
	RoundPlay,
	RoundEnd,
	RoundEndUI,
	MatchEndUI,
	MatchEndedPrematurely,
	EAthenaRoundsMutatorPhase_MAX
};

enum EFortMutatorOverridePriority
{
	None,
	Low,
	Medium,
	High,
	EFortMutatorOverridePriority_MAX
};

enum EUraniumRoundPhase
{
	None,
	EndOfRoundStart,
	ShowRoundEnd,
	HideRoundEnd,
	FadeOut,
	SetupForNextRound,
	ShowRoundIntro,
	ShowPOICamera,
	PerkSelect,
	RoundActive,
	FadeIn,
	EndOfRoundFinish,
	FadeBeforeReleasePlayerIntoGameplay,
	ReleasePlayersIntoGameplay,
	EndGame,
	MAX
};

enum EUraniumEventId
{
	Intro,
	CartBecomesPushable,
	CheckPointReached,
	NearCheckPoint,
	NearFinalCheckPoint,
	TimeIsLow,
	TimeIsExtraLow,
	OvertimeStarted,
	RoundEnd_PushersWin,
	RoundEnd_DefendersWin,
	RoundStart,
	GameEnded_AllRoundsPlayed,
	GameEndedEarly_BlowOut,
	GameEndedEarly_OutNumbered,
	EUraniumEventId_MAX
};

enum EUraniumCartMovementRuleOnNewRound
{
	NoChange,
	MoveToNextCheckpoint,
	MoveToStartOfNewSpline,
	MoveToRandomCheckPointOfNewSpline,
	MoveToNextCheckPointOfNewSpline,
	EUraniumCartMovementRuleOnNewRound_MAX
};

enum EUraniumRoundEndCondition
{
	RanOutOfTime_Or_CheckpointReached,
	RanOutOfTime_Or_LastCheckpointReached,
	RanOutOfTime_Or_MAX
};

enum EFortMinigameState
{
	PreGame,
	Setup,
	Transitioning,
	WaitingForCameras,
	Warmup,
	InProgress,
	PostGameTimeDilation,
	PostRoundEnd,
	PostGameEnd,
	PostGameAbandon,
	PostGameReset,
	EFortMinigameState_MAX
};

enum ESpecialEventInputButton
{
	PrimaryFire,
	SecondaryFire,
	RollLeft,
	RollRight,
	Jump,
	Reload,
	SpecialEventInputButton_MAX
};

enum EBuildingGameplayActorSentry_State
{
	PassiveIdle,
	ActiveIdle,
	Tracking,
	Aggro,
	Dormant,
	Deactivated,
	ReturningToIdle,
	LocatingDamager,
	EBuildingGameplayActorSentry_MAX
};

enum ELockState
{
	INVALID,
	UNLOCKED,
	LOCKED,
	ELockState_MAX
};

enum EJoinInProgress
{
	Spectate,
	JoinOnNextRound,
	JoinImmediately,
	JoinSpecificTeam,
	EJoinInProgress_MAX
};

enum EMMSPrivacy
{
	Public,
	Private,
	EMMSPrivacy_MAX
};

enum EPortalLinkCodeLockStatus
{
	Unlocked_NotSet,
	Unlocked,
	Locked,
	EPortalLinkCodeLockStatus_MAX
};

enum EHeldObjectState
{
	Unheld,
	Held,
	Thrown,
	Placed,
	Dropped,
	HeldInVehicle,
	EHeldObjectState_MAX
};

enum EFortHomingStyle
{
	None,
	LockOn,
	LaserTargeted,
	LaserTargeted_NoTrace,
	DrunkArtillery,
	EFortHomingStyle_MAX
};

enum ECaptureState
{
	Neutral,
	Capturing,
	Contested,
	Disabled,
	Decapturing,
	Neutralizing,
	Deneutralizing,
	Captured,
	ECaptureState_MAX
};

enum EHitTraceRule
{
	Visibility,
	Terrain,
	None,
	EHitTraceRule_MAX
};

enum ETransformationType
{
	Translation,
	Rotation,
	Scale,
	None,
	ETransformationType_MAX
};

enum EScaleAxis
{
	All,
	X,
	Y,
	Z,
	EScaleAxis_MAX
};

enum ESelectionProperty
{
	SingleObject,
	MultipleObjects,
	MultipleObjectsMoveOnGrid,
	None,
	ESelectionProperty_MAX
};

enum ECreativeKeyLockState
{
	LOCKED,
	UNLOCKED,
	INVALID,
	ECreativeKeyLockState_MAX
};

enum EFortPrototypingStatus
{
	Unknown,
	Disabled,
	Enabled,
	EFortPrototypingStatus_MAX
};

enum EFortQuestObjectiveStatEvent
{
	Kill,
	TeamKill,
	KillContribution,
	Damage,
	SquadDamage,
	Visit,
	VisitDiscoverPOI,
	Land,
	Emote,
	Spray,
	Toy,
	Build,
	BuildingEdit,
	BuildingRepair,
	BuildingUpgrade,
	PlaceTrap,
	Complete,
	Craft,
	Collect,
	Win,
	Interact,
	TeamInteract,
	Destroy,
	Ability,
	WaveComplete,
	Custom,
	ComplexCustom,
	Client,
	AthenaRank,
	AthenaOutlive,
	RevivePlayer,
	Heal,
	EarnVehicleTrickPoints,
	VehicleAirTime,
	TimeElapsed,
	Death,
	AthenaMarker,
	PlacementUpdate,
	StormPhase,
	DistanceTraveled,
	DownOrElim,
	Accolade,
	TakeDamage,
	AthenaCollection,
	UsedNPCService,
	ReceivedNPCGift,
	InitiatedNPCConversation,
	AthenaCraft,
	AthenaTurnInQuest,
	AthenaVehicleMod,
	AthenaOpenedFriendChest,
	AthenaAcquire,
	InitiatedServiceStationConversation,
	AthenaAccumulatedItem,
	AthenaVehicleFlip,
	RevealedPawnDisguise,
	NumGameplayEvents,
	Acquire,
	Consume,
	OpenCardPack,
	PurchaseCardPack,
	Convert,
	Upgrade,
	UpgradeRarity,
	QuestComplete,
	AssignWorker,
	CollectExpedition,
	CollectSuccessfulExpedition,
	LevelUpCollectionBook,
	LevelUpAthenaSeason,
	LevelUpBattlePass,
	GainAthenaSeasonXp,
	HasItem,
	HasAccumulatedItem,
	SlotInCollection,
	AlterationRespec,
	AlterationUpgrade,
	HasCompletedQuest,
	HasAssignedWorker,
	HasUpgraded,
	HasConverted,
	HasUpgradedRarity,
	HasLeveledUpCollectionBook,
	SlotHeroInLoadout,
	HasLeveledUpAthenaSeason,
	HasLeveledUpBattlePass,
	HasGainedAthenaSeasonXp,
	MinigameDynamicEvent,
	MinigameComplete,
	MinigameDeath,
	MinigameAssist,
	Max_None,
	EFortQuestObjectiveStatEvent_MAX
};

enum EFortMinigameEnd
{
	AbandonGame,
	EndGame,
	EndRound,
	EFortMinigameEnd_MAX
};

enum EPlayerIndicatorDisplayMode
{
	DontOverride,
	TeamOnly,
	Enemies,
	Anyone,
	Never,
	EPlayerIndicatorDisplayMode_MAX
};

enum ECreativeBossDisplayMode
{
	DontOverride,
	Yes,
	No,
	ECreativeBossDisplayMode_MAX
};

enum EFortMissionVisibilityOverride
{
	Visible,
	Invisible,
	Max_None,
	EFortMissionVisibilityOverride_MAX
};

enum EMatchmakingCompleteResult
{
	NotStarted,
	UpdateNeeded,
	OutpostNotFound,
	Cancelled,
	NoResults,
	Failure,
	CreateFailure,
	Success,
	EMatchmakingCompleteResult_MAX
};

enum EMatchmakingState
{
	NotMatchmaking,
	CancelingMatchmaking,
	ReleasingLock,
	AcquiringLock,
	LockAcquistionFailure,
	FindingEmptyServer,
	FindingExistingSession,
	TestingEmptyServers,
	TestingExistingSessions,
	JoiningExistingSession,
	NoMatchesAvailable,
	CleaningUpExisting,
	HandlingFailure,
	JoinSuccess,
	EMatchmakingState_MAX
};

enum EMatchmakingUtilityState
{
	Idle,
	MatchmakingUtilityBegin,
	MatchmakingUtilityCompleted,
	Failed,
	CheckSittingOutState,
	InitializePlaylistSelection,
	JoinMatchInProgress,
	CheckingIfBanned,
	PrivateMatchMinPartySize,
	CreativeContentDownloadModal,
	CrossplayOptIn_GameMode,
	CrossplayOptIn_Fill,
	CrossplayDevices,
	FactionChoice,
	SetPartyReadiness,
	WaitingForPartyReadinessConfirmed,
	TournamentEligibility,
	TournamentRegionLock,
	TournamentMFA,
	TournamentEULA,
	AppEnvSecurity,
	WaitingForInitialPlaylistSelection,
	ServerBrowsers,
	WaitingForServerBrowserPlaylistSwap,
	DelayReadyUp,
	QoSSettings_UpdateManager,
	QoSSettings_UpdateManager_Success,
	SetAthenaReady,
	WaitingForAthenaReadinessConfirmed,
	PreloadAthena,
	UpdateHiddenMatchmakingDelay,
	WaitingForUpdateHiddenMatchmakingDelay,
	HiddenMatchmakingDelay,
	FindExistingEditSession,
	SelectingFlow,
	PreloadingForFlow,
	ReadyingUpforFlow,
	CallingMatchmakingForFlow,
	WaitingForJoinMatchInProgressResponse,
	WaitingForMatchmakingResponse,
	WaitingForMatchmakingToCompleteAsPartyMember,
	WaitingForEntirePartyReady,
	WaitingToRestartMatchmaking_TooFrequentRequest,
	RequestingSpectateMatch,
	ProcessingMatchmakingResults_Success,
	ProcessingMatchmakingResults_Fail,
	PreloadingForJoiningMatchInProgress,
	ProcessingJoinMatchInProgressResults_Success,
	ProcessingJoinMatchInProgressResults_Fail,
	DownloadingAdditionalContent,
	RestartingMatchmaking,
	INVALID,
	EMatchmakingUtilityState_MAX
};

enum EAthenaPartyMemberReadyType
{
	Ready,
	NotReady,
	Playing,
	Spectating,
	WatchingReplay,
	EAthenaPartyMemberReadyType_MAX
};

enum EFortFriendRequestStatus
{
	None,
	Accepted,
	PendingReceived,
	PendingSent,
	EFortFriendRequestStatus_MAX
};

enum EFortPartyMemberLocation
{
	PreLobby,
	ConnectingToLobby,
	Lobby,
	JoiningGame,
	ProcessingRejoin,
	InGame,
	Spectating,
	WatchingReplay,
	ReturningToFrontEnd,
	EFortPartyMemberLocation_MAX
};

enum EFortPartyState
{
	Undetermined,
	WorldView,
	TheaterView,
	Matchmaking,
	PostMatchmaking,
	ReturningToFrontEnd,
	BattleRoyaleView,
	BattleRoyalePreloading,
	BattleRoyaleMatchmaking,
	BattleRoyalePostMatchmaking,
	EFortPartyState_MAX
};

enum EFortPartyMemberDisplayState
{
	Open,
	Connecting,
	Connected,
	Max
};

enum ESpectateBlendEasing
{
	Linear,
	EaseOutQuad,
	ESpectateBlendEasing_MAX
};

enum EFortSpectatorBlendType
{
	None,
	Orbit,
	Default,
	EFortSpectatorBlendType_MAX
};

enum ESpectatorSquadIdMode
{
	AlwaysOff,
	AlwaysOn,
	HoldToDisplay,
	ESpectatorSquadIdMode_MAX
};

enum EPlayEventType
{
	None,
	TeamFlight,
	Zone,
	Elimination,
	PlayerMoves,
	SinglePlayerMove,
	ActorsPosition,
	GameHighlights,
	Timecode,
	EPlayEventType_MAX
};

enum ECameraShotNotificationTypes
{
	Notification,
	HighlightAnnotation,
	TitleScreen,
	ECameraShotNotificationTypes_MAX
};

enum EThirdPersonAutoFollowMode
{
	Off,
	Auto,
	Lazy,
	EThirdPersonAutoFollowMode_MAX
};

enum EFortPlayerSurveyFinishReason
{
	Submitted,
	Canceled,
	Disallowed,
	EFortPlayerSurveyFinishReason_MAX
};

enum EFortPlayerSurveyAnswerContainerChangeReason
{
	AnswerChange,
	QuestionChange,
	ProxyChange,
	EFortPlayerSurveyAnswerContainerChangeReason_MAX
};

enum EInteriorAudioState
{
	Indoors,
	Outdoors,
	PartialOutdoors,
	Max_None,
	EInteriorAudioState_MAX
};

enum ECollectionBookRewardType
{
	Uninitialized,
	Book,
	Page,
	Section,
	ECollectionBookRewardType_MAX
};

enum ECampaignCustomizationCategory
{
	None,
	PersonalVehicle,
	ECampaignCustomizationCategory_MAX
};

enum EFortDialogFeedbackType
{
	FriendRequestSent,
	FriendRequestReceived,
	FriendRequestAccepted,
	Default,
	EFortDialogFeedbackType_MAX
};

enum EFortWeakPointState
{
	Uninitialized,
	Inactive,
	Active,
	Destroyed,
	EFortWeakPointState_MAX
};

enum EDualWeaponHand
{
	LEFT,
	RIGHT,
	MAX
};

enum EFortEncounterDirection
{
	North,
	NorthEast,
	East,
	SouthEast,
	South,
	SouthWest,
	West,
	NorthWest,
	Max_None,
	EFortEncounterDirection_MAX
};

enum EQuestUpdateType
{
	ObjectiveCompleted,
	QuestCompleted,
	ObjectiveUpdated,
	QuestUpdated,
	QuestAndObjectiveUpdate,
	EQuestUpdateType_MAX
};

enum EQuestVisibilityResponse
{
	Hide,
	Show,
	Custom,
	EQuestVisibilityResponse_MAX
};

enum EItemWrapMaterialType
{
	WeaponWrap,
	VehicleWrap_Opaque,
	VehicleWrap_Masked,
	Character,
	EItemWrapMaterialType_MAX
};

enum EFortRarity
{
	Common,
	Uncommon,
	Rare,
	Epic,
	Legendary,
	Mythic,
	Transcendent,
	Unattainable,
	NumRarityValues,
	EFortRarity_MAX
};

enum EAIHotSpotAssignmentFilter
{
	All,
	WithSlots,
	WaitingList,
	EAIHotSpotAssignmentFilter_MAX
};

enum EAIHotSpotSlotFilter
{
	All,
	AvailableOnly,
	UnavailableOnly,
	EAIHotSpotSlotFilter_MAX
};

enum EAIHotSpotSlot
{
	Free,
	Claimed,
	Occupied,
	Blocked,
	Disabled,
	EAIHotSpotSlot_MAX
};

enum EBoundingBoxSlotDirectionCalculation
{
	Auto,
	FaceWall,
	FaceAwayFromWall,
	FaceCenter,
	EBoundingBoxSlotDirectionCalculation_MAX
};

enum EBarrierFlagState
{
	FlagUp,
	FlagDown,
	EBarrierFlagState_MAX
};

enum EBarrierState
{
	BarrierUp,
	BarrierComingDown,
	BarrierDown,
	EBarrierState_MAX
};

enum EAthenaBroadcastKillFeedEntryType
{
	Elimination,
	Storm,
	FallDamage,
	Explosion,
	DBNO,
	EAthenaBroadcastKillFeedEntryType_MAX
};

enum ECapturePointUnlockRules
{
	Reset,
	MaintainState,
	ResetDeactivate,
	ECapturePointUnlockRules_MAX
};

enum EContentionRuleType
{
	MajorityWins,
	OneTeamOnlyWins,
	EContentionRuleType_MAX
};

enum EVariantUnlockType
{
	UnlockAll,
	ExclusiveChoice,
	EVariantUnlockType_MAX
};

enum EAthenaPIEStartupMode
{
	UseDefaults,
	Warmup,
	WarmupPaused,
	Aircraft,
	AircraftPaused,
	Gameplay,
	EAthenaPIEStartupMode_MAX
};

enum EEventTokenType
{
	Invite,
	Creation,
	EEventTokenType_MAX
};

enum ELayeredAudioTriggerDirection
{
	AnyDirection,
	Forwards,
	Sideways,
	Backwards,
	Count,
	ELayeredAudioTriggerDirection_MAX
};

enum EItemWrapSectionNames
{
	Section_0,
	Section_1,
	Section_2,
	Section_3,
	Section_4,
	Section_5,
	Section_6,
	Section_7,
	Section_8,
	Section_9,
	Section_10,
	Section_11,
	Section_12,
	Section_13,
	Section_14,
	Section_15,
	Section_16,
	Section_17,
	Section_18,
	Section_19,
	Section_20,
	Section_21,
	Section_22,
	Section_23,
	Section_24,
	Section_25,
	Section_26,
	Section_27,
	Section_28,
	Section_29,
	Section_30,
	Section_31,
	Section_MAX
};

enum ECancelMarkerReason
{
	Ping,
	MapOrDeath,
	ECancelMarkerReason_MAX
};

enum EInBoundsState
{
	NoBounds,
	NotInBounds,
	InBounds,
	EInBoundsState_MAX
};

enum EAthenaPetAttachRule
{
	AttachToBackpack,
	AttachToBody,
	EAthenaPetAttachRule_MAX
};

enum EAthenaQuickChatFilteringType
{
	AlwaysVisible,
	ActiveMaterial,
	FacingPickup,
	ActiveHotbarItem,
	ActiveHotbarItemAmmo,
	FacingPickupOrActiveHotbarItem,
	NoWeaponEquippedRequiringAmmo,
	WeaponEquippedOfAmmoType,
	EAthenaQuickChatFilteringType_MAX
};

enum EAthenaSeasonRewardTrack
{
	Invalid,
	SeasonProgressionTrack,
	CompendiumFreeTrack,
	CompendiumPaidTrack,
	CompendiumAdditionalTrack,
	EAthenaSeasonRewardTrack_MAX
};

enum EAthenaRewardVisualImportanceType
{
	Normal,
	Hot,
	CrazyHot,
	Crazy,
	EAthenaRewardVisualImportanceType_MAX
};

enum EAthenaRewardItemType
{
	Normal,
	HiddenReward,
	GiftboxHiddenReward,
	NonExportedFakeReward,
	EAthenaRewardItemType_MAX
};

enum EBattlePassRewardSource
{
	None,
	ChallengeBundle,
	Quest,
	EBattlePassRewardSource_MAX
};

enum EPageItemTileSize
{
	Size_1_x_1,
	Size_2_x_1,
	Size_2_x_2,
	Size_3_x_2,
	Size_3_x_3,
	Count,
	EPageItemTileSize_MAX
};

enum ETraversePointState
{
	None,
	Hidden,
	Active,
	TouchedByPlayer,
	Finished,
	ETraversePointState_MAX
};

enum EFortVehicleDecoType
{
	Unknown,
	Flag,
	HoodOrnament,
	Wings,
	EFortVehicleDecoType_MAX
};

enum EWrapPreviewCamera
{
	Weapon,
	LargeWeapon,
	Vehicle,
	EWrapPreviewCamera_MAX
};

enum EAutoFrameMode
{
	Off,
	ManualOverride,
	AutoFrame,
	EAutoFrameMode_MAX
};

enum EBacchusHUDStateType
{
	DoNothing,
	Hide,
	Show,
	FallbackToDefault,
	EBacchusHUDStateType_MAX
};

enum EBattleMapHoveredReason
{
	None,
	Mouse,
	Code,
	BattleMapHoveredReason_MAX
};

enum EBuildingTickReason
{
	Dynamic,
	Damaged,
	GameplayCue_Damage,
	GameplayCue_Healing,
	GameplayCue_InstantDeath,
	UnderConstruction,
	UnderRepair,
	Editing,
	BuildingAnimation,
	DynamicLODAnim,
	DynamicShrinkAnim,
	AutoBuild,
	FullHealthEffect,
	BounceAnimation,
	DoorOpenStyleChanged,
	DoorOpenChanged,
	DoorInteract,
	EBuildingTickReason_MAX
};

enum ECalendarDrivenState
{
	ForceEnable,
	ForceDisable,
	ECalendarDrivenState_MAX
};

enum EBinaryToggleValues
{
	BTV_Active,
	BTV_Inactive,
	BTV_Either,
	BTV_MAX
};

enum EAuxIndicatorStates
{
	AIS_GuidingArrow,
	AIS_ConfirmedArrow,
	AIS_Inactive,
	AIS_Active,
	AIS_MAX
};

enum EDynamicFoundationEnabledState
{
	Unknown,
	Enabled,
	Disabled,
	EDynamicFoundationEnabledState_MAX
};

enum EDynamicFoundationType
{
	Static,
	StartEnabled_Stationary,
	StartEnabled_Dynamic,
	StartDisabled,
	EDynamicFoundationType_MAX
};

enum EBuildingFoundationType
{
	BFT_3x3,
	BFT_5x5,
	BFT_5x10,
	BFT_None,
	BFT_MAX
};

enum ESpawnMachineSubTextState
{
	NoCards,
	VanInUse,
	None,
	ESpawnMachineSubTextState_MAX
};

enum EFortItemCollectorTrackingType
{
	Player,
	Team,
	EFortItemCollectorTrackingType_MAX
};

enum EFortItemCollectorBehavior
{
	FirstToGoal,
	FreeForAll,
	EFortItemCollectorBehavior_MAX
};

enum EFortItemCollectorState
{
	CanInteract,
	Active,
	Inactive,
	Captured,
	Invalid,
	EFortItemCollectorState_MAX
};

enum ELayoutRequirementStatus
{
	Inactive_Invisible,
	Active_Invisible,
	Active_Visible,
	ELayoutRequirementStatus_MAX
};

enum ECaptureAreaItemFilters
{
	None,
	Both,
	ForPeriodicScoring,
	ToTakeControl,
	ECaptureAreaItemFilters_MAX
};

enum EPlayerCaptureKnobOptions
{
	Off,
	EachPlayer,
	OnePlayerPerTeam,
	OwningTeam,
	EPlayerCaptureKnobOptions_MAX
};

enum ECreativeVendingMachineState
{
	DisplayingItem,
	AcceptingItem,
	AcceptedItem,
	RejectedItem,
	ECreativeVendingMachineState_MAX
};

enum EMusicTrackPlayback
{
	Disabled,
	Enabled,
	EMusicTrackPlayback_MAX
};

enum ERiftCosmeticState
{
	None,
	Intro,
	Idle,
	RampUp,
	ShouldDie,
	ERiftCosmeticState_MAX
};

enum EFortRiftSpawnSlotSelectionMode
{
	Random,
	BestScore,
	EFortRiftSpawnSlotSelectionMode_MAX
};

enum EFortRiftSlotStatus
{
	Reserved,
	Occupied,
	Max_None,
	EFortRiftSlotStatus_MAX
};

enum EBuildingNavObstacleType
{
	UnwalkableAll,
	UnwalkableHuskOnly,
	SmashWhenLowHeight,
	SmashOnlyLowHeight,
	SmashSmasherOnly,
	SmashAll,
	EBuildingNavObstacleType_MAX
};

enum EFortDamageVisualsState
{
	UnDamaged,
	DamagedAndAnimating,
	DamagedAndStatic,
	EFortDamageVisualsState_MAX
};

enum EStructuralSupportCheck
{
	Stable,
	Unstable,
	Max_None,
	EStructuralSupportCheck_MAX
};

enum ESavedSupportStatus
{
	UnknownState,
	Supported,
	UnSupported,
	ESavedSupportStatus_MAX
};

enum EPlacementType
{
	Free,
	Grid,
	None,
	EPlacementType_MAX
};

enum EBuildingAttachmentSide
{
	Front,
	Back,
	Any,
	EBuildingAttachmentSide_MAX
};

enum EBuildingAttachmentSlot
{
	SLOT_Floor,
	SLOT_Wall,
	SLOT_Ceiling,
	SLOT_None,
	SLOT_MAX
};

enum EBuildingAnim
{
	EBA_None,
	EBA_Building,
	EBA_Breaking,
	EBA_Destruction,
	EBA_Placement,
	EBA_DynamicLOD,
	EBA_DynamicShrink,
	EBA_MAX
};

enum EStructuralFloorPosition
{
	Top,
	Bottom,
	EStructuralFloorPosition_MAX
};

enum EStructuralWallPosition
{
	Left,
	Right,
	Front,
	Back,
	EStructuralWallPosition_MAX
};

enum EFortDefenderInteractionError
{
	None,
	Obstructed,
	NoEditPermission,
	UsedByAnotherPlayer,
	EFortDefenderInteractionError_MAX
};

enum EFortBuildingDestroyedReason
{
	Unknown,
	WeaponDamage,
	LostSupport,
	Edit,
	FireDirect,
	FireIndirect,
	EFortBuildingDestroyedReason_MAX
};

enum EFortBounceType
{
	Hit,
	Interact,
	EditPlaced,
	EFortBounceType_MAX
};

enum EFortConnectivityCubeFace
{
	Front,
	Left,
	Back,
	Right,
	Upper,
	Lower,
	MAX
};

enum EFortDecoPlacementQueryResults
{
	CanAdd,
	ExistingTrap,
	ExistingObject,
	Obstructed,
	NoLocation,
	WrongType,
	WrongShape,
	BeingModified,
	WrongTeam,
	BlueprintFailure,
	AbilityFailure,
	RequiresPlayerBuildableActor,
	NoEditPermission,
	WrongZone,
	EFortDecoPlacementQueryResults_MAX
};

enum EFortStructuralGridQueryResults
{
	CanAdd,
	ExistingActor,
	Obstructed,
	NoStructuralSupport,
	InvalidActor,
	ReachedLimit,
	NoEditPermission,
	PatternNotPermittedByLayoutRequirement,
	ResourceTypeNotPermittedByLayoutRequirement,
	BuildingAtRequirementsDisabled,
	BuildingOtherThanRequirementsDisabled,
	EFortStructuralGridQueryResults_MAX
};

enum EFortBuildingState
{
	Placement,
	EditMode,
	None,
	EFortBuildingState_MAX
};

enum EFortTextureDataSlot
{
	Primary,
	Secondary,
	Tertiary,
	Fourth,
	NumSlots,
	EFortTextureDataSlot_MAX
};

enum EFortTextureDataType
{
	Any,
	OuterWall,
	InnerWall,
	Corner,
	Floor,
	Ceiling,
	Trim,
	Roof,
	Pillar,
	Shingle,
	None,
	EFortTextureDataType_MAX
};

enum EBuildingAttachmentType
{
	ATTACH_Floor,
	ATTACH_Wall,
	ATTACH_Ceiling,
	ATTACH_Corner,
	ATTACH_All,
	ATTACH_WallThenFloor,
	ATTACH_FloorAndStairs,
	ATTACH_CeilingAndStairs,
	ATTACH_None,
	ATTACH_MAX
};

enum EDoorOpenStyle
{
	Open,
	SlamOpen,
	SmashOpen,
	Close,
	EDoorOpenStyle_MAX
};

enum ESmoothProgressState
{
	Enabled,
	DisabledByForceProgress,
	DisabledByState,
	ESmoothProgressState_MAX
};

enum EClientContentReadiness
{
	AwaitingServerResponse,
	ReceivingContentNames,
	DownloadingContent,
	MountingContent,
	ReadyToJoin,
	FailedToMount,
	NotConnected,
	ConnectionFailed,
	CelluarDataRefusal,
	CancelledDownloadContent,
	None,
	EClientContentReadiness_MAX
};

enum EContentRequestStatus
{
	None,
	Active,
	Finished,
	EContentRequestStatus_MAX
};

enum ECreativeMinimapComponentIconColorType
{
	None,
	White,
	Red,
	Orange,
	Yellow,
	Green,
	Teal,
	Blue,
	Purple,
	ECreativeMinimapComponentIconColorType_MAX
};

enum EShowProgressMode
{
	Total,
	Remaining,
	Off,
	MAX
};

enum ECreativeQuestSharing
{
	Individual,
	Team,
	All,
	MAX
};

enum ECreativeQuestStat
{
	None,
	Eliminations,
	Eliminated,
	Score,
	MAX
};

enum EScoreDistributionType
{
	Default,
	DivideByDamage,
	DivideEvenly,
	AllToKiller,
	EScoreDistributionType_MAX
};

enum EAccessoryColorName
{
	EAccessoryColorName_AccessoryColor1,
	EAccessoryColorName_AccessoryColor2,
	EAccessoryColorName_AccessoryColor3,
	EAccessoryColorName_NumTypes,
	EAccessoryColorName_MAX
};

enum ECustomHatType
{
	ECustomHatType_None,
	ECustomHatType_Cap,
	ECustomHatType_Helmet,
	ECustomHatType_Mask,
	ECustomHatType_Hat,
	ECustomHatType_HeadReplacement,
	ECustomHatType_MAX
};

enum ECharacterPartAttachmentTargetType
{
	RootComponent,
	SkeletalMeshForAssociatedPlayerPawnPartType,
	ECharacterPartAttachmentTargetType_MAX
};

enum EClothingColorName
{
	EClothingColorName_AccessoryColor1,
	EClothingColorName_AccessoryColor2,
	EClothingColorName_NumTypes,
	EClothingColorName_MAX
};

enum EColorSwatchType
{
	EColorSwatchType_Skin,
	EColorSwatchType_Hair,
	EColorSwatchType_BodyAccessory,
	EColorSwatchType_Accessory,
	EColorSwatchType_NumTypes,
	EColorSwatchType_MAX
};

enum ECharacterColorSwatchType
{
	ECharacterColorSwatchType_Skin,
	ECharacterColorSwatchType_Hair,
	ECharacterColorSwatchType_NumTypes,
	ECharacterColorSwatchType_MAX
};

enum EVehicleEnteredCosmeticReaction
{
	Driver,
	Passenger,
	Both,
	EVehicleEnteredCosmeticReaction_MAX
};

enum EDadBroHealthType
{
	None,
	Weakpoints,
	Horn,
	Guy,
	EDadBroHealthType_MAX
};

enum EDeployableBaseConstructionStatus
{
	Constructing,
	Destroying,
	Finished,
	EDeployableBaseConstructionStatus_MAX
};

enum EDeployableBaseBuildingState
{
	Empty,
	Built,
	Unoccupied,
	WaitingToBuild,
	Building,
	WaitingToDestroy,
	Destroying,
	WaitingToReset,
	Resetting,
	EDeployableBaseBuildingState_MAX
};

enum EDeployableBaseBoxType
{
	BuildSpace,
	SaveSpace,
	PlotSpace,
	NumSpaceTypes,
	EDeployableBaseBoxType_MAX
};

enum EReferenceType
{
	Hard,
	Soft,
	Dynamic,
	EReferenceType_MAX
};

enum EFortSharedAnimationState
{
	Anim_Walk,
	Anim_Run,
	Anim_Turn,
	Anim_Attack,
	Anim_Death,
	Anim_Knockback,
	Anim_FullBodyHit,
	Anim_Pushed,
	Anim_Dance,
	Anim_Idle,
	Anim_RangedAttack,
	Anim_MAX
};

enum EFortStatDisplayType
{
	Category,
	Buff,
	Debuff,
	Neutral,
	DoNotDisplay,
	EFortStatDisplayType_MAX
};

enum EFortAbilityTargetSelectionUsage
{
	BothTargetingAndCanHit,
	OnlyTargeting,
	OnlyCanHit,
	EFortAbilityTargetSelectionUsage_MAX
};

enum EFortDirectedMovementSpace
{
	WorldSpace,
	ActorLocRelative,
	ActorLocRotRelative,
	CameraRelative,
	EFortDirectedMovementSpace_MAX
};

enum EFortAbilityTargetDataPolicy
{
	ReplicateToServer,
	SimulateOnServer,
	EFortAbilityTargetDataPolicy_MAX
};

enum EFortAbilityChargeState
{
	None,
	ChargingUp,
	MaxCharge,
	Discharge,
	ChargingDown,
	EFortAbilityChargeState_MAX
};

enum EFortAccoladeType
{
	Acknowledgement,
	Accolade,
	Medal,
	EFortAccoladeType_MAX
};

enum EFortActorIndicatorContainerWidget
{
	Top,
	Middle,
	Bottom,
	EFortActorIndicatorContainerWidget_MAX
};

enum EFortActorSpawnerAuthority
{
	ServerAuthoritative,
	ClientAuthoritative,
	EFortActorSpawnerAuthority_MAX
};

enum EAthenaAITelemetryEventType
{
	Spawn,
	Despawn,
	EAthenaAITelemetryEventType_MAX
};

enum EDespawnAIType
{
	Relevancy,
	Distance,
	EDespawnAIType_MAX
};

enum EFortEncounterUtilityDesire
{
	Low,
	Medium,
	High,
	VeryHigh,
	Max_None,
	EFortEncounterUtilityDesire_MAX
};

enum EFortAIDirectorFactorContribution
{
	Direct,
	Inverse,
	EFortAIDirectorFactorContribution_MAX
};

enum EFortAIDirectorEventContribution
{
	Increment,
	Set,
	EFortAIDirectorEventContribution_MAX
};

enum EFortAIWaveProgressSection
{
	SectionOne,
	SectionTwo,
	Max_None,
	EFortAIWaveProgressSection_MAX
};

enum EFortEncounterState
{
	Uninitialized,
	InitializingProperties,
	InitializingRiftManager,
	AwaitingActivation,
	Active,
	ReplacingRifts,
	Max_None,
	EFortEncounterState_MAX
};

enum EFortEncounterPacingState
{
	Ramp,
	Peak,
	Fade,
	Rest,
	Max_None,
	EFortEncounterPacingState_MAX
};

enum EFortEncounterSequenceResult
{
	Success,
	FailedEncounterInProgress,
	Failed,
	EFortEncounterSequenceResult_MAX
};

enum EAssignmentCreationResult
{
	AssignmentNotFoundOrCreated,
	AssignmentCreated,
	AssignmentFound,
	EAssignmentCreationResult_MAX
};

enum ETagGoalScoringCategory
{
	Ignore,
	HighInterest,
	NumCategories,
	ETagGoalScoringCategory_MAX
};

enum EFortAIPawnGender
{
	FAPG_Default,
	FAPG_Female,
	FAPG_Male,
	FAPG_MAX
};

enum EFortAILevelRatingDisplayType
{
	DisplayRatingBasedOnDifficulty,
	DisplayAIDifficultyAsRating,
	DontDisplayRating,
	EFortAILevelRatingDisplayType_MAX
};

enum EFortressAIType
{
	FAT_Dormant,
	FAT_Cleaner,
	FAT_DayWanderer,
	FAT_NightWanderer,
	FAT_DebugOnly,
	FAT_Encounter,
	FAT_MAX
};

enum ECorePerceptionTypes
{
	Sight,
	Hearing,
	Damage,
	Touch,
	Team,
	Prediction,
	MAX
};

enum EAIScalableFloatScalingType
{
	Disabled,
	ReceivedDamageByTarget,
	EAIScalableFloatScalingType_MAX
};

enum ETargetDistanceComparisonType
{
	TwoDimensions,
	ThreeDimensions,
	CollisionHalfHeightMultiplier,
	ETargetDistanceComparisonType_MAX
};

enum EFortPartialPathUsage
{
	Always,
	OnlyGoalsOnDestructible,
	Never,
	EFortPartialPathUsage_MAX
};

enum EHotspotTypeConfigMode
{
	AlwaysAdd,
	WhenNotDefined,
	WhenNotValid,
	EHotspotTypeConfigMode_MAX
};

enum EFortHotSpotPreview
{
	None,
	Smashing,
	Shooting,
	EFortHotSpotPreview_MAX
};

enum EFortHotSpotDirection
{
	PositiveX,
	NegativeX,
	PositiveY,
	NegativeY,
	PositiveZ,
	NegativeZ,
	Any,
	EFortHotSpotDirection_MAX
};

enum EFortHotSpotSlot
{
	Melee,
	MeleeHuge,
	Ranged,
	None,
	EFortHotSpotSlot_MAX
};

enum EBuildingFloorRailing
{
	None,
	Balcony,
	EBuildingFloorRailing_MAX
};

enum EBuildingStairsRailing
{
	None,
	Partial,
	Full,
	EBuildingStairsRailing_MAX
};

enum EBuildingWallArea
{
	Regular,
	Flat,
	Special,
	EBuildingWallArea_MAX
};

enum EAssignmentType
{
	Invalid,
	Encounter,
	World,
	Enemy,
	NumAssignmentTypes,
	EAssignmentType_MAX
};

enum EFortAILODLevel
{
	Invalid,
	MIN,
	Dormant,
	BelowLower,
	Lower,
	AboveLower,
	BelowNormal,
	Normal,
	AboveNormal,
	MAX
};

enum EFortAnalyticsClientEngagementEventType
{
	None,
	DamageReceivedFromPlayerPawn,
	DamageDealtToPlayerPawn,
	DamageDealtToPlayerBuild,
	DamageDealtToOther,
	EngagementTimeout,
	PlayerWon,
	PlayerDeathOnWin,
	TeamWon,
	TeamLost,
	PlayerLost,
	PlayerKilledPlayer,
	PlayerFiredWeapon,
	ManagerStopped,
	PlayerPawnDied,
	PlayerPawnSpawned,
	Count,
	EFortAnalyticsClientEngagementEventType_MAX
};

enum EFortAnalyticsEventBlacklistPlaylistKey
{
	PlaylistType,
	PlaylistName,
	All,
	EFortAnalyticsEventBlacklistPlaylistKey_MAX
};

enum EAlphaFromDeltaTypes
{
	TranslationX,
	TranslationY,
	TranslationZ,
	Scale,
	ScaleX,
	ScaleY,
	ScaleZ,
	EulerX,
	EulerY,
	EulerZ,
	QuaternionTwist,
	EAlphaFromDeltaTypes_MAX
};

enum ESkydivingDirection
{
	Center,
	Right,
	Left,
	Forward,
	Back,
	ESkydivingDirection_MAX
};

enum EAnimRelaxedState
{
	None,
	WeaponRaised,
	RelaxedLevel1,
	RelaxedLevel2,
	EAnimRelaxedState_MAX
};

enum ESourceSelectionMode
{
	MaxDifference,
	ESourceSelectionMode_MAX
};

enum ESpeedWarpingAxisMode
{
	IKFootRootLocalX,
	IKFootRootLocalY,
	IKFootRootLocalZ,
	WorldSpaceVectorInput,
	ComponentSpaceVectorInput,
	ActorSpaceVectorInput,
	ESpeedWarpingAxisMode_MAX
};

enum EFortNotifyAudioParamsStoreSource
{
	Weapon,
	Pawn,
	Controller,
	EFortNotifyAudioParamsStoreSource_MAX
};

enum EMontageSyncTargetType
{
	Pet,
	CustomPartType,
	EMontageSyncTargetType_MAX
};

enum EMontageInterrupt
{
	Any,
	RootMotionOnly,
	None,
	EMontageInterrupt_MAX
};

enum EDeimosAnimState
{
	Idle,
	Running,
	Attack,
	Dance,
	Dying,
	Died,
	FullBodyHitReact,
	AdditiveHitReact,
	ActiveIdle,
	Falling,
	Frozen,
	RangedAttack,
	Walking,
	Sprinting,
	EDeimosAnimState_MAX
};

enum EFortHandIKOverrideType
{
	UseDefault,
	ForceFK,
	ForceIK,
	ForceFKSnap,
	EFortHandIKOverrideType_MAX
};

enum EFortPlayerAnimBodyType
{
	Small,
	Medium,
	Large,
	All,
	EFortPlayerAnimBodyType_MAX
};

enum EFortCardinalDirection
{
	North,
	East,
	South,
	West,
	EFortCardinalDirection_MAX
};

enum EPlaneDirection
{
	Center,
	Right,
	Left,
	Up,
	Down,
	EPlaneDirection_MAX
};

enum EGliderType
{
	HangGlider,
	Umbrella,
	Surfing,
	Cape,
	NoGlider,
	EGliderType_MAX
};

enum EFortFacialAnimTypes
{
	Default,
	FaceOnly,
	FullHead,
	FromAmplitude,
	Max
};

enum EWaxTokenState
{
	None,
	FirstSpline,
	Interpolation,
	LastSpline,
	Finished,
	EWaxTokenState_MAX
};

enum EAIBotBuildingTemplate
{
	SingleWall,
	SingleRamp,
	SingleRoof,
	SingleBrace,
	SingleWallWindow,
	MAX
};

enum EBotNamingMode
{
	RealName,
	SkinName,
	Anonymous,
	Custom,
	EBotNamingMode_MAX
};

enum EReachLocationValidationMode
{
	None,
	Storm,
	Leash,
	SoftLeash,
	EReachLocationValidationMode_MAX
};

enum EHarvestResult
{
	None,
	InProgress,
	Success,
	Fail,
	EHarvestResult_MAX
};

enum EObstacleType
{
	IncomingSmashable,
	BlockingSmashable,
	BlockingDetectedTrap,
	Unknown,
	Count,
	EObstacleType_MAX
};

enum EBotDataOverrideCosmeticMode
{
	SpecificLoadout,
	CosmeticLibrary,
	BotDataOverrideCosmeticMode_MAX
};

enum EEvasiveManeuverType
{
	Crouch,
	Dodge,
	Jump,
	JetpackStrafe,
	None,
	EEvasiveManeuverType_MAX
};

enum EFreeFallingMode
{
	Idle,
	Random,
	TowardNearestAlly,
	EFreeFallingMode_MAX
};

enum EFocusingBehavior
{
	FocusCurrentTarget,
	IgnoreThreatAfterTimer,
	IgnoreThreatAlways,
	LookAtInvestigate,
	LookAtHeardSound,
	LookAtScanAround,
	LookAtScanAroundOnly,
	Invalid,
	EFocusingBehavior_MAX
};

enum EAILootBlackListReason
{
	Invalid,
	ExecutionError,
	CannotReachLootLocation,
	OutsideSafeZoneRadius,
	NoInventorySpace,
	LootStateUnavailable,
	PathNotFound,
	GoalOffNavmesh,
	AgentBlocked,
	IsolatedIsland,
	UnsupportedItem,
	Count,
	EAILootBlackListReason_MAX
};

enum EBotMovementState
{
	None,
	InProgress,
	Failed_NoPathFound,
	Failed_Aborted,
	Failed_AgentOffNavmesh,
	Failed_GoalOffNavmesh,
	Failed_Blocked,
	Failed_OffPath,
	Failed_Falling,
	Success,
	Success_Partial,
	EBotMovementState_MAX
};

enum EExecutionStatus
{
	ExecutionError,
	ExecutionDenied,
	ExecutionSuccess,
	ExecutionPending,
	ExecutionAllowed,
	EExecutionStatus_MAX
};

enum ELootElementType
{
	Pickup,
	Chest,
	SupplyDrop,
	Invalid,
	ELootElementType_MAX
};

enum ETeleportReason
{
	AgentNotOnNavmesh,
	AgentDestinationNotOnNavMesh,
	AgentStuckInRepetitivePartialPaths,
	AgentBlocked,
	Unknown,
	ETeleportReason_MAX
};

enum EActionState
{
	TryingToEquip,
	EquippingItem,
	UsingItem,
	WaitingItemTermination,
	ActionEndedWithNoError,
	ActionEndedWithError,
	EActionState_MAX
};

enum EFreelookMode
{
	None,
	Mouse,
	Analog,
	EFreelookMode_MAX
};

enum EExitCraftState
{
	None,
	Spawned,
	Landed,
	SpawnBalloon,
	GettingIntoPosition,
	GettingIntoPosition_Simple,
	WaitingForPawns,
	Exiting,
	EExitCraftState_MAX
};

enum EFortExitRequirements
{
	AnyPlayer,
	WholeSquad,
	EFortExitRequirements_MAX
};

enum EFortMutatorReturnValue
{
	Ignore,
	Override,
	OverrideReturn,
	EFortMutatorReturnValue_MAX
};

enum EAshtonStoneStateType
{
	NotSpawned,
	Spawned,
	Captured,
	MAX
};

enum EAshtonStoneType
{
	Purple,
	Blue,
	Red,
	Orange,
	Green,
	Yellow,
	MAX
};

enum EBagelDifficulty
{
	Easy,
	Medium,
	Hard,
	EBagelDifficulty_MAX
};

enum EBagelScoreEvent
{
	FiendKill,
	BruteKill,
	RangedKill,
	ExplodingKill,
	ChillKill,
	PoisonKill,
	GoldKill,
	RiftDestroyed,
	ScoreMultiplierUsed,
	HeadshotKill,
	RespawnPenalty,
	AmmoBoxOpened,
	ChestOpened,
	FinalBossKill,
	MAX
};

enum EBagelPhase
{
	NotStarted,
	Preparation,
	Survival,
	AfterMath,
	Moving,
	FinalPhase,
	FinalPhaseFullStorm,
	MAX
};

enum EBarrierObjectiveDamageState
{
	Health_75,
	Health_50,
	Health_25,
	Health_10,
	Health_5,
	Health_4,
	Health_3,
	Health_2,
	Health_1,
	MAX
};

enum EBarrierFoodTeam
{
	Burger,
	Tomato,
	MAX
};

enum EAllowedToEdit
{
	Default,
	Anyone,
	EAllowedToEdit_MAX
};

enum ECreativeRespawnWaveType
{
	None,
	WaveStartingOnElimination,
	ECreativeRespawnWaveType_MAX
};

enum EFortCrucibleWorkType
{
	Invalid,
	SetupPlayer,
	GetFriendsList,
	GetFriendsStats,
	GetGlobalLeaderboard,
	GetDisplayNames,
	EFortCrucibleWorkType_MAX
};

enum EFortCrucibleLeaderboardState
{
	Disabled,
	ReadyForQuery,
	WaitingForQueryResults,
	NeedsUserInfoQueried,
	Complete,
	EFortCrucibleLeaderboardState_MAX
};

enum EFortCrucibleLeaderboardId
{
	GlobalGamepad,
	GlobalKBM,
	GlobalTouch,
	GlobalAll,
	FriendsGamepad,
	FriendsKBM,
	FriendsTouch,
	FriendsAll,
	Count,
	EFortCrucibleLeaderboardId_MAX
};

enum EFortCrucibleStatSource
{
	None,
	Backend,
	CurrentSession,
	EFortCrucibleStatSource_MAX
};

enum EFortCrucibleStatId
{
	Gamepad_CourseOverall,
	Gamepad_CourseSegment1,
	Gamepad_CourseSegment2,
	Gamepad_CourseSegment3,
	Gamepad_CourseSegment4,
	Gamepad_CourseSegment5,
	KBM_CourseOverall,
	KBM_CourseSegment1,
	KBM_CourseSegment2,
	KBM_CourseSegment3,
	KBM_CourseSegment4,
	KBM_CourseSegment5,
	Touch_CourseOverall,
	Touch_CourseSegment1,
	Touch_CourseSegment2,
	Touch_CourseSegment3,
	Touch_CourseSegment4,
	Touch_CourseSegment5,
	Count,
	EFortCrucibleStatId_MAX
};

enum EFortCrucibleControlType
{
	Gamepad,
	KBM,
	Touch,
	Count,
	EFortCrucibleControlType_MAX
};

enum EFortCrucibleStatType
{
	CourseOverall,
	CourseSegment1,
	CourseSegment2,
	CourseSegment3,
	CourseSegment4,
	CourseSegment5,
	Count,
	EFortCrucibleStatType_MAX
};

enum EDadBroState
{
	NotYet,
	Initializing,
	Active,
	EDadBroState_MAX
};

enum EControlPointState
{
	None,
	Disabled,
	Enabled,
	EControlPointState_MAX
};

enum EEnvironmentDamageFilter
{
	Off,
	PlayerBuiltOnly,
	All,
	EEnvironmentDamageFilter_MAX
};

enum EBuildingDamageTeamFilter
{
	Default,
	OwnerOnly,
	TeamOnly,
	EnemyOnly,
	EnemyAndOwnerOnly,
	None,
	EBuildingDamageTeamFilter_MAX
};

enum EEQSActorSpawnerStopSpawningReason
{
	Success,
	ManualStop,
	Requeued,
	RanOutOfRetries,
	EEQSActorSpawnerStopSpawningReason_MAX
};

enum EEQSActorSpawnerSpawnType
{
	Actor,
	AIPawn,
	Pickup,
	EEQSActorSpawnerSpawnType_MAX
};

enum EEQSActorSpawnerTriggerType
{
	Manual,
	SafeZoneState,
	EEQSActorSpawnerTriggerType_MAX
};

enum EAthenaMutatorEvaluators
{
	NoOverride,
	ForceOverride,
	Add,
	Multiply,
	EAthenaMutatorEvaluators_MAX
};

enum EHeistExitCraftState
{
	None,
	Incoming,
	Spawned,
	Exited,
	EHeistExitCraftState_MAX
};

enum EFortReticleVisibiltyOption
{
	DoNotOverride,
	ShowAlways,
	ShowPickaxeOnly,
	ShowNonPickaxeOnly,
	HideAlways,
	EFortReticleVisibiltyOption_MAX
};

enum EFortHUDElementVisibiltyOption
{
	DoNotOverride,
	ShowElement,
	HideElement,
	EFortHUDElementVisibiltyOption_MAX
};

enum EInfiltrationTeam
{
	Attacking,
	Defending,
	NumOfTeams,
	EInfiltrationTeam_MAX
};

enum EAthenaInventorySpawnOverride
{
	NoOverride,
	Always,
	IntialSpawn,
	AircraftPhaseOnly,
	AfterWarmup,
	EAthenaInventorySpawnOverride_MAX
};

enum EAthenaLootDropOverride
{
	NoOverride,
	ForceDrop,
	ForceKeep,
	ForceDestroy,
	ForceDropUnlessRespawning,
	ForceDestroyUnlessRespawning,
	DropUnlessTeamSelectionUpdated,
	EAthenaLootDropOverride_MAX
};

enum ERespawnRequirements
{
	RespawnOnly,
	NoRespawnOnly,
	Both,
	ERespawnRequirements_MAX
};

enum ECustomLootSelection
{
	Default,
	HighExplosives,
	CloseEncounters,
	SolidGold,
	WildWest,
	SniperShootout,
	OneShot,
	ECustomLootSelection_MAX
};

enum EMashDifficulty
{
	Easy,
	Medium,
	Hard,
	EMashDifficulty_MAX
};

enum EMashScoreEvent
{
	FiendKill,
	BruteKill,
	RangedKill,
	ExplodingKill,
	ChillKill,
	PoisonKill,
	GoldKill,
	RiftDestroyed,
	ScoreMultiplierUsed,
	HeadshotKill,
	RespawnPenalty,
	AmmoBoxOpened,
	ChestOpened,
	FinalBossKill,
	MAX
};

enum EMashPhase
{
	NotStarted,
	Preparation,
	Survival,
	AfterMath,
	Moving,
	FinalPhase,
	FinalPhaseFullStorm,
	MAX
};

enum EMatchConditionMutatorTeamStatus
{
	None,
	Won,
	Lost,
	Placed,
	EMatchConditionMutatorTeamStatus_MAX
};

enum EIndicatorDisplayMode
{
	Default,
	Always,
	Never,
	MiniMap,
	EIndicatorDisplayMode_MAX
};

enum EOmahaTeam
{
	Home,
	Away,
	MAX
};

enum EOperationsTeamFaction
{
	Alter,
	Ego,
	NumFactions,
	EOperationsTeamFaction_MAX
};

enum EPlayerDamageHeightRatioDetectionType
{
	None,
	FromBottom,
	FromTop,
	EPlayerDamageHeightRatioDetectionType_MAX
};

enum ERespawnAndSpectateServerPayloadId
{
	SpectateTargetSelected,
	RespawnTargetSelected,
	SpectateAndRespawnTargetSelected,
	ERespawnAndSpectateServerPayloadId_MAX
};

enum ERespawnAndSpectateClientEventId
{
	ShowUI,
	HideUI,
	ShowRespawnAvailableUI,
	MAX
};

enum EShowPlacardPhase
{
	None,
	StartShow,
	WaitBeforeInitialFadeOut,
	PreShowFadeOut,
	Show,
	FadeOut,
	PostShowFadeIn,
	DoneShowingScreen,
	MAX
};

enum ESkyCapState
{
	Hidden,
	StormStarting,
	ESkyCapState_MAX
};

enum ESynchronizedTeleportHealthAndShieldResetType
{
	None,
	EvaluateHealthAndShieldMutators,
	MaxHealth,
	ESynchronizedTeleportHealthAndShieldResetType_MAX
};

enum EAthenaTODPostProcess
{
	NoOverride,
	Blueprint1,
	Blueprint2,
	Blueprint3,
	Blueprint4,
	Blueprint5,
	Blueprint6,
	Blueprint7,
	Blueprint8,
	Blueprint9,
	Blueprint10,
	Blueprint11,
	EAthenaTODPostProcess_MAX
};

enum EAthenaTODColor
{
	NoOverride,
	Black,
	White,
	Red,
	Green,
	Blue,
	Yellow,
	Magenta,
	Cyan,
	EAthenaTODColor_MAX
};

enum EAthenaFogDensityOverride
{
	NoOverride,
	Fog0,
	Fog1,
	Fog2,
	Fog3,
	Fog4,
	Fog5,
	Fog6,
	Fog7,
	Fog8,
	Fog9,
	Fog10,
	EAthenaFogDensityOverride_MAX
};

enum EAthenaLightIntensityOverride
{
	NoOverride,
	Intensity0,
	Intensity1,
	Intensity2,
	Intensity3,
	Intensity4,
	Intensity5,
	Intensity6,
	Intensity7,
	Intensity8,
	Intensity9,
	Intensity10,
	EAthenaLightIntensityOverride_MAX
};

enum EAthenaTimeOfDayOverride
{
	NoOverride,
	ForceDay,
	ForceNight,
	Custom,
	Hour0,
	Hour1,
	Hour2,
	Hour3,
	Hour4,
	Hour5,
	Hour6,
	Hour7,
	Hour8,
	Hour9,
	Hour10,
	Hour11,
	Hour12,
	Hour13,
	Hour14,
	Hour15,
	Hour16,
	Hour17,
	Hour18,
	Hour19,
	Hour20,
	Hour21,
	Hour22,
	Hour23,
	Random,
	EAthenaTimeOfDayOverride_MAX
};

enum EUraniumGameEndedReason
{
	GameIsStillInProgress,
	AllRoundsPlayed,
	EarlyGameEnd_BlowOut,
	EarlyGameEnd_OutNumbered,
	EUraniumGameEndedReason_MAX
};

enum EFortAthenaMutator_VoiceChatChannelType
{
	Default,
	None,
	SquadOnly,
	TeamOnly,
	WholeServer,
	ScopeOnly,
	ScopeSquadOnly,
	ScopeTeamOnly,
	EFortAthenaMutator_MAX
};

enum EWaxState
{
	None,
	SomewhatVisible,
	ModeratelyVisible,
	VeryVisible,
	Undeniable,
	EWaxState_MAX
};

enum EWaxMinimapDrawMode
{
	NoDrawing,
	DrawCloseAndMoveOrShoot,
	DrawMoveOrShoot,
	DrawAlways,
	EWaxMinimapDrawMode_MAX
};

enum EEncampmentRole
{
	Guard,
	Build,
	Count,
	EEncampmentRole_MAX
};

enum EPatrollingMode
{
	BackAndForth,
	Loop,
	EPatrollingMode_MAX
};

enum ECoastState
{
	Idle,
	Mount,
	Coasting,
	Pedaling,
	PreDismount,
	Dismount,
	EndCoast,
	ECoastState_MAX
};

enum EFortAthenaTutorial_StandaloneStep
{
	None,
	HealingInterrupted,
	Count,
	EFortAthenaTutorial_MAX
};

enum EDebugVehicleFlags
{
	Status,
	Input,
	Shocks,
	Exits,
	Water,
	Wheels,
	Friction,
	AirControl,
	CenterOfMass,
	Gravity,
	Forces,
	Damage,
	Collisions,
	OrientationCorrection,
	BoundarySpline,
	Sleeping,
	Misc,
	LeanBreak,
	EDebugVehicleFlags_MAX
};

enum EBounceCompressionState
{
	None,
	Crouching,
	Crouched,
	Jumping,
	Recoiling,
	EBounceCompressionState_MAX
};

enum ENaturalSlideState
{
	None,
	Entering,
	InProgress,
	Exiting,
	ENaturalSlideState_MAX
};

enum EPowerSlideState
{
	None,
	Entering,
	InProgress,
	Exiting,
	EPowerSlideState_MAX
};

enum EFortFuelGadgetVisualType
{
	FuelMeter,
	ChargeText,
	Invalid,
	EFortFuelGadgetVisualType_MAX
};

enum EFortAttributeDisplay
{
	BasicInt,
	NegativeImpliesInfiniteInt,
	BasicFloat,
	NegativeImpliesInfiniteFloat,
	BasicString,
	NormalizedPercentage,
	StringArray,
	SlateBrush,
	DoNotDisplay,
	None_Max,
	EFortAttributeDisplay_MAX
};

enum EAudioAnalysisParameterTypes
{
	Scalar,
	Vector4,
	Count,
	EAudioAnalysisParameterTypes_MAX
};

enum EVectorCurveType
{
	Original,
	Modified,
	EVectorCurveType_MAX
};

enum EVectorCurveFloat
{
	VectorCurve_X,
	VectorCurve_Y,
	VectorCurve_Z,
	VectorCurve_MAX
};

enum EAudioShapeComponentState
{
	Inactive,
	Active,
	Count,
	EAudioShapeComponentState_MAX
};

enum EBiplaneSimEvent
{
	EngineStart,
	EngineStop,
	Takeoff,
	Landing,
	BoostBegin,
	BoostEnd,
	BoostChargeAboveThreshold,
	AileronRoll,
	ControlContextChange,
	EBiplaneSimEvent_MAX
};

enum EAileronRollDirection
{
	None,
	Right,
	Left,
	EAileronRollDirection_MAX
};

enum EDoghouseControlMode
{
	GroundControls,
	AirControls,
	MaxCount,
	EDoghouseControlMode_MAX
};

enum EIDScannerResult
{
	Success_PlayerBelongsToFaction,
	Success_DownedPlayer,
	Success_DownedGuard,
	Success_PlayerIsDisguised,
	Failure_NotFromFaction,
	Other,
	EIDScannerResult_MAX
};

enum EEnvironmentalItemEndReason
{
	None,
	PlayerExit,
	PlayerDeath,
	ObjectDestroyed,
	EEnvironmentalItemEndReason_MAX
};

enum ESpawnResult
{
	Success,
	Failure_NoLocationFound,
	Failure_NoActorClass,
	Failure_BadQueryData,
	ESpawnResult_MAX
};

enum EFortBuildingSoundType
{
	Construction,
	GenericDestruction,
	PlayerBuiltDestruction,
	None,
	EFortBuildingSoundType_MAX
};

enum ESmartBuildMode
{
	None,
	Auto,
	Box,
	Bridge,
	Tower,
	ESmartBuildMode_MAX
};

enum EFortUICameraFrameTargetBoundingBehavior
{
	NoBounds,
	AllBounds,
	TopBoundOnly,
	EFortUICameraFrameTargetBoundingBehavior_MAX
};

enum ECameraOrigin
{
	ViewTargetTransform,
	BoneTransform,
	ECameraOrigin_MAX
};

enum EChallengeBundleQuestUnlockType
{
	Manually,
	GrantWithBundle,
	RequiresBattlePass,
	DaysFromEventStart,
	ChallengesCompletedToUnlock,
	BundleLevelup,
	EChallengeBundleQuestUnlockType_MAX
};

enum EChallengeScheduleUnlockType
{
	Manually,
	OnScheduleGranted,
	DaysSinceEventStart,
	EChallengeScheduleUnlockType_MAX
};

enum EBundleStyleColor
{
	Primary,
	Secondary,
	Accent,
	EBundleStyleColor_MAX
};

enum EManagedCosmeticType
{
	Glider,
	GliderAnimSet,
	Pickaxe,
	Pet,
	ItemWrap,
	Charm,
	MAX_COUNT,
	EManagedCosmeticType_MAX
};

enum EFortCustomMovement
{
	Default,
	Driving,
	Passenger,
	Parachuting,
	Skydiving,
	SkydiveFollowing,
	Hover,
	RemoteControl_Flying,
	Ziplining,
	ZipliningOnSpline,
	Ballooning,
	SurfaceSwimming,
	DBNOCarried,
	Floating,
	Goop,
	Count,
	EFortCustomMovement_MAX
};

enum EOstrichDetonationState
{
	None,
	Detonate,
	SelfDestruct,
	Instant,
	EOstrichDetonationState_MAX
};

enum EChatRoomJoinHelperState
{
	Ready,
	AttemptingJoin,
	Joined,
	AttemptingLeave,
	EChatRoomJoinHelperState_MAX
};

enum EFortAnnouncementDisplayPreference
{
	Default_HUD,
	QuestIntroduction,
	QuestJournal,
	EFortAnnouncementDisplayPreference_MAX
};

enum EFortAnnouncementChannel
{
	Primary,
	Conversation,
	Tutorial,
	Max_None,
	EFortAnnouncementChannel_MAX
};

enum EFortAnnouncementDelivery
{
	Created,
	Received,
	Ignored,
	Active,
	Stopped,
	Max_None,
	EFortAnnouncementDelivery_MAX
};

enum ECreativeBotItemTestingState
{
	ITS_NONE,
	ITS_Teleporting,
	ITS_Landing,
	ITS_Grant,
	ITS_Equip,
	ITS_Place,
	ITS_Cleanup,
	ITS_MAX
};

enum ECreativeBotIslandLoadingState
{
	ILS_NONE,
	ILS_LoadNotStarted,
	ILS_GrantPlotItem,
	ILS_WaitingUserPlotReady,
	ILS_LoadInProgress,
	ILS_LoadComplete,
	ILS_Teleporting,
	ILS_Returning,
	ILS_Items,
	ILS_MAX
};

enum ECreativeBotIslandIterationType
{
	CBI_NONE,
	CBI_Mnemonics,
	CBI_User,
	CBI_MAX
};

enum ELicensedAudioTreatment
{
	None,
	MuteOthers,
	MuteAll,
	ELicensedAudioTreatment_MAX
};

enum ESelectableVoiceChatSetting
{
	Off,
	FriendsOnly,
	Everyone,
	ESelectableVoiceChatSetting_MAX
};

enum EFortAutoMulchMode
{
	Off,
	Common,
	Uncommon,
	Rare,
	Epic,
	Num,
	EFortAutoMulchMode_MAX
};

enum EFortAutoMulchCategory
{
	Invalid,
	Weapon,
	Trap,
	Survivor,
	Defender,
	Hero,
	Num,
	EFortAutoMulchCategory_MAX
};

enum EQuestMapScreenMode
{
	Invalid,
	MainCampaign,
	Event,
	EQuestMapScreenMode_MAX
};

enum EFortAllowBackgroundAudioSetting
{
	Off,
	NotificationsOnly,
	AllSounds,
	Num,
	EFortAllowBackgroundAudioSetting_MAX
};

enum EAndroidAppStoreTypes
{
	Unset,
	Epic,
	Samsung,
	EAndroidAppStoreTypes_MAX
};

enum EColorBlindMode
{
	Off,
	Deuteranope,
	Protanope,
	Tritanope,
	EColorBlindMode_MAX
};

enum ECloudFileState
{
	Unitialized,
	Saving,
	Loading,
	Idle,
	ECloudFileState_MAX
};

enum ECodeTokenPlatform
{
	PC,
	PS4,
	XBOX,
	ECodeTokenPlatform_MAX
};

enum EFortCollectionBookState
{
	Active,
	Completed,
	Claimed,
	EFortCollectionBookState_MAX
};

enum EFortPIDValueGraphElements
{
	Proportional,
	Integral,
	Max_None,
	EFortPIDValueGraphElements_MAX
};

enum EFortIntensityGraphElements
{
	ActualIntensity,
	DesiredIntensity,
	Max_None,
	EFortIntensityGraphElements_MAX
};

enum EFortContributionGraphElements
{
	ProportionalLine,
	IntegralLine,
	TotalLine,
	PendingLine,
	ActionLine,
	Max_None,
	EFortContributionGraphElements_MAX
};

enum EFortFactorContributionType
{
	CurrentValue_Direct,
	CurrentValue_Inverse,
	AverageValue_Direct,
	AverageValue_Inverse,
	EFortFactorContributionType_MAX
};

enum EFortAIDirectorFactor
{
	PlayerDamageThreat,
	ObjectiveDamageThreat,
	ObjectivePathCost,
	PlayerPathCost,
	PlayerMovement,
	TrapsEffective,
	PlayerWander,
	NearbyEnemyPresence,
	OffensiveResources,
	DefensiveResources,
	Boredom,
	ArtilleryVulnerability,
	Max_None,
	EFortAIDirectorFactor_MAX
};

enum EFortCombatFactors
{
	PlayerDamageThreat,
	ObjectiveDamageThreat,
	ObjectivePathCost,
	PlayerPathCost,
	PlayerMovement,
	TrapsEffective,
	PlayerWander,
	NearbyEnemyPresence,
	OffensiveResources,
	DefensiveResources,
	Boredom,
	ArtilleryVulnerability,
	Max_None,
	EFortCombatFactors_MAX
};

enum EFortCombatEventContribution
{
	Linear,
	Inverse,
	Accumulator,
	Max_None,
	EFortCombatEventContribution_MAX
};

enum EFortCombatEvents
{
	HuskFollowing,
	SmasherFollowing,
	TrollFollowing,
	FlingerFollowing,
	TakerFollowing,
	HuskCombatNearby,
	SmasherCombatNearby,
	TrollCombatNearby,
	FlingerCombatNearby,
	TakerCombatNearby,
	PlayerTakeDamage,
	PlayerHealth,
	PlayerLookAtEnemy,
	PlayerDamageEnemy,
	PlayerKilledEnemy,
	AtlasTakeDamage,
	AtlasHealth,
	AtlasDestroyed,
	TrapFiring,
	BuildingTakeDamage,
	FoodHealingPotential,
	PlayerAmmo,
	EnemiesNear,
	PlayerMovement,
	BuildingDamagedNearObjective,
	TrapDamageEnemy,
	ObjectivePathCost,
	PlayerPathCost,
	Max_None,
	EFortCombatEvents_MAX
};

enum EFortAIDirectorEventParticipant
{
	Target,
	Source,
	Either,
	Max_None,
	EFortAIDirectorEventParticipant_MAX
};

enum EFortCombatThresholds
{
	Low,
	Medium,
	High,
	Extreme,
	Max_None,
	EFortCombatThresholds_MAX
};

enum EFortWeaponCoreAnimation
{
	Melee,
	Pistol,
	Shotgun,
	PaperBlueprint,
	Rifle,
	MeleeOneHand,
	MachinePistol,
	RocketLauncher,
	GrenadeLauncher,
	GoingCommando,
	AssaultRifle,
	TacticalShotgun,
	SniperRifle,
	TrapPlacement,
	ShoulderLauncher,
	AbilityDecoTool,
	Crossbow,
	C4,
	RemoteControl,
	DualWield,
	AR_BullPup,
	AR_ForwardGrip,
	MedPackPaddles,
	SMG_P90,
	AR_DrumGun,
	Consumable_Small,
	Consumable_Large,
	Balloon,
	MountedTurret,
	CreativeTool,
	ExplosiveBow,
	AshtonIndigo,
	AshtonChicago,
	MeleeDualWield,
	MAX
};

enum EFortTargetingFXState
{
	TargetingStart,
	TargetingEnd,
	Max_None,
	EFortTargetingFXState_MAX
};

enum EFortReloadFXState
{
	ReloadStart,
	ReloadCartridge,
	ReloadEnd,
	Max_None,
	EFortReloadFXState_MAX
};

enum EFortWeaponSoundChannel
{
	NormalA,
	NormalB,
	LowAmmo,
	Degraded,
	Max_None,
	EFortWeaponSoundChannel_MAX
};

enum EFortWeaponSoundState
{
	Normal,
	LowAmmo,
	Degraded,
	Max_None,
	EFortWeaponSoundState_MAX
};

enum EFortMontageInputType
{
	WindowClickOrHold,
	WindowHoldOnly,
	InstantClick,
	EFortMontageInputType_MAX
};

enum EFortAmmoType
{
	Pistol,
	Shotgun,
	Assault,
	Sniper,
	Energy,
	EFortAmmoType_MAX
};

enum EFortConditionalResourceItemTest
{
	CanEarnMtx,
	EFortConditionalResourceItemTest_MAX
};

enum EFortContentEncryptionAllowedReferences
{
	None,
	SoftOnly,
	Any,
	EFortContentEncryptionAllowedReferences_MAX
};

enum EFortContentEncryptionCollectionGrouping
{
	Individual,
	Combined,
	ByCosmeticSet,
	EFortContentEncryptionCollectionGrouping_MAX
};

enum EFortEncryptionStatus
{
	ENCRYPTED,
	RELEASED,
	EFortEncryptionStatus_MAX
};

enum EFortContextualTutorialPlatform
{
	Any,
	PcAndConsole,
	Mobile,
	EFortContextualTutorialPlatform_MAX
};

enum ECollectionsComponentValidityResult
{
	IsValid,
	IsNotValid,
	ECollectionsComponentValidityResult_MAX
};

enum EContextualEvent
{
	Generic,
	Location,
	InventoryAdded,
	InventoryRemoved,
	StartSkydiving,
	NewQuests,
	EContextualEvent_MAX
};

enum EContextualContext
{
	DoNotShow,
	BusPhase,
	Skydiving,
	Gameplay,
	EContextualContext_MAX
};

enum EShareActorWithMask
{
	None,
	SquadOnTeam,
	AllTeam,
	Target,
	EShareActorWithMask_MAX
};

enum EShareActorWith
{
	None,
	SquadOnTeam,
	AllTeam,
	EShareActorWith_MAX
};

enum EIndicatorStateImage
{
	FIRST_FRIENDLY_STATE,
	Default,
	InCombat,
	DBNO,
	BeingRevived,
	Dead,
	LAST_FRIENDLY_STATE,
	FIRST_CHAT_MESSAGE,
	NeedAmmoHeavy,
	NeedAmmoLight,
	NeedAmmoMedium,
	NeedAmmoShells,
	NeedAmmoRocket,
	ChatBubble,
	EnemySpotted,
	NeedBandages,
	NeedMaterials,
	NeedShields,
	NeedWeapon,
	LAST_CHAT_MESSAGE,
	FIRST_MATE_STATE,
	Squadmate,
	Teammate,
	LAST_MATE_STATE,
	FIRST_ENEMY_STATE,
	Enemy,
	LAST_ENEMY_STATE,
	FIRST_NPC_STATE,
	FriendlyNPC,
	EnemyNPC,
	EliteFriendlyNPC,
	EliteEnemyNPC,
	LAST_NPC_STATE,
	FIRST_WORLDITEM_STATE,
	Interactable,
	InteractableLarge,
	GameplayItem,
	TreasureChest,
	LAST_WORLDITEM_STATE,
	FIRST_HARDCORE_STATE,
	HardCoreBeacon,
	LAST_HARDCORE_STATE,
	LAST_STATE,
	None,
	MAX
};

enum ETouchInteractMode
{
	Off,
	InWorld,
	Buttons,
	ETouchInteractMode_MAX
};

enum EStashInventoryServiceSyncState
{
	Uninitialized,
	Syncing,
	Ready,
	EStashInventoryServiceSyncState_MAX
};

enum EMinigameActivityStat
{
	Score,
	Time,
	Distance,
	RaceProgress,
	CurrentLap,
	MaxLaps,
	BestLapTime,
	COUNT,
	EMinigameActivityStat_MAX
};

enum EMinigameActivityComponentValidityResult
{
	Valid,
	IsNotValid,
	EMinigameActivityComponentValidityResult_MAX
};

enum ESkydiveFeedbackPhase
{
	Initial,
	WithGlider,
	InVortex,
	None,
	ESkydiveFeedbackPhase_MAX
};

enum EConversationEventQueryMethod
{
	CheckAgainstCurrentConversationParticipant,
	CheckAgainstConversationEntryTag,
	EConversationEventQueryMethod_MAX
};

enum EFortSentenceAudioPreference
{
	AudioAsset,
	FeedbackBank,
	EFortSentenceAudioPreference_MAX
};

enum EContextRequirementMatchPolicy
{
	RequireAll,
	RequireAny,
	EContextRequirementMatchPolicy_MAX
};

enum EPreventAbilityUseReason
{
	CannotAfford,
	AlreadyActive,
	AbilityActivationBlocked,
	None,
	Count,
	EPreventAbilityUseReason_MAX
};

enum ERequirementMatchPolicy
{
	RequireAll,
	RequireAny,
	ERequirementMatchPolicy_MAX
};

enum EDataDrivenEffectRecipient
{
	Player,
	NPC,
	EDataDrivenEffectRecipient_MAX
};

enum EPreventUseStormCircleServiceReason
{
	CannotAfford,
	AlreadyActive,
	StormLocationsAlreadyRevealed,
	None,
	EPreventUseStormCircleServiceReason_MAX
};

enum EPreventSupplyDropUseReason
{
	CannotAfford,
	OutOfStock,
	None,
	Count,
	EPreventSupplyDropUseReason_MAX
};

enum ESupplyDropSpawnLocationPolicy
{
	RadiusAroundLocation,
	SafeZone,
	ESupplyDropSpawnLocationPolicy_MAX
};

enum EFCRP_LoopBehavior
{
	StopAtEnd,
	PingPong,
	WrapAround,
	EFCRP_MAX
};

enum EFortRichColorConflictResolutionRules
{
	NoConflictsAllowed,
	BlackOrWhiteCannotConflict,
	EFortRichColorConflictResolutionRules_MAX
};

enum EFortCosmeticSwapRequirementPart
{
	None,
	Glider,
	Pickaxe,
	Backpack,
	Character,
	FullLoadout,
	MAX
};

enum ELoadoutVariantInsertType
{
	StartOfArray,
	EndOfArray,
	ELoadoutVariantInsertType_MAX
};

enum EAnimInstanceClassSwapType
{
	None,
	SwapOnMatch,
	EAnimInstanceClassSwapType_MAX
};

enum EAssetChangeMethod
{
	Latest,
	Minimum,
	OnlyAddNew,
	BelowDoubleTheFixedCost,
	EAssetChangeMethod_MAX
};

enum EFortCreativeDiscoverySkippedEntries
{
	None,
	ByCount,
	ByPercent,
	EFortCreativeDiscoverySkippedEntries_MAX
};

enum EFortCreativeDiscoveryPanelType
{
	CuratedList,
	MetricDriven,
	Recommendations,
	EFortCreativeDiscoveryPanelType_MAX
};

enum EFortCreativeDiscoveryDeterminism
{
	Always,
	Random,
	PlayerDeterministic,
	PartyDeterministic,
	EpicEmployee,
	Never,
	EFortCreativeDiscoveryDeterminism_MAX
};

enum EFortLockDeviceVisibilityDuringGames
{
	No,
	Yes,
	HologramOnly,
	EFortLockDeviceVisibilityDuringGames_MAX
};

enum EBuildingAsPropSetting
{
	None,
	SnapToEdge,
	SnapToCenter,
	EBuildingAsPropSetting_MAX
};

enum ECameraSpaceHoldPosition
{
	AsPickup,
	Left,
	Center,
	Right,
	ECameraSpaceHoldPosition_MAX
};

enum EAddToSelectionResult
{
	Added,
	Removed,
	AtLimit,
	None,
	EAddToSelectionResult_MAX
};

enum EHitTraceType
{
	Single,
	Multi,
	EHitTraceType_MAX
};

enum ECreativePortalManagerValidityResult
{
	Valid,
	Invalid,
	ECreativePortalManagerValidityResult_MAX
};

enum EFortCreativePlotPermission
{
	Private,
	Public,
	EFortCreativePlotPermission_MAX
};

enum ERealEstateOffsetType
{
	CustomOffsetFromCorner,
	Center,
	ERealEstateOffsetType_MAX
};

enum EFortSourDropperColorType
{
	None,
	Bright,
	MidBright,
	MidDark,
	Dark,
	EFortSourDropperColorType_MAX
};

enum EFortCreativeTeleporterEvent
{
	Entered,
	Exited,
	Enabled,
	Disabled,
	None,
	EFortCreativeTeleporterEvent_MAX
};

enum EFortCreativeTeleporterGroup
{
	Group_A,
	Group_B,
	Group_C,
	Group_D,
	Group_E,
	Group_F,
	Group_G,
	Group_H,
	Group_I,
	Group_J,
	Group_K,
	Group_L,
	Group_M,
	Group_N,
	Group_O,
	Group_P,
	Group_Q,
	Group_R,
	Group_S,
	Group_T,
	Group_U,
	Group_V,
	Group_W,
	Group_X,
	Group_Y,
	Group_Z,
	Group_None,
	Group_MAX
};

enum EMMSPlayersPerTeamPreset
{
	Solos,
	Duos,
	Trios,
	Squads,
	SplitEvenly,
	Dynamic,
	EMMSPlayersPerTeamPreset_MAX
};

enum EMMSRulePreset
{
	RespectParties,
	KeepFull,
	Off,
	EMMSRulePreset_MAX
};

enum EFortCurieApplicationEvent
{
	OnHit,
	OnBeginOverlap,
	OnEndOverlap,
	MaxValue,
	EFortCurieApplicationEvent_MAX
};

enum EFortCurieExecutionType
{
	Application,
	Interaction,
	EFortCurieExecutionType_MAX
};

enum EFortNativeCurieFXCueResponse
{
	IgnoreCue,
	AllowCue,
	OverrideCue,
	EFortNativeCurieFXCueResponse_MAX
};

enum ECurieManagerComponentPriority
{
	Priority_1,
	Priority_2,
	Priority_3,
	Priority_4,
	Priority_5,
	Priority_6,
	Priority_7,
	Priority_8,
	Priority_9,
	Priority_10,
	Priority_Default,
	Priority_MAX
};

enum EFortCurieToggleComponentDeactivationBehavior
{
	NeverDeactivate,
	TimedDeactivationAllowRefresh,
	TimedDeactivationNoRefresh,
	EFortCurieToggleComponentDeactivationBehavior_MAX
};

enum EFortCurieToggleComponentActivationBehavior
{
	OnValidAttachment,
	EFortCurieToggleComponentActivationBehavior_MAX
};

enum EFortCurieNativeFXType
{
	None,
	Electricity,
	Fire,
	Charred,
	EFortCurieNativeFXType_MAX
};

enum ETimespanUnitDisplayFormat
{
	Full,
	Abbreviated,
	ETimespanUnitDisplayFormat_MAX
};

enum ETimespanAsTextFormat
{
	DaysHoursMinutesSeconds,
	Colons,
	ColonsWithMilliseconds,
	Approximate,
	ApproximateWithWeeks,
	ApproximateWithMonths,
	ApproximateWithYears,
	ETimespanAsTextFormat_MAX
};

enum EDeathCauseReason
{
	SelfInflicted,
	SelfInflictedDBNO,
	Eliminated,
	EliminatedDBNO,
	EDeathCauseReason_MAX
};

enum EFortDefenderSubtype
{
	AssaultRifle,
	Pistol,
	Melee,
	Sniper,
	Shotgun,
	Invalid,
	EFortDefenderSubtype_MAX
};

enum EHordeTierStartStatus
{
	ReadyToStart,
	WaitingForPlayer,
	WaitingForDBM,
	GenericNotReadyToStart,
	EHordeTierStartStatus_MAX
};

enum EHordeWaveStingerType
{
	WaveSuccess,
	WaveFailure,
	WaveIncoming,
	WaveStarted,
	EHordeWaveStingerType_MAX
};

enum EQueueActionType
{
	Plot,
	ZoneCleanup,
	EnvironmentActorRestoration,
	EQueueActionType_MAX
};

enum EFDynamicBuildOrder
{
	X,
	Y,
	Z,
	None,
	FDynamicBuildOrder_MAX
};

enum ENavOptionFallbackDir
{
	Left,
	Right,
	Up,
	Down,
	Num,
	Invalid,
	ENavOptionFallbackDir_MAX
};

enum EFortFactionAttitude
{
	Friendly,
	Neutral,
	Hostile,
	MAX
};

enum EFortFeedbackBroadcastFilter
{
	FFBF_Speaker,
	FFBF_SpeakerTeam,
	FFBF_SpeakerAdressee,
	FFBF_HumanPvP_Team1,
	FFBF_HumanPvP_Team2,
	FFBF_None_Max,
	FFBF_MAX
};

enum EFortFeedbackSelectionMethod
{
	FFSM_Instigator,
	FFSM_Recipient,
	FFSM_TeamWitness,
	FFSM_EnemyWitness,
	FFSM_Random,
	FFSM_Priority_IRTE,
	FFSM_AllPawns,
	FFSM_Announcer,
	FFSM_MAX
};

enum EFortFeedbackAddressee
{
	FFA_Instigator,
	FFA_Recipient,
	FFA_All,
	FFA_MAX
};

enum EFortFeedbackContext
{
	FFC_Instigator,
	FFC_Recipient,
	FFC_TeamWitness,
	FFC_EnemyWitness,
	FFC_AllPawns,
	FFC_Announcer,
	FFC_None_Max,
	FFC_MAX
};

enum EFortFoleyHitAudioType
{
	Body,
	Shield,
	Crit,
	Death,
	DeathCrit,
	Fall,
	FallDeath,
	Max_None,
	EFortFoleyHitAudioType_MAX
};

enum EFortFootstepPosition
{
	Parallel,
	Above,
	Below,
	AboveOrBelowAndVisible,
	Max_None,
	EFortFootstepPosition_MAX
};

enum EFortFootstepSurfaceType
{
	Default,
	Wood,
	Stone,
	Metal,
	Water,
	Snow,
	Ice,
	Lava,
	Dirt,
	Grass,
	Sand,
	Max_None,
	EFortFootstepSurfaceType_MAX
};

enum EFortFootstepAudioType
{
	Crouch,
	CrouchSprint,
	Walk,
	Sprint,
	Jump,
	Land,
	LandHard,
	Max_None,
	EFortFootstepAudioType_MAX
};

enum EFriendChestTimePeriod
{
	Daily,
	Weekly,
	EFriendChestTimePeriod_MAX
};

enum EFriendChestInstancedType
{
	ItemDefinition,
	FreshDripCounter,
	GrantXp,
	Undefined,
	EFriendChestInstancedType_MAX
};

enum EFortGameActivityType
{
	Undefined,
	STW,
	BR,
	LTM,
	CreativePublishedIsland,
	CreativePersonalIsland,
	Dummy,
	EFortGameActivityType_MAX
};

enum EFortActivityValidationResult
{
	NotFound,
	InvalidKeyTooShort,
	InvalidKeyCharacters,
	IneligibleParty,
	IslandPrivate,
	CreativePublishedSuccess,
	EFortActivityValidationResult_MAX
};

enum ESubGameAccessReason
{
	NoAccess,
	OpenAccess,
	TokenItemAccess,
	XboxHomeSharingAccess,
	XboxServiceOutageAccess,
	LimitedAccess,
	ESubGameAccessReason_MAX
};

enum EClampType
{
	Minimum,
	Maximum,
	EClampType_MAX
};

enum ERatingsEnforcementType
{
	Default,
	IgnoreMaximums,
	IgnoreParty,
	IgnorePartyMaximum,
	ERatingsEnforcementType_MAX
};

enum EDynamicSoundOverride
{
	Cue,
	Wave,
	Class,
	EDynamicSoundOverride_MAX
};

enum EPlayerQueueType
{
	Player,
	BroadcastSpectator,
	EPlayerQueueType_MAX
};

enum EFortInputFilterLevel
{
	None,
	Touch,
	Gamepad,
	Mouse,
	EFortInputFilterLevel_MAX
};

enum EAircraftLaunchReason
{
	StdTimerAllPlayers,
	EarlyTimerAllPlayers,
	StdTimerMostPlayers,
	EarlyTimerMostPlayers,
	StdTimerFewPlayers,
	EAircraftLaunchReason_MAX
};

enum EFortGamepadLookInputCurve
{
	Linear,
	Exponential,
	EFortGamepadLookInputCurve_MAX
};

enum EFortGamepadSensitivity
{
	Invalid,
	Slow,
	SlowPlus,
	SlowPlusPlus,
	Normal,
	NormalPlus,
	NormalPlusPlus,
	Fast,
	FastPlus,
	FastPlusPlus,
	Insane,
	MAX
};

enum EFortAbilityCostSource
{
	Stamina,
	Durability,
	AmmoMagazine,
	AmmoPrimary,
	Item,
	EFortAbilityCostSource_MAX
};

enum EFortGameplayAbilityActivation
{
	Passive,
	Triggered,
	Active,
	EFortGameplayAbilityActivation_MAX
};

enum EFortGetPlayerPawnExecutions
{
	ValidFortPlayerPawn,
	AvatarCastFailed,
	EFortGetPlayerPawnExecutions_MAX
};

enum EFortAIWeaponUsage
{
	NoWeaponUsage,
	UsesRangedWeapon,
	UsesMeleeWeapon,
	EFortAIWeaponUsage_MAX
};

enum EFortGameplayAbilityMontageSectionToPlay
{
	FirstSection,
	RandomSection,
	TestedRandomSection,
	EFortGameplayAbilityMontageSectionToPlay_MAX
};

enum EJumpBoostPackState
{
	Idle,
	Boost,
	Hovering,
	Falling,
	None,
	EJumpBoostPackState_MAX
};

enum EMedicPackState
{
	Idle,
	Active,
	None,
	EMedicPackState_MAX
};

enum EFortGameplayCueSourceCondition
{
	AnySource,
	LocalPlayerSource,
	NonLocalPlayerSource,
	EFortGameplayCueSourceCondition_MAX
};

enum EFortGameplayCueAttachType
{
	AttachToTarget,
	DoNotAttach,
	EFortGameplayCueAttachType_MAX
};

enum EFortGameplayDataTrackerEventContributionType
{
	Accumulate,
	Set,
	EFortGameplayDataTrackerEventContributionType_MAX
};

enum ESetCVarType
{
	Numeric,
	String,
	ESetCVarType_MAX
};

enum EFortServerGameMode
{
	Idle,
	LobbyPvE,
	LobbyPvP,
	ZonePvP,
	ZonePvE,
	EFortServerGameMode_MAX
};

enum EFortBanHammerNotificationAction
{
	BanAndKick,
	Kick,
	EFortBanHammerNotificationAction_MAX
};

enum EFortServerContentRestartReason
{
	None,
	CreativeCuratedHubChanged,
	CreativeFeaturedIslandsChanged,
	CreativePreloadRevisionChanged,
	CreativePlaylistConditionalFlagsChanged,
	GameFeaturePluginDisabled,
	ForceRestartEventFlagsChanged,
	ForceRestartFlagActiveStateChanged,
	EFortServerContentRestartReason_MAX
};

enum EServerRestartReason
{
	HotfixApplied,
	GracefulShutdown,
	BeaconJoinDelayRestart,
	Other,
	EServerRestartReason_MAX
};

enum ETeamChatRoomState
{
	NotCreated,
	Creating,
	Created,
	Timeout,
	ETeamChatRoomState_MAX
};

enum EPlayerIndicatorFlags
{
	None,
	Minimap,
	WorldArrow,
	WorldName,
	DBNOCountDown,
	EPlayerIndicatorFlags_MAX
};

enum EAthenaAerialPhase
{
	None,
	BusCantExit,
	BusCanExit,
	BusCanExitEndZebulonDrone,
	Skydiving,
	Parachuting,
	Falling,
	EAthenaAerialPhase_MAX
};

enum ERadiusTrackingGroupingType
{
	Global,
	Team,
	Squad,
	ERadiusTrackingGroupingType_MAX
};

enum EAnalyticMatchCounts
{
	IDScannerDoorFailed,
	IDSCannerDoorSuccessBelongsToFaction,
	IDSCannerDoorSuccessPlayerDisguised,
	IDSCannerDoorSuccessCarriedHenchman,
	IDSCannerDoorSuccessCarriedHuman,
	IDScannerChestFailed,
	IDSCannerChestSuccessBelongsToFaction,
	IDSCannerChestSuccessPlayerDisguised,
	IDSCannerChestSuccessCarriedHenchman,
	IDSCannerChestSuccessCarriedHuman,
	HenchmenEnteredAlertedState,
	HenchmenEnteredLKPState,
	HenchmenEnteredThreatenedState,
	HenchmenDowned,
	HenchmenEliminated,
	HenchmenInterrogated,
	BossesEliminated,
	DisguisePhoneBoothTimesEntered,
	DisguiseItemTimesUsed,
	HiddenPassagesTimesEntered,
	CameraOrSentryTimesEnteredCautionState,
	CameraOrSentryTimesEnteredAltertedState,
	CameraOrSentryTimesDestroyed,
	GeneratorTimesDisabled,
	UmbrellaNumberOfDashes,
	UmbrellaNumberUsedToFloat,
	UmbrellaAmountDamageBlocked,
	UmbrellaBulletsBlocked,
	UmbrellaMeleeHitsBlocked,
	Count,
	EAnalyticMatchCounts_MAX
};

enum EDefenderSpawnFailureReason
{
	None,
	AllPlayerSlotsFull,
	DefendersNotUnlocked,
	CurrentlySimulatingDefender,
	NotOutpostOwner,
	EDefenderSpawnFailureReason_MAX
};

enum ERHIType
{
	D3D11,
	D3D12,
	Performance,
	ERHIType_MAX
};

enum EShowInGamePictureInPicture
{
	Default,
	Hide,
	Show,
	EShowInGamePictureInPicture_MAX
};

enum EFortScalabilityMode
{
	LowPower,
	Frontend,
	EFortScalabilityMode_MAX
};

enum ESavedAccountType
{
	None,
	Facebook,
	Google,
	Epic,
	Device,
	Headless,
	Refresh,
	ESavedAccountType_MAX
};

enum EFortMobileFPSMode
{
	Mode_20Fps,
	Mode_30Fps,
	Mode_45Fps,
	Mode_60Fps,
	Mode_90Fps,
	Mode_120Fps,
	Num,
	EFortMobileFPSMode_MAX
};

enum EFortOfferSeenLevel
{
	Unseen,
	Notification,
	ItemShopVisited,
	OfferSectionVisited,
	Purchased,
	EFortOfferSeenLevel_MAX
};

enum EFortGiftWrapType
{
	System,
	UserFree,
	UserUnlock,
	UserConsumable,
	Message,
	Ungift,
	EFortGiftWrapType_MAX
};

enum ETimeLimitForReplayCinematic
{
	OpenTimeLimit,
	DurationTimeLimit,
	DurationExtraTime,
	ETimeLimitForReplayCinematic_MAX
};

enum EFortGlobalAction
{
	TrapConfirm,
	TrapPicker,
	BuildConfirm,
	PerformBuildingEditInteraction,
	PerformBuildingImprovementInteraction,
	SwitchQuickBar,
	Use,
	Reload,
	InventoryOrChatHold,
	GamepadChangeMaterialOrHarvestHold,
	GamepadNextWeaponOrHarvestHold,
	GamepadNextWeaponOrAltInteractOrHarvestHold,
	ChangeMaterial,
	Fire,
	RotatePrimitiveClockwise,
	Gadget1,
	Gadget2,
	Ability1,
	Ability2,
	Ability3,
	ToggleFullScreenMap,
	ToggleInventory,
	Jump,
	Crouch,
	ShoppingCartCoast,
	GolfCartEBrake,
	GolfCartForward,
	GolfCartReverse,
	GolfCartHonk,
	GamepadToggleHarvestOrHoldCreativePhone,
	GamepadToggleCreativePhoneWeapon,
	Count,
	EFortGlobalAction_MAX
};

enum EFortClientUpdateType
{
	Client,
	ContentOnly,
	EFortClientUpdateType_MAX
};

enum EContentInstallState
{
	NotInstalled,
	Pending,
	Installed,
	Unknown,
	Error,
	EContentInstallState_MAX
};

enum EFortAccountLinkingUIConfig
{
	Disabled,
	Default,
	ExternalViewerOnly,
	FullExternal,
	EFortAccountLinkingUIConfig_MAX
};

enum EGravityGunHolderRotationAxis
{
	XZ,
	YZ,
	ZX,
	MAX
};

enum EGravityGunHolderObjectType
{
	Invalid,
	PhysicsObject,
	Vehicle,
	Projectile,
	PickUp,
	EGravityGunHolderObjectType_MAX
};

enum EGravityGunHolderState
{
	Invalid,
	Unrooting,
	Catching,
	Holding,
	Detached,
	EGravityGunHolderState_MAX
};

enum EFortHardcoreModifierTier
{
	Bronze,
	Silver,
	Gold,
	Platinum,
	Diamond,
	EFortHardcoreModifierTier_MAX
};

enum EFortHelpContentLocation
{
	Top,
	Bottom,
	Max
};

enum EFortHelpItemType
{
	Header,
	Entry,
	Max
};

enum ESpecializationType
{
	Tier1,
	Tier2,
	Tier3,
	Tier4,
	NumTiers,
	ESpecializationType_MAX
};

enum EFortHeroLoadoutPerkType
{
	Commander,
	Standard,
	EFortHeroLoadoutPerkType_MAX
};

enum EFortSupportBonusType
{
	Normal,
	Tactical,
	Max_None,
	EFortSupportBonusType_MAX
};

enum EFortHexTileAdjacency
{
	North,
	NorthEast,
	SouthEast,
	South,
	SouthWest,
	NorthWest,
	Max_None,
	EFortHexTileAdjacency_MAX
};

enum EBannerUsageContext
{
	Unknown,
	BannerIcon,
	PhysicalBanner,
	Spray,
	EBannerUsageContext_MAX
};

enum ESquadSlotType
{
	HeroSquadMissionDefender,
	SurvivorSquadLeadSurvivor,
	SurvivorSquadSurvivor,
	DefenderSquadMember,
	ExpeditionSquadMember,
	ESquadSlotType_MAX
};

enum EFortHomebaseSquadType
{
	AttributeSquad,
	CombatSquad,
	DefenderSquad,
	ExpeditionSquad,
	Max_None,
	EFortHomebaseSquadType_MAX
};

enum EHomebaseNodeType
{
	Gadget,
	Utility,
	Hidden,
	EHomebaseNodeType_MAX
};

enum EFortHuskAnimType
{
	Basic,
	Dwarf,
	BlasterBig,
	Weak,
	TinyHead,
	Beehive,
	Husky,
	Sploder,
	Zapper,
	EFortHuskAnimType_MAX
};

enum EMapCaptureMethod
{
	None,
	LiveCapture,
	StaticCapture,
	EMapCaptureMethod_MAX
};

enum EFortInputGameMode
{
	SaveTheWorld,
	Athena,
	EFortInputGameMode_MAX
};

enum EFortIntensityCurveSequenceType
{
	Sequence,
	Loop,
	Random,
	Max_None,
	EFortIntensityCurveSequenceType_MAX
};

enum EFortInteractContextInfoType
{
	Standard,
	Crafting,
	EFortInteractContextInfoType_MAX
};

enum EInteriorAudioBuildingType
{
	None,
	Wall,
	Floor,
	CenterCell,
	EInteriorAudioBuildingType_MAX
};

enum EInteriorAudioBuildingDefaultRotation
{
	PositiveY,
	NegativeX,
	NegativeY,
	PositiveX,
	EInteriorAudioBuildingDefaultRotation_MAX
};

enum EInteriorAudioQuadrant
{
	None,
	Left,
	Right,
	Top,
	Bottom,
	EInteriorAudioQuadrant_MAX
};

enum EInteriorAudioBuildingTags
{
	None,
	HasDoors,
	RotationDependant,
	UseConditionalEvaluation,
	EInteriorAudioBuildingTags_MAX
};

enum EInteriorAudioBuildingRelativePosition
{
	SameCell,
	SameCellQuadrantTestFailed,
	OtherCellParallelToForward,
	OtherCellParallelToRight,
	OtherCellParallelToUp,
	Max_None,
	EInteriorAudioBuildingRelativePosition_MAX
};

enum EInteriorAudioBuildingDirection
{
	Left,
	Right,
	Forward,
	Backward,
	Upward,
	EInteriorAudioBuildingDirection_MAX
};

enum EInteriorAudioBuildingEvaluation
{
	Invalid,
	Partial,
	Solid,
	EInteriorAudioBuildingEvaluation_MAX
};

enum EInteriorAudioRoomSize
{
	Small,
	Medium,
	Large,
	Max_None,
	EInteriorAudioRoomSize_MAX
};

enum EMinigameStatSavePolicy
{
	Never,
	Always,
	OnlyIfLower,
	OnlyIfHigher,
	EMinigameStatSavePolicy_MAX
};

enum EItemEvolutionRestrictionReason
{
	NoEvolutions,
	BelowMaximumLevel,
	VaultOverflow,
	MissingCatalyst,
	MissingCosts,
	NoRarityUpgrade,
	InUseByCrafting,
	EItemEvolutionRestrictionReason_MAX
};

enum EItemUpgradeRestrictionReason
{
	NoAdditionalLevels,
	MaximumLevelAchieved,
	VaultOverflow,
	EItemUpgradeRestrictionReason_MAX
};

enum EFortTemplateAccess
{
	Normal,
	Trusted,
	Private,
	EFortTemplateAccess_MAX
};

enum EItemProfileType
{
	Common,
	Campaign,
	Athena,
	EItemProfileType_MAX
};

enum EFortJackalSimEvent
{
	Jumped,
	EFortJackalSimEvent_MAX
};

enum ELayeredAudioTriggerDir
{
	Forward,
	Backward,
	ELayeredAudioTriggerDir_MAX
};

enum ELayeredAudioInterpolationType
{
	None,
	CustomCurve,
	Linear,
	ELayeredAudioInterpolationType_MAX
};

enum EUnableToLoadReason
{
	None,
	PackageDoesNotExist,
	EUnableToLoadReason_MAX
};

enum EFLightOverrideLevel
{
	Default,
	High,
	Count,
	FLightOverrideLevel_MAX
};

enum ELobbyMissionGeneratorDetailsRequirement
{
	Unknown,
	NotRequired,
	Required,
	ELobbyMissionGeneratorDetailsRequirement_MAX
};

enum ELootQuotaLevel
{
	Unlimited,
	Level1,
	Level2,
	Level3,
	Level4,
	Level5,
	Level6,
	Level7,
	Level8,
	Level9,
	Level10,
	Level11,
	Level12,
	Level13,
	Level14,
	Level15,
	Level16,
	Level17,
	NumLevels,
	ELootQuotaLevel_MAX
};

enum EMarkableResult
{
	Markable,
	Block,
	Continue,
	EMarkableResult_MAX
};

enum EFortMatchmakingType
{
	Gathering,
	CriticalMission,
	QuickPlay,
	Session,
	EFortMatchmakingType_MAX
};

enum EFortSessionHelperJoinResult
{
	NoResult,
	ReservationSuccess,
	ReservationFailure_PartyLimitReached,
	ReservationFailure_IncorrectPlayerCount,
	ReservationFailure_RequestTimedOut,
	ReservationFailure_ReservationNotFound,
	ReservationFailure_ReservationDenied,
	ReservationFailure_ReservationDenied_Banned,
	ReservationFailure_ReservationRequestCanceled,
	ReservationFailure_ReservationInvalid,
	ReservationFailure_BadSessionId,
	ReservationFailure_ReservationDenied_ContainsExistingPlayers,
	ReservationFailure_GeneralError,
	ReservationFailure_NoSubsystem,
	ReservationFailure_NoIdentity,
	ReservationFailure_InvalidSession,
	ReservationFailure_InvalidUser,
	ReservationFailure_EncryptionKey,
	ReservationFailure_RefreshAuth,
	ReservationFailure_AlreadyJoiningDuringReserve,
	ReservationFailure_AlreadyJoiningDuringSkip,
	JoinSessionSuccess,
	JoinSessionFailure_SessionIsFull,
	JoinSessionFailure_SessionDoesNotExist,
	JoinSessionFailure_CouldNotRetrieveAddress,
	JoinSessionFailure_AlreadyInSession,
	JoinSessionFailure_UnknownError,
	JoinSessionFailure_InvalidSession,
	JoinSessionFailure_InvalidSearchResultIndex,
	JoinSessionFailure_AlreadyJoiningDuringJoin,
	SearchPassFailure_NoSessionHelper,
	SearchPassFailure_InvalidUser,
	SearchPassFailure_NoIdentity,
	SearchPassFailure_InvalidSearchResult,
	SearchPassFailure_InvalidSearchResultIndex,
	JoinSessionCanceled,
	EFortSessionHelperJoinResult_MAX
};

enum EFortMatchmakingPool
{
	Any,
	Desktop,
	PS4,
	XboxOne,
	Mobile,
	Test,
	Switch,
	Console,
	All,
	EFortMatchmakingPool_MAX
};

enum EFortMatchmakingPrivacyConfiguration
{
	UserPartyConfigured,
	ForcePrivate,
	ForcePublic,
	EFortMatchmakingPrivacyConfiguration_MAX
};

enum EMatchmakingFlags
{
	None,
	CreateNewOnly,
	NoReservation,
	Private,
	UseWorldDataOwner,
	EMatchmakingFlags_MAX
};

enum EMatchmakingStartLocation
{
	Lobby,
	Game,
	CreateNew,
	FindSingle,
	EMatchmakingStartLocation_MAX
};

enum EFortTournamentAlertType
{
	Warning,
	Info,
	EFortTournamentAlertType_MAX
};

enum EPlaylistUpdateReason
{
	LocalPlayerJoinedParty,
	LocalPlayerLocationChangedToFrontend,
	LocalPlayerSubgameSelected,
	LocalPlayerLeftParty,
	LocalPlayerDeclinedCrossplayPermission,
	PartyGameSessionKeyChanged,
	PartySquadFillChanged,
	PartyMemberJoined,
	PartyMemberLeft,
	PartyMemberReadinessChanged,
	PartyMemberInGameReadyCheckChanged,
	PartyMemberLocationChanged,
	PartyMemberSpectateAvailabilityChanged,
	PartyMemberSessionJoinInfoChanged,
	ValidatePlaylist_QoSCheckFailed,
	ValidatePlaylist_TournamentEventOver,
	ValidatePlaylist_BlackoutChanged,
	ValidatePlaylist_MatchmakingEventsChanged,
	ValidatePlaylist_TournamentNewEventStarted,
	ValidatePlaylist_TournamentNewEventCountdown,
	ValidatePlaylist_TournamentsChanged,
	ValidatePlaylist_PlayerTournamentDataRefreshed,
	Initialization,
	EPlaylistUpdateReason_MAX
};

enum EMatchmakingUtilityFlows
{
	Automatic,
	JoinMatchInProgress,
	SpectateMatch,
	Legacy,
	LinkCode,
	JoinEditingSession,
	Internal_Unselected,
	EMatchmakingUtilityFlows_MAX
};

enum EMatchmakingSourceV2
{
	None,
	AthenaMatchmakingWidget,
	ActivityMatchmakingWidget,
	ReadyUpScreenWidget,
	ForcedIntro,
	KeepPlayingTogetherWidget,
	Unknown,
	EMatchmakingSourceV2_MAX
};

enum EUseInputWithPartyResult
{
	Success,
	LocalPlayerNeedsToAllowCrossplay,
	LocalPlayerRestricted,
	RemotePlayerRestricted,
	UnknownFailure,
	EUseInputWithPartyResult_MAX
};

enum EMatchmakingErrorV2
{
	Success,
	Canceled,
	NeedUpdate,
	VersionMismatch,
	UpdateFailed,
	NotLoggedIn,
	NoIdentityInterface,
	NoSessionInterface,
	AlreadyInSession,
	FindSessionFailure,
	InvalidSessionId,
	FailedToQueryEncryptionKey,
	FailedToRefreshAuthToken,
	FailedToCleanupSession,
	FailedToJoinSession,
	FailedToTravelToSession,
	Unauthorized,
	BannedFromAthena,
	BannedFromAthenaForExploit,
	BannedFromAthenaForTeaming,
	BannedFromAthenaForTeamKilling,
	InvalidCustomMatchKey,
	FailedToContactGameServices,
	FailedToConnectToMMS,
	MMSCommunicationIssue,
	ServiceReturnedError,
	PlaylistNoLongerAvailable,
	CrossplayUnsetWithInputDevicePoolShift,
	CrossplayNeededForTournamentMatch,
	MatchmakingDisabled,
	AccountLevelTooLow,
	JoinInProgressError,
	SpectateInProgressError,
	MatchmakingInProgress,
	TooFrequentRequests,
	FailedToAcquireContent,
	CellularDataRefusal,
	CancelledDownloadContent,
	DataAssetDirectoryUpdateFailed,
	InvalidPlaylistRevision,
	UnknownError,
	EMatchmakingErrorV2_MAX
};

enum EBadMatchType
{
	None,
	Ping,
	PacketLoss,
	NotMonitored,
	EBadMatchType_MAX
};

enum ECharacterEncounterType
{
	Converstation,
	Attack,
	Count,
	ECharacterEncounterType_MAX
};

enum ESavePlayerQuestUpdate
{
	QuestGiven,
	QuestCompleted,
	ESavePlayerQuestUpdate_MAX
};

enum EAthenaFilterDisplayType
{
	UseCategoryName,
	ShowFilterString,
	EAthenaFilterDisplayType_MAX
};

enum EMcpSubscriptionState
{
	Inactive,
	Active,
	Canceled,
	PaymentProcessError,
	BlockedBenefits,
	Unknown,
	EMcpSubscriptionState_MAX
};

enum ESocialImportPanelPlatform
{
	Facebook,
	VK,
	Steam,
	Xbox,
	Playstation,
	Switch,
	None,
	ESocialImportPanelPlatform_MAX
};

enum EPublishStatus
{
	Banned,
	CannotPublish,
	NeedsCreatorName,
	CanPublishProvisionally,
	CanPublish,
	EPublishStatus_MAX
};

enum ETwitchViewerStatusType
{
	TwitchViewerStatus_Unknown,
	TwitchViewerStatus_Nonsubscriber,
	TwitchViewerStatus_Subscriber,
	TwitchViewerStatus_Broadcaster,
	TwitchViewerStatus_Max
};

enum EMegaStormState
{
	GatheringActorList,
	DamagingActors,
	EMegaStormState_MAX
};

enum EMinigameGameEndCallout
{
	WinLose,
	Placement,
	Cooperative,
	EMinigameGameEndCallout_MAX
};

enum EMinigameScoreboardStates
{
	RoundEndDisplayWinner,
	GameEndDisplayWinner,
	RoundEndDisplayScoreboard,
	GameEndDisplayScoreboard,
	Max
};

enum EMinigameFullscreenMapWidgetType
{
	Default_Map,
	Creative_Scoreboard,
	EMinigameFullscreenMapWidgetType_MAX
};

enum EFortMinigameExec
{
	Yes,
	No,
	EFortMinigameExec_MAX
};

enum EMinigamePlayerPersistence
{
	None,
	PartyLeaderOnly,
	AllPlayers,
	EMinigamePlayerPersistence_MAX
};

enum EMinigameWinCondition
{
	MostRoundWins,
	MostScoreWins,
	EMinigameWinCondition_MAX
};

enum EMinigameTeamListType
{
	Blacklist,
	Whitelist,
	EMinigameTeamListType_MAX
};

enum EMinigameCaptureObjectiveState
{
	NotCaptured,
	Captured,
	EMinigameCaptureObjectiveState_MAX
};

enum EObjectiveType
{
	DestructionObjective,
	CaptureObjective,
	EObjectiveType_MAX
};

enum ETrackedObjectiveQuery
{
	All,
	ExactTeam,
	Friendly,
	Neutral,
	Hostile,
	NotFriendly,
	MAX
};

enum EMinigameScoreType
{
	Time,
	PointTotal,
	EMinigameScoreType_MAX
};

enum EFortMinigameStatOperation
{
	Equal,
	Less,
	Greater,
	LessOrEqual,
	GreaterOrEqual,
	EFortMinigameStatOperation_MAX
};

enum EFortMinigameStatScope
{
	Group,
	Team,
	Player,
	EFortMinigameStatScope_MAX
};

enum EBuildingMode
{
	None,
	BuildingsOnly,
	TrapsOnly,
	All,
	EBuildingMode_MAX
};

enum EFortMinigameClassResetType
{
	Never,
	RoundEnd,
	GameEnd,
	PlayerDeath,
	EFortMinigameClassResetType_MAX
};

enum EFortMinigameClassSlot
{
	ZeroIndex,
	None,
	EFortMinigameClassSlot_MAX
};

enum EFortMinigamePostGameSpawnLocationSetting
{
	IslandStart,
	PreGameLocation,
	EFortMinigamePostGameSpawnLocationSetting_MAX
};

enum EFortMinigamePlayerSpawnLocationSetting
{
	SpawnPads,
	Air,
	CurrentLocation,
	EFortMinigamePlayerSpawnLocationSetting_MAX
};

enum EMiniMapIconParameterDataType
{
	None,
	Scalar,
	Vector,
	Texture,
	EMiniMapIconParameterDataType_MAX
};

enum EFortMiniMapIconRotation
{
	EFMMIR_None,
	EFMMIR_Absolute,
	EFMMIR_Relative,
	EFMMIR_MAX
};

enum EFortMiniMapContext
{
	EFMC_MiniMap,
	EFMC_FullScreenMap,
	EFMC_MAX
};

enum EFortMiniMapHeight
{
	EFMH_Equal,
	EFMH_Below,
	EFMH_Above,
	EFMH_MAX
};

enum EFortMiniMapDrawCategory
{
	AthenaBackground,
	MapLocation,
	SafeZone,
	BusPath,
	SpecialActorIcon,
	SquadPin,
	MapIndicator,
	MapCursor,
	Elimination,
	MAX
};

enum EFortCheatMissionGenType
{
	NewGeneration,
	OldGeneration,
	Max_None,
	EFortCheatMissionGenType_MAX
};

enum EFortOptionGenerationResult
{
	NoOptionsGenerated,
	NewOptionsGenerated,
	ExistingOptionsGenerated,
	EFortOptionGenerationResult_MAX
};

enum EPollActorsInVolumeTypes
{
	DesignerPlacedOnly,
	PlayerBuiltOnly,
	All,
	EPollActorsInVolumeTypes_MAX
};

enum EMissionReplyTypes
{
	Handled,
	NotHandled,
	EMissionReplyTypes_MAX
};

enum ETimerOverrideSetting
{
	DefaultBehavior,
	ForceShow,
	ForceHide,
	ShowAtEnd,
	ETimerOverrideSetting_MAX
};

enum ESchemaModificationType
{
	AddOrModify,
	Remove,
	Count,
	ESchemaModificationType_MAX
};

enum EMontageVisibilityRule
{
	RequiredItem,
	ForbiddenItem,
	EMontageVisibilityRule_MAX
};

enum EFortMtxPlatform
{
	Epic,
	PSN,
	Live,
	Shared,
	EpicPC,
	EpicPCKorea,
	IOSAppStore,
	EpicAndroid,
	Nintendo,
	WeGame,
	Samsung,
	GooglePlay,
	EFortMtxPlatform_MAX
};

enum EFortMusicSectionType
{
	Intro,
	Loop,
	Outro,
	Max_None,
	EFortMusicSectionType_MAX
};

enum EFortMusicSectionStopBehavior
{
	Crossfade,
	AllowFadeOut,
	EFortMusicSectionStopBehavior_MAX
};

enum EFortMusicCombatIntensity
{
	Low,
	Medium,
	High,
	VeryHigh,
	Max_None,
	EFortMusicCombatIntensity_MAX
};

enum EMusicChannel
{
	VoiceA,
	VoiceB,
	Max_None,
	EMusicChannel_MAX
};

enum EMusicFadeStyles
{
	CrossFade,
	FadeOutThenIn,
	Max_None,
	EMusicFadeStyles_MAX
};

enum EMutatorListInitState
{
	Default,
	Enabled,
	Disabled,
	EMutatorListInitState_MAX
};

enum EFortAreaFlag
{
	Default,
	Obstacle,
	Smashable,
	Unwalkable,
	Interactable,
	EFortAreaFlag_MAX
};

enum EVisibilityResponse
{
	Hide,
	Show,
	Custom,
	EVisibilityResponse_MAX
};

enum EFortNavLinkPattern
{
	Floor,
	Stairs,
	Roof,
	Manual,
	EFortNavLinkPattern_MAX
};

enum EFortNamedNavmesh
{
	Husk,
	Smasher,
	MAX
};

enum EFXType
{
	GenericAnimNotify,
	TrailAnimNotify,
	WeaponImpactEffect,
	WeaponMeleeImpactEffect,
	Contrail,
	Emote,
	Trap,
	Skin,
	Glider,
	Vehicle,
	BackpackBling,
	Water,
	LootChest,
	EnvironmentalAmbient,
	WeaponRangedBeam,
	WeaponBulletShells,
	WeaponMuzzleFlashes,
	PickAxe,
	Curie,
	Projectile,
	EFXType_MAX
};

enum EFortOctopusSimEvent
{
	BeginBoostCooldown,
	EFortOctopusSimEvent_MAX
};

enum EOutpostBuildings
{
	StormShield,
	HarvestingOptimizer,
	StorageVault,
	POST,
	EOutpostBuildings_MAX
};

enum EPartyMemberVoiceChatStatus
{
	Disabled,
	Enabled,
	PartyVoice,
	GameVoice,
	PlatformVoice,
	EPartyMemberVoiceChatStatus_MAX
};

enum EGameReadiness
{
	NotReady,
	Ready,
	SittingOut,
	EGameReadiness_MAX
};

enum ESquadChangeType
{
	JoinSquad,
	BenchSelf,
	UnbenchSelf,
	Swap,
	None,
	ESquadChangeType_MAX
};

enum EPartyMemberSidekickStatus
{
	None,
	Linked,
	Connected,
	EPartyMemberSidekickStatus_MAX
};

enum EFortPartyMemberReadyCheckStatus
{
	None,
	InProgress,
	Complete,
	Canceled,
	EFortPartyMemberReadyCheckStatus_MAX
};

enum EPathUndermineEvent
{
	Predicted,
	Started,
	Finished,
	EPathUndermineEvent_MAX
};

enum EPathObstacleAction
{
	Melee,
	Ignore,
	AbortMoveAsFailed,
	FinishMoveAsSucceeded,
	EPathObstacleAction_MAX
};

enum EWardAffectType
{
	AffectsBothStartAndEndPoints,
	AffectsOnlyStartPoints,
	AffectsOnlyEndPoints,
	EWardAffectType_MAX
};

enum EFortWeaponListRemovalBehavior
{
	DestroyImmediately,
	DeferredLifespan,
	DoNotDestroy,
	EFortWeaponListRemovalBehavior_MAX
};

enum EFortControlRecoveryBehavior
{
	DefaultControl,
	LimitedControl,
	ChainControl,
	EFortControlRecoveryBehavior_MAX
};

enum EFortPawnPushSize
{
	FFPS_Normal,
	FPPS_Player,
	FPPS_Large,
	FPPS_SuperLarge,
	EFortPawnPushSize_MAX
};

enum EFortAnnouncerTeamVocalChords
{
	Team1,
	Team2,
	Max_None,
	EFortAnnouncerTeamVocalChords_MAX
};

enum EFortPawnComponent_DisguiseRevealReason
{
	ByDamage,
	ByConversation,
	ByProximity,
	Unknown,
	EFortPawnComponent_MAX
};

enum EFortBadMatchTriggerType
{
	Unspecified,
	SmallTeam,
	LargeTeam,
	LetoTeam,
	EFortBadMatchTriggerType_MAX
};

enum EFortBadMatchTriggerOperation
{
	LessThan,
	LessThanOrEqual,
	Equal,
	GreaterThan,
	GreaterThanOrEqual,
	EFortBadMatchTriggerOperation_MAX
};

enum EFortRewardType
{
	Default,
	Missed,
	Max_None,
	EFortRewardType_MAX
};

enum EFortReplicatedStat
{
	MonsterKills,
	MonsterDamagePoints,
	PlayerKills,
	WoodGathered,
	StoneGathered,
	MetalGathered,
	Deaths,
	BluGloActivity,
	BuildingsBuilt,
	BuildingsBuilt_Wood,
	BuildingsBuilt_Stone,
	BuildingsBuilt_Metal,
	BuildingsUpgraded_Wood2,
	BuildingsUpgraded_Wood3,
	BuildingsUpgraded_Stone2,
	BuildingsUpgraded_Stone3,
	BuildingsUpgraded_Metal2,
	BuildingsUpgraded_Metal3,
	BuildingsDestroyed,
	Repair_Wood,
	Repair_Stone,
	Repair_Metal,
	FlagsCaptured,
	FlagsReturned,
	ContainersLooted,
	CraftingPoints,
	TrapPlacementPoints,
	TrapActivationPoints,
	TotalScore,
	OldTotalScore,
	CombatScore,
	BuildingScore,
	UtilityScore,
	BadgesScore,
	None,
	MAX
};

enum EFortReplenishmentType
{
	Restricted,
	ClampMin,
	Add,
	Ability,
	EFortReplenishmentType_MAX
};

enum EFortPhysicsObjectMovementState
{
	None,
	Flying,
	Rolling,
	Sliding,
	Floating,
	EFortPhysicsObjectMovementState_MAX
};

enum EFortPhysicsObjectAwakeState
{
	Invalid,
	Awake,
	Asleep,
	EFortPhysicsObjectAwakeState_MAX
};

enum EFortPhysicsSimulationRepEvent
{
	LinearVelocity,
	AngularVelocity,
	Impulse,
	ImpulseAtLocation,
	AngularImpulse,
	Force,
	Torque,
	None,
	EFortPhysicsSimulationRepEvent_MAX
};

enum EFortPhysicsObjectNetworkPolicy
{
	ClientOnly,
	ServerAuthoritative,
	EFortPhysicsObjectNetworkPolicy_MAX
};

enum EFortPhysicsSimSize
{
	Small,
	Average,
	Medium,
	Large,
	Invalid,
	EFortPhysicsSimSize_MAX
};

enum EFortPickupTossState
{
	NotTossed,
	InProgress,
	AtRest,
	EFortPickupTossState_MAX
};

enum EFortPickupSpawnSource
{
	Unset,
	PlayerElimination,
	Chest,
	SupplyDrop,
	AmmoBox,
	Drone,
	ItemSpawner,
	BotElimination,
	NPCElimination,
	LootDrop,
	EFortPickupSpawnSource_MAX
};

enum EFortPickupSourceTypeFlag
{
	Other,
	Player,
	Destruction,
	Container,
	AI,
	Tossed,
	FloorLoot,
	Fishing,
	EFortPickupSourceTypeFlag_MAX
};

enum EManagedPickupContext
{
	Unknown,
	PlayerDropped,
	Overflow,
	Spawned,
	EManagedPickupContext_MAX
};

enum EManagedPickupBucket
{
	Default,
	Junk,
	Normal,
	Important,
	EManagedPickupBucket_MAX
};

enum EConstructionBuildingType
{
	Wall,
	Floor,
	Stairs,
	Roof,
	Brace,
	WallWindow,
	Count,
	EConstructionBuildingType_MAX
};

enum EOrientedConstructionBuildingType
{
	WallX,
	WallY,
	Floor,
	StairsUpX,
	StairsUpY,
	StairsDownX,
	StairsDownY,
	Roof,
	BraceLeftX,
	BraceRightX,
	BraceLeftY,
	BraceRightY,
	WallWindowX,
	WallWindowY,
	Count,
	EOrientedConstructionBuildingType_MAX
};

enum ELeashReturnLocationMode
{
	Closest,
	Random,
	ELeashReturnLocationMode_MAX
};

enum ELookAtType
{
	ScanAround,
	Investigate,
	HeardSound,
	MAX
};

enum EPerceptionSoundType
{
	Default,
	Explosion,
	ProjectileFlyBy,
	ProjectileImpact,
	WeaponFiring,
	Building,
	MeleeImpact,
	MAX
};

enum ETrackingOffsetModifierCurve
{
	Default,
	CombatStart,
	TargetLowHealth,
	MAX
};

enum EPerceptionState
{
	Threat_Seeing,
	Threat_LKP,
	Threat_Alerted,
	ObstacleIncoming,
	ObstacleBlockedBy,
	ObstacleDetectedTrap,
	Unknown,
	Count,
	Threat_Count,
	EPerceptionState_MAX
};

enum EStimType
{
	Seeing,
	Seen,
	MightHaveSeen,
	Hurt,
	Heard,
	Obstacle,
	Enemy,
	Unknown,
	Count,
	EStimType_MAX
};

enum EDBNOPlayStyle
{
	Thirsty,
	Default,
	Passive,
	ThristyButPassiveOnPlayers,
	DefaultButPassiveOnPlayers,
	EDBNOPlayStyle_MAX
};

enum EFortCrucibleWhitelistLevel
{
	None,
	Basic,
	Advanced,
	EFortCrucibleWhitelistLevel_MAX
};

enum ECreativeQuickbarSlots
{
	Phone,
	ECreativeQuickbarSlots_MAX
};

enum EQuickbarSlots
{
	HarvestingTool,
	Weapon1,
	Weapon2,
	Weapon3,
	Gadget1,
	Gadget2,
	Ability1,
	Ability2,
	Ability3,
	EQuickbarSlots_MAX
};

enum EFortPickerToDisplay
{
	TrapPicker,
	WeaponPicker,
	SocialPicker,
	ChatPicker,
	NotePicker,
	EmotePicker,
	SquadQuickChatPicker,
	BattleLabDevicePicker,
	MusicSourcePicker,
	EFortPickerToDisplay_MAX
};

enum EMapZoomingMode
{
	None,
	ZoomingIn,
	ZoomingOut,
	EMapZoomingMode_MAX
};

enum EFortIdleCheckResult
{
	Invalid,
	ActivityDetected,
	Idle,
	IdlePastMaxTime,
	EFortIdleCheckResult_MAX
};

enum EHighlightReelTypes
{
	Generic,
	GameSummary,
	ExtendedGameSummary,
	Builder,
	FastMover,
	LongDistance,
	Multikill,
	StormCloud,
	EHighlightReelTypes_MAX
};

enum ECameraStateRestoreReason
{
	Unknown,
	ChangedFollowTarget,
	ChangedCameraType,
	InvokedHotKey,
	Scrubbed,
	Restored,
	SpecialAction,
	MAX
};

enum EGyroButtonResponse
{
	NoChange,
	Disable,
	Trackball,
	Invert,
	EGyroButtonResponse_MAX
};

enum EGyroAcceleration
{
	Off,
	Low,
	Medium,
	High,
	Custom,
	Legacy,
	EGyroAcceleration_MAX
};

enum EGyroActiveMode
{
	ScopeOnly,
	AimDownSightsOrFiring,
	ADSOrFiring,
	ADSOrFiringOrHarvestEquipped,
	All,
	EGyroActiveMode_MAX
};

enum EFortMotionYawAxis
{
	Yaw,
	Roll,
	EFortMotionYawAxis_MAX
};

enum EFortInputActionGroup
{
	AllModes,
	Combat,
	Building,
	Movement,
	Edit,
	Death,
	Cinematic,
	Picker,
	Other,
	Interaction,
	AthenaLTMAbilities,
	ShoppingCart,
	ShoppingCartDriver,
	ShoppingCartPassenger,
	Cannon,
	CannonDriver,
	CannonPassenger,
	GolfCart,
	GolfCartDriver,
	GolfCartPassenger,
	QuadCrasher,
	QuadCrasherDriver,
	QuadCrasherPassenger,
	Biplane,
	BiplaneDriver,
	BiplanePassenger,
	Hamsterball,
	Jackal,
	Ostrich,
	OstrichDriver,
	OstrichPassenger,
	Meatball,
	MeatballDriver,
	MeatballPassenger,
	HoagieDriver,
	Dagwood,
	DagwoodDriver,
	DagwoodPassenger,
	Nevada,
	NevadaDriver,
	MountedTurret,
	Spectating,
	FullscreenMap,
	CreativeAll,
	CreativeModeratorMode,
	CreativeMoveToolEquipped,
	CreativeMoveObjectsFreely,
	CreativeMoveBuildingsOnGrid,
	CreativeFlying,
	CreativeIslandPanel,
	PropSelectorEquipped,
	DBNOCarryStart,
	DBNOCarryStop,
	DBNOCarry,
	InteractionStart,
	InteractionStop,
	BattleLab,
	SuperDuper,
	Tether,
	CombatAndBuilding,
	CombatAndAthenaLTMAbilities,
	CombatBuildingAndAthenaLTMAbilities,
	EFortInputActionGroup_MAX
};

enum EFortInputActionType
{
	Press,
	Click,
	Hold,
	Release,
	EFortInputActionType_MAX
};

enum EFortInputDevice
{
	Mouse,
	Keyboard,
	Gamepad,
	Touch,
	EFortInputDevice_MAX
};

enum EItemInteractionStatus
{
	Interrupted,
	Completed,
	TimedOut,
	EItemInteractionStatus_MAX
};

enum EItemInteractionType
{
	Search,
	LockOnSearch,
	None,
	EItemInteractionType_MAX
};

enum EBodyPartVisibilityGrouping
{
	AllParts,
	AllButHead,
	OnlyBackBling,
	BackBlingAndCharm,
	OnlyBody,
	OnlyHead,
	OnlyTail,
	EBodyPartVisibilityGrouping_MAX
};

enum EUpdateCustomDepthOptimDirtyFlags
{
	None,
	CharacterParts,
	Weapon,
	PossessedProp,
	EUpdateCustomDepthOptimDirtyFlags_MAX
};

enum EBackpackType
{
	Jetpack,
	Medic,
	StormTracker,
	Glider,
	EBackpackType_MAX
};

enum EKeepPlayingTogetherAnalyticEventPhase
{
	PrePostGamePhase,
	Countdown,
	TimedOut,
	AllSquadMembersVoted,
	EKeepPlayingTogetherAnalyticEventPhase_MAX
};

enum EFortPawnState
{
	Default,
	InCombat,
	DBNO,
	IsReviving,
	BeingRevived,
	Dead,
	EFortPawnState_MAX
};

enum ECustomFeedFilterParticipantNames
{
	NoFiltering,
	AllPlayers,
	ECustomFeedFilterParticipantNames_MAX
};

enum EFortPlayerSurveyAnalyticsFinishReason
{
	Submitted,
	Canceled,
	EFortPlayerSurveyAnalyticsFinishReason_MAX
};

enum EFortPlayerSurveyQuestionPresentationStyle
{
	Invalid,
	Standard,
	MultipleChoice_Rating,
	Num,
	EFortPlayerSurveyQuestionPresentationStyle_MAX
};

enum EFortPlayerSurveyQuestionTypeLegacy
{
	Invalid,
	MultipleChoice,
	MultipleSelection,
	FreeFormText,
	Num,
	EFortPlayerSurveyQuestionTypeLegacy_MAX
};

enum EVehicleTrickType
{
	None,
	RollIncrement,
	ReverseRollIncrement,
	YawIncrement,
	ReverseYawIncrement,
	PitchIncrement,
	ReversePitchIncrement,
	HeightIncrement,
	DistanceIncrement,
	AirTimeIncrement,
	ShoppingCart_Flying,
	ShoppingCart_Stooping,
	StartedLanding,
	FailedLanding,
	Cancelled,
	StuckLanding,
	Count,
	EVehicleTrickType_MAX
};

enum EVehicleTrickAxis
{
	X,
	XNeg,
	Y,
	YNeg,
	Z,
	ZNeg,
	Count,
	EVehicleTrickAxis_MAX
};

enum EFortGameType
{
	BR,
	Creative,
	CreativeLTM,
	Playground,
	STW,
	BRArena,
	BRLTM,
	Social,
	MAX
};

enum EDBNOType
{
	On,
	Off,
	NotWhenRespawning,
	EDBNOType_MAX
};

enum EWeaponSelectionPreservationType
{
	KeepSelectionWhenRespawning,
	NeverKeepSelection,
	EWeaponSelectionPreservationType_MAX
};

enum ERewardPlacementBonusType
{
	Solo,
	Duo,
	Squad,
	LargeTeam,
	None,
	TwoTeam,
	MediumTeam,
	QuickSolo,
	QuickDuo,
	QuickSquad,
	QuickLargeTeam,
	QuickTwoTeam,
	QuickMediumTeam,
	SinglePlacement,
	ERewardPlacementBonusType_MAX
};

enum ERewardTimePlayedType
{
	Default,
	NoReward,
	FlatValue,
	ERewardTimePlayedType_MAX
};

enum EAthenaWinCondition
{
	LastManStanding,
	LastManStandingIncludingAllies,
	TimedTeamFinalFight,
	FirstToGoalScore,
	TimedLastMenStanding,
	MutatorControlled,
	MutatorControlledGoalScore,
	MutatorControlledChinaSupported,
	EAthenaWinCondition_MAX
};

enum EAthenaRespawnLocation
{
	LastDeath,
	CreativePlayerStart,
	EAthenaRespawnLocation_MAX
};

enum EAthenaRespawnType
{
	None,
	InfiniteRespawn,
	InfiniteRespawnExceptStorm,
	EAthenaRespawnType_MAX
};

enum ECustomGameVoiceChannel
{
	Squad,
	FullTeam,
	ECustomGameVoiceChannel_MAX
};

enum EPlaylistVisibilityState
{
	Enabled,
	Disabled,
	EnabledButInvisible,
	Hidden,
	EPlaylistVisibilityState_MAX
};

enum EPlaylistAdvertisementType
{
	None,
	New,
	Updated,
	EPlaylistAdvertisementType_MAX
};

enum EPlaysetOffsetType
{
	CustomOffsetFromCorner,
	Center,
	EPlaysetOffsetType_MAX
};

enum EFortPlayspaceMatchmakingRules
{
	AllPlaylists,
	SpecificPlaylists,
	EFortPlayspaceMatchmakingRules_MAX
};

enum EFortPlayspaceUserAcceptanceType
{
	CustomLogic,
	Matchmaking,
	VolumeBased,
	EFortPlayspaceUserAcceptanceType_MAX
};

enum EPortalLinkCodeLockMode
{
	NeverLocked,
	WindowLocked,
	AlwaysLocked,
	EPortalLinkCodeLockMode_MAX
};

enum EFortPreferredItemSlotItemType
{
	Unassigned,
	AssaultRifle,
	Shotgun,
	SMG,
	Pistol,
	SniperAndBow,
	Launcher,
	Utility,
	Consumable,
	Num,
	EFortPreferredItemSlotItemType_MAX
};

enum EProfileGoState
{
	None,
	SettlingLocation,
	RunningCommands,
	CompletedScenario,
	Summary,
	Completed,
	EProfileGoState_MAX
};

enum EProjectileWaterHitBehavior
{
	Overlap,
	StopIfStopSimulatingOnHit,
	StopOnOverlap,
	EProjectileWaterHitBehavior_MAX
};

enum EFortIdleDetectionState
{
	Disabled,
	Default,
	Suspicious,
	Problematic,
	Inactive,
	EFortIdleDetectionState_MAX
};

enum EFortPointsFromNavGraphGoalPathDistanceFilterOperator
{
	AllGoalsInRange,
	AnyGoalInRange,
	EFortPointsFromNavGraphGoalPathDistanceFilterOperator_MAX
};

enum EFortTestGoalActorDot
{
	Dot3D,
	Dot2D,
	EFortTestGoalActorDot_MAX
};

enum EDistanceMode
{
	DistItemToContext,
	DistItemGoalActorToContext,
	DistItemToItemGoalActor,
	EDistanceMode_MAX
};

enum ECountAIAssignedToType
{
	Goal,
	Actor,
	Assignment,
	ECountAIAssignedToType_MAX
};

enum ETwoPointSolverRotationA
{
	PointAToQuerier,
	QuerierToPointA,
	PointAToQuerierWithRandomOffset,
	QuerierToPointAWithRandomOffset,
	Custom,
	ETwoPointSolverRotationA_MAX
};

enum EObjectiveStatusUpdateType
{
	Always,
	OnPercent,
	OnComplete,
	Never,
	EObjectiveStatusUpdateType_MAX
};

enum EFortQuestRewardType
{
	BasicRewards,
	BasicPlusSingleChoice,
	EFortQuestRewardType_MAX
};

enum EFortQuestSubtype
{
	None,
	WeeklyChallenge,
	PunchCard,
	QuickChallenge,
	Milestone,
	UrgentQuest,
	EFortQuestSubtype_MAX
};

enum EFortQuestType
{
	Task,
	Optional,
	DailyQuest,
	TransientQuest,
	SurvivorQuest,
	Achievement,
	Onboarding,
	StreamBroadcaster,
	StreamViewer,
	StreamSubscriber,
	Athena,
	AthenaDailyQuest,
	AthenaEvent,
	AthenaChallengeBundleQuest,
	AthenaTransientQuest,
	All,
	EFortQuestType_MAX
};

enum ECosmeticType
{
	Image,
	Widget,
	ECosmeticType_MAX
};

enum EFortQuestMapNodeLabelPosition
{
	Top,
	Bottom,
	EFortQuestMapNodeLabelPosition_MAX
};

enum EFortQuestMapNodeType
{
	MandatoryQuest,
	SideQuest,
	EFortQuestMapNodeType_MAX
};

enum EInlineObjectiveStatTagCheckEntryType
{
	Target,
	Source,
	Context,
	EInlineObjectiveStatTagCheckEntryType_MAX
};

enum EFortXPPropagationRule
{
	Self,
	Party,
	Squad,
	EFortXPPropagationRule_MAX
};

enum EFortQuestObjectiveItemEvent
{
	Craft,
	Collect,
	Acquire,
	Consume,
	OpenCardPack,
	PurchaseCardPack,
	Convert,
	Upgrade,
	UpgradeRarity,
	QuestComplete,
	AssignWorker,
	LevelUpCollectionBook,
	LevelUpAthenaSeason,
	LevelUpBattlePass,
	GainAthenaSeasonXp,
	HasItem,
	HasAccumulatedItem,
	SlotInCollection,
	AlterationRespec,
	AlterationUpgrade,
	HasCompletedQuest,
	HasAssignedWorker,
	HasUpgraded,
	HasConverted,
	HasUpgradedRarity,
	HasLeveledUpCollectionBook,
	SlotHeroInLoadout,
	HasLeveledUpAthenaSeason,
	HasLeveledUpBattlePass,
	HasGainedAthenaSeasonXp,
	MinigameTime,
	Max_None,
	EFortQuestObjectiveItemEvent_MAX
};

enum EFortChallengeBundleInfoLockedReasonCode
{
	Unlocked,
	NoKnownUnlockMethod,
	PurchaseTheBattlePass,
	ReachSpecificTier,
	TimeLeftBeforeUnlock,
	EFortChallengeBundleInfoLockedReasonCode_MAX
};

enum EFortQuestState
{
	Inactive,
	Active,
	Completed,
	Claimed,
	EFortQuestState_MAX
};

enum ERegisteredPlayerUnregistrationStatus
{
	Registered,
	UnregistrationStarting,
	UnregistrationWaitingForInitialLock,
	UnregistrationWaitingForScoreSave,
	UnregistrationWaitingForFinalSave,
	UnregistrationWaitingForProfileUnlock,
	UnregistrationComplete,
	ERegisteredPlayerUnregistrationStatus_MAX
};

enum EJoinServerState
{
	Inactive,
	Rejoin,
	Tutorial,
	Abandon,
	EJoinServerState_MAX
};

enum EEventMatchScreen
{
	None,
	ActivePlayerGrid,
	EliminatedPlayerGrid,
	MatchStatus,
	Scoreboard,
	EEventMatchScreen_MAX
};

enum EDroneFollowMode
{
	None,
	ForceFacingLocation,
	ForceFacingFollowedPlayer,
	TetherToFollowedPlayer,
	MAX
};

enum EHighlightSignificances
{
	NotSignificant,
	SomewhatSignificant,
	Significant,
	VerySignificant,
	Critical,
	EHighlightSignificances_MAX
};

enum EHighlightFeatures
{
	INVALID,
	FollowingPlayerKill,
	FollowingPlayerDeath,
	InterestingDeathCause,
	BusyBuilder,
	FastMover,
	LongDistanceKill,
	Multikill,
	StormCloudAction,
	WinningMoment,
	PlacementScore,
	FollowingPlayerKillDBNO,
	FollowingPlayerDBNOFinished,
	MaxLongKillDistance,
	VehicleMultikill,
	VehiclePlayerLaunchDistance,
	VehicleKills,
	MaxMidFallKillTime,
	MaxMidFallNoScopeKillTime,
	MaxMidFallKillSpeed,
	MaxMidFallNoScopeSpeed,
	FallingElimination,
	VehicleUsage,
	COUNT,
	EHighlightFeatures_MAX
};

enum EClassRepNodeMapping
{
	NotRouted,
	RelevantAllConnections,
	RelevantAllInsideFortVolume,
	Custom,
	Spatialize_Static,
	Spatialize_Dynamic,
	Spatialize_Dormancy,
	Instance_Dynamic,
	EClassRepNodeMapping_MAX
};

enum ECrucibleWhitelistOverride
{
	DoNothing,
	ForceOn,
	ForceOff,
	ECrucibleWhitelistOverride_MAX
};

enum EFortItemShopSection
{
	RMTItemOffer,
	Featured,
	Daily,
	SpecialFeatured,
	SpecialDaily,
	Standalone,
	CommunityChoice,
	MegaBundle,
	BattlePass,
	MAX_None,
	EFortItemShopSection_MAX
};

enum EFortRuntimeOptionTabStateTarget
{
	All,
	Primary,
	Secondary,
	EFortRuntimeOptionTabStateTarget_MAX
};

enum EFortRuntimeOptionTabState
{
	Default,
	Disabled,
	Hidden,
	EFortRuntimeOptionTabState_MAX
};

enum ENewsExternalURLMode
{
	PatchNotes,
	UpdateNotes,
	MoreInformation,
	ENewsExternalURLMode_MAX
};

enum EFortSafeZoneState
{
	None,
	Starting,
	Holding,
	Shrinking,
	EFortSafeZoneState_MAX
};

enum EFortUIScoreType
{
	Combat,
	Building,
	Utility,
	Badges,
	Bonus,
	Total,
	Max_None,
	EFortUIScoreType_MAX
};

enum EFortScriptedActionEnvironment
{
	FrontEnd,
	GameServer,
	GameClient,
	Max_None,
	EFortScriptedActionEnvironment_MAX
};

enum EFortScriptedActionSource
{
	Quest,
	Token,
	Manual,
	Max_None,
	EFortScriptedActionSource_MAX
};

enum EServerManifestOutputFormat
{
	FlatFile,
	Json,
	HTTP,
	EServerManifestOutputFormat_MAX
};

enum EServerManifestCommandType
{
	Move,
	Copy,
	Unknown,
	EServerManifestCommandType_MAX
};

enum EFortSessionHelperJoinState
{
	NotJoining,
	RequestingReservation,
	FailedReservation,
	WaitingOnGame,
	AttemptingJoin,
	JoiningSession,
	FailedJoin,
	CanceledJoin,
	EFortSessionHelperJoinState_MAX
};

enum ESkeletalAudioBoneVelocityType
{
	Linear,
	Rotational,
	Custom,
	ESkeletalAudioBoneVelocityType_MAX
};

enum ESkeletalAudioBoneSpace
{
	Relative,
	World,
	ESkeletalAudioBoneSpace_MAX
};

enum ESkeletalAudioBoneEvent
{
	None,
	SlowThresholdStart,
	SlowThresholdStop,
	MediumThreshold,
	FastThreshold,
	ESkeletalAudioBoneEvent_MAX
};

enum EFortPowerRatingComparison
{
	InRange,
	OverLevel,
	UnderLevel,
	Unknown,
	EFortPowerRatingComparison_MAX
};

enum EFortInvalidActivityReason
{
	None,
	PartyTooBig,
	PartyTooSmall,
	NotPartyLeader,
	MatchmakingAlready,
	InvalidData,
	EFortInvalidActivityReason_MAX
};

enum EAcceptFriendInviteFailureReason
{
	InviterTooManyFriends,
	SelfTooManyFriends,
	UnknownError,
	EAcceptFriendInviteFailureReason_MAX
};

enum ESendFriendInviteFailureReason
{
	NotFound,
	AlreadyFriends,
	InvitePending,
	AddingSelfFail,
	AddingBlockedFail,
	AutoDeclined,
	BlockedByTarget,
	InviteeInboxFull,
	SelfOutboxFull,
	UnknownError,
	ESendFriendInviteFailureReason_MAX
};

enum EFortSocialFriendRequestMethod
{
	LobbySuggestion,
	LobbyPlayerPanel,
	MassImportPlatformFriends,
	AutoImportPlatformFriends,
	PartyContextRewriteHelper,
	EFortSocialFriendRequestMethod_MAX
};

enum EFortSoundIndicatorTypes
{
	Generic,
	FootStep,
	Gunshot,
	Chest,
	Glider,
	Vehicle,
	Infected,
	COUNT,
	EFortSoundIndicatorTypes_MAX
};

enum EFortSpawnActorTime
{
	PostPlaylistLoad,
	StartOfStormHoldTime,
	EFortSpawnActorTime_MAX
};

enum ESplineWaterAudioFacingDirection
{
	None,
	Inwards,
	Outwards,
	ESplineWaterAudioFacingDirection_MAX
};

enum ESplineWaterAudioWindingOrder
{
	Clockwise,
	Counterclockwise,
	ESplineWaterAudioWindingOrder_MAX
};

enum EFortEventConditionType
{
	EFEC_StatCompare,
	EFEC_MAX
};

enum EFortCompare
{
	EFC_LessThan,
	EFC_LessThanOrEqual,
	EFC_GreaterThan,
	EFC_GreaterThanOrEqual,
	EFC_Equals,
	EFC_MAX
};

enum EFortEventRepeat
{
	EFER_Inactive,
	EFER_Always,
	EFER_OncePerPlayer,
	EFER_OncePerCampaign,
	EFER_OncePerMap,
	EFER_MAX
};

enum EFortStatType
{
	Fortitude,
	Offense,
	Resistance,
	Technology,
	Fortitude_Team,
	Offense_Team,
	Resistance_Team,
	Technology_Team,
	Invalid,
	EFortStatType_MAX
};

enum EUCPTypes
{
	UCPAudio,
	UCPVideo,
	UCPBoth,
	UCPNone,
	UCPTypes_MAX
};

enum ESupplyDropItemTrackType
{
	SpecialActors,
	ESupplyDropItemTrackType_MAX
};

enum ESupplyDropSpawnType
{
	SafeZoneDriven,
	ItemDeliveryManagement,
	MutatorManaged,
	ESupplyDropSpawnType_MAX
};

enum EFortSwimmingAudioType
{
	Normal,
	Sprint,
	SprintStart,
	PickaxeSwing,
	NormalDBNO,
	SprintBoostStart,
	SwimStart,
	SwimEnd,
	Max_None,
	EFortSwimmingAudioType_MAX
};

enum EFortAutoTestState
{
	InitialLoad,
	Login,
	FrontendLoad,
	FrontendPvELoad,
	FrontendPvETest,
	PvEMatchmaking,
	ZoneLoad,
	ZoneTest,
	Finished,
	MAX
};

enum EReplaySmokeTestStep
{
	Setup,
	TogglePause,
	StepForward,
	StepBackward,
	StepToEnd,
	StepToBeginning,
	SpeedUpPlayback,
	SlowDownPlayback,
	ToggleHideTimeline,
	IterateCameraModes,
	TogglePlayerOutlines,
	ToggleNamePlates,
	ToggleReplayRegion,
	ZoomIn,
	ZoomOut,
	ToggleAutoFollowThirdPerson,
	IncreaseExposure,
	DecreaseExposure,
	SetAutoExposure,
	IncreaseAperture,
	DecreaseAperture,
	IncreaseFocalLength,
	DecreaseFocalLength,
	IncreaseFocusDistance,
	DecreaseFocusDistance,
	SetAutoFocus,
	ToggleDamageEffects,
	ToggleHideUI,
	ToggleMap,
	Reset,
	MAX
};

enum EFortTheaterType
{
	Standard,
	Elder,
	PvP,
	PvP2,
	Tutorial,
	TutorialGate,
	Max_None,
	EFortTheaterType_MAX
};

enum EFortTheaterMapTileType
{
	Normal,
	CriticalMission,
	AlwaysActive,
	Outpost,
	NonMission,
	PvPFOB,
	EFortTheaterMapTileType_MAX
};

enum EFortMapNavigationDirection
{
	North,
	NorthEast,
	East,
	SouthEast,
	South,
	SouthWest,
	West,
	NorthWest,
	Invalid,
	EFortMapNavigationDirection_MAX
};

enum EFortMissionQuestValidityResult
{
	Invalid,
	InvalidNotPlayable,
	ValidLinked,
	ValidObjectiveCondition,
	ValidFallback,
	EFortMissionQuestValidityResult_MAX
};

enum EThresholdRequirement
{
	LessThan,
	LessThanOrEqual,
	Equal,
	GreaterThan,
	GreaterThanOrEqual,
	EThresholdRequirement_MAX
};

enum ECollectionSelectionMethod
{
	TierAsIndex,
	TierAsIndexOverflowToLastValid,
	Modulo,
	Random,
	None,
	ECollectionSelectionMethod_MAX
};

enum EGlobalWeatherState
{
	Inactive,
	Active,
	BlendingIn,
	BlendingOut,
	EGlobalWeatherState_MAX
};

enum EWidgetInterfaceTimerStatus
{
	None,
	Active,
	Paused,
	Inactive,
	EWidgetInterfaceTimerStatus_MAX
};

enum EFortTipDisplayPlatformGroup
{
	None,
	Desktop,
	Console,
	Switch,
	Mobile,
	EFortTipDisplayPlatformGroup_MAX
};

enum EPayoutScoringType
{
	Invalid,
	Points,
	Ranking,
	Percentile,
	Persistent,
	EPayoutScoringType_MAX
};

enum EPayoutRewardType
{
	Invalid,
	Commerce,
	GameRelated,
	Token,
	Score,
	EPayoutRewardType_MAX
};

enum ETournamentDisplayType
{
	Default,
	Creative,
	ETournamentDisplayType_MAX
};

enum EScoreMatchOperator
{
	Invalid,
	LessThan,
	LessThanOrEqual,
	Equal,
	GreaterThan,
	GreaterThanOrEqual,
	EScoreMatchOperator_MAX
};

enum EFTowhookExtensionState
{
	Hold,
	Extend,
	Contract,
	FTowhookExtensionState_MAX
};

enum EMessageFeedSubject
{
	ToyOwner,
	OtherPlayerInteractingWithToy,
	EMessageFeedSubject_MAX
};

enum EMessageFeedRelationshipFilter
{
	Anyone,
	SquadAndTeamMembers,
	SquadMembersOnly,
	SelfOnly,
	EMessageFeedRelationshipFilter_MAX
};

enum ETrackVerticality
{
	Floor,
	Ramp,
	GradualRamp,
	Max_None,
	ETrackVerticality_MAX
};

enum ETrackIncline
{
	NoNeighbor,
	Flat,
	Up,
	Down,
	GradualUp,
	GradualDown,
	Max_None,
	ETrackIncline_MAX
};

enum ETrackPieceType
{
	None,
	Straight,
	Turn,
	TShape,
	Cross,
	Max_None,
	ETrackPieceType_MAX
};

enum ETrackDirection
{
	YNegative,
	XPositive,
	YPositive,
	XNegative,
	Max_None,
	ETrackDirection_MAX
};

enum EFortServerTickRate
{
	UseDefault,
	Twenty,
	Thirty,
	EFortServerTickRate_MAX
};

enum EDuelState
{
	Started,
	Won,
	Lost,
	EDuelState_MAX
};

enum EMissionStormShieldState
{
	IDLE,
	GROWING,
	SHRINKING,
	MAX
};

enum EDynamicBackgroudKey
{
	Lobby,
	Vault,
	Max_None,
	EDynamicBackgroudKey_MAX
};

enum EFortLeaderboardMetric
{
	Score,
	Kills,
	TeamScore,
	EFortLeaderboardMetric_MAX
};

enum EUFortMatchmakingKnobsDataSource
{
	None,
	Playlist,
	Mutator,
	GameMode,
	Permissions,
	UISettings,
	CreativeGlobalOption,
	Max
};

enum ECreativeItemCategory
{
	Prefabs,
	Devices,
	Weapons,
	Consumables,
	Gallery,
	None,
	ECreativeItemCategory_MAX
};

enum EFortPlaylistTeamSizeType
{
	Solo,
	Duo,
	Trio,
	Squad,
	LargeTeam,
	MAX
};

enum EFortGameFeatureState
{
	Unknown,
	Unavailable,
	Downloading,
	DownloadFailed,
	Available,
	Preloading,
	Preloaded,
	Loading,
	Loaded,
	Count,
	EFortGameFeatureState_MAX
};

enum EFortGameFeature
{
	EarlyStartup,
	DedicatedServer,
	KairosInitial,
	KairosCapture,
	KairosLoadElectraPlayer,
	DanceRoyale,
	Frontend,
	GameplayAthena,
	GameplayCampaignTutorial,
	GameplayCampaign,
	GameplayCreative,
	Invalid,
	Count,
	EFortGameFeature_MAX
};

enum EFortStartupPhase
{
	InitializingEngine,
	EarlyStartupLoading,
	EarlyStartupFinished,
	GameStartupLoading,
	GameStartupFinished,
	Invalid,
	Count,
	EFortStartupPhase_MAX
};

enum EFortCreativeAdColorPreset
{
	Default,
	Emphasized,
	Common,
	Uncommon,
	Rare,
	Epic,
	Legendary,
	EFortCreativeAdColorPreset_MAX
};

enum EFortCreativeAdType
{
	Default,
	Playset,
	Toy,
	Game,
	Island,
	Knob,
	EFortCreativeAdType_MAX
};

enum EEventTournamentType
{
	Online,
	Onsite,
	Test,
	Unknown,
	EEventTournamentType_MAX
};

enum EFortLobbyType
{
	Default,
	Tournament,
	Creative,
	Division,
	EFortLobbyType_MAX
};

enum EFortMatchmakingViolatorStyle
{
	None,
	Basic,
	HighStakes,
	Showdown,
	EFortMatchmakingViolatorStyle_MAX
};

enum EFortMatchmakingTileStyle
{
	None,
	Special,
	HighStakes,
	Showdown,
	EFortMatchmakingTileStyle_MAX
};

enum EFortErrorSeverity
{
	Unspecified,
	Silent,
	Passive,
	Blocking,
	SevereBlocking,
	EFortErrorSeverity_MAX
};

enum EFortPickerMode
{
	BuildingCategory,
	TrapCategory,
	WeaponCategory,
	SocialCategory,
	Building,
	Trap,
	TrapRadial,
	Weapon,
	Social,
	DirectPickEmote,
	DirectPickSpray,
	SquadQuickChat,
	WeaponsSlotted,
	BattleLabDevice,
	MusicSource,
	EFortPickerMode_MAX
};

enum EPTTState
{
	Enabled,
	MicDisabled,
	AllSoundDisabled,
	EPTTState_MAX
};

enum EMobileInteractionIconTypes
{
	Interact,
	Swap,
	Revive,
	Blocked,
	MAX
};

enum EAthenaPickerType
{
	EditMode,
	Interact,
	MAX
};

enum EFortGliderType
{
	Glider,
	Umbrella,
	EFortGliderType_MAX
};

enum EItemListViewDisplayType
{
	World,
	Outpost,
	Account,
	DeployableBase,
	Max
};

enum EStormShieldDefense
{
	NotSSD,
	StormShieldExpansion,
	WargameSimulation,
	EStormShieldDefense_MAX
};

enum EFortClientAnnouncementQueueType
{
	Toasts,
	Stickies,
	Max
};

enum EFortNotificationQueueType
{
	Toasts,
	Stickies,
	Messages,
	Max
};

enum EFortDialogResult
{
	Confirmed,
	Declined,
	Ignored,
	Killed,
	TimedOut,
	Unknown,
	EFortDialogResult_MAX
};

enum ETutorialType
{
	None,
	Callout,
	GuardScreen,
	WidgetIntro,
	Highlight,
	ETutorialType_MAX
};

enum EFortBangType
{
	Invalid,
	Custom,
	PlayTab,
	VaultTab,
	StoreTab,
	FriendsButton,
	PartyInviteButton,
	SubGameAccessChanged,
	DailyRewardsButton,
	QuestsButton,
	CompletedExpeditions,
	MainMenu,
	HelpMenu,
	VaultSchematics,
	VaultLeadSurvivors,
	VaultSurvivors,
	VaultHeroes,
	VaultDefenders,
	VaultResources,
	VaultMelee,
	VaultRanged,
	VaultConsumables,
	VaultIngredients,
	VaultTraps,
	BattlePassTab,
	SpecialEventTab,
	CosmeticsTab,
	CosmeticsOutfit,
	CosmeticGlider,
	CosmeticContrail,
	CosmeticBattleBus,
	CosmeticVehicle,
	CosmeticItemWrap,
	CosmeticCallingCard,
	CosmeticMapMarker,
	CosmeticMusicPack,
	CosmeticPetSkin,
	CosmeticLoadingScreen,
	CosmeticBackpack,
	CosmeticHat,
	CosmeticPickaxe,
	CosmeticDance,
	CosmeticCharm,
	AthenaDirectedAcquisitionTab,
	ItemShop_RMTItemOfferTab,
	ItemShop_FeaturedTab,
	ItemShop_DailyTab,
	ItemShop_SpecialFeaturedTab,
	ItemShop_SpecialDailyTab,
	ItemShop_StandaloneTab,
	ItemShop_CommunityVotingTab,
	ItemShop_MegaBundleTab,
	ItemShop_BattlePassTab,
	AthenaChallengesTab,
	AthenaShowdownTab,
	AthenaCareerTab,
	PlayerBanners,
	STWCommand,
	STWCommand_Heroes,
	STWCommand_Heroes_Manage,
	STWCommand_Heroes_HeroLoadout,
	STWCommand_Heroes_Defenders,
	STWCommand_Heroes_Expeditions,
	STWCommand_Survivors,
	STWCommand_Survivors_Manage,
	STWCommand_Survivors_Squads,
	STWCommand_Upgrades,
	STWCommand_Research,
	STWCommand_AccountXP,
	STWArmory,
	STWArmory_Schematics,
	STWArmory_Backpack,
	STWArmory_Storage,
	STWArmory_CollectionBook,
	STWArmory_Transform,
	STWArmory_Resources,
	STWLocker,
	EFortBangType_MAX
};

enum EFortEventNameType
{
	Mission,
	Client,
	EFortEventNameType_MAX
};

enum ECraftingUpgradeDataFlags
{
	None,
	OverrideWrap,
	Durability,
	PhantomAmmo,
	LoadedAmmo,
	ModSlots,
	All,
	ECraftingUpgradeDataFlags_MAX
};

enum EFortCraftFailCause
{
	Unknown,
	NotEnoughResources,
	InventoryFull,
	InsufficientTeamLevel,
	CraftingQueueFull,
	CurrentlyLocked,
	OverflowSchematic,
	EFortCraftFailCause_MAX
};

enum EKeepDefenseState
{
	Inactive,
	Warmup,
	Defense,
	LastAlive,
	Max
};

enum EKeepContainerType
{
	Base,
	Storeroom,
	FirstAid,
	Armory,
	Workshop,
	AmmoStash,
	Max
};

enum EFortIndicatorState
{
	Hidden,
	OnlyFriendsVisible,
	Visible,
	EFortIndicatorState_MAX
};

enum EFortAIUtility
{
	KillPlayersMelee,
	KillPlayersRanged,
	KillPlayersArtillery,
	DestroyBuildingsMelee,
	DestroyBuildingsRanged,
	DestroyBuildingsArtillery,
	DestroyTraps,
	Tank,
	Infiltrate,
	Mob,
	Support,
	Kiting,
	AreaOfDenial,
	DisableTraps,
	Dormant,
	Assassin,
	MAX
};

enum EFortTileEdgeType
{
	Undefined,
	Outer_1,
	Transition_2,
	Inner_3,
	Border_4,
	BorderTransitionSingle_5,
	BorderTransitionDouble_6,
	MAX
};

enum EFortMovementUrgency
{
	None,
	Low,
	Medium,
	High,
	NumLevels,
	EFortMovementUrgency_MAX
};

enum EFortMovementStyle
{
	Running,
	Walking,
	Charging,
	Sprinting,
	PersonalVehicle,
	Flying,
	Tethered,
	Burrowing,
	EFortMovementStyle_MAX
};

enum EFortWeaponTriggerType
{
	OnPress,
	Automatic,
	OnRelease,
	OnPressAndRelease,
	EFortWeaponTriggerType_MAX
};

enum EFortDayPhasePrio
{
	Default,
	DailySummary,
	EFortDayPhasePrio_MAX
};

enum EFortPawnDisplayContext
{
	BattleRoyale,
	VaultItemsFromOffer,
	VaultItem,
	VaultItems,
	VaultItemFromDefinition,
	CampaignFrontEndPlayer,
	CampaignNPC,
	CampaignHeroInspect,
	CampaignHeroLoadoutSupport,
	CampaignOutfitPicker,
	EFortPawnDisplayContext_MAX
};

enum EFortCustomCharmType
{
	Weapon,
	Lapel,
	Backpack1,
	Backpack2,
	NumTypes,
	EFortCustomCharmType_MAX
};

enum EFortDisplayGender
{
	Unknown,
	Male,
	Female,
	NumTypes,
	EFortDisplayGender_MAX
};

enum EFortTargetSelectionFilter
{
	Damageable,
	Everything,
	Pawns,
	Buildings,
	Walls,
	Traps,
	Players,
	AIPawns,
	Instigator,
	WeakSpots,
	World,
	Max
};

enum EFortWeaponType
{
	None,
	RangedAny,
	Pistol,
	Shotgun,
	Rifle,
	SMG,
	Sniper,
	GrenadeLauncher,
	RocketLauncher,
	Bow,
	Minigun,
	LMG,
	BiplaneGun,
	MeleeAny,
	Harvesting,
	MAX
};

enum EFortTargetSelectionTestType
{
	Overlap,
	Swept,
	Ballistic,
	EFortTargetSelectionTestType_MAX
};

enum EFortTargetSelectionShape
{
	Sphere,
	Cone,
	Box,
	Capsule,
	Line,
	Cylinder,
	Custom,
	CustomOnSource,
	EFortTargetSelectionShape_MAX
};

enum EFortBrushSize
{
	VeryVerySmall,
	VerySmall,
	Small,
	Medium,
	Large,
	VeryLarge,
	EFortBrushSize_MAX
};

enum ESpecialActorStatType
{
	NumEliminationsNearby,
	TimeInWorld,
	PickupNumSpawns,
	PickupNumDespawns,
	PickupNumDropped,
	PickupNumTaken,
	PlayerWon,
	PlayerNumEliminations,
	PlayerNum,
	TotalStats,
	ESpecialActorStatType_MAX
};

enum EFortItemViewRotationMode
{
	None,
	BoundsPivot,
	World,
	Relative,
	EFortItemViewRotationMode_MAX
};

enum EChangeInStructDetected
{
	Dirty,
	Clean,
	EChangeInStructDetected_MAX
};

enum EFortInventoryCustomFilter
{
	Mythic,
	Legendary,
	Epic,
	Rare,
	Uncommon,
	Common,
	EFortInventoryCustomFilter_MAX
};

enum EInventoryContentSortType
{
	ByName,
	ByRating,
	ByLevel,
	ByCategory,
	ByRarity,
	ByLocation,
	ByPersonality,
	ByBonus,
	BySubtype,
	ByGrantTime,
	Invalid,
	EInventoryContentSortType_MAX
};

enum EFortFrontendInventoryFilter
{
	Schematics,
	WorldItems,
	WorldItemsInGame,
	WorldItemsStorage,
	WorldItemsTransfer,
	ConsumablesAndAccountResources,
	Heroes,
	Defenders,
	Survivors,
	AthenaCharacter,
	AthenaBackpack,
	AthenaPickaxe,
	AthenaGliders,
	AthenaContrails,
	AthenaEmotes,
	AthenaItemWraps,
	AthenaLoadingScreens,
	AthenaLobbyMusic,
	AthenaCharm,
	HestiaWeapons,
	HestiaResources,
	StarlightInventory,
	Invisible,
	Max_None,
	EFortFrontendInventoryFilter_MAX
};

enum EFortInventoryFilter
{
	WeaponMelee,
	WeaponRanged,
	Ammo,
	Traps,
	Consumables,
	Ingredients,
	Gadget,
	Decorations,
	Badges,
	Heroes,
	LeadSurvivors,
	Survivors,
	Defenders,
	Resources,
	ConversionControl,
	AthenaCosmetics,
	Playset,
	CreativePlot,
	TeamPerk,
	Workers,
	Invisible,
	Max_None,
	EFortInventoryFilter_MAX
};

enum EFortItemCategoryOrdinal
{
	Primary,
	Secondary,
	Tertiary,
	EFortItemCategoryOrdinal_MAX
};

enum ESubGameMatchmakingStatus
{
	Disabled,
	Enabled,
	ESubGameMatchmakingStatus_MAX
};

enum ESubGameAccessStatus
{
	Disabled,
	LimitedAccess,
	OpenAccess,
	ESubGameAccessStatus_MAX
};

enum ESettingTab
{
	None,
	Video,
	Game,
	GameUI,
	Brightness,
	Audio,
	Accessibility,
	Input,
	MouseAndKeyboard,
	Controller,
	ControllerSensitivity,
	TouchAndMotion,
	Account,
	CreativeWorld,
	CreativePlayer,
	ESettingTab_MAX
};

enum EUIExtensionSlot
{
	Primary,
	TopRightCorner,
	GameInfoBox,
	Quickbar,
	QuickbarUnderlay,
	UpperCenter,
	CrosshairRight,
	UnderSquadInfo,
	FullScreenMap,
	BelowRespawnWidget,
	BelowCompass,
	UnderTeammateStatus,
	ControllerBindingCallout,
	AboveStormMessageSlot,
	CustomMinigameCallouts,
	UnderLocalPlayerInfo,
	PlayerHealthbarOverlay,
	InventoryScreenReplacement,
	Reticle,
	KillfeedSlot,
	PrioritizedContextualSlot,
	RightOfTeammateStatus,
	TeammateStatusPortraitOverlay,
	MobileHUDBottomRight,
	InventoryScreenTab,
	InventoryPanelSubTab,
	MiniMapOverlay,
	InventoryEquipSlot,
	InventoryItemInfo,
	HUDEquippedItemInfo,
	UnderneathTeammateStatusList,
	FullScreenMapSquadEliminationsOverlay,
	PrimaryQuickBarSlot,
	MainMenuButtonListInZone,
	MainMenuButtonListFrontEnd,
	LobbyMatchmakingWidgetOverride,
	EUIExtensionSlot_MAX
};

enum EFortUIFriendNotificationType
{
	Default,
	FriendRequest,
	PartyInvite,
	InviteFriendToParty,
	PartyRequest,
	AutoImportFriendSuggestion,
	PartyMemberCreated,
	EFortUIFriendNotificationType_MAX
};

enum EFortNotificationPriority
{
	Vote,
	Friend,
	BoostedXP,
	TwitchHigh,
	GeneralSendNotification,
	TwitchLow,
	Max
};

enum EFortNotificationType
{
	Default,
	Power,
	HealthWarning,
	Error,
	GiftSent,
	VoiceChannel,
	FriendSubscriptionNudge,
	DonutChallenge,
	HousepartyWelcome,
	HousepartyFOMO,
	HousepartyMic,
	IncomingFriendRequest,
	SocialNotification,
	BattlePassPageUnlock,
	Max
};

enum EFortContextualReticleTypes
{
	FCR_GenericFailure,
	FCR_Upgrade,
	FCR_Repair,
	FCR_Locked,
	FCR_Placement,
	FCR_Edit,
	FCR_NoTarget,
	FCR_InProgress,
	FCR_None,
	FCR_MAX
};

enum EFortUserCloudRequestResult
{
	Success,
	Failure_CloudStorageDisabled,
	Failure_CloudStorageError,
	Failure_FileNotFoundInUserFileList,
	Failure_SavingNotAllowedForSpecifiedUser,
	EFortUserCloudRequestResult_MAX
};

enum EFortUserCloudRequestType
{
	LoadCloudFile,
	SaveCloudFile,
	EFortUserCloudRequestType_MAX
};

enum EFortValetVehicleType
{
	Default,
	Sport,
	BasicTruck,
	EFortValetVehicleType_MAX
};

enum EVehicleAudioTriggerDir
{
	Forward,
	Backward,
	EVehicleAudioTriggerDir_MAX
};

enum EVehicleAudioInterpolationType
{
	None,
	CustomCurve,
	Linear,
	EVehicleAudioInterpolationType_MAX
};

enum EVehicleFuelState
{
	Uninitialized,
	UsingVehicleFuel,
	NotUsingVehicleFuel,
	EVehicleFuelState_MAX
};

enum EVehicleSessionEndReason
{
	Invalid,
	NoPassengers,
	EVehicleSessionEndReason_MAX
};

enum EVehicleMovementMode
{
	OnGround,
	InAir,
	WipeOut,
	MaxCount,
	EVehicleMovementMode_MAX
};

enum ESlotEnvironmentExposure
{
	Unknown,
	Exposed,
	Protected,
	ESlotEnvironmentExposure_MAX
};

enum ETryExitVehicleBehavior
{
	DoNotForce,
	ForceOnBlocking,
	ForceAlways,
	ETryExitVehicleBehavior_MAX
};

enum EFortVisibilityBehavior
{
	Proximity,
	OnceSeenAlwaysSeen,
	AlwaysSeen,
	Invalid,
	EFortVisibilityBehavior_MAX
};

enum EVolumeValidityResult
{
	Valid,
	Invalid,
	EVolumeValidityResult_MAX
};

enum EFortWeaponAbilityRemovalPolicy
{
	GameDefault,
	Remove,
	Persist,
	EFortWeaponAbilityRemovalPolicy_MAX
};

enum EFortWeaponAbilityRemovalReason
{
	RemovedFromInventory,
	EndPlay,
	Unequipped,
	INVALID_MAX,
	EFortWeaponAbilityRemovalReason_MAX
};

enum EFortWeaponReduceMeshWorkSetting
{
	DisableTick,
	DontReduceWork,
	HandledInAnimInstance,
	EFortWeaponReduceMeshWorkSetting_MAX
};

enum EFortWeaponChargeStateForFireFX
{
	Partial,
	Full,
	Over,
	Max_None,
	EFortWeaponChargeStateForFireFX_MAX
};

enum EFortReloadMontageSection
{
	Intro,
	Loop,
	Outro,
	EFortReloadMontageSection_MAX
};

enum EFortRecoilCurveType
{
	WithTime,
	WithOverheat,
	EFortRecoilCurveType_MAX
};

enum EFortDisplayTier
{
	Invalid,
	Handmade,
	Copper,
	Silver,
	Malachite,
	Obsidian,
	Brightcore,
	Spectrolite,
	Shadowshard,
	Sunbeam,
	Moonglow,
	EFortDisplayTier_MAX
};

enum EFortMeleeFX
{
	Idle,
	Swing,
	AnimTrail,
	EFortMeleeFX_MAX
};

enum EFortDualWieldStance
{
	TwoPicksInUse,
	SinglePickDataDriven,
	EFortDualWieldStance_MAX
};

enum EFortDualWieldSwingState
{
	None,
	MainHand,
	OffHand,
	EFortDualWieldSwingState_MAX
};

enum EWorldItemDropBehavior
{
	DropAsPickup,
	DestroyOnDrop,
	DropAsPickupDestroyOnEmpty,
	DropAsPickupEvenWhenEmpty,
	EWorldItemDropBehavior_MAX
};

enum EFortWorldManagerState
{
	WMS_Created,
	WMS_QueryingWorld,
	WMS_WorldQueryComplete,
	WMS_CreatingNewWorld,
	WMS_LoadingExistingWorld,
	WMS_Running,
	WMS_Failed,
	WMS_MAX
};

enum EFortLevelStreamingState
{
	LSS_Unloaded,
	LSS_Loaded,
	LSS_UnconditionalFoundationsUpdated,
	LSS_AllFoundationsUpdated,
	LSS_NewActorsCreatedButNotUpdated,
	LSS_AllUpdated,
	LSS_Ready,
	LSS_MAX
};

enum EFortQueuedActionUserStatus
{
	Succeeded,
	Failed,
	WaitingForCloudRequest,
	WaitingForProfileSave,
	EFortQueuedActionUserStatus_MAX
};

enum EFortWorldRecordState
{
	StartProcessing,
	WaitingForResponse,
	RetrievingTheaterInformation,
	RetrievingZoneInformation,
	LoadingWorldRecord,
	LoadingZoneRecord,
	SavingZoneRecord,
	SavingPlayerProfiles,
	SavingPlayerDeployableBases,
	Succeeded,
	Failed,
	Max_None,
	EFortWorldRecordState_MAX
};

enum EFortWorldRecordAction
{
	LoadWorldOnly,
	SaveWorldOnly,
	SaveZoneAndWorld,
	LoadZoneAndWorld,
	SaveDeployableBasesAndWorld,
	Max_None,
	EFortWorldRecordAction_MAX
};

enum EDeployableBaseUseType
{
	Neighborhood,
	PvECombat,
	EDeployableBaseUseType_MAX
};

enum EFortZoneType
{
	PVE,
	PVP,
	Keep,
	SingleZone,
	Max_None,
	EFortZoneType_MAX
};

enum EFrontEndCamera
{
	Invalid,
	Command,
	Command_HeroLoadout,
	LegacyHeroLoadout,
	Cosmetics,
	Expeditions,
	FrontendDefault,
	Heroes,
	HeroSelect,
	HeroLoadout,
	Home,
	HomeBase,
	Login,
	Manage1,
	Manage2,
	Manage3,
	Manage4,
	MissionControl,
	Party,
	Play,
	Research,
	SkillTrees,
	SmallCosmetics,
	SpatialUI,
	SpecialEvent,
	SpecialEvent2,
	SpecialEvent3,
	Store,
	StoreItemInspect,
	StwFrontendDefault,
	SurvivorSquadBuilding1,
	SurvivorSquadBuilding2,
	SurvivorSquadBuilding3,
	SurvivorSquadBuilding4,
	TutorialPhaseOne,
	TutorialPhaseTwo,
	TutorialPhaseThree,
	Upgrades,
	Vault,
	WorldMap,
	LobbyCentered,
	CosmeticDisplay,
	BattlePass,
	Rewards,
	SpecialEventRewards,
	Armory,
	EFrontEndCamera_MAX
};

enum EPlayerAttributeClampType
{
	Minimum,
	Maximum,
	EPlayerAttributeClampType_MAX
};

enum EFortWeaponUpgradeInteractionResult
{
	Upgradable,
	NotEnoughResources,
	CannotUpgrade,
	CannotInteract,
	EFortWeaponUpgradeInteractionResult_MAX
};

enum EFortWeaponUpgradeDirection
{
	NotSet,
	Vertical,
	Horizontal,
	EFortWeaponUpgradeDirection_MAX
};

enum EFortWeaponUpgradeCosts
{
	NotSet,
	WoodUncommon,
	WoodRare,
	WoodVeryRare,
	WoodSuperRare,
	MetalUncommon,
	MetalRare,
	MetalVeryRare,
	MetalSuperRare,
	BrickUncommon,
	BrickRare,
	BrickVeryRare,
	BrickSuperRare,
	HorizontalWoodCommon,
	HorizontalWoodUncommon,
	HorizontalWoodRare,
	HorizontalWoodVeryRare,
	HorizontalWoodSuperRare,
	HorizontalMetalCommon,
	HorizontalMetalUncommon,
	HorizontalMetalRare,
	HorizontalMetalVeryRare,
	HorizontalMetalSuperRare,
	HorizontalBrickCommon,
	HorizontalBrickUncommon,
	HorizontalBrickRare,
	HorizontalBrickVeryRare,
	HorizontalBrickSuperRare,
	EFortWeaponUpgradeCosts_MAX
};

enum EDroneFacingLocationMode
{
	NotFacingLocation,
	FindingPoint,
	TrackingPoint,
	EDroneFacingLocationMode_MAX
};

enum EFireModeType
{
	Unset,
	TapToShoot,
	FireButton,
	AutoFire,
	ForceTouch,
	Custom,
	MAX
};

enum ELayoutDataType
{
	Custom,
	DefaultToolLayout,
	DefaultGameLayout,
	MAX_Local,
	CustomCloudLayout,
	ELayoutDataType_MAX
};

enum ESpawnPointState
{
	Inactive,
	Active_CarryObjectInRange,
	Active_CarryObjectOutOfRange,
	IntelCaptured,
	IntelDownloaded,
	ESpawnPointState_MAX
};

enum EIslandInspectorState
{
	Initializing,
	Ready,
	AwaitingProcessCommand,
	ProcessingCommand,
	EIslandInspectorState_MAX
};

enum ELevelSaveRecordVersion
{
	CloudSaveInfoAdded,
	TimestampConversion,
	SoftActorClassReferences,
	SoftActorComponentClassReferences,
	DuplicateNewActorRecordsRemoved,
	StartOfResaveWhenNotLatestVersion,
	LowerCloseThresholdForDuplicates,
	DeprecatedDeleteAndNewActorRecords,
	DependenciesFromSerializedWorld,
	RemovingSerializedDependencies,
	AddingVolumeInfoRecordsMap,
	AddingVolumeGridDependency,
	AddingScale,
	AddingDataHash,
	AddedIslandTemplateId,
	AddedLevelStreamedDeleteRecord,
	UsingSaveActorGUID,
	UsingActorFNameForEditorSpawning,
	AddedPlayerPersistenceUserWipeNumber,
	VersionPlusOne,
	LatestVersion,
	ELevelSaveRecordVersion_MAX
};

enum ELevelSaveCategory
{
	ActorInstance,
	VolumeInfoActor,
	ELevelSaveCategory_MAX
};

enum EFortEncounterSpawnLimitType
{
	NoLimit,
	NumPawnsLimit,
	SpawnPointLimit,
	UserDefined,
	MAX
};

enum EFortEncounterUtilitiesMode
{
	LockedOnly,
	LockedAndFree,
	EFortEncounterUtilitiesMode_MAX
};

enum EFortEncounterSpawnLocationManagementMode
{
	Spawn,
	Search,
	EFortEncounterSpawnLocationManagementMode_MAX
};

enum EFortEncounterSpawnLocationPlacementMode
{
	Directional,
	Ring,
	Volume,
	Custom,
	Max_None,
	EFortEncounterSpawnLocationPlacementMode_MAX
};

enum EFortEncounterPacingMode
{
	SpawnPointsPercentageCurve,
	IntensityCurve,
	Burst,
	Fixed,
	EFortEncounterPacingMode_MAX
};

enum EFortMissionAudibility
{
	UseVisibility,
	Audible,
	Inaudible,
	EFortMissionAudibility_MAX
};

enum EFortMissionType
{
	Primary,
	Secondary,
	Max_None,
	EFortMissionType_MAX
};

enum EFortObjectiveRequirement
{
	Optional,
	Required,
	RequiredButCanFail,
	EFortObjectiveRequirement_MAX
};

enum EFortMissionStatus
{
	Created,
	InProgress,
	Succeeded,
	Failed,
	NeutralCompletion,
	Quit,
	Max_None,
	EFortMissionStatus_MAX
};

enum EMissionGenerationCategory
{
	Primary,
	Secondary,
	Tertiary,
	Survivor,
	Max_None,
	EMissionGenerationCategory_MAX
};

enum EClipMessageSettings
{
	DontShow,
	ShowString,
	DeduceUnicornAnnotation,
	ShotTitleScreen,
	EClipMessageSettings_MAX
};

enum EPegasusTimelineCategories
{
	Unassigned,
	Player,
	Combat,
	Building,
	Inventory,
	Social,
	Resources,
	EPegasusTimelineCategories_MAX
};

enum ESceneQueryShape
{
	Sphere,
	Box,
	Capsule,
	ESceneQueryShape_MAX
};

enum EPrimitiveCrosstalkFunctions
{
	SendTagAsPayload,
	SendTaggedObjectPayload,
	SendTaggedStringPayload,
	SendTaggedIntPayload,
	GetTaggedObjectPayload,
	GetTaggedStringPayload,
	GetTaggedIntPayload,
	BindNoParamEventToTaggedDelegate,
	UnbindNoParamEventFromTaggedDelegate,
	UnbindObjectCallbacksFromAllTaggedDelegates,
	EPrimitiveCrosstalkFunctions_MAX
};

enum EQuickHealingRequirementFlags
{
	Nothing,
	NeedsHealing,
	NeedsShields,
	NeedsBoth,
	Invalid,
	EQuickHealingRequirementFlags_MAX
};

enum EVideoManagerJobTypes
{
	EliminationExtraction,
	DefinedShotSequence,
	EVideoManagerJobTypes_MAX
};

enum EVideoManagerStates
{
	INVALID,
	LoadingReplay,
	ScrubbingReplay,
	WaitingForShotSetup,
	WatchingShot,
	ExportingShot,
	PostExportedShot,
	EVideoManagerStates_MAX
};

enum ERespawnAndSpectatePlayerRespawningState
{
	None,
	WaitingOnTimer,
	WaitingOnPlayerSelection,
	WaitingOnClientReady,
	Respawning,
	RespawnFinishing,
	ERespawnAndSpectatePlayerRespawningState_MAX
};

enum ESaveLocation
{
	Local_ForDevice,
	Local_MAX
};

enum EIndexNavigationResult
{
	Succeeded,
	Modified,
	Clamped,
	StepOff,
	NoMove,
	EIndexNavigationResult_MAX
};

enum ESpecialRelevancyMode
{
	NormalRelevancy,
	SoloRelevancy,
	SquadRelevancy,
	MultiSquad,
	MAX
};

enum EFortThreatDeactivationType
{
	Off,
	Dormant,
	EFortThreatDeactivationType_MAX
};

enum EUnicornSocialFeatures
{
	INVALID,
	FriendPlay,
	DancePartyManpower,
	CongaLineManpower,
	SocialPartyDuration,
	COUNT,
	EUnicornSocialFeatures_MAX
};

enum EHighlightReelIds
{
	INVALID,
	MainHighlightReel,
	ShortHighlightReel,
	MicroHighlights,
	EntireGameReel,
	ShortExtendedHighlightReel,
	MediumHighlightReel,
	MediumExtendedHighlightReel,
	ShorterHighlightReel,
	PlayerSpotlightReel,
	PlayerSpotlightNoDeathsReel,
	VATReel,
	COUNT,
	EHighlightReelIds_MAX
};

enum EWrapPreviewGridLockerMode
{
	IgnoreLockerConfiguration,
	SupportedWeaponsOnly,
	UnsupportedWeaponsOnly,
	EWrapPreviewGridLockerMode_MAX
};

enum EFortItemCooldownType
{
	None,
	AmmoRegeneration,
	ItemActivation,
	WeaponReloading,
	Death,
	AthenaWeaponFireCooldown,
	AbilitySetActivateByInputAbility,
	EFortItemCooldownType_MAX
};

enum EFortUIFeatureStateReason
{
	Default,
	Tutorial,
	ContentInstall,
	AccountRestrictions,
	Platform,
	SeasonOrEventNotActive,
	NoPlayerController,
	UnexpectedFeature,
	Invalid,
	EFortUIFeatureStateReason_MAX
};

enum EFortUIFeatureState
{
	Enabled,
	Disabled,
	Hidden,
	Invalid,
	EFortUIFeatureState_MAX
};

enum EFortUIFeature
{
	ShowHome,
	ShowPlay,
	ShowCommand,
	ShowHeros,
	ShowSquads,
	ShowArmory,
	ShowLocker,
	ShowSkillTree,
	ShowStore,
	ShowQuests,
	ShowMainStore,
	ShowContextHelpMenu,
	ShowCampaign,
	ShowActiveBoost,
	ShowJournal,
	ShowPartyBar,
	ShowChatWindow,
	ShowFriendsMenu,
	ShowObjectives,
	ShowRatingIconsInTopbar,
	ShowAntiAddictionMessage,
	ShowMinorShutdownMessage,
	ShowHealthWarningScreen,
	ShowSimplifiedHUD,
	LargeHeaderFooterButtons,
	ShowAthenaFavoriting,
	ShowAthenaItemRandomization,
	ShowBattlePass,
	ShowNewBattlePass,
	ShowDynamicBattlePass,
	ShowBattlePassFAQ,
	ShowReplays,
	ShowProfileStatsUI,
	ShowAthenaItemShop,
	ShowNewAthenaItemShop,
	ShowAthenaCataba,
	ShowShowdown,
	ShowSpecialEvent,
	ShowSpatialUI,
	ShowSocial,
	ShowAccountBoosts,
	ShowCustomerSupport,
	ShowGlobalChat,
	ShowEULA,
	ShowEndOfZoneCinematic,
	ShowOnboardingCinematics,
	ShowFounderBannerIcons,
	ShowCurrentRegionInLobby,
	ShowPrerollLlamas,
	EnableFoundersDailyRewards,
	EnableTwitchIntegration,
	EnableMatchmakingRegionSetting,
	EnableLanguageSetting,
	EnableFriendCodeSetting,
	EnableEarlyAccessLoadingScreenBanner,
	EnableAlterationModifications,
	EnableSchematicRarityUpgrade,
	EnableMissionActivationVote,
	EnableLtmRetrieveTheData,
	EnableUpgradesVideos,
	ShowPreviewTestTab,
	Max_None,
	EFortUIFeature_MAX
};

enum EFlagStatus
{
	FlagPresent,
	FlagNotPresent,
	EFlagStatus_MAX
};

enum EInputPriority
{
	Normal,
	Menu,
	Chat,
	Modal,
	Confirmation,
	Error,
	HUD,
	EInputPriority_MAX
};

enum EFortInputMode
{
	Frontend,
	InGame,
	InGameCursor,
	EFortInputMode_MAX
};

enum EFortUrlType
{
	NormalWebLink,
	AccountCreationLink,
	HelpLink,
	EULALink,
	EFortUrlType_MAX
};

enum EFortHitPointModificationReason
{
	Invalid,
	InitalSet,
	AutoRegen,
	ItemRegen,
	DamageOverTime,
	DamageReceived,
	EFortHitPointModificationReason_MAX
};

enum EFortBuildingInteraction
{
	None,
	Build,
	Repair,
	Upgrade,
	Edit,
	BeingModified,
	ConfirmEdit,
	Creative,
	EFortBuildingInteraction_MAX
};

enum EFortBuildingHealthDisplayRule
{
	Never,
	Allowed,
	Always,
	EFortBuildingHealthDisplayRule_MAX
};

enum EItemDisassembleRestrictionReason
{
	InnatelyCannotDisassemble,
	ItemWasGifted,
	EItemDisassembleRestrictionReason_MAX
};

enum EItemRecyclingRestrictionReason
{
	InnatelyUnrecyclable,
	IsSlottedGroundOperative,
	MissingCatalyst,
	ItemOutOnExpedition,
	InUseByCrafting,
	MulchingNotAllowed,
	IsSlottedAttributeWorker,
	EItemRecyclingRestrictionReason_MAX
};

enum EItemRecyclingWarning
{
	HighLevel,
	HighRarity,
	CanSlotInCollectionBook,
	EItemRecyclingWarning_MAX
};

enum EConversionControlKeyRequest
{
	AllKeys,
	NonConsumableKeys,
	ConsumableKeys,
	EConversionControlKeyRequest_MAX
};

enum EVaultItemLimitStatus
{
	WellBelowCapacity,
	NearCapacity,
	AtCapacity,
	OverCapacity,
	EVaultItemLimitStatus_MAX
};

enum EFortUISpecialEvents
{
	NoEvent,
	FritTemp,
	EFortUISpecialEvents_MAX
};

enum EFortTutorialGlowType
{
	None,
	Look,
	Click,
	EFortTutorialGlowType_MAX
};

enum EFortBangSize
{
	XXS,
	XS,
	S,
	M,
	L,
	XL,
	EFortBangSize_MAX
};

enum EFortUIState
{
	Invalid,
	Login,
	JoinServer,
	SubgameSelect,
	FrontEnd,
	InGame_Custom,
	UNUSED,
	InGame_STW,
	Cinematic,
	InGame_BR,
	AthenaSpectator,
	Replay,
	InGame_STW_Custom,
	MAX
};

enum EFortStoreState
{
	Error,
	Closed,
	CardPackStore,
	CurrencyStore,
	WebPayment,
	PurchaseOpen,
	PackOpen,
	CardEnter,
	CardBackReveal,
	CardFlip,
	CardFrontReveal,
	CardExit,
	SummaryAdd,
	PackDestroy,
	Summary,
	PresentChoice,
	ChoiceMade,
	SummaryAddTransition,
	MAX_None,
	EFortStoreState_MAX
};

enum EPostGameHUDMode
{
	None,
	AllHidden,
	Spectating,
	AllHiddenExceptXP,
	EPostGameHUDMode_MAX
};

enum EPostGameClickCatcherMode
{
	Catch_None,
	Catch_MobileOnly,
	Catch_MouseOnly,
	Catch_All,
	Catch_MAX
};

enum EFortFrontEndFeatureStateReason
{
	Default,
	NoHeroes,
	Tutorial,
	BROnly,
	NoPlayerController,
	UnexpectedFeature,
	Invalid,
	EFortFrontEndFeatureStateReason_MAX
};

enum EFortFrontEndFeatureState
{
	Enabled,
	Disabled,
	Hidden,
	Invalid,
	EFortFrontEndFeatureState_MAX
};

enum EFortFrontEndFeature
{
	ShowHomeBase,
	ShowHeroList,
	ShowVault,
	ShowStore,
	PlayZone,
	ShowDailyRewards,
	ShowHeroSelect,
	RecruitHero,
	ShowHomeBaseOverview,
	STWArmory_Transform,
	STWArmory_CollectionBook,
	MAX_None,
	EFortFrontEndFeature_MAX
};

enum EActivatePanelOption
{
	Show,
	Hide,
	PlatformDefault,
	EActivatePanelOption_MAX
};

enum EFortPlayerPowerRatingType
{
	Auto,
	Campaign,
	Phoenix,
	Max_None,
	EFortPlayerPowerRatingType_MAX
};

enum EGridSortKind
{
	None,
	ByNumber,
	ByString,
	ByNumberThenString,
	ByStringThenNumber,
	EGridSortKind_MAX
};

enum EFortItemInspectionMode
{
	Overview,
	Details,
	View,
	Evolution,
	Upgrade,
	UpgradeRarity,
	Promotion,
	EFortItemInspectionMode_MAX
};

enum EFortItemCardSize
{
	XXS,
	XS,
	Wide_S,
	S,
	M,
	L,
	XL,
	XXL,
	EFortItemCardSize_MAX
};

enum EFortCollectionBookPopupButtonType
{
	Invalid,
	SelectItem,
	Preview,
	Purchase,
	Unslot,
	Back,
	EFortCollectionBookPopupButtonType_MAX
};

enum ECollectionBookSectionNavTarget
{
	SlotSelect,
	SlotPicker,
	ECollectionBookSectionNavTarget_MAX
};

enum EViewerNavigationDirection
{
	Previous,
	Next,
	EViewerNavigationDirection_MAX
};

enum EHeroLoadoutSlotType
{
	CommanderSlot,
	TeamPerkSlot,
	CrewSlot,
	GadgetSlot,
	EHeroLoadoutSlotType_MAX
};

enum ESquadSlotSortType
{
	ByRating,
	ByLevel,
	ByRarity,
	ByBonus,
	ByMatch,
	ESquadSlotSortType_MAX
};

enum EAvailableSquadSlotsListEntryState
{
	Detailed,
	Simplified,
	EAvailableSquadSlotsListEntryState_MAX
};

enum EBattleMapTimelineWidgetState
{
	None,
	HasNext,
	HasPrevious,
	IsInReplay,
	CanBeScrubbed,
	IsStreaming,
	Invalid,
	EBattleMapTimelineWidgetState_MAX
};

enum ECountdownDisplay
{
	EventEndTime,
	ChallengeUnlockTime,
	ChallengeBundleUnlockTime,
	UnlockAlreadySet,
	MAX
};

enum EChallengeInfoPage
{
	PartyAssist,
	Daily,
	Suggested,
	Contextual,
	Selected,
	EChallengeInfoPage_MAX
};

enum EChallengeListSection
{
	SpecialOffers,
	CompletionRewards,
	AllChallenges,
	FreeChallenges,
	PaidChallenges,
	Objectives,
	EChallengeListSection_MAX
};

enum EAthenaChallengeTimerState
{
	Hidden,
	WeeksRemaining,
	DaysRemaining,
	HoursRemaining,
	MinutesRemaining,
	Urgent,
	EAthenaChallengeTimerState_MAX
};

enum ELocationEntryState
{
	Found,
	NotFound,
	Unused,
	ELocationEntryState_MAX
};

enum EAthenaConfirmationResult
{
	Confirmed,
	Declined,
	Canceled,
	Max_NONE,
	EAthenaConfirmationResult_MAX
};

enum EEquippedWeaponDisplay
{
	None,
	Resource,
	Magazine,
	Utility,
	Chargeable,
	EEquippedWeaponDisplay_MAX
};

enum EAthenaEventMatchInfoSortMethod
{
	Eliminations,
	Place,
	Count,
	EAthenaEventMatchInfoSortMethod_MAX
};

enum EAthenaGameFeatureStatus
{
	InProgress,
	ProgressPaused,
	ErrorOccured,
	EAthenaGameFeatureStatus_MAX
};

enum EAthenaPlayerActionAlert
{
	PlayerDown,
	PlayerKill,
	EnteredStorm,
	NewZebulonDrone,
	NewZebulonDroneYou,
	EAthenaPlayerActionAlert_MAX
};

enum EViolatorIntensity
{
	Low,
	Medium,
	High,
	EViolatorIntensity_MAX
};

enum EItemShopTileSize
{
	Mini,
	Small,
	Normal,
	DoubleWide,
	TripleWide,
	Max
};

enum EItemShopCurrency
{
	VBucks,
	RealMOney,
	EItemShopCurrency_MAX
};

enum EFortAthenaPlaylist
{
	AthenaSolo,
	AthenaDuo,
	AthenaSquad,
	EFortAthenaPlaylist_MAX
};

enum EAthenaLockerInfoRestrictionWarning
{
	UnsatisfiedExclusiveItem,
	LockedEmote,
	CosmeticRestriction,
	Unknown,
	EAthenaLockerInfoRestrictionWarning_MAX
};

enum EAtheaMapTabType
{
	Invalid,
	Quest,
	Map,
	Collection,
	EAtheaMapTabType_MAX
};

enum EFortMarkerActions
{
	None,
	Cancel,
	Confirm,
	EFortMarkerActions_MAX
};

enum EMinigameActivityWidgetStatFormat
{
	Score,
	Time,
	AddTime,
	Distance,
	Laps,
	EMinigameActivityWidgetStatFormat_MAX
};

enum EAthenaNewsStyle
{
	None,
	SpecialEvent,
	SpecialEvent2,
	EAthenaNewsStyle_MAX
};

enum EAthenaNewsEntryType
{
	Text,
	Item,
	Website,
	NavigateToTab,
	Challenge,
	Playlist,
	Setting,
	SpatialScreen,
	SmallNews,
	NavigateToBattlePassSubPage,
	Invalid,
	EAthenaNewsEntryType_MAX
};

enum EPurchaseButtonsMode
{
	Default,
	ClaimSubscriptionRewards,
	SubscriptionPaymentError,
	SubscriptionBlockedBenefits,
	EPurchaseButtonsMode_MAX
};

enum EPassOfferSelected
{
	BattlePass,
	Subscription,
	Max_None,
	EPassOfferSelected_MAX
};

enum EHealthBarType
{
	Health,
	Shield,
	Stamina,
	VehicleHealth,
	SignalInStorm,
	EHealthBarType_MAX
};

enum EBPStatus
{
	UnpurchasedBP,
	PurchasedBP,
	EBPStatus_MAX
};

enum EPunchType
{
	Horizontal,
	Vertical,
	EPunchType_MAX
};

enum EPunchCardLocation
{
	NONE,
	HUD,
	Map,
	EndGame,
	Lobby,
	EPunchCardLocation_MAX
};

enum EAthenaSquadListUpdateType
{
	None,
	CanResurrect,
	FindResurrectChip,
	EAthenaSquadListUpdateType_MAX
};

enum ERespawnUIState
{
	Hidden,
	ShowRespawnEnabled,
	ShowRespawnDisabled,
	ERespawnUIState_MAX
};

enum EFortSocialDateTimeStyle
{
	Default,
	Short,
	Medium,
	Long,
	Full,
	EFortSocialDateTimeStyle_MAX
};

enum ESpectatorBuildCountType
{
	BuildCount,
	Wood,
	Stone,
	Metal,
	ESpectatorBuildCountType_MAX
};

enum ESpectatorMapPlayerListState
{
	Default,
	Irrelevant,
	Eliminated,
	ESpectatorMapPlayerListState_MAX
};

enum EAthenaSpectatorNameplateDetailState
{
	High,
	Low,
	Arrow,
	Off,
	EAthenaSpectatorNameplateDetailState_MAX
};

enum EAthenaSpectatorNameplateDistanceState
{
	Near,
	MidDistance,
	FurtherThanMaxDistance,
	EAthenaSpectatorNameplateDistanceState_MAX
};

enum ESpectatorPlayerListSortMethod
{
	SquadId,
	PlayerName,
	Eliminations,
	EventScore,
	State,
	Count,
	ESpectatorPlayerListSortMethod_MAX
};

enum EStormSurgeThresholdType
{
	None,
	Above,
	Below,
	Equal,
	EStormSurgeThresholdType_MAX
};

enum EOptionalFlowSteps
{
	TryShowMobileGuidedTutorial,
	TryPlayTrailer,
	TryShowMOTDs,
	TryShowNormalBanModal,
	TryShowSocialBanModal,
	TryShowMFAModal,
	TryShowCrossplayDialog,
	TryShowSocialImport,
	TryShowSurveys,
	TryShowFireModeModal,
	TryShowBadMatchPopup,
	TryShowMobileInGameAppRating,
	TryShowSamsungSensorWarning,
	TryShowBattlePassPurchaseScreen,
	TryPushGiftingScreen,
	TryPushMessagingScreen,
	TryGoToBattlePassTab,
	EOptionalFlowSteps_MAX
};

enum EFrontendVisibilityMode
{
	Normal,
	HideTopTabsOnly,
	HideTopTabsOnlyWithoutBottomBar,
	OnlyBottom,
	OnlyTop,
	OnlyTitleBar,
	Empty,
	EFrontendVisibilityMode_MAX
};

enum EWinConditionParentType
{
	None,
	Desktop,
	Mobile,
	EWinConditionParentType_MAX
};

enum EComboSlotType
{
	Primary,
	Secondary,
	Combo,
	Creative,
	COUNT,
	EComboSlotType_MAX
};

enum EBacchusSignalQuality
{
	None,
	Low,
	Medium,
	High,
	EBacchusSignalQuality_MAX
};

enum EBattleLabAlertType
{
	QuestComplete,
	QuestGranted,
	Reward,
	EBattleLabAlertType_MAX
};

enum EBattlePassInputs
{
	Back,
	ViewItem,
	PreviewAction,
	ReplayTrailer,
	ShowAbout,
	BulkBuyRewards,
	ShowAboutCustomization,
	COUNT,
	EBattlePassInputs_MAX
};

enum EBattlePassPurchaseButtonLayout
{
	Normal,
	Bundle,
	Normal_PaysForSelf,
	EBattlePassPurchaseButtonLayout_MAX
};

enum ERewardPageType
{
	Reward,
	Quest,
	Bonus,
	MAX
};

enum EBattlePassCurrencyType
{
	BattleStar,
	CustomSkin,
	TOTAL_CURRENCIES,
	EBattlePassCurrencyType_MAX
};

enum EBattlePassFeatures
{
	None,
	BuyBattlePass,
	GiftBattlePass,
	BuySubscription,
	ViewDetails,
	WatchVideo,
	ShowAbout,
	PurchaseResources,
	EBattlePassFeatures_MAX
};

enum EBattlePassView
{
	None,
	LandingPage,
	RewardOverview,
	ItemDetails,
	BulkBuyRewards,
	CharacterCustomizer,
	BonusRewards,
	Quests,
	EBattlePassView_MAX
};

enum ECobaltStatusTeam
{
	Ally,
	Enemy,
	MAX
};

enum EBracketNodeState
{
	LocalTeam,
	EnemyTeam,
	Neutral,
	EBracketNodeState_MAX
};

enum ESurvivalObjectiveIconState
{
	None,
	Spawned,
	Destroyed,
	ESurvivalObjectiveIconState_MAX
};

enum EDiscoCaptureUIState
{
	None,
	Hidden,
	Dance,
	Capturing,
	Contested,
	EDiscoCaptureUIState_MAX
};

enum EDiscoScoreProgressTypes
{
	None,
	ProgressSoundMild,
	ProgressSoundMedium,
	ProgressSoundStrong,
	CountdownSoundNormal,
	CountdownSoundStrong,
	EDiscoScoreProgressTypes_MAX
};

enum EDiscoCaptureProgressState
{
	None,
	AllyProgress,
	EnemyProgress,
	EDiscoCaptureProgressState_MAX
};

enum EDiscoCaptureIconState
{
	None,
	Hidden,
	Neutral,
	AllyCaptured,
	EnemyCaptured,
	EDiscoCaptureIconState_MAX
};

enum EFortDonutIdleGameObject
{
	UnsetObject,
	Tree,
	Mound,
	SmallMound,
	JumpPad,
	Torch,
	PeelMonster,
	Chimichanga,
	Arm,
	EFortDonutIdleGameObject_MAX
};

enum EEndOfMatchQuestCategoryType
{
	Completed,
	InProgress,
	EEndOfMatchQuestCategoryType_MAX
};

enum EUFortActivatableVideoEvents
{
	Finished,
	Skipped,
	UFortActivatableVideoEvents_MAX
};

enum EUFortActivatableVideoPanelEvents
{
	Finished,
	Skipped,
	UFortActivatableVideoPanelEvents_MAX
};

enum EFortAlterationOptionType
{
	Upgrade,
	Replacement,
	Max_NONE,
	EFortAlterationOptionType_MAX
};

enum EFortAlterationWidgetState
{
	Normal,
	Upgrade,
	Evolution,
	EFortAlterationWidgetState_MAX
};

enum EFillDisableReason
{
	Enabled,
	FillDisabledOnPlaylist,
	NotPartyLeader,
	AlreadyMatchmaking,
	PartyTooSmall,
	PartyTooBig,
	InactiveTournament,
	NoSplitscreen,
	EFillDisableReason_MAX
};

enum EServerAccessSetting
{
	Invalid,
	FriendsOfCurrentPlayers,
	LeaderInviteOnly,
	EServerAccessSetting_MAX
};

enum ESpectatorQueueType
{
	Invalid,
	Player,
	BroadcastSpectator,
	ESpectatorQueueType_MAX
};

enum ESquadFillSetting
{
	Invalid,
	Fill,
	NoFill,
	ESquadFillSetting_MAX
};

enum ETutorialButtonInteractionType
{
	Click,
	Press,
	ETutorialButtonInteractionType_MAX
};

enum EHighlightType
{
	ESquareHighlight,
	ECircleHighlight_Big,
	ECircleHighlight_Small,
	EHighlightType_MAX
};

enum EFortAthenaTutorialScreenExtraWidget
{
	None,
	DragToTurn,
	Completed,
	Count,
	EFortAthenaTutorialScreenExtraWidget_MAX
};

enum EFortAthenaTutorialSubstep
{
	ScreenSwipeToLook,
	ScreenFindMarker,
	ScreenUseLeftStick,
	ScreenMoveToBuilding,
	ScreenJump,
	ScreenCrouch,
	ScreenReachMarker,
	ScreenUsePickaxe,
	ScreenReachPickUp,
	ScreenPickUpItems,
	ScreenEquipItem,
	ScreenShootTargets,
	ScreenReload,
	ScreenReachLocation,
	ScreenDefendYourself,
	ScreenHealthAlert,
	ScreenDestroyEnemies,
	ScreenCollectLoot,
	ScreenUseMedkit,
	ScreenUseShield,
	ScreenShieldInfo,
	ScreenLookForChest,
	ScreenChestFound,
	ScreenSelectBuildMode,
	ScreenShowMaterials,
	ScreenShowBuildPieces,
	ScreenSelectStairs,
	ScreenPlaceStairs,
	ScreenReachChest,
	ScreenExitBuildMode,
	ScreenLootChest,
	ScreenCollectLootChest,
	ScreenEquipRifle,
	ScreenUseScope,
	ScreenShootTargetsScoping,
	ScreenCompleted,
	HealingInterrupted,
	Count,
	EFortAthenaTutorialSubstep_MAX
};

enum EFortAthenaTutorialStep
{
	Look,
	Move,
	Harvest,
	Pickup,
	Shoot,
	Ambush,
	Heal,
	Build,
	Chest,
	Scoping,
	Completed,
	Count,
	EFortAthenaTutorialStep_MAX
};

enum EFuelTankState
{
	Empty,
	LowFuel,
	RegularFuel,
	EFuelTankState_MAX
};

enum EFortMemberConnectionState
{
	Open,
	Connecting,
	Connected,
	Invalid,
	EFortMemberConnectionState_MAX
};

enum ECollectionBookRewardStatus
{
	Unknown,
	Available,
	Claimed,
	ECollectionBookRewardStatus_MAX
};

enum ECollectionBookPrimaryNavTarget
{
	Overview,
	SectionTileView,
	ECollectionBookPrimaryNavTarget_MAX
};

enum EColorPickerColorRepresentation
{
	HSV,
	RGB,
	Max_NONE,
	EColorPickerColorRepresentation_MAX
};

enum EColorPickerType
{
	Swatches,
	CustomColor,
	Both,
	EColorPickerType_MAX
};

enum ELeaderboardDisplayType
{
	Default,
	Floating,
	ELeaderboardDisplayType_MAX
};

enum EFortLoadoutCardType
{
	Items,
	RandomTile,
	AddTile,
	AddPreviewItems,
	Blank,
	EFortLoadoutCardType_MAX
};

enum EFortCosmeticLoadoutScreenMode
{
	Browse,
	Save,
	EFortCosmeticLoadoutScreenMode_MAX
};

enum EFortCreativeItemType
{
	Chest,
	Item,
	Collection,
	SubItems,
	FNStudio,
	EFortCreativeItemType_MAX
};

enum EFortCreativeIslandLinkCategory
{
	Default,
	Favorite,
	Published,
	Recent,
	EFortCreativeIslandLinkCategory_MAX
};

enum EFortCreativeIslandLinkValidationResult
{
	Unknown,
	Success,
	IslandNotFound,
	InvalidKeyTooShort,
	InvalidKeyTooLong,
	InvalidKeyCharacters,
	IneligibleParty,
	IslandPrivate,
	EFortCreativeIslandLinkValidationResult_MAX
};

enum EFortCreativeIslandSelectTabType
{
	IslandCode,
	ListView,
	EFortCreativeIslandSelectTabType_MAX
};

enum EFortContentBrowserQuickbarState
{
	Disabled,
	Creative,
	Primary,
	Trap,
	EFortContentBrowserQuickbarState_MAX
};

enum EFortCreativeServerPrivacySetting
{
	Unknown,
	Private,
	Public,
	EFortCreativeServerPrivacySetting_MAX
};

enum EFortDefenderSlotType
{
	Invalid,
	Defender,
	Weapon,
	EFortDefenderSlotType_MAX
};

enum EDynamicEntryPatternDirection
{
	FirstToLast,
	LastToFirst,
	EDynamicEntryPatternDirection_MAX
};

enum EDateFormat
{
	CountdownTextual,
	CountdownNumeric,
	CountdownNumUnder12Hours,
	Date,
	DateFormat_MAX
};

enum EFortExpeditionListSort
{
	ByRating,
	ByDuration,
	ByName,
	EFortExpeditionListSort_MAX
};

enum EShareButtonType
{
	IOS,
	Android,
	Generic,
	EShareButtonType_MAX
};

enum EFrontEndRewardType
{
	Mission,
	Quest,
	EpicNewQuest,
	Expedition,
	CollectionBook,
	MissionAlert,
	DifficultyIncrease,
	GiftBox,
	ItemCache,
	PhoenixLevelUp,
	EFrontEndRewardType_MAX
};

enum ESelectionState
{
	Unselected,
	Selected,
	CannotGift,
	ESelectionState_MAX
};

enum EFortHeroPerkDisplayType
{
	CommanderPerk,
	TeamPerk,
	ClassPerk,
	StandardPerk,
	Max_None,
	EFortHeroPerkDisplayType_MAX
};

enum EFortSupportPerkWidgetState
{
	Normal,
	Upgrade,
	Evolution,
	EFortSupportPerkWidgetState_MAX
};

enum ECenterPopupMessageStateEnum
{
	NotVisible,
	WaitingForOutpostOwner,
	ECenterPopupMessageStateEnum_MAX
};

enum EBuildingFocusType
{
	Contextual,
	Interactable,
	Normal,
	Count,
	EBuildingFocusType_MAX
};

enum EContextPositionPlatform
{
	NonMobile,
	Mobile,
	EContextPositionPlatform_MAX
};

enum EFortItemCountStyle
{
	StackCount,
	OverrideCount,
	StackCountOverOverride,
	EFortItemCountStyle_MAX
};

enum EFortItemManagementMode
{
	Details,
	Comparison,
	Mulch,
	EFortItemManagementMode_MAX
};

enum EPresentationMode
{
	ItemList,
	VariantList,
	EPresentationMode_MAX
};

enum EItemContextAction
{
	Equip,
	GoToBattlePassRewards,
	GoToBattlePassCustomization,
	Count,
	EItemContextAction_MAX
};

enum ESceneTransitionType
{
	NoTransition,
	Clockwise,
	CounterClockwise,
	ESceneTransitionType_MAX
};

enum EFortKeybindForcedHoldStatus
{
	NoForcedHold,
	ForcedHold,
	NeverShowHold,
	EFortKeybindForcedHoldStatus_MAX
};

enum EFortLegacySlateWidget
{
	None,
	Minimap,
	MAX
};

enum ELocalUserOnlineStatus
{
	Online,
	Offline,
	Away,
	ExtendedAway,
	DoNotDisturb,
	Chat,
	ELocalUserOnlineStatus_MAX
};

enum EFortLoginInteraction
{
	Begin,
	PlayedBefore,
	CredentialSelect,
	NamePassword,
	RedirectEpicAccount,
	AccountNotFound,
	CreateDisplayName,
	MultiFactorAuth,
	EULA,
	AccountLink,
	AccountPinLink,
	WebLogin,
	WebAccountCreation,
	AgeGate,
	EFortLoginInteraction_MAX
};

enum EFortModifiedStatus
{
	IsDefault,
	IsModified,
	Unsupported,
	EFortModifiedStatus_MAX
};

enum EFortMaterialProgressBarSectionOverflowBehavior
{
	PreserveValues,
	ReverseCollapse,
	EFortMaterialProgressBarSectionOverflowBehavior_MAX
};

enum EFortMaterialProgressBarSectionColorNumber
{
	Color1,
	Color2,
	EFortMaterialProgressBarSectionColorNumber_MAX
};

enum EFortMaterialProgressBarSection
{
	Primary,
	Secondary,
	Tertiary,
	Negative,
	MAX_PROGRESS_BAR_SECTIONS,
	EFortMaterialProgressBarSection_MAX
};

enum EFortMissionActivationWidgetState
{
	Default,
	StartObjective,
	IncreaseDifficulty,
	Invalid,
	EFortMissionActivationWidgetState_MAX
};

enum EActionBindingComparisonType
{
	NoneBound,
	AnyBound,
	AllBound,
	EActionBindingComparisonType_MAX
};

enum ETagComparisonType
{
	HasAny,
	HasAll,
	HasNone,
	ETagComparisonType_MAX
};

enum EFortFortMobileShareButtonOS
{
	Android,
	iOS,
	EFortFortMobileShareButtonOS_MAX
};

enum EModalContainerSlot
{
	Top,
	Middle,
	Bottom,
	Background,
	Max
};

enum EFortMtxOfferDisplaySize
{
	Small,
	Medium,
	Large,
	EFortMtxOfferDisplaySize_MAX
};

enum EFortMtxStoreOfferType
{
	FoundersPack,
	CurrencyPack,
	Unknown,
	Max_None,
	EFortMtxStoreOfferType_MAX
};

enum EFortNotificationEntryCompletionStatus
{
	New,
	Active,
	Completed,
	MAX
};

enum ESettingType
{
	None,
	Header,
	WindowMode,
	DisplayResolution,
	FrameRateLimit,
	VideoQuality,
	ThreeDResolution,
	ViewDistance,
	Shadows,
	AntiAliasing,
	Textures,
	Effects,
	PostProcessing,
	VSync,
	MotionBlur,
	ShowGrass,
	MobileFPSType,
	ShowFPS,
	AllowLowPower,
	AllowVideoPlayback,
	AllowDynamicResolution,
	AllowMultithreadedRendering,
	RenderingAPI,
	UseGPUCrashDebugging,
	RegionHeader,
	Language,
	Region,
	MouseSensitivityYaw,
	MouseSensitivityPitch,
	MouseSensitivityMultiplierForAircraft,
	TouchDragSensitivity,
	ControllerLookSensitivityYaw,
	ControllerLookSensitivityPitch,
	MouseTargetingMultiplier,
	MouseScopedMultiplier,
	GamepadTargetingMultiplier,
	GamepadScopedMultiplier,
	GamepadBuildingMultiplier,
	GamepadEditModeMultiplier,
	TouchLookAccelerationMultiplier,
	TouchDragTargetingSensitivity,
	TouchDragScopedSensitivity,
	TouchBuildingMultiplier,
	TouchEditModeMultiplier,
	TouchVerticalSensitivity,
	InvertPitch,
	InvertYaw,
	InvertPitchForMotion,
	InvertPitchForAircraftPrimary,
	InvertPitchForAircraftSecondary,
	InvertYawForMotion,
	GyroEnabled,
	GyroYawAxis,
	GyroSensitivity,
	GyroTargetingSensitivity,
	GyroScopedSensitivity,
	GyroHarvestingToolSensitivity,
	SafeZone,
	AnonymousMode,
	AnonymousCharacterMode,
	HideOtherPlayerNames,
	HiddenMatchmakingDelay,
	AutoJoinGameVoiceChannel,
	ShowVoiceIndicators,
	HUDScale,
	ShowViewerCount,
	FirstPersonCamera,
	PeripheralLighting,
	PingPlaceDangerMarkerWhenTargeting,
	ShowGlobalChat,
	ConsoleUnlockedFPS,
	ToggleSprint,
	SprintByDefault,
	SprintCancelsReload,
	TapInteract,
	InWorldInteract,
	ToggleTargeting,
	HoldToSwapPickup,
	AutoEquipBetterItems,
	EquipFirstBuildingPieceWhenSwappingQuickbars,
	EquipFirstBuildingPieceWhenSwappingQuickbarsAthena,
	DisablePreEditsWhenPlacingBuilding,
	AimAssist,
	EditModeAimAssist,
	TouchEdit,
	EditConfirmOnRelease,
	QuickEdit,
	TurboBuild,
	CreativeTurboDelete,
	AutoChangeMaterial,
	GamepadAutoRun,
	CrossplayPreference,
	AutoOpenDoors,
	AutoPickupWeapons,
	AutoPickupWeaponsConsolePC,
	AutoSortConsumablesToRight,
	EnableTryBuildOnFocus,
	EditButtonHoldTime,
	AccessoriesHeader,
	ForceFeedback,
	ContextTutorial,
	ReplayRecording,
	ReplayRecordingLargeTeams,
	ReplayRecordingCreativeMode,
	UsePowerSavingMode,
	ShadowPlayHighlights,
	ShowTemperature,
	BuildingPossession,
	GammaValue,
	MusicVolume,
	SoundFXVolume,
	DialogVolume,
	VoiceChatVolume,
	CinematicsVolume,
	Subtitles,
	Quality,
	VoiceChat,
	PushToTalk,
	ProximityVoiceChat,
	VoiceChatInputDevice,
	VoiceChatOutputDevice,
	AllowBackgroundAudio,
	ColorBlindMode,
	ColorBlindStrength,
	IgnoreGamepadInput,
	LockPrimaryInputMethodToMouse,
	EnableRudderControl,
	RudderDeadZone,
	RudderMaxThrottle,
	VisualizeSoundEffects,
	VisualizeSoundEffectsHeader,
	MoveStickDeadZone,
	LookStickDeadZone,
	LookSensitivityPreset,
	LookSensitivityPresetAds,
	LookBuildModeMultiplier,
	LookEditModeMultiplier,
	UseAdvancedOptions,
	LookHorizontalSpeed,
	LookVerticalSpeed,
	LookHorizontalSpeedAds,
	LookVerticalSpeedAds,
	LookHorizontalBoostSpeed,
	LookVerticalBoostSpeed,
	LookBoostAccelerationTime,
	LookHorizontalBoostSpeedAds,
	LookVerticalBoostSpeedAds,
	LookBoostAccelerationTimeAds,
	InstantBoostWhenBuilding,
	LookEaseTime,
	LookInputCurve,
	AimAssistStrength,
	UseLegacyControls,
	PlayerSurveysAllowed,
	NotifyWhenPlaying,
	LocalNotifications,
	FireMode,
	COUNT,
	ESettingType_MAX
};

enum EParentalControlsViewState
{
	Invalid,
	EnterPin,
	AskToEnableControls,
	VerifyEmail,
	SetupEmail,
	SetupPin,
	DisplaySettings,
	DisableParentalControls,
	AskToReEnable,
	ReEnabling,
	EParentalControlsViewState_MAX
};

enum EFortPerksWidgetState
{
	Normal,
	Upgrade,
	Evolution,
	EFortPerksWidgetState_MAX
};

enum ESaveProfileForBanners
{
	SaveTheWorld,
	BattleRoyale,
	ESaveProfileForBanners_MAX
};

enum EFortPlayerSurveyAggregateOp
{
	Sum,
	Max
};

enum EFortPlayerSurveyAthenaSeasonStat
{
	XP,
	Level,
	BookXP,
	BookLevel,
	EFortPlayerSurveyAthenaSeasonStat_MAX
};

enum EFortPlayerSurveyButtonListMultipleSelectionAnswerWidgetFocusType
{
	Unknown,
	ChoiceButton,
	EFortPlayerSurveyButtonListMultipleSelectionAnswerWidgetFocusType_MAX
};

enum EFortPlayerSurveyCMSDataAggregateOp
{
	s,
	mx,
	EFortPlayerSurveyCMSDataAggregateOp_MAX
};

enum EFortPlayerSurveyCMSDataAthenaSeasonStat
{
	sx,
	sl,
	bx,
	bl,
	EFortPlayerSurveyCMSDataAthenaSeasonStat_MAX
};

enum EFortPlayerSurveyCMSDataBinaryComparisonOp
{
	e,
	n,
	l,
	g,
	le,
	ge,
	EFortPlayerSurveyCMSDataBinaryComparisonOp_MAX
};

enum EFortPlayerSurveyCMSDataGameMode
{
	c,
	a,
	pc,
	EFortPlayerSurveyCMSDataGameMode_MAX
};

enum EFortPlayerSurveyCMSDataGameplayTagQueryExprType
{
	n,
	s,
	a,
	EFortPlayerSurveyCMSDataGameplayTagQueryExprType_MAX
};

enum EFortPlayerSurveyCMSDataPlaylistCategory
{
	a,
	s,
	d,
	q,
	lt,
	c,
	pl,
	EFortPlayerSurveyCMSDataPlaylistCategory_MAX
};

enum EFortPlayerSurveyCMSDataPresentationStyle
{
	standard,
	rating,
	EFortPlayerSurveyCMSDataPresentationStyle_MAX
};

enum EFortPlayerSurveyCMSDataQuestState
{
	i,
	a,
	co,
	cl,
	EFortPlayerSurveyCMSDataQuestState_MAX
};

enum EFortPlayerSurveyCMSDataRelativeSurveyKeyType
{
	s,
	a,
	o,
	EFortPlayerSurveyCMSDataRelativeSurveyKeyType_MAX
};

enum EFortPlayerSurveyCMSDataTrigger
{
	rm,
	EFortPlayerSurveyCMSDataTrigger_MAX
};

enum EFortPlayerSurveyPlaylistCategory
{
	All,
	Solo,
	Duo,
	Squad,
	LTM,
	Creative,
	Playground,
	EFortPlayerSurveyPlaylistCategory_MAX
};

enum EFortPlayerSurveyQuestionType
{
	SingleChoice,
	MultipleChoice,
	Rating,
	EFortPlayerSurveyQuestionType_MAX
};

enum EFortPlayerSurveyResponseChoiceType
{
	CheckBox,
	Radio,
	EFortPlayerSurveyResponseChoiceType_MAX
};

enum EFortPlayerSurveyTrigger
{
	Invalid,
	Any,
	ReturnToMainMenu,
	EFortPlayerSurveyTrigger_MAX
};

enum EPostGameScreenContinueBehavior
{
	Next,
	Previous,
	EPostGameScreenContinueBehavior_MAX
};

enum EPurchaseReturnStep
{
	ItemSelection,
	ReasonSelection,
	FinalSubmission,
	EPurchaseReturnStep_MAX
};

enum ECalloutNavigationDirection
{
	Previous,
	Next,
	ECalloutNavigationDirection_MAX
};

enum EFortRadialControllingStick
{
	Right,
	Left,
	EFortRadialControllingStick_MAX
};

enum ERedeemCodeFailureReason
{
	InvalidCode,
	CodeAlreadyUsed,
	AlreadyHasAccess,
	FailedToGetItem,
	Unknown,
	ERedeemCodeFailureReason_MAX
};

enum EFortRewardItemType
{
	RewardedBadges,
	MissedBadges,
	RewardedItems,
	RewardedAccountItems,
	EFortRewardItemType_MAX
};

enum EFortServerBrowserAction
{
	BattleLabServerCreate,
	PlaygroundServerCreate,
	CreativeServerCreate,
	Play,
	CreativeIslandCode,
	CreativeDiscovery,
	EFortServerBrowserAction_MAX
};

enum EFortServerItemIneligibleReason
{
	None,
	PartyTooBig,
	PartyTooSmall,
	NotPartyLeader,
	MatchmakingAlready,
	NotSupportedByLeto,
	InvalidData,
	EFortServerItemIneligibleReason_MAX
};

enum EFortSettingGameVisibility
{
	None,
	Campaign,
	Athena,
	EFortSettingGameVisibility_MAX
};

enum EFortShowdownPinState
{
	None,
	Locked,
	Unlocked,
	EFortShowdownPinState_MAX
};

enum EFortEventWindowEligibility
{
	Unknown,
	Public,
	Private,
	Locked,
	EFortEventWindowEligibility_MAX
};

enum EFortShowdownEventState
{
	Unknown,
	FutureTBD,
	FutureScheduled,
	FutureNext,
	Live,
	LiveParticipating,
	LiveNotParticipating,
	Completed,
	CompletedParticipated,
	CompletedNotPartipated,
	Cancelled,
	EFortShowdownEventState_MAX
};

enum EFortShowdownMatchType
{
	Unknown,
	Solo,
	Duos,
	Squads,
	EFortShowdownMatchType_MAX
};

enum EFortDateTimeStyle
{
	Default,
	Short,
	Medium,
	Long,
	Full,
	EFortDateTimeStyle_MAX
};

enum ERadialOrderingMode
{
	CounterClockwise,
	Clockwise,
	Cardinal,
	Custom,
	ERadialOrderingMode_MAX
};

enum ECardinalPoint
{
	E,
	NE,
	N,
	NW,
	W,
	SW,
	S,
	SE,
	None,
	ECardinalPoint_MAX
};

enum ESocialImportPanelType
{
	Athena,
	SaveTheWorld,
	ESocialImportPanelType_MAX
};

enum EFriendLinkShareButtonType
{
	IOS,
	Android,
	Generic,
	EFriendLinkShareButtonType_MAX
};

enum EListHeaderType
{
	TeamMember,
	PartyMember,
	JoinableParty,
	PlatformOnlineFriend,
	McpOnlineFriend,
	OfflineFriend,
	Blocked,
	FriendInvite,
	SuggestedFriend,
	RecentPlayer,
	SearchResults,
	JoinRequests,
	GameVoice,
	PartyVoice,
	PlatformVoice,
	SidekickVoice,
	Invalid,
	EListHeaderType_MAX
};

enum EFortSquadSlottingRestrictionReason
{
	ItemIsInInventoryOverflow,
	MandatorySlotWouldBeEmptied,
	ItemIsOnActiveExpedition,
	HeroRequiresMissingGameplayTag,
	HeroAlreadyEquippedInLoadout,
	EFortSquadSlottingRestrictionReason_MAX
};

enum EPauseType
{
	NoPause,
	Rare,
	New,
	NewAndRare,
	EPauseType_MAX
};

enum ECardPackPurchaseError
{
	PendingServerConfirmation,
	CannotAffordItem,
	NoneLeft,
	PurchaseAlreadyPending,
	NoConnection,
	ECardPackPurchaseError_MAX
};

enum ESubgameTileType
{
	Campaign,
	Athena,
	Creative,
	ESubgameTileType_MAX
};

enum ESubgameLoadFromCMS
{
	DoNotLoadFromCMS,
	LoadImageFromCMS,
	ESubgameLoadFromCMS_MAX
};

enum ESubscriptionCancellability
{
	NotCancellable,
	CancellabeOnPlatform,
	CancellableAnywhere,
	ESubscriptionCancellability_MAX
};

enum EFortUISurvivorSquadMatchType
{
	Multi,
	Single,
	Summary,
	Max_None,
	EFortUISurvivorSquadMatchType_MAX
};

enum ETouchState
{
	None,
	WaitingForStart,
	Started,
	Active,
	COUNT,
	ETouchState_MAX
};

enum EFortControlType
{
	None,
	CameraAndMovement,
	Picking,
	COUNT,
	EFortControlType_MAX
};

enum EFortTouchControlRegion
{
	MovePlayer,
	RotateCamera,
	NoRegion,
	COUNT,
	EFortTouchControlRegion_MAX
};

enum ETournmentPosterViolatorState
{
	Hidden,
	Live,
	Countdown,
	Info,
	ETournmentPosterViolatorState_MAX
};

enum EFortNamedBundle
{
	Menu,
	MenuCampaign,
	MenuAthena,
	Zone,
	ZoneCampaign,
	ZoneAthena,
	Client,
	ClientCampaign,
	ClientAthena,
	EFortNamedBundle_MAX
};

enum EFortReturnToFrontendBehavior
{
	NotSpecified,
	HomeScreen,
	MapScreen,
	MapScreenWithAutoLaunch,
	MapScreenWithMinimalAutoLaunch,
	EFortReturnToFrontendBehavior_MAX
};

enum ELetoDisplayMode
{
	PrimaryOnly,
	SingleToggle,
	Simultaneous,
	ELetoDisplayMode_MAX
};

enum EFortUINavigationOp
{
	PopContentStack,
	FeatureSwitch,
	NavigateToSkillTree,
	NavigateToSquadSlot,
	NavigateToQuest,
	NavigateToItemManagement,
	NavigateToExpeditions,
	NavigateToCollectionBook,
	None,
	EFortUINavigationOp_MAX
};

enum EFortLoginStage
{
	Begin,
	SplashScreen,
	UpdateCheck,
	SignIn,
	PostSignin,
	SafeZoneEditor,
	Benchmark,
	RejoinCheck,
	LoadingAthenaProfile,
	HealthWarning,
	WaitingForDynamicContent,
	Complete,
	EFortLoginStage_MAX
};

enum EFortLoginDisplay
{
	LoginStatus,
	SplashScreen,
	SignIn,
	SafeZoneEditor,
	HealthWarning,
	EFortLoginDisplay_MAX
};

enum EFortLoginAccountType
{
	None,
	EpicAccount,
	Facebook,
	Google,
	PSN,
	XBLive,
	Erebus,
	EFortLoginAccountType_MAX
};

enum EItemShopNavigationAction
{
	None,
	ShowOfferDetails,
	NavigateToOffer,
	EItemShopNavigationAction_MAX
};

enum ESubscriptionContentTab
{
	SubscriptionManagementScreen,
	ProgressiveItemScreen,
	ESubscriptionContentTab_MAX
};

enum EPlayerReportReasons
{
	None,
	Communication,
	Offensive,
	AFK,
	IgnoringObjective,
	Harassment,
	Exploiting,
	TradeScam,
	CommunicationsAbuse,
	OffensiveName,
	TeamingUpWithEnemies,
	InappropriateContent,
	ExploitingOrHacking,
	Harassment_Threatening,
	Harassment_Annoying,
	Harassment_Selling,
	Harassment_Verbal,
	Harassment_GameBehavior,
	EPlayerReportReasons_MAX
};

enum EFortComparisonType
{
	None,
	HigherValue,
	LowerValue,
	Upgrade,
	EFortComparisonType_MAX
};

enum EFortClampState
{
	NoClamp,
	MinClamp,
	MaxClamp,
	EFortClampState_MAX
};

enum EFortBuffState
{
	NoChange,
	Better,
	Worse,
	EFortBuffState_MAX
};

enum EFortStatValueDisplayType
{
	BasicPaired,
	BasicSingle,
	Unique,
	ElementalFire,
	ElementalIce,
	ElementalElectric,
	EFortStatValueDisplayType_MAX
};

enum EFortAnimSpeed
{
	Instant,
	Fast,
	Normal,
	EFortAnimSpeed_MAX
};

enum EFortSocialPanelTab
{
	PartyInvitations,
	Friends,
	RecentPlayers,
	Max
};

enum EFortSocialPanelType
{
	Join,
	Invite,
	Max
};

enum EModalContainerSize
{
	ExtraSmall,
	Small,
	Medium,
	Large,
	Max
};

enum ENotificationType
{
	Basic,
	Friends,
	ENotificationType_MAX
};

enum ENotificationResult
{
	Confirmed,
	Declined,
	Unknown,
	ENotificationResult_MAX
};

enum EFortInventoryContext
{
	Game,
	Lobby,
	FrontEnd,
	Pickup,
	EFortInventoryContext_MAX
};

enum EFortToastType
{
	Default,
	Subdued,
	Impactful,
	EFortToastType_MAX
};

enum EUpgradeInfoImageSize
{
	Small,
	Large,
	EUpgradeInfoImageSize_MAX
};

enum EChannelSpeakerStyle
{
	InGame,
	InLobby,
	OutOfClient,
	Max
};

enum EHeistExitCraftIconState
{
	None,
	Incoming,
	Spawned,
	Exited,
	EHeistExitCraftIconState_MAX
};

enum EHeistBlingIconState
{
	None,
	SupplyDrop,
	PickupItem,
	CarriedEnemy,
	CarriedAlly,
	EHeistBlingIconState_MAX
};

enum EHeistExitCraftUIState
{
	None,
	OnTheWay,
	Incoming,
	Arrived,
	EHeistExitCraftUIState_MAX
};

enum ENumericalIndicatorActionType
{
	Adding,
	Removing,
	Nothing,
	ENumericalIndicatorActionType_MAX
};

enum ELinkAcrossSimpleAction
{
	AddToAll,
	RemovedFromAll,
	Custom,
	Nothing,
	ELinkAcrossSimpleAction_MAX
};

enum EHUDLayoutToolPopupType
{
	Warning,
	Generic,
	EHUDLayoutToolPopupType_MAX
};

enum EHUDLayoutToolTextInputPopupType
{
	Rename,
	ImportLayoutID,
	EHUDLayoutToolTextInputPopupType_MAX
};

enum EHUDLayoutToolToasterType
{
	Success,
	Failure,
	LocalFailure,
	CloudFailure,
	EHUDLayoutToolToasterType_MAX
};

enum ELiveStreamStandaloneBlocked
{
	StreamInWorldActive,
	ELiveStreamStandaloneBlocked_MAX
};

enum EMatchmakingInputSource
{
	Local,
	Remote,
	Pool,
	EMatchmakingInputSource_MAX
};

enum ESpectatorLeaderboardEntryType
{
	ScoreWithEndScore,
	NoEndScore,
	Time,
	Invalid,
	ESpectatorLeaderboardEntryType_MAX
};

enum EMinigameCaptureObjectiveIconState
{
	NotCaptured,
	Captured,
	EMinigameCaptureObjectiveIconState_MAX
};

enum EFortPlayerFeedbackFlags
{
	SubscreenFlow_ForceDisplayFreeText,
	SubscreenFlow_IncludeScreenshotSubscreen,
	DoNotDisplay_SaveTheWorld,
	DoNotDisplay_Athena,
	DoNotDisplay_Creative,
	Submit_TryDisplayBlockUser,
	Submit_TryDisplayCommunityRulesURL,
	Filter_AddSidekick,
	Filter_OnlySidekick,
	EFortPlayerFeedbackFlags_MAX
};

enum EPlayerFeedback_EpicQAState
{
	SignInPage,
	SigningInFailed,
	SigningIn,
	SelectBugComponent,
	EPlayerFeedback_MAX
};

enum EPlayerFeedbackSubmitState
{
	Start,
	Submitting,
	SubmitFailed,
	SubmitSucceeded,
	EPlayerFeedbackSubmitState_MAX
};

enum EFortPrioritizedWidgetInterruptedBehavior
{
	RemainInQueue,
	Drop,
	EFortPrioritizedWidgetInterruptedBehavior_MAX
};

enum EFortPrioritizedWidgetContestedBehavior
{
	QueueBehind,
	StompOther,
	EFortPrioritizedWidgetContestedBehavior_MAX
};

enum EFortPrioritizedWidgetPriority
{
	Priority_0,
	Priority_1,
	Priority_2,
	Priority_3,
	Priority_4,
	Priority_5,
	NumberOfPrios
};

enum ECooldownTrackingType
{
	Cue,
	AbilityCooldownTags,
	COUNT,
	ECooldownTrackingType_MAX
};

enum ESidekickVoiceChatVideoContentType
{
	Default,
	Screencast,
	ESidekickVoiceChatVideoContentType_MAX
};

enum ESidekickVoiceChatVideoStatus
{
	Enabled,
	Disabled,
	NetworkDisabled,
	BackgroundDisabled,
	ESidekickVoiceChatVideoStatus_MAX
};

enum ESidekickVoiceChatAudioStatus
{
	Enabled,
	Disabled,
	ESidekickVoiceChatAudioStatus_MAX
};

enum ESpatialCustomizationRoomState
{
	None,
	CategorySelection,
	CustomizationSelection,
	ESpatialCustomizationRoomState_MAX
};

enum ESpatialCustomizationCategoryState
{
	LockedByBattlePass,
	LockedByChallenge,
	LockedByChallengeCompletion,
	UnlockAvailable,
	UnlockUsed,
	Max_NONE,
	ESpatialCustomizationCategoryState_MAX
};

enum ESpatialStyleCharacterUnlockPrerequisite
{
	BattlepassPurchase,
	BattlepassLevel,
	ESpatialStyleCharacterUnlockPrerequisite_MAX
};

enum ESubscriptionDisclaimerState
{
	None,
	Asterisk,
	Received,
	ESubscriptionDisclaimerState_MAX
};

enum ETDMScoreProgressTypes
{
	None,
	ProgressSoundMild,
	ProgressSoundMedium,
	ProgressSoundStrong,
	CountdownSoundNormal,
	CountdownSoundStrong,
	ETDMScoreProgressTypes_MAX
};

enum EFortEarnedSubRewardType
{
	XP,
	Bars,
	EFortEarnedSubRewardType_MAX
};

enum EFriendChestResult
{
	Ignored,
	Partial,
	Abandoned,
	Opened,
	SpawnFailure,
	EFriendChestResult_MAX
};

enum EGameplayEffectGrantedAbilityRemovePolicy
{
	CancelAbilityImmediately,
	RemoveAbilityOnEnd,
	DoNothing,
	EGameplayEffectGrantedAbilityRemovePolicy_MAX
};

enum EGameplayEffectAttributeCaptureSource
{
	Source,
	Target,
	EGameplayEffectAttributeCaptureSource_MAX
};

enum EGameplayAbilityActivationMode
{
	Authority,
	NonAuthority,
	Predicting,
	Confirmed,
	Rejected,
	EGameplayAbilityActivationMode_MAX
};

enum EAbilityGenericReplicatedEvent
{
	GenericConfirm,
	GenericCancel,
	InputPressed,
	InputReleased,
	GenericSignalFromClient,
	GenericSignalFromServer,
	GameCustom1,
	GameCustom2,
	GameCustom3,
	GameCustom4,
	GameCustom5,
	GameCustom6,
	MAX
};

enum EGameplayCueEvent
{
	OnActive,
	WhileActive,
	Executed,
	Removed,
	EGameplayCueEvent_MAX
};

enum EGameplayEffectReplicationMode
{
	Minimal,
	Mixed,
	Full,
	EGameplayEffectReplicationMode_MAX
};

enum EAbilityTaskWaitState
{
	WaitingOnGame,
	WaitingOnUser,
	WaitingOnAvatar,
	EAbilityTaskWaitState_MAX
};

enum ERootMotionMoveToActorTargetOffsetType
{
	AlignFromTargetToSource,
	AlignToTargetForward,
	AlignToWorldSpace,
	ERootMotionMoveToActorTargetOffsetType_MAX
};

enum EAbilityTaskNetSyncType
{
	BothWait,
	OnlyServerWait,
	OnlyClientWait,
	EAbilityTaskNetSyncType_MAX
};

enum EWaitAttributeChangeComparison
{
	None,
	GreaterThan,
	LessThan,
	GreaterThanOrEqualTo,
	LessThanOrEqualTo,
	NotEqualTo,
	ExactlyEqualTo,
	MAX
};

enum EGameplayAbilityInputBinds
{
	Ability1,
	Ability2,
	Ability3,
	Ability4,
	Ability5,
	Ability6,
	Ability7,
	Ability8,
	Ability9,
	EGameplayAbilityInputBinds_MAX
};

enum ETargetDataFilterSelf
{
	TDFS_Any,
	TDFS_NoSelf,
	TDFS_NoOthers,
	TDFS_MAX
};

enum EGameplayAbilityTargetingLocationType
{
	LiteralTransform,
	ActorTransform,
	SocketTransform,
	EGameplayAbilityTargetingLocationType_MAX
};

enum EGameplayTargetingConfirmation
{
	Instant,
	UserConfirmed,
	Custom,
	CustomMulti,
	EGameplayTargetingConfirmation_MAX
};

enum ERepAnimPositionMethod
{
	Position,
	CurrentSectionId,
	ERepAnimPositionMethod_MAX
};

enum EGameplayAbilityTriggerSource
{
	GameplayEvent,
	OwnedTagAdded,
	OwnedTagPresent,
	EGameplayAbilityTriggerSource_MAX
};

enum EGameplayAbilityReplicationPolicy
{
	ReplicateNo,
	ReplicateYes,
	EGameplayAbilityReplicationPolicy_MAX
};

enum EGameplayAbilityNetSecurityPolicy
{
	ClientOrServer,
	ServerOnlyExecution,
	ServerOnlyTermination,
	ServerOnly,
	EGameplayAbilityNetSecurityPolicy_MAX
};

enum EGameplayAbilityNetExecutionPolicy
{
	LocalPredicted,
	LocalOnly,
	ServerInitiated,
	ServerOnly,
	EGameplayAbilityNetExecutionPolicy_MAX
};

enum EGameplayAbilityInstancingPolicy
{
	NonInstanced,
	InstancedPerActor,
	InstancedPerExecution,
	EGameplayAbilityInstancingPolicy_MAX
};

enum EGameplayCuePayloadType
{
	CueParameters,
	FromSpec,
	EGameplayCuePayloadType_MAX
};

enum EGameplayEffectPeriodInhibitionRemovedPolicy
{
	NeverReset,
	ResetPeriod,
	ExecuteAndResetPeriod,
	EGameplayEffectPeriodInhibitionRemovedPolicy_MAX
};

enum EGameplayEffectStackingExpirationPolicy
{
	ClearEntireStack,
	RemoveSingleStackAndRefreshDuration,
	RefreshDuration,
	EGameplayEffectStackingExpirationPolicy_MAX
};

enum EGameplayEffectStackingPeriodPolicy
{
	ResetOnSuccessfulApplication,
	NeverReset,
	EGameplayEffectStackingPeriodPolicy_MAX
};

enum EGameplayEffectStackingDurationPolicy
{
	RefreshOnSuccessfulApplication,
	NeverRefresh,
	EGameplayEffectStackingDurationPolicy_MAX
};

enum EGameplayEffectDurationType
{
	Instant,
	Infinite,
	HasDuration,
	EGameplayEffectDurationType_MAX
};

enum EGameplayEffectScopedModifierAggregatorType
{
	CapturedAttributeBacked,
	Transient,
	EGameplayEffectScopedModifierAggregatorType_MAX
};

enum EAttributeBasedFloatCalculationType
{
	AttributeMagnitude,
	AttributeBaseValue,
	AttributeBonusMagnitude,
	AttributeMagnitudeEvaluatedUpToChannel,
	EAttributeBasedFloatCalculationType_MAX
};

enum EGameplayEffectMagnitudeCalculation
{
	ScalableFloat,
	AttributeBased,
	CustomCalculationClass,
	SetByCaller,
	EGameplayEffectMagnitudeCalculation_MAX
};

enum EGameplayTagEventType
{
	NewOrRemoved,
	AnyCountChange,
	EGameplayTagEventType_MAX
};

enum EGameplayEffectStackingType
{
	None,
	AggregateBySource,
	AggregateByTarget,
	EGameplayEffectStackingType_MAX
};

enum EGameplayModOp
{
	Additive,
	Multiplicitive,
	Division,
	Override,
	Max
};

enum EGameplayModEvaluationChannel
{
	Channel0,
	Channel1,
	Channel2,
	Channel3,
	Channel4,
	Channel5,
	Channel6,
	Channel7,
	Channel8,
	Channel9,
	Channel_MAX,
	EGameplayModEvaluationChannel_MAX
};

enum EGameplayBehaviorInstantiationPolicy
{
	Instantiate,
	ConditionallyInstantiate,
	DontInstantiate,
	EGameplayBehaviorInstantiationPolicy_MAX
};

enum EInitialOscillatorOffset
{
	EOO_OffsetRandom,
	EOO_OffsetZero,
	EOO_MAX
};

enum EOscillatorWaveform
{
	SineWave,
	PerlinNoise,
	EOscillatorWaveform_MAX
};

enum EInitialWaveOscillatorOffsetType
{
	Random,
	Zero,
	EInitialWaveOscillatorOffsetType_MAX
};

enum EGameplayMessageMatchType
{
	ExactMatch,
	PartialMatch,
	EGameplayMessageMatchType_MAX
};

enum EGameplayTagQueryExprType
{
	Undefined,
	AnyTagsMatch,
	AllTagsMatch,
	NoTagsMatch,
	AnyExprMatch,
	AllExprMatch,
	NoExprMatch,
	EGameplayTagQueryExprType_MAX
};

enum EGameplayContainerMatchType
{
	Any,
	All,
	EGameplayContainerMatchType_MAX
};

enum EGameplayTagMatchType
{
	Explicit,
	IncludeParentTags,
	EGameplayTagMatchType_MAX
};

enum EGameplayTagSelectionType
{
	None,
	NonRestrictedOnly,
	RestrictedOnly,
	All,
	EGameplayTagSelectionType_MAX
};

enum EGameplayTagSourceType
{
	Native,
	DefaultTagList,
	TagList,
	RestrictedTagList,
	DataTable,
	Invalid,
	EGameplayTagSourceType_MAX
};

enum ETaskResourceOverlapPolicy
{
	StartOnTop,
	StartAtEnd,
	ETaskResourceOverlapPolicy_MAX
};

enum EGameplayTaskRunResult
{
	Error,
	Failed,
	Success_Paused,
	Success_Active,
	Success_Finished,
	EGameplayTaskRunResult_MAX
};

enum EGameplayTaskState
{
	Uninitialized,
	AwaitingActivation,
	Paused,
	Active,
	Finished,
	EGameplayTaskState_MAX
};

enum ECatalogRequirementType
{
	RequireFulfillment,
	DenyOnFulfillment,
	RequireItemOwnership,
	DenyOnItemOwnership,
	ECatalogRequirementType_MAX
};

enum ECatalogOfferType
{
	StaticPrice,
	DynamicBundle,
	ECatalogOfferType_MAX
};

enum ECatalogSaleType
{
	NotOnSale,
	UndecoratedNewPrice,
	AmountOff,
	PercentOff,
	PercentOn,
	Strikethrough,
	MAX
};

enum EAppStore
{
	DebugStore,
	EpicPurchasingService,
	IOSAppStore,
	WeGameStore,
	GooglePlayAppStore,
	KindleStore,
	PlayStation4Store,
	XboxLiveStore,
	NintendoEShop,
	SamsungGalaxyAppStore,
	MicrosoftStore,
	PlayStation5Store,
	MAX
};

enum EStoreCurrencyType
{
	RealMoney,
	MtxCurrency,
	GameItem,
	Other,
	MAX
};

enum EChaosBreakingSortMethod
{
	SortNone,
	SortByHighestMass,
	SortByHighestSpeed,
	SortByNearestFirst,
	Count,
	EChaosBreakingSortMethod_MAX
};

enum EChaosCollisionSortMethod
{
	SortNone,
	SortByHighestMass,
	SortByHighestSpeed,
	SortByHighestImpulse,
	SortByNearestFirst,
	Count,
	EChaosCollisionSortMethod_MAX
};

enum EChaosTrailingSortMethod
{
	SortNone,
	SortByHighestMass,
	SortByHighestSpeed,
	SortByNearestFirst,
	Count,
	EChaosTrailingSortMethod_MAX
};

enum EGeometryCollectionDebugDrawActorHideGeometry
{
	HideNone,
	HideWithCollision,
	HideSelected,
	HideWholeCollection,
	HideAll,
	EGeometryCollectionDebugDrawActorHideGeometry_MAX
};

enum ECollectionGroupEnum
{
	Chaos_Traansform,
	Chaos_Max
};

enum ECollectionAttributeEnum
{
	Chaos_Active,
	Chaos_DynamicState,
	Chaos_CollisionGroup,
	Chaos_Max
};

enum EGiftingPresentationMode
{
	ConfirmPurchase,
	WrapOptions,
	GiftingProcess,
	GiftingError,
	GiftConfirmed,
	None,
	EGiftingPresentationMode_MAX
};

enum EGiftingPricePresentationMode
{
	MtxCurrency,
	RealMoney,
	Hidden,
	EGiftingPricePresentationMode_MAX
};

enum EGiftingScreenPresentationMode
{
	ItemList,
	NoContent,
	EGiftingScreenPresentationMode_MAX
};

enum EFilterType
{
	All,
	Party,
	EFilterType_MAX
};

enum ERecipientPresentationMode
{
	Loading,
	GiftPrice,
	AlreadyOwned,
	Unavailable,
	ERecipientPresentationMode_MAX
};

enum EGooglePADCellularDataConfirmStatus
{
	AssetPack_CONFIRM_UNKNOWN,
	AssetPack_CONFIRM_PENDING,
	AssetPack_CONFIRM_USER_APPROVED,
	AssetPack_CONFIRM_USER_CANCELED,
	AssetPack_CONFIRM_MAX
};

enum EGooglePADStorageMethod
{
	AssetPack_STORAGE_FILES,
	AssetPack_STORAGE_APK,
	AssetPack_STORAGE_UNKNOWN,
	AssetPack_STORAGE_NOT_INSTALLED,
	AssetPack_STORAGE_MAX
};

enum EGooglePADDownloadStatus
{
	AssetPack_UNKNOWN,
	AssetPack_DOWNLOAD_PENDING,
	AssetPack_DOWNLOADING,
	AssetPack_TRANSFERRING,
	AssetPack_DOWNLOAD_COMPLETED,
	AssetPack_DOWNLOAD_FAILED,
	AssetPack_DOWNLOAD_CANCELED,
	AssetPack_WAITING_FOR_WIFI,
	AssetPack_NOT_INSTALLED,
	AssetPack_INFO_PENDING,
	AssetPack_INFO_FAILED,
	AssetPack_REMOVAL_PENDING,
	AssetPack_REMOVAL_FAILED,
	AssetPack_MAX
};

enum EGooglePADErrorCode
{
	AssetPack_NO_ERROR,
	AssetPack_APP_UNAVAILABLE,
	AssetPack_UNAVAILABLE,
	AssetPack_INVALID_REQUEST,
	AssetPack_DOWNLOAD_NOT_FOUND,
	AssetPack_API_NOT_AVAILABLE,
	AssetPack_NETWORK_ERROR,
	AssetPack_ACCESS_DENIED,
	AssetPack_INSUFFICIENT_STORAGE,
	AssetPack_PLAY_STORE_NOT_FOUND,
	AssetPack_NETWORK_UNRESTRICTED,
	AssetPack_INTERNAL_ERROR,
	AssetPack_INITIALIZATION_NEEDED,
	AssetPack_INITIALIZATION_FAILED,
	AssetPack_MAX
};

enum EXRVisualType
{
	Controller,
	Hand,
	EXRVisualType_MAX
};

enum EHandKeypoint
{
	Palm,
	Wrist,
	ThumbMetacarpal,
	ThumbProximal,
	ThumbDistal,
	ThumbTip,
	IndexMetacarpal,
	IndexProximal,
	IndexIntermediate,
	IndexDistal,
	IndexTip,
	MiddleMetacarpal,
	MiddleProximal,
	MiddleIntermediate,
	MiddleDistal,
	MiddleTip,
	RingMetacarpal,
	RingProximal,
	RingIntermediate,
	RingDistal,
	RingTip,
	LittleMetacarpal,
	LittleProximal,
	LittleIntermediate,
	LittleDistal,
	LittleTip,
	EHandKeypoint_MAX
};

enum EXRTrackedDeviceType
{
	HeadMountedDisplay,
	Controller,
	TrackingReference,
	Other,
	Invalid,
	Any,
	EXRTrackedDeviceType_MAX
};

enum ESpectatorScreenMode
{
	Disabled,
	SingleEyeLetterboxed,
	Undistorted,
	Distorted,
	SingleEye,
	SingleEyeCroppedToFill,
	Texture,
	TexturePlusEye,
	ESpectatorScreenMode_MAX
};

enum EXRSystemFlags
{
	NoFlags,
	IsAR,
	IsTablet,
	IsHeadMounted,
	SupportsHandTracking,
	EXRSystemFlags_MAX
};

enum EXRDeviceConnectionResult
{
	NoTrackingSystem,
	FeatureNotSupported,
	NoValidViewport,
	MiscFailure,
	Success,
	EXRDeviceConnectionResult_MAX
};

enum EHMDWornState
{
	Unknown,
	Worn,
	NotWorn,
	EHMDWornState_MAX
};

enum EHMDTrackingOrigin
{
	Floor,
	Eye,
	Stage,
	Unbounded,
	EHMDTrackingOrigin_MAX
};

enum EOrientPositionSelector
{
	Orientation,
	Position,
	OrientationAndPosition,
	EOrientPositionSelector_MAX
};

enum ETrackingStatus
{
	NotTracked,
	InertialOnly,
	Tracked,
	ETrackingStatus_MAX
};

enum ESpatialInputGestureAxis
{
	None,
	Manipulation,
	Navigation,
	NavigationRails,
	ESpatialInputGestureAxis_MAX
};

enum EHoagieBoostState
{
	Unknown,
	Ready,
	Started,
	Finished,
	Failed,
	EHoagieBoostState_MAX
};

enum EHoagieState
{
	STARTUP,
	STARTUP_LIFT,
	FLIGHT,
	AUTO_LANDING,
	SPIN_CRASHING,
	CRASHING_NO_SPIN,
	CRASH_LANDED,
	LANDED,
	EXPLODING,
	NONE,
	EHoagieState_MAX
};

enum EHotfixResult
{
	Failed,
	Success,
	SuccessNoChange,
	SuccessNeedsReload,
	SuccessNeedsRelaunch,
	EHotfixResult_MAX
};

enum EUpdateCompletionStatus
{
	UpdateUnknown,
	UpdateSuccess,
	UpdateSuccess_NoChange,
	UpdateSuccess_NeedsReload,
	UpdateSuccess_NeedsRelaunch,
	UpdateSuccess_NeedsPatch,
	UpdateFailure_PatchCheck,
	UpdateFailure_HotfixCheck,
	UpdateFailure_NotLoggedIn,
	EUpdateCompletionStatus_MAX
};

enum EUpdateState
{
	UpdateIdle,
	UpdatePending,
	CheckingForPatch,
	DetectingPlatformEnvironment,
	CheckingForHotfix,
	WaitingOnInitialLoad,
	InitialLoadComplete,
	UpdateComplete,
	EUpdateState_MAX
};

enum EBitmapCSType
{
	BCST_BLCS_CALIBRATED_RGB,
	BCST_LCS_sRGB,
	BCST_LCS_WINDOWS_COLOR_SPACE,
	BCST_PROFILE_LINKED,
	BCST_PROFILE_EMBEDDED,
	BCST_MAX
};

enum EBitmapHeaderVersion
{
	BHV_BITMAPINFOHEADER,
	BHV_BITMAPV2INFOHEADER,
	BHV_BITMAPV3INFOHEADER,
	BHV_BITMAPV4HEADER,
	BHV_BITMAPV5HEADER,
	BHV_MAX
};

enum EDesiredImageFormat
{
	PNG,
	JPG,
	BMP,
	EXR,
	EDesiredImageFormat_MAX
};

enum ETouchIndex
{
	Touch1,
	Touch2,
	Touch3,
	Touch4,
	Touch5,
	Touch6,
	Touch7,
	Touch8,
	Touch9,
	Touch10,
	CursorPointerIndex,
	MAX_TOUCHES,
	ETouchIndex_MAX
};

enum EControllerHand
{
	Left,
	Right,
	AnyHand,
	Pad,
	ExternalCamera,
	Gun,
	Special_1,
	Special_2,
	Special_3,
	Special_4,
	Special_5,
	Special_6,
	Special_7,
	Special_8,
	Special_9,
	Special_10,
	Special_11,
	ControllerHand_Count,
	EControllerHand_MAX
};

enum ETouchType
{
	Began,
	Moved,
	Stationary,
	ForceChanged,
	FirstMove,
	Ended,
	NumTypes,
	ETouchType_MAX
};

enum EConsoleForGamepadLabels
{
	None,
	XBoxOne,
	PS4,
	EConsoleForGamepadLabels_MAX
};

enum EInputCaptureState
{
	Begin,
	Continue,
	End,
	Ignore,
	EInputCaptureState_MAX
};

enum EInputCaptureRequestType
{
	Begin,
	Ignore,
	EInputCaptureRequestType_MAX
};

enum EInputCaptureSide
{
	None,
	Left,
	Right,
	Both,
	Any,
	EInputCaptureSide_MAX
};

enum EInputDevices
{
	None,
	Keyboard,
	Mouse,
	Gamepad,
	OculusTouch,
	HTCViveWands,
	AnySpatialDevice,
	TabletFingers,
	EInputDevices_MAX
};

enum ETransformGizmoSubElements
{
	None,
	TranslateAxisX,
	TranslateAxisY,
	TranslateAxisZ,
	TranslateAllAxes,
	TranslatePlaneXY,
	TranslatePlaneXZ,
	TranslatePlaneYZ,
	TranslateAllPlanes,
	RotateAxisX,
	RotateAxisY,
	RotateAxisZ,
	RotateAllAxes,
	ScaleAxisX,
	ScaleAxisY,
	ScaleAxisZ,
	ScaleAllAxes,
	ScalePlaneYZ,
	ScalePlaneXZ,
	ScalePlaneXY,
	ScaleAllPlanes,
	ScaleUniform,
	StandardTranslateRotate,
	TranslateRotateUniformScale,
	FullTranslateRotateScale,
	ETransformGizmoSubElements_MAX
};

enum EToolChangeTrackingMode
{
	NoChangeTracking,
	UndoToExit,
	FullUndoRedo,
	EToolChangeTrackingMode_MAX
};

enum EToolSide
{
	Left,
	Mouse,
	Right,
	EToolSide_MAX
};

enum EViewInteractionState
{
	None,
	Hovered,
	Focused,
	EViewInteractionState_MAX
};

enum ESelectedObjectsModificationType
{
	Replace,
	Add,
	Remove,
	Clear,
	ESelectedObjectsModificationType_MAX
};

enum EToolMessageLevel
{
	Internal,
	UserMessage,
	UserNotification,
	UserWarning,
	UserError,
	EToolMessageLevel_MAX
};

enum EToolContextCoordinateSystem
{
	World,
	Local,
	EToolContextCoordinateSystem_MAX
};

enum EStandardToolContextMaterials
{
	VertexColorMaterial,
	EStandardToolContextMaterials_MAX
};

enum ESceneSnapQueryTargetType
{
	None,
	MeshVertex,
	MeshEdge,
	Grid,
	All,
	ESceneSnapQueryTargetType_MAX
};

enum ESceneSnapQueryType
{
	Position,
	Rotation,
	ESceneSnapQueryType_MAX
};

enum EKairosAvatarCaptureState
{
	Unloaded,
	LoadingAssets,
	ReadyToSpawn,
	FinalizingSpawn,
	Displaying,
	EKairosAvatarCaptureState_MAX
};

enum EKairosHeroAnimationType
{
	Unknown,
	Idle,
	Emote,
	EKairosHeroAnimationType_MAX
};

enum EKairosHeroAnimationState
{
	Unknown,
	Idling,
	Emoting,
	Face_Live,
	Face_Playback,
	EKairosHeroAnimationState_MAX
};

enum EKairosHeroSkeletonType
{
	Unknown,
	Male,
	Female,
	EKairosHeroSkeletonType_MAX
};

enum EKairosAppMode
{
	Unknown,
	AvatarCapture,
	AvatarRender,
	EKairosAppMode_MAX
};

enum ELabradorDespawnReason
{
	DeathByPlayer,
	DeathByBot,
	DeathByAIPawn,
	DeathByStorm,
	Teleport,
	Unknown,
	ELabradorDespawnReason_MAX
};

enum ELivingWorldPointProviderSpawnLimiterBehavior
{
	Lifetime,
	Concurrent,
	ELivingWorldPointProviderSpawnLimiterBehavior_MAX
};

enum ELivingWorldCalendarEventConditionRatioBehavior
{
	Less,
	LessOrEqual,
	Greater,
	GreaterOrEqual,
	InBetween,
	ELivingWorldCalendarEventConditionRatioBehavior_MAX
};

enum ELivingWorldCalendarEventConditionBehavior
{
	IsActive,
	Ratio,
	ELivingWorldCalendarEventConditionBehavior_MAX
};

enum EBrushFalloffMode
{
	Angle,
	Width,
	EBrushFalloffMode_MAX
};

enum EBrushBlendType
{
	AlphaBlend,
	Min,
	Max,
	Additive
};

enum ELandscapeBlendMode
{
	LSBM_AdditiveBlend,
	LSBM_AlphaBlend,
	LSBM_MAX
};

enum EWeightmapRTType
{
	WeightmapRT_Scratch_RGBA,
	WeightmapRT_Scratch1,
	WeightmapRT_Scratch2,
	WeightmapRT_Scratch3,
	WeightmapRT_Mip0,
	WeightmapRT_Mip1,
	WeightmapRT_Mip2,
	WeightmapRT_Mip3,
	WeightmapRT_Mip4,
	WeightmapRT_Mip5,
	WeightmapRT_Mip6,
	WeightmapRT_Mip7,
	WeightmapRT_Count,
	WeightmapRT_MAX
};

enum EHeightmapRTType
{
	HeightmapRT_CombinedAtlas,
	HeightmapRT_CombinedNonAtlas,
	HeightmapRT_Scratch1,
	HeightmapRT_Scratch2,
	HeightmapRT_Scratch3,
	HeightmapRT_Mip1,
	HeightmapRT_Mip2,
	HeightmapRT_Mip3,
	HeightmapRT_Mip4,
	HeightmapRT_Mip5,
	HeightmapRT_Mip6,
	HeightmapRT_Mip7,
	HeightmapRT_Count,
	HeightmapRT_MAX
};

enum ERTDrawingType
{
	RTAtlas,
	RTAtlasToNonAtlas,
	RTNonAtlasToAtlas,
	RTNonAtlas,
	RTMips,
	ERTDrawingType_MAX
};

enum ELandscapeSetupErrors
{
	LSE_None,
	LSE_NoLandscapeInfo,
	LSE_CollsionXY,
	LSE_NoLayerInfo,
	LSE_MAX
};

enum ELandscapeClearMode
{
	Clear_Weightmap,
	Clear_Heightmap,
	Clear_All,
	Clear_MAX
};

enum ELandscapeGizmoType
{
	LGT_None,
	LGT_Height,
	LGT_Weight,
	LGT_MAX
};

enum EGrassScaling
{
	Uniform,
	Free,
	LockXY,
	EGrassScaling_MAX
};

enum ESplineModulationColorMask
{
	Red,
	Green,
	Blue,
	Alpha,
	ESplineModulationColorMask_MAX
};

enum ELandscapeLODFalloff
{
	Linear,
	SquareRoot,
	ELandscapeLODFalloff_MAX
};

enum ELandscapeLayerDisplayMode
{
	Default,
	Alphabetical,
	UserSpecific,
	ELandscapeLayerDisplayMode_MAX
};

enum ELandscapeLayerPaintingRestriction
{
	None,
	UseMaxLayers,
	ExistingOnly,
	UseComponentWhitelist,
	ELandscapeLayerPaintingRestriction_MAX
};

enum ELandscapeImportAlphamapType
{
	Additive,
	Layered,
	ELandscapeImportAlphamapType_MAX
};

enum ELandscapeSplineMeshOrientation
{
	LSMO_XUp,
	LSMO_YUp,
	LSMO_MAX
};

enum ELandscapeLayerBlendType
{
	LB_WeightBlend,
	LB_AlphaBlend,
	LB_HeightBlend,
	LB_MAX
};

enum ELandscapeCustomizedCoordType
{
	LCCT_None,
	LCCT_CustomUV0,
	LCCT_CustomUV1,
	LCCT_CustomUV2,
	LCCT_WeightMapUV,
	LCCT_MAX
};

enum ETerrainCoordMappingType
{
	TCMT_Auto,
	TCMT_XY,
	TCMT_XZ,
	TCMT_YZ,
	TCMT_MAX
};

enum ELiveLinkCameraProjectionMode
{
	Perspective,
	Orthographic,
	ELiveLinkCameraProjectionMode_MAX
};

enum ELiveLinkSourceMode
{
	Latest,
	EngineTime,
	Timecode,
	ELiveLinkSourceMode_MAX
};

enum ELiveLinkAxis
{
	X,
	Y,
	Z,
	XNeg,
	YNeg,
	ZNeg,
	ELiveLinkAxis_MAX
};

enum ELiveLinkTimecodeProviderEvaluationType
{
	Lerp,
	Nearest,
	Latest,
	ELiveLinkTimecodeProviderEvaluationType_MAX
};

enum ELiveStreamAnimationRole
{
	Proxy,
	Processor,
	Tracker,
	ELiveStreamAnimationRole_MAX
};

enum ELobbyBeaconJoinState
{
	None,
	SentJoinRequest,
	JoinRequestAcknowledged,
	ELobbyBeaconJoinState_MAX
};

enum ELocalNotificationEventType
{
	None,
	FirstLogin,
	Max
};

enum ELocalNotificationParams
{
	BestHour24ToNotify,
	SpecificFireDateTime,
	HoursSinceLastSession,
	HoursAfterEvent_Max,
	HoursAfterEvent_Min,
	Max
};

enum ELocalNotificationType
{
	SpecificDateTime,
	RangeDateTime,
	Max
};

enum ECreativeMannequinAnalyticsInteractType
{
	Equip,
	OpenStore,
	ECreativeMannequinAnalyticsInteractType_MAX
};

enum EFortMantisBranchPath
{
	Default,
	Path_1,
	Path_2,
	Path_3,
	Path_4,
	Path_5,
	Blocked,
	EFortMantisBranchPath_MAX
};

enum EFortMantisBranchRule
{
	Off,
	Any,
	Starter,
	EFortMantisBranchRule_MAX
};

enum EFortMantisNotifyRotationWarpRateRule
{
	Snap,
	WindowDurationLerp,
	EFortMantisNotifyRotationWarpRateRule_MAX
};

enum EFortMantisNotifyWindow
{
	Invalid,
	Input,
	Execution,
	Damage,
	RootMotionWarp,
	EFortMantisNotifyWindow_MAX
};

enum EFortMantisNotifyEvent
{
	Invalid,
	Branch,
	EFortMantisNotifyEvent_MAX
};

enum EMobileShadowQuality
{
	NoFiltering,
	PCF_1x1,
	PCF_2x2,
	PCF_3x3,
	EMobileShadowQuality_MAX
};

enum EMediaWebcamCaptureDeviceFilter
{
	None,
	DepthSensor,
	Front,
	Rear,
	Unknown,
	EMediaWebcamCaptureDeviceFilter_MAX
};

enum EMediaVideoCaptureDeviceFilter
{
	None,
	Card,
	Software,
	Unknown,
	Webcam,
	EMediaVideoCaptureDeviceFilter_MAX
};

enum EMediaAudioCaptureDeviceFilter
{
	None,
	Card,
	Microphone,
	Software,
	Unknown,
	EMediaAudioCaptureDeviceFilter_MAX
};

enum EMediaPlayerTrack
{
	Audio,
	Caption,
	Metadata,
	Script,
	Subtitle,
	Text,
	Video,
	EMediaPlayerTrack_MAX
};

enum EMediaSoundComponentFFTSize
{
	Min_64,
	Small_256,
	Medium_512,
	Large_1024,
	EMediaSoundComponentFFTSize_MAX
};

enum EMediaSoundChannels
{
	Mono,
	Stereo,
	Surround,
	EMediaSoundChannels_MAX
};

enum EMediaTextureOrientation
{
	MTORI_Original,
	MTORI_CW90,
	MTORI_CW180,
	MTORI_CW270,
	MTORI_MAX
};

enum EMediaTextureOutputFormat
{
	MTOF_Default,
	MTOF_SRGB_LINOUT,
	MTOF_MAX
};

enum EMediaPlayerOptionBooleanOverride
{
	UseMediaPlayerSetting,
	Enabled,
	Disabled,
	EMediaPlayerOptionBooleanOverride_MAX
};

enum EComputeNTBsOptions
{
	None,
	Normals,
	Tangents,
	WeightedNTBs,
	EComputeNTBsOptions_MAX
};

enum EMeshNetworkNodeType
{
	Root,
	Inner,
	Edge,
	Client,
	Unknown,
	EMeshNetworkNodeType_MAX
};

enum EMeshNetworkRelevancy
{
	NotRelevant,
	RelevantToEdgeNodes,
	RelevantToClients,
	EMeshNetworkRelevancy_MAX
};

enum EMIDIEventType
{
	Unknown,
	NoteOff,
	NoteOn,
	NoteAfterTouch,
	ControlChange,
	ProgramChange,
	ChannelAfterTouch,
	PitchBend,
	EMIDIEventType_MAX
};

enum EMoviePlaybackType
{
	MT_Normal,
	MT_Looped,
	MT_LoadingLoop,
	MT_MAX
};

enum EHDRCaptureGamut
{
	HCGM_Rec709,
	HCGM_P3DCI,
	HCGM_Rec2020,
	HCGM_ACES,
	HCGM_ACEScg,
	HCGM_Linear,
	HCGM_MAX
};

enum EMovieSceneCaptureProtocolState
{
	Idle,
	Initialized,
	Capturing,
	Finalizing,
	EMovieSceneCaptureProtocolState_MAX
};

enum EMovieScene3DPathSection_Axis
{
	X,
	Y,
	Z,
	NEG_X,
	NEG_Y,
	NEG_Z,
	MovieScene3DPathSection_MAX
};

enum EFireEventsAtPosition
{
	AtStartOfEvaluation,
	AtEndOfEvaluation,
	AfterSpawn,
	EFireEventsAtPosition_MAX
};

enum ELevelVisibility
{
	Visible,
	Hidden,
	ELevelVisibility_MAX
};

enum EParticleKey
{
	Activate,
	Deactivate,
	Trigger,
	EParticleKey_MAX
};

enum EMovieSceneKeyInterpolation
{
	Auto,
	User,
	Break,
	Linear,
	Constant,
	EMovieSceneKeyInterpolation_MAX
};

enum EMovieSceneBlendType
{
	Invalid,
	Absolute,
	Additive,
	Relative,
	AdditiveFromBase,
	EMovieSceneBlendType_MAX
};

enum EMovieSceneCompletionMode
{
	KeepState,
	RestoreState,
	ProjectDefault,
	EMovieSceneCompletionMode_MAX
};

enum EMovieSceneBuiltInEasing
{
	Linear,
	SinIn,
	SinOut,
	SinInOut,
	QuadIn,
	QuadOut,
	QuadInOut,
	CubicIn,
	CubicOut,
	CubicInOut,
	QuartIn,
	QuartOut,
	QuartInOut,
	QuintIn,
	QuintOut,
	QuintInOut,
	ExpoIn,
	ExpoOut,
	ExpoInOut,
	CircIn,
	CircOut,
	CircInOut,
	EMovieSceneBuiltInEasing_MAX
};

enum EEvaluationMethod
{
	Static,
	Swept,
	EEvaluationMethod_MAX
};

enum EMovieSceneServerClientMask
{
	None,
	Server,
	Client,
	All,
	EMovieSceneServerClientMask_MAX
};

enum EMovieSceneSequenceFlags
{
	None,
	Volatile,
	BlockingEvaluation,
	InheritedFlags,
	EMovieSceneSequenceFlags_MAX
};

enum EUpdateClockSource
{
	Tick,
	Platform,
	Audio,
	RelativeTimecode,
	Timecode,
	Custom,
	EUpdateClockSource_MAX
};

enum EMovieSceneEvaluationType
{
	FrameLocked,
	WithSubFrames,
	EMovieSceneEvaluationType_MAX
};

enum EMovieScenePlayerStatus
{
	Stopped,
	Playing,
	Scrubbing,
	Jumping,
	Stepping,
	Paused,
	MAX
};

enum EMovieSceneObjectBindingSpace
{
	Local,
	Root,
	Unused,
	EMovieSceneObjectBindingSpace_MAX
};

enum ESectionEvaluationFlags
{
	None,
	PreRoll,
	PostRoll,
	ESectionEvaluationFlags_MAX
};

enum EMovieScenePositionType
{
	Frame,
	Time,
	MarkedFrame,
	EMovieScenePositionType_MAX
};

enum EUpdatePositionMethod
{
	Play,
	Jump,
	Scrub,
	EUpdatePositionMethod_MAX
};

enum ESpawnOwnership
{
	InnerSequence,
	MasterSequence,
	External,
	ESpawnOwnership_MAX
};

enum EMeshTrackerVertexColorMode
{
	None,
	Confidence,
	Block,
	EMeshTrackerVertexColorMode_MAX
};

enum ERuntimeGenerationType
{
	Static,
	DynamicModifiersOnly,
	Dynamic,
	LegacyGeneration,
	ERuntimeGenerationType_MAX
};

enum ENavCostDisplay
{
	TotalCost,
	HeuristicOnly,
	RealCostOnly,
	ENavCostDisplay_MAX
};

enum ENavSystemOverridePolicy
{
	Override,
	Append,
	Skip,
	ENavSystemOverridePolicy_MAX
};

enum ERecastPartitioning
{
	Monotone,
	Watershed,
	ChunkyMonotone,
	ERecastPartitioning_MAX
};

enum EFastArraySerializerDeltaFlags
{
	None,
	HasBeenSerialized,
	HasDeltaBeenRequested,
	IsUsingDeltaSerialization,
	EFastArraySerializerDeltaFlags_MAX
};

enum ENetworkLOD
{
	Interpolated,
	SimExtrapolate,
	ForwardPredict,
	All,
	ENetworkLOD_MAX
};

enum ENetworkPredictionTickingPolicy
{
	Independent,
	Fixed,
	All,
	ENetworkPredictionTickingPolicy_MAX
};

enum ENetworkPredictionStateRead
{
	Simulation,
	Presentation,
	ENetworkPredictionStateRead_MAX
};

enum ETractorBeamState
{
	TBS_Inactive,
	TBS_Charging,
	TBS_Active,
	TBS_MAX
};

enum ENevadaFlightStates
{
	FLIGHT,
	CRASHING,
	CRASHED,
	REBOOTING,
	LANDING,
	LANDED,
	IDLE,
	NONE,
	ENevadaFlightStates_MAX
};

enum EFNiagaraCompileEventSeverity
{
	Log,
	Warning,
	Error,
	FNiagaraCompileEventSeverity_MAX
};

enum ENiagaraSystemSpawnSectionEndBehavior
{
	SetSystemInactive,
	Deactivate,
	None,
	ENiagaraSystemSpawnSectionEndBehavior_MAX
};

enum ENiagaraSystemSpawnSectionEvaluateBehavior
{
	ActivateIfInactive,
	None,
	ENiagaraSystemSpawnSectionEvaluateBehavior_MAX
};

enum ENiagaraSystemSpawnSectionStartBehavior
{
	Activate,
	ENiagaraSystemSpawnSectionStartBehavior_MAX
};

enum ENiagaraBakerViewMode
{
	Perspective,
	OrthoFront,
	OrthoBack,
	OrthoLeft,
	OrthoRight,
	OrthoTop,
	OrthoBottom,
	Num,
	ENiagaraBakerViewMode_MAX
};

enum ENiagaraCollisionMode
{
	None,
	SceneGeometry,
	DepthBuffer,
	DistanceField,
	ENiagaraCollisionMode_MAX
};

enum ENiagaraFunctionDebugState
{
	NoDebug,
	Basic,
	ENiagaraFunctionDebugState_MAX
};

enum ENiagaraSystemInstanceState
{
	None,
	PendingSpawn,
	PendingSpawnPaused,
	Spawning,
	Running,
	Paused,
	Num,
	ENiagaraSystemInstanceState_MAX
};

enum ENCPoolMethod
{
	None,
	AutoRelease,
	ManualRelease,
	ManualRelease_OnComplete,
	FreeInPool,
	ENCPoolMethod_MAX
};

enum ENiagaraLegacyTrailWidthMode
{
	FromCentre,
	FromFirst,
	FromSecond,
	ENiagaraLegacyTrailWidthMode_MAX
};

enum ENiagaraRendererSourceDataMode
{
	Particles,
	Emitter,
	ENiagaraRendererSourceDataMode_MAX
};

enum ENiagaraBindingSource
{
	ImplicitFromSource,
	ExplicitParticles,
	ExplicitEmitter,
	ExplicitSystem,
	ExplicitUser,
	MaxBindingSource,
	ENiagaraBindingSource_MAX
};

enum ENiagaraIterationSource
{
	Particles,
	DataInterface,
	ENiagaraIterationSource_MAX
};

enum ENiagaraScriptGroup
{
	Particle,
	Emitter,
	System,
	Max
};

enum ENiagaraScriptContextStaticSwitch
{
	System,
	Emitter,
	Particle,
	ENiagaraScriptContextStaticSwitch_MAX
};

enum ENiagaraCompileUsageStaticSwitch
{
	Spawn,
	Update,
	Event,
	SimulationStage,
	Default,
	ENiagaraCompileUsageStaticSwitch_MAX
};

enum ENiagaraScriptUsage
{
	Function,
	Module,
	DynamicInput,
	ParticleSpawnScript,
	ParticleSpawnScriptInterpolated,
	ParticleUpdateScript,
	ParticleEventScript,
	ParticleSimulationStageScript,
	ParticleGPUComputeScript,
	EmitterSpawnScript,
	EmitterUpdateScript,
	SystemSpawnScript,
	SystemUpdateScript,
	ENiagaraScriptUsage_MAX
};

enum ENiagaraScriptCompileStatus
{
	NCS_Unknown,
	NCS_Dirty,
	NCS_Error,
	NCS_UpToDate,
	NCS_BeingCreated,
	NCS_UpToDateWithWarnings,
	NCS_ComputeUpToDateWithWarnings,
	NCS_MAX
};

enum ENiagaraInputNodeUsage
{
	Undefined,
	Parameter,
	Attribute,
	SystemConstant,
	TranslatorConstant,
	RapidIterationParameter,
	ENiagaraInputNodeUsage_MAX
};

enum ENiagaraDataSetType
{
	ParticleData,
	Shared,
	Event,
	ENiagaraDataSetType_MAX
};

enum ENiagaraStatDisplayMode
{
	Percent,
	Absolute,
	ENiagaraStatDisplayMode_MAX
};

enum ENiagaraStatEvaluationType
{
	Average,
	Maximum,
	ENiagaraStatEvaluationType_MAX
};

enum ENiagaraAgeUpdateMode
{
	TickDeltaTime,
	DesiredAge,
	DesiredAgeNoSeek,
	ENiagaraAgeUpdateMode_MAX
};

enum ENiagaraSimTarget
{
	CPUSim,
	GPUComputeSim,
	ENiagaraSimTarget_MAX
};

enum ENiagaraRendererMotionVectorSetting
{
	AutoDetect,
	Precise,
	Approximate,
	Disable,
	ENiagaraRendererMotionVectorSetting_MAX
};

enum ENiagaraDefaultRendererMotionVectorSetting
{
	Precise,
	Approximate,
	ENiagaraDefaultRendererMotionVectorSetting_MAX
};

enum ENiagaraDefaultMode
{
	Value,
	Binding,
	Custom,
	FailIfPreviouslyNotSet,
	ENiagaraDefaultMode_MAX
};

enum ENiagaraMipMapGeneration
{
	Disabled,
	PostStage,
	PostSimulate,
	ENiagaraMipMapGeneration_MAX
};

enum ENiagaraGpuBufferFormat
{
	Float,
	HalfFloat,
	UnsignedNormalizedByte,
	Max
};

enum ENiagaraTickBehavior
{
	UsePrereqs,
	UseComponentTickGroup,
	ForceTickFirst,
	ForceTickLast,
	ENiagaraTickBehavior_MAX
};

enum ENDIExport_GPUAllocationMode
{
	FixedSize,
	PerParticle,
	ENDIExport_MAX
};

enum ENDILandscape_SourceMode
{
	Default,
	Source,
	AttachParent,
	ENDILandscape_MAX
};

enum ESetResolutionMethod
{
	Independent,
	MaxAxis,
	CellSize,
	ESetResolutionMethod_MAX
};

enum ENDISkeletalMesh_SkinningMode
{
	Invalid,
	None,
	SkinOnTheFly,
	PreSkin,
	ENDISkeletalMesh_MAX
};

enum ENDISkeletalMesh_SourceMode
{
	Default,
	Source,
	AttachParent,
	ENDISkeletalMesh_MAX
};

enum ENDIStaticMesh_SourceMode
{
	Default,
	Source,
	AttachParent,
	DefaultMeshOnly,
	ENDIStaticMesh_MAX
};

enum ENiagaraDebugHudVerbosity
{
	None,
	Basic,
	Verbose,
	ENiagaraDebugHudVerbosity_MAX
};

enum ENiagaraDebugHudFont
{
	Small,
	Normal,
	ENiagaraDebugHudFont_MAX
};

enum ENiagaraDebugHudVAlign
{
	Top,
	Center,
	Bottom,
	ENiagaraDebugHudVAlign_MAX
};

enum ENiagaraDebugHudHAlign
{
	Left,
	Center,
	Right,
	ENiagaraDebugHudHAlign_MAX
};

enum ENiagaraDebugPlaybackMode
{
	Play,
	Loop,
	Paused,
	Step,
	ENiagaraDebugPlaybackMode_MAX
};

enum ENiagaraCullProxyMode
{
	None,
	Instanced_Rendered,
	ENiagaraCullProxyMode_MAX
};

enum ENiagaraScalabilityUpdateFrequency
{
	SpawnOnly,
	Low,
	Medium,
	High,
	Continuous,
	ENiagaraScalabilityUpdateFrequency_MAX
};

enum ENiagaraCullReaction
{
	Deactivate,
	DeactivateImmediate,
	DeactivateResume,
	DeactivateImmediateResume,
	ENiagaraCullReaction_MAX
};

enum EParticleAllocationMode
{
	AutomaticEstimate,
	ManualEstimate,
	EParticleAllocationMode_MAX
};

enum EScriptExecutionMode
{
	EveryParticle,
	SpawnedParticles,
	SingleParticle,
	EScriptExecutionMode_MAX
};

enum ENiagaraSortMode
{
	None,
	ViewDepth,
	ViewDistance,
	CustomAscending,
	CustomDecending,
	ENiagaraSortMode_MAX
};

enum ENiagaraMeshLockedAxisSpace
{
	Simulation,
	World,
	Local,
	ENiagaraMeshLockedAxisSpace_MAX
};

enum ENiagaraMeshPivotOffsetSpace
{
	Mesh,
	Simulation,
	World,
	Local,
	ENiagaraMeshPivotOffsetSpace_MAX
};

enum ENiagaraMeshFacingMode
{
	Default,
	Velocity,
	CameraPosition,
	CameraPlane,
	ENiagaraMeshFacingMode_MAX
};

enum ENiagaraPlatformSetState
{
	Disabled,
	Enabled,
	Active,
	Unknown,
	ENiagaraPlatformSetState_MAX
};

enum ENiagaraPlatformSelectionState
{
	Default,
	Enabled,
	Disabled,
	ENiagaraPlatformSelectionState_MAX
};

enum ENiagaraPreviewGridResetMode
{
	Never,
	Individual,
	All,
	ENiagaraPreviewGridResetMode_MAX
};

enum ENiagaraRibbonUVDistributionMode
{
	ScaledUniformly,
	ScaledUsingRibbonSegmentLength,
	TiledOverRibbonLength,
	TiledFromStartOverRibbonLength,
	ENiagaraRibbonUVDistributionMode_MAX
};

enum ENiagaraRibbonUVEdgeMode
{
	SmoothTransition,
	Locked,
	ENiagaraRibbonUVEdgeMode_MAX
};

enum ENiagaraRibbonTessellationMode
{
	Automatic,
	Custom,
	Disabled,
	ENiagaraRibbonTessellationMode_MAX
};

enum ENiagaraRibbonShapeMode
{
	Plane,
	MultiPlane,
	Tube,
	Custom,
	ENiagaraRibbonShapeMode_MAX
};

enum ENiagaraRibbonDrawDirection
{
	FrontToBack,
	BackToFront,
	ENiagaraRibbonDrawDirection_MAX
};

enum ENiagaraRibbonAgeOffsetMode
{
	Scale,
	Clip,
	ENiagaraRibbonAgeOffsetMode_MAX
};

enum ENiagaraRibbonFacingMode
{
	Screen,
	Custom,
	CustomSideVector,
	ENiagaraRibbonFacingMode_MAX
};

enum ENiagaraScriptTemplateSpecification
{
	None,
	Template,
	Behavior,
	ENiagaraScriptTemplateSpecification_MAX
};

enum ENiagaraScriptLibraryVisibility
{
	Invalid,
	Unexposed,
	Library,
	Hidden,
	ENiagaraScriptLibraryVisibility_MAX
};

enum ENiagaraModuleDependencyScriptConstraint
{
	SameScript,
	AllScripts,
	ENiagaraModuleDependencyScriptConstraint_MAX
};

enum ENiagaraModuleDependencyType
{
	PreDependency,
	PostDependency,
	ENiagaraModuleDependencyType_MAX
};

enum EUnusedAttributeBehaviour
{
	Copy,
	Zero,
	None,
	MarkInvalid,
	PassThrough,
	EUnusedAttributeBehaviour_MAX
};

enum ENDISkelMesh_AdjacencyTriangleIndexFormat
{
	Full,
	Half,
	ENDISkelMesh_MAX
};

enum ENDISkelMesh_GpuUniformSamplingFormat
{
	Full,
	Limited_24_8,
	Limited_23_9,
	ENDISkelMesh_MAX
};

enum ENDISkelMesh_GpuMaxInfluences
{
	AllowMax4,
	AllowMax8,
	Unlimited,
	ENDISkelMesh_MAX
};

enum ENiagaraSpriteFacingMode
{
	FaceCamera,
	FaceCameraPlane,
	CustomFacingVector,
	FaceCameraPosition,
	FaceCameraDistanceBlend,
	ENiagaraSpriteFacingMode_MAX
};

enum ENiagaraSpriteAlignment
{
	Unaligned,
	VelocityAligned,
	CustomAlignment,
	ENiagaraSpriteAlignment_MAX
};

enum ENiagaraOrientationAxis
{
	XAxis,
	YAxis,
	ZAxis,
	ENiagaraOrientationAxis_MAX
};

enum ENiagaraPythonUpdateScriptReference
{
	None,
	ScriptAsset,
	DirectTextEntry,
	ENiagaraPythonUpdateScriptReference_MAX
};

enum ENiagaraCoordinateSpace
{
	Simulation,
	World,
	Local,
	ENiagaraCoordinateSpace_MAX
};

enum ENiagaraExecutionState
{
	Active,
	Inactive,
	InactiveClear,
	Complete,
	Disabled,
	Num,
	ENiagaraExecutionState_MAX
};

enum ENiagaraExecutionStateSource
{
	Scalability,
	Internal,
	Owner,
	InternalCompletion,
	ENiagaraExecutionStateSource_MAX
};

enum ENiagaraNumericOutputTypeSelectionMode
{
	None,
	Largest,
	Smallest,
	Scalar,
	ENiagaraNumericOutputTypeSelectionMode_MAX
};

enum ENiagaraVariantMode
{
	None,
	Object,
	DataInterface,
	Bytes,
	ENiagaraVariantMode_MAX
};

enum EGfeSDKHighlightSignificance
{
	NONE,
	ExtremelyBad,
	VeryBad,
	Bad,
	Neutral,
	Good,
	VeryGood,
	ExtremelyGood,
	MAX
};

enum EGfeSDKHighlightType
{
	NONE,
	Milestone,
	Achievement,
	Incident,
	StateChange,
	MAX
};

enum EGfeSDKPermission
{
	Granted,
	Denied,
	MustAsk,
	Unknown,
	MAX
};

enum EGfeSDKScope
{
	Highlights,
	HighlightsRecordVideo,
	HighlightsRecordScreenshot,
	MAX
};

enum EGfeSDKReturnCode
{
	Success,
	SuccessIpcOldSdk,
	SuccessIpcOldGfe,
	Error,
	ErrorGfeVersion,
	ErrorSdkVersion,
	ErrorModuleNotLoaded,
	EGfeSDKReturnCode_MAX
};

enum EInAppPurchaseStatus
{
	Invalid,
	Failed,
	Deferred,
	Canceled,
	Purchased,
	Restored,
	EInAppPurchaseStatus_MAX
};

enum EOnlineProxyStoreOfferDiscountType
{
	NotOnSale,
	Percentage,
	DiscountAmount,
	PayAmount,
	EOnlineProxyStoreOfferDiscountType_MAX
};

enum EBeaconConnectionState
{
	Invalid,
	Closed,
	Pending,
	Open,
	EBeaconConnectionState_MAX
};

enum EClientRequestType
{
	NonePending,
	ExistingSessionReservation,
	ReservationUpdate,
	EmptyServerReservation,
	Reconnect,
	Abandon,
	ReservationRemoveMembers,
	AddOrUpdateReservation,
	EClientRequestType_MAX
};

enum EPartyReservationResult
{
	NoResult,
	RequestPending,
	GeneralError,
	PartyLimitReached,
	IncorrectPlayerCount,
	RequestTimedOut,
	ReservationDuplicate,
	ReservationNotFound,
	ReservationAccepted,
	ReservationDenied,
	ReservationDenied_CrossPlayRestriction,
	ReservationDenied_Banned,
	ReservationRequestCanceled,
	ReservationInvalid,
	BadSessionId,
	ReservationDenied_ContainsExistingPlayers,
	EPartyReservationResult_MAX
};

enum ESpectatorClientRequestType
{
	NonePending,
	ExistingSessionReservation,
	ReservationUpdate,
	EmptyServerReservation,
	Reconnect,
	Abandon,
	ESpectatorClientRequestType_MAX
};

enum ESpectatorReservationResult
{
	NoResult,
	RequestPending,
	GeneralError,
	SpectatorLimitReached,
	IncorrectPlayerCount,
	RequestTimedOut,
	ReservationDuplicate,
	ReservationNotFound,
	ReservationAccepted,
	ReservationDenied,
	ReservationDenied_CrossPlayRestriction,
	ReservationDenied_Banned,
	ReservationRequestCanceled,
	ReservationInvalid,
	BadSessionId,
	ReservationDenied_ContainsExistingPlayers,
	ESpectatorReservationResult_MAX
};

enum EInAppPurchaseState
{
	Unknown,
	Success,
	Failed,
	Cancelled,
	Invalid,
	NotAllowed,
	Restored,
	AlreadyOwned,
	EInAppPurchaseState_MAX
};

enum EMPMatchOutcome
{
	None,
	Quit,
	Won,
	Lost,
	Tied,
	TimeExpired,
	First,
	Second,
	Third,
	Fourth,
	EMPMatchOutcome_MAX
};

enum EOodleEnableMode
{
	AlwaysEnabled,
	WhenCompressedPacketReceived,
	EOodleEnableMode_MAX
};

enum EFlipbookCollisionMode
{
	NoCollision,
	FirstFrameCollision,
	EachFrameCollision,
	EFlipbookCollisionMode_MAX
};

enum EPaperSpriteAtlasPadding
{
	DilateBorder,
	PadWithZero,
	EPaperSpriteAtlasPadding_MAX
};

enum ETileMapProjectionMode
{
	Orthogonal,
	IsometricDiamond,
	IsometricStaggered,
	HexagonalStaggered,
	ETileMapProjectionMode_MAX
};

enum ESpritePivotMode
{
	Top_Left,
	Top_Center,
	Top_Right,
	Center_Left,
	Center_Center,
	Center_Right,
	Bottom_Left,
	Bottom_Center,
	Bottom_Right,
	Custom,
	ESpritePivotMode_MAX
};

enum ESpritePolygonMode
{
	SourceBoundingBox,
	TightBoundingBox,
	ShrinkWrapped,
	FullyCustom,
	Diced,
	ESpritePolygonMode_MAX
};

enum ESpriteShapeType
{
	Box,
	Circle,
	Polygon,
	ESpriteShapeType_MAX
};

enum ESpriteCollisionMode
{
	None,
	Use2DPhysics,
	Use3DPhysics,
	ESpriteCollisionMode_MAX
};

enum ESocialPartyInviteFailureReason
{
	Success,
	NotOnline,
	NotAcceptingMembers,
	NotFriends,
	AlreadyInParty,
	OssValidationFailed,
	PlatformInviteFailed,
	PartyInviteFailed,
	InviteRateLimitExceeded,
	ESocialPartyInviteFailureReason_MAX
};

enum ESocialPartyInviteMethod
{
	Other,
	Notification,
	ESocialPartyInviteMethod_MAX
};

enum EApprovalAction
{
	Approve,
	Enqueue,
	EnqueueAndStartBeacon,
	Deny,
	EApprovalAction_MAX
};

enum EPartyJoinDenialReason
{
	NoReason,
	JoinAttemptAborted,
	Busy,
	OssUnavailable,
	PartyFull,
	GameFull,
	NotPartyLeader,
	PartyPrivate,
	JoinerCrossplayRestricted,
	MemberCrossplayRestricted,
	GameModeRestricted,
	Banned,
	NotLoggedIn,
	CheckingForRejoin,
	TargetUserMissingPresence,
	TargetUserUnjoinable,
	TargetUserAway,
	AlreadyLeaderInPlatformSession,
	TargetUserPlayingDifferentGame,
	TargetUserMissingPlatformSession,
	PlatformSessionMissingJoinInfo,
	FailedToStartFindConsoleSession,
	MissingPartyClassForTypeId,
	TargetUserBlocked,
	CustomReason0,
	CustomReason1,
	CustomReason2,
	CustomReason3,
	CustomReason4,
	CustomReason5,
	CustomReason6,
	CustomReason7,
	CustomReason8,
	CustomReason9,
	CustomReason10,
	CustomReason11,
	CustomReason12,
	CustomReason13,
	CustomReason14,
	CustomReason15,
	CustomReason16,
	CustomReason17,
	CustomReason18,
	CustomReason19,
	CustomReason20,
	CustomReason21,
	CustomReason22,
	CustomReason23,
	CustomReason24,
	CustomReason25,
	CustomReason26,
	CustomReason27,
	CustomReason28,
	CustomReason29,
	CustomReason30,
	CustomReason31,
	CustomReason32,
	CustomReason33,
	CustomReason34,
	CustomReason35,
	CustomReason36,
	CustomReason37,
	CustomReason38,
	CustomReason39,
	MAX
};

enum EPartyInviteRestriction
{
	AnyMember,
	LeaderOnly,
	NoInvites,
	EPartyInviteRestriction_MAX
};

enum EPartyType
{
	Public,
	FriendsOnly,
	Private,
	EPartyType_MAX
};

enum ESocialChannelType
{
	General,
	Founder,
	Party,
	Team,
	System,
	Private,
	ESocialChannelType_MAX
};

enum EPlatformIconDisplayRule
{
	Always,
	AlwaysIfDifferent,
	AlwaysWhenInCrossplayParty,
	AlwaysIfDifferentWhenInCrossplayParty,
	Never,
	EPlatformIconDisplayRule_MAX
};

enum ECrossplayPreference
{
	NoSelection,
	OptedIn,
	OptedOut,
	OptedOutRestricted,
	ECrossplayPreference_MAX
};

enum ESocialRelationship
{
	Any,
	FriendInviteReceived,
	FriendInviteSent,
	PartyInvite,
	Friend,
	BlockedPlayer,
	SuggestedFriend,
	RecentPlayer,
	JoinRequest,
	ESocialRelationship_MAX
};

enum ESocialSubsystem
{
	Primary,
	Platform,
	MAX
};

enum ESocialFriendRequestMethod
{
	SocialInteraction_AddFriend,
	SocialInteraction_AddPlatformFriend,
	Other,
	ESocialFriendRequestMethod_MAX
};

enum EPhysicalSurface
{
	SurfaceType_Default,
	SurfaceType1,
	SurfaceType2,
	SurfaceType3,
	SurfaceType4,
	SurfaceType5,
	SurfaceType6,
	SurfaceType7,
	SurfaceType8,
	SurfaceType9,
	SurfaceType10,
	SurfaceType11,
	SurfaceType12,
	SurfaceType13,
	SurfaceType14,
	SurfaceType15,
	SurfaceType16,
	SurfaceType17,
	SurfaceType18,
	SurfaceType19,
	SurfaceType20,
	SurfaceType21,
	SurfaceType22,
	SurfaceType23,
	SurfaceType24,
	SurfaceType25,
	SurfaceType26,
	SurfaceType27,
	SurfaceType28,
	SurfaceType29,
	SurfaceType30,
	SurfaceType31,
	SurfaceType32,
	SurfaceType33,
	SurfaceType34,
	SurfaceType35,
	SurfaceType36,
	SurfaceType37,
	SurfaceType38,
	SurfaceType39,
	SurfaceType40,
	SurfaceType41,
	SurfaceType42,
	SurfaceType43,
	SurfaceType44,
	SurfaceType45,
	SurfaceType46,
	SurfaceType47,
	SurfaceType48,
	SurfaceType49,
	SurfaceType50,
	SurfaceType51,
	SurfaceType52,
	SurfaceType53,
	SurfaceType54,
	SurfaceType55,
	SurfaceType56,
	SurfaceType57,
	SurfaceType58,
	SurfaceType59,
	SurfaceType60,
	SurfaceType61,
	SurfaceType62,
	SurfaceType_Max,
	EPhysicalSurface_MAX
};

enum ERadialImpulseFalloff
{
	RIF_Constant,
	RIF_Linear,
	RIF_MAX
};

enum ESleepFamily
{
	Normal,
	Sensitive,
	Custom,
	ESleepFamily_MAX
};

enum EBodyCollisionResponse
{
	BodyCollision_Enabled,
	BodyCollision_Disabled,
	BodyCollision_MAX
};

enum EPhysicsType
{
	PhysType_Default,
	PhysType_Kinematic,
	PhysType_Simulated,
	PhysType_MAX
};

enum ECollisionTraceFlag
{
	CTF_UseDefault,
	CTF_UseSimpleAndComplex,
	CTF_UseSimpleAsComplex,
	CTF_UseComplexAsSimple,
	CTF_MAX
};

enum ELinearConstraintMotion
{
	LCM_Free,
	LCM_Limited,
	LCM_Locked,
	LCM_MAX
};

enum EConstraintFrame
{
	Frame1,
	Frame2,
	EConstraintFrame_MAX
};

enum EAngularConstraintMotion
{
	ACM_Free,
	ACM_Limited,
	ACM_Locked,
	ACM_MAX
};

enum EFrictionCombineMode
{
	Average,
	Min,
	Multiply,
	Max
};

enum EWheelSweepType
{
	SimpleAndComplex,
	Simple,
	Complex,
	EWheelSweepType_MAX
};

enum EVehicleDifferential4W
{
	LimitedSlip_4W,
	LimitedSlip_FrontDrive,
	LimitedSlip_RearDrive,
	Open_4W,
	Open_FrontDrive,
	Open_RearDrive,
	EVehicleDifferential4W_MAX
};

enum EPlayspaceCreationType
{
	ChildOfRoot,
	RootDestroy,
	RootInserted,
	EPlayspaceCreationType_MAX
};

enum EProcMeshSliceCapOption
{
	NoCap,
	CreateNewSectionForCap,
	UseLastSectionForCap,
	EProcMeshSliceCapOption_MAX
};

enum EProceduralParameterModifierBlendMode
{
	Min,
	Max,
	Additive,
	Subtractive,
	Multiply,
	Interpolate
};

enum EProceduralRotationFormat
{
	VectorXAxis,
	VectorXAxisNegative,
	VectorYAxis,
	VectorYAxisNegative,
	VectorZAxis,
	VectorZAxisNegative,
	Vector2DXAxis,
	Vector2DXAxisNegative,
	Vector2DYAxis,
	Vector2DYAxisNegative,
	Vector2DZAxis,
	Vector2DZAxisNegative,
	RangedRotator,
	EProceduralRotationFormat_MAX
};

enum EProceduralScatterContentPivotMode
{
	UsePivot,
	UseBoundingBoxCenter,
	UseBoundingBoxBottomCenter,
	UseFootprintBoundingBoxBottomCenter,
	EProceduralScatterContentPivotMode_MAX
};

enum EProceduralScatterMethod
{
	Density,
	SourcePoints,
	Grid,
	EProceduralScatterMethod_MAX
};

enum EProceduralScatterTileRandomGenerator
{
	PseudoRandom,
	HaltonSequence,
	EProceduralScatterTileRandomGenerator_MAX
};

enum EProceduralScatterStaticMeshMode
{
	FromStaticMesh,
	FromActor,
	EProceduralScatterStaticMeshMode_MAX
};

enum EProceduralTextureColorChannel
{
	Red,
	Green,
	Blue,
	Alpha,
	EProceduralTextureColorChannel_MAX
};

enum EPropertyAccessCopyBatch
{
	InternalUnbatched,
	ExternalUnbatched,
	InternalBatched,
	ExternalBatched,
	Count,
	EPropertyAccessCopyBatch_MAX
};

enum EPropertyAccessCopyType
{
	None,
	Plain,
	Complex,
	Bool,
	Struct,
	Object,
	Name,
	Array,
	PromoteBoolToByte,
	PromoteBoolToInt32,
	PromoteBoolToInt64,
	PromoteBoolToFloat,
	PromoteByteToInt32,
	PromoteByteToInt64,
	PromoteByteToFloat,
	PromoteInt32ToInt64,
	PromoteInt32ToFloat,
	EPropertyAccessCopyType_MAX
};

enum EPropertyAccessObjectType
{
	None,
	Object,
	WeakObject,
	SoftObject,
	EPropertyAccessObjectType_MAX
};

enum EPropertyAccessIndirectionType
{
	Offset,
	Object,
	Array,
	ScriptFunction,
	NativeFunction,
	EPropertyAccessIndirectionType_MAX
};

enum EQosResponseType
{
	NoResponse,
	Success,
	Failure,
	EQosResponseType_MAX
};

enum EQosCompletionResult
{
	Invalid,
	Success,
	Failure,
	Canceled,
	EQosCompletionResult_MAX
};

enum EQosDatacenterResult
{
	Invalid,
	Success,
	Incomplete,
	EQosDatacenterResult_MAX
};

enum ERejoinStatus
{
	NoMatchToRejoin,
	RejoinAvailable,
	UpdatingStatus,
	NeedsRecheck,
	NoMatchToRejoin_MatchEnded,
	ERejoinStatus_MAX
};

enum EResonanceRenderMode
{
	StereoPanning,
	BinauralLowQuality,
	BinauralMediumQuality,
	BinauralHighQuality,
	RoomEffectsOnly,
	EResonanceRenderMode_MAX
};

enum ERaMaterialName
{
	TRANSPARENT,
	ACOUSTIC_CEILING_TILES,
	BRICK_BARE,
	BRICK_PAINTED,
	CONCRETE_BLOCK_COARSE,
	CONCRETE_BLOCK_PAINTED,
	CURTAIN_HEAVY,
	FIBER_GLASS_INSULATION,
	GLASS_THIN,
	GLASS_THICK,
	GRASS,
	LINOLEUM_ON_CONCRETE,
	MARBLE,
	METAL,
	PARQUET_ONCONCRETE,
	PLASTER_ROUGH,
	PLASTER_SMOOTH,
	PLYWOOD_PANEL,
	POLISHED_CONCRETE_OR_TILE,
	SHEETROCK,
	WATER_OR_ICE_SURFACE,
	WOOD_CEILING,
	WOOD_PANEL,
	UNIFORM,
	ERaMaterialName_MAX
};

enum ERaDistanceRolloffModel
{
	LOGARITHMIC,
	LINEAR,
	NONE,
	ERaDistanceRolloffModel_MAX
};

enum ERaSpatializationMethod
{
	STEREO_PANNING,
	HRTF,
	ERaSpatializationMethod_MAX
};

enum ERaQualityMode
{
	STEREO_PANNING,
	BINAURAL_LOW,
	BINAURAL_MEDIUM,
	BINAURAL_HIGH,
	ERaQualityMode_MAX
};

enum ERigVMParameterType
{
	Input,
	Output,
	Invalid,
	ERigVMParameterType_MAX
};

enum ERigVMOpCode
{
	Execute_0_Operands,
	Execute_1_Operands,
	Execute_2_Operands,
	Execute_3_Operands,
	Execute_4_Operands,
	Execute_5_Operands,
	Execute_6_Operands,
	Execute_7_Operands,
	Execute_8_Operands,
	Execute_9_Operands,
	Execute_10_Operands,
	Execute_11_Operands,
	Execute_12_Operands,
	Execute_13_Operands,
	Execute_14_Operands,
	Execute_15_Operands,
	Execute_16_Operands,
	Execute_17_Operands,
	Execute_18_Operands,
	Execute_19_Operands,
	Execute_20_Operands,
	Execute_21_Operands,
	Execute_22_Operands,
	Execute_23_Operands,
	Execute_24_Operands,
	Execute_25_Operands,
	Execute_26_Operands,
	Execute_27_Operands,
	Execute_28_Operands,
	Execute_29_Operands,
	Execute_30_Operands,
	Execute_31_Operands,
	Execute_32_Operands,
	Execute_33_Operands,
	Execute_34_Operands,
	Execute_35_Operands,
	Execute_36_Operands,
	Execute_37_Operands,
	Execute_38_Operands,
	Execute_39_Operands,
	Execute_40_Operands,
	Execute_41_Operands,
	Execute_42_Operands,
	Execute_43_Operands,
	Execute_44_Operands,
	Execute_45_Operands,
	Execute_46_Operands,
	Execute_47_Operands,
	Execute_48_Operands,
	Execute_49_Operands,
	Execute_50_Operands,
	Execute_51_Operands,
	Execute_52_Operands,
	Execute_53_Operands,
	Execute_54_Operands,
	Execute_55_Operands,
	Execute_56_Operands,
	Execute_57_Operands,
	Execute_58_Operands,
	Execute_59_Operands,
	Execute_60_Operands,
	Execute_61_Operands,
	Execute_62_Operands,
	Execute_63_Operands,
	Execute_64_Operands,
	Zero,
	BoolFalse,
	BoolTrue,
	Copy,
	Increment,
	Decrement,
	Equals,
	NotEquals,
	JumpAbsolute,
	JumpForward,
	JumpBackward,
	JumpAbsoluteIf,
	JumpForwardIf,
	JumpBackwardIf,
	ChangeType,
	Exit,
	BeginBlock,
	EndBlock,
	Invalid,
	ERigVMOpCode_MAX
};

enum ERigVMPinDirection
{
	Input,
	Output,
	IO,
	Visible,
	Hidden,
	Invalid,
	ERigVMPinDirection_MAX
};

enum ERigVMRegisterType
{
	Plain,
	String,
	Name,
	Struct,
	Invalid,
	ERigVMRegisterType_MAX
};

enum ERigVMMemoryType
{
	Work,
	Literal,
	External,
	Invalid,
	ERigVMMemoryType_MAX
};

enum ESequenceTimeUnit
{
	DisplayRate,
	TickResolution,
	ESequenceTimeUnit_MAX
};

enum ESkyfirePhase
{
	None,
	Prison,
	CoreGameplay,
	AllPlayersExited,
	ESkyfirePhase_MAX
};

enum ESkyfireBackpackComponentCachedExec
{
	Cached,
	Success,
	Fail,
	ESkyfireBackpackComponentCachedExec_MAX
};

enum ESkyfireBackpackComponentYesNoExec
{
	Yes,
	No,
	ESkyfireBackpackComponentYesNoExec_MAX
};

enum EOrbSpawnerConfigCategory
{
	Solo,
	Duos,
	Trios,
	Squads,
	None,
	EOrbSpawnerConfigCategory_MAX
};

enum EUINavigation
{
	Left,
	Right,
	Up,
	Down,
	Next,
	Previous,
	Num,
	Invalid,
	EUINavigation_MAX
};

enum ECheckBoxState
{
	Unchecked,
	Checked,
	Undetermined,
	ECheckBoxState_MAX
};

enum EWidgetClipping
{
	Inherit,
	ClipToBounds,
	ClipToBoundsWithoutIntersecting,
	ClipToBoundsAlways,
	OnDemand,
	EWidgetClipping_MAX
};

enum ESlateBrushImageType
{
	NoImage,
	FullColor,
	Linear,
	ESlateBrushImageType_MAX
};

enum ESlateBrushMirrorType
{
	NoMirror,
	Horizontal,
	Vertical,
	Both,
	ESlateBrushMirrorType_MAX
};

enum ESlateBrushTileType
{
	NoTile,
	Horizontal,
	Vertical,
	Both,
	ESlateBrushTileType_MAX
};

enum ESlateBrushDrawType
{
	NoDrawType,
	Box,
	Border,
	Image,
	ESlateBrushDrawType_MAX
};

enum ESlateColorStylingMode
{
	UseColor_Specified,
	UseColor_Specified_Link,
	UseColor_Foreground,
	UseColor_Foreground_Subdued,
	UseColor_MAX
};

enum EUINavigationRule
{
	Escape,
	Explicit,
	Wrap,
	Stop,
	Custom,
	CustomBoundary,
	Invalid,
	EUINavigationRule_MAX
};

enum EFlowDirectionPreference
{
	Inherit,
	Culture,
	LeftToRight,
	RightToLeft,
	EFlowDirectionPreference_MAX
};

enum EColorVisionDeficiency
{
	NormalVision,
	Deuteranope,
	Protanope,
	Tritanope,
	EColorVisionDeficiency_MAX
};

enum ESelectInfo
{
	OnKeyPress,
	OnNavigation,
	OnMouseClick,
	Direct,
	ESelectInfo_MAX
};

enum ETextCommit
{
	Default,
	OnEnter,
	OnUserMovedFocus,
	OnCleared,
	ETextCommit_MAX
};

enum ETextShapingMethod
{
	Auto,
	KerningOnly,
	FullShaping,
	ETextShapingMethod_MAX
};

enum EMenuPlacement
{
	MenuPlacement_BelowAnchor,
	MenuPlacement_CenteredBelowAnchor,
	MenuPlacement_BelowRightAnchor,
	MenuPlacement_ComboBox,
	MenuPlacement_ComboBoxRight,
	MenuPlacement_MenuRight,
	MenuPlacement_AboveAnchor,
	MenuPlacement_CenteredAboveAnchor,
	MenuPlacement_AboveRightAnchor,
	MenuPlacement_MenuLeft,
	MenuPlacement_Center,
	MenuPlacement_RightLeftCenter,
	MenuPlacement_MatchBottomLeft,
	MenuPlacement_MAX
};

enum EHorizontalAlignment
{
	HAlign_Fill,
	HAlign_Left,
	HAlign_Center,
	HAlign_Right,
	HAlign_MAX
};

enum EVerticalAlignment
{
	VAlign_Fill,
	VAlign_Top,
	VAlign_Center,
	VAlign_Bottom,
	VAlign_MAX
};

enum EFontLayoutMethod
{
	Metrics,
	BoundingBox,
	EFontLayoutMethod_MAX
};

enum EFontLoadingPolicy
{
	LazyLoad,
	Stream,
	Inline,
	EFontLoadingPolicy_MAX
};

enum EFontHinting
{
	Default,
	Auto,
	AutoLight,
	Monochrome,
	None,
	EFontHinting_MAX
};

enum EFocusCause
{
	Mouse,
	Navigation,
	SetDirectly,
	Cleared,
	OtherWidgetLostFocus,
	WindowActivate,
	EFocusCause_MAX
};

enum ESlateDebuggingFocusEvent
{
	FocusChanging,
	FocusLost,
	FocusReceived,
	MAX
};

enum ESlateDebuggingNavigationMethod
{
	Unknown,
	Explicit,
	CustomDelegateBound,
	CustomDelegateUnbound,
	NextOrPrevious,
	HitTestGrid,
	ESlateDebuggingNavigationMethod_MAX
};

enum ESlateDebuggingStateChangeEvent
{
	MouseCaptureGained,
	MouseCaptureLost,
	ESlateDebuggingStateChangeEvent_MAX
};

enum ESlateDebuggingInputEvent
{
	MouseMove,
	MouseEnter,
	MouseLeave,
	PreviewMouseButtonDown,
	MouseButtonDown,
	MouseButtonUp,
	MouseButtonDoubleClick,
	MouseWheel,
	TouchStart,
	TouchEnd,
	TouchForceChanged,
	TouchFirstMove,
	TouchMoved,
	DragDetected,
	DragEnter,
	DragLeave,
	DragOver,
	DragDrop,
	DropMessage,
	PreviewKeyDown,
	KeyDown,
	KeyUp,
	KeyChar,
	AnalogInput,
	TouchGesture,
	MotionDetected,
	MAX
};

enum EScrollDirection
{
	Scroll_Down,
	Scroll_Up,
	Scroll_MAX
};

enum EOrientation
{
	Orient_Horizontal,
	Orient_Vertical,
	Orient_MAX
};

enum ENavigationGenesis
{
	Keyboard,
	Controller,
	User,
	ENavigationGenesis_MAX
};

enum ENavigationSource
{
	FocusedWidget,
	WidgetUnderCursor,
	ENavigationSource_MAX
};

enum EUINavigationAction
{
	Accept,
	Back,
	Num,
	Invalid,
	EUINavigationAction_MAX
};

enum EButtonPressMethod
{
	DownAndUp,
	ButtonPress,
	ButtonRelease,
	EButtonPressMethod_MAX
};

enum EButtonTouchMethod
{
	DownAndUp,
	Down,
	PreciseTap,
	EButtonTouchMethod_MAX
};

enum EButtonClickMethod
{
	DownAndUp,
	MouseDown,
	MouseUp,
	PreciseClick,
	EButtonClickMethod_MAX
};

enum ESlateCheckBoxType
{
	CheckBox,
	ToggleButton,
	ESlateCheckBoxType_MAX
};

enum ESlateParentWindowSearchMethod
{
	ActiveWindow,
	MainWindow,
	ESlateParentWindowSearchMethod_MAX
};

enum EConsumeMouseWheel
{
	WhenScrollingPossible,
	Always,
	Never,
	EConsumeMouseWheel_MAX
};

enum ETextJustify
{
	Left,
	Center,
	Right,
	ETextJustify_MAX
};

enum ETextFlowDirection
{
	Auto,
	LeftToRight,
	RightToLeft,
	ETextFlowDirection_MAX
};

enum EVirtualKeyboardDismissAction
{
	TextChangeOnDismiss,
	TextCommitOnAccept,
	TextCommitOnDismiss,
	EVirtualKeyboardDismissAction_MAX
};

enum EVirtualKeyboardTrigger
{
	OnFocusByPointer,
	OnAllFocusEvents,
	EVirtualKeyboardTrigger_MAX
};

enum ETextWrappingPolicy
{
	DefaultWrapping,
	AllowPerCharacterWrapping,
	ETextWrappingPolicy_MAX
};

enum ETextTransformPolicy
{
	None,
	ToLower,
	ToUpper,
	ETextTransformPolicy_MAX
};

enum ETableViewMode
{
	List,
	Tile,
	Tree,
	ETableViewMode_MAX
};

enum ESelectionMode
{
	None,
	Single,
	SingleToggle,
	Multi,
	ESelectionMode_MAX
};

enum EMultiBlockType
{
	None,
	ButtonRow,
	EditableText,
	Heading,
	MenuEntry,
	Separator,
	ToolBarButton,
	ToolBarComboButton,
	Widget,
	EMultiBlockType_MAX
};

enum EMultiBoxType
{
	MenuBar,
	ToolBar,
	VerticalToolBar,
	UniformToolBar,
	Menu,
	ButtonRow,
	EMultiBoxType_MAX
};

enum EProgressBarFillType
{
	LeftToRight,
	RightToLeft,
	FillFromCenter,
	TopToBottom,
	BottomToTop,
	EProgressBarFillType_MAX
};

enum EStretch
{
	None,
	Fill,
	ScaleToFit,
	ScaleToFitX,
	ScaleToFitY,
	ScaleToFill,
	ScaleBySafeZone,
	UserSpecified,
	EStretch_MAX
};

enum EStretchDirection
{
	Both,
	DownOnly,
	UpOnly,
	EStretchDirection_MAX
};

enum EScrollWhenFocusChanges
{
	NoScroll,
	InstantScroll,
	AnimatedScroll,
	EScrollWhenFocusChanges_MAX
};

enum EDescendantScrollDestination
{
	IntoView,
	TopOrLeft,
	Center,
	BottomOrRight,
	EDescendantScrollDestination_MAX
};

enum EListItemAlignment
{
	EvenlyDistributed,
	EvenlySize,
	EvenlyWide,
	LeftAligned,
	RightAligned,
	CenterAligned,
	Fill,
	EListItemAlignment_MAX
};

enum ECustomizedToolMenuVisibility
{
	None,
	Visible,
	Hidden,
	ECustomizedToolMenuVisibility_MAX
};

enum EMultipleKeyBindingIndex
{
	Primary,
	Secondary,
	NumChords,
	EMultipleKeyBindingIndex_MAX
};

enum EUserInterfaceActionType
{
	None,
	Button,
	ToggleButton,
	RadioButton,
	Check,
	CollapsedButton,
	EUserInterfaceActionType_MAX
};

enum ESmartObjectTagPolicy
{
	Any,
	All,
	MAX
};

enum ESOReferenceDrawingMode
{
	Default,
	Sequence,
	MAX
};

enum ESmartObjectRequiredTagsPolicy
{
	RequireAny,
	RequireAnyExact,
	RequireAll,
	RequireAllExact,
	RequireNone,
	ESmartObjectRequiredTagsPolicy_MAX
};

enum ESmartObjectSlotState
{
	Free,
	Claimed,
	Occupied,
	MAX
};

enum ESoundContainerType
{
	Concatenate,
	Randomize,
	Mix,
	ESoundContainerType_MAX
};

enum ESoundLibraryNotifyTriggerType
{
	Play,
	Stop,
	None,
	ESoundLibraryNotifyTriggerType_MAX
};

enum ESpecialEventPhaseState
{
	Unregistered,
	Inactive,
	Active,
	Deactivated,
	ESpecialEventPhaseState_MAX
};

enum EScriptStartupState
{
	WaitingForWarmupToEnd,
	PreShow,
	ShowTime,
	EScriptStartupState_MAX
};

enum EOnlineRadioSourceType
{
	Epic,
	EOnlineRadioSourceType_MAX
};

enum ERadioSource
{
	Invalid,
	Vehicle,
	ERadioSource_MAX
};

enum EStreamingRadioSourceState
{
	None,
	Loading,
	LoadingPlayer,
	LoadedPlayer,
	Playing,
	EStreamingRadioSourceState_MAX
};

enum EOptionalLabel
{
	Label_DS,
	Label_NM,
	Max
};

enum ESubtitleDisplayBackgroundOpacity
{
	Clear,
	Low,
	Medium,
	High,
	Solid,
	ESubtitleDisplayBackgroundOpacity_MAX
};

enum ESubtitleDisplayTextBorder
{
	None,
	Outline,
	DropShadow,
	ESubtitleDisplayTextBorder_MAX
};

enum ESubtitleDisplayTextColor
{
	White,
	Yellow,
	ESubtitleDisplayTextColor_MAX
};

enum ESubtitleDisplayTextSize
{
	ExtraSmall,
	Small,
	Medium,
	Large,
	ExtraLarge,
	ESubtitleDisplayTextSize_MAX
};

enum ESynth1PatchDestination
{
	Osc1Gain,
	Osc1Frequency,
	Osc1Pulsewidth,
	Osc2Gain,
	Osc2Frequency,
	Osc2Pulsewidth,
	FilterFrequency,
	FilterQ,
	Gain,
	Pan,
	LFO1Frequency,
	LFO1Gain,
	LFO2Frequency,
	LFO2Gain,
	Count,
	ESynth1PatchDestination_MAX
};

enum ESynth1PatchSource
{
	LFO1,
	LFO2,
	Envelope,
	BiasEnvelope,
	Count,
	ESynth1PatchSource_MAX
};

enum ESynthStereoDelayMode
{
	Normal,
	Cross,
	PingPong,
	Count,
	ESynthStereoDelayMode_MAX
};

enum ESynthFilterAlgorithm
{
	OnePole,
	StateVariable,
	Ladder,
	Count,
	ESynthFilterAlgorithm_MAX
};

enum ESynthFilterType
{
	LowPass,
	HighPass,
	BandPass,
	BandStop,
	Count,
	ESynthFilterType_MAX
};

enum ESynthModEnvBiasPatch
{
	PatchToNone,
	PatchToOscFreq,
	PatchToFilterFreq,
	PatchToFilterQ,
	PatchToLFO1Gain,
	PatchToLFO2Gain,
	PatchToLFO1Freq,
	PatchToLFO2Freq,
	Count,
	ESynthModEnvBiasPatch_MAX
};

enum ESynthModEnvPatch
{
	PatchToNone,
	PatchToOscFreq,
	PatchToFilterFreq,
	PatchToFilterQ,
	PatchToLFO1Gain,
	PatchToLFO2Gain,
	PatchToLFO1Freq,
	PatchToLFO2Freq,
	Count,
	ESynthModEnvPatch_MAX
};

enum ESynthLFOPatchType
{
	PatchToNone,
	PatchToGain,
	PatchToOscFreq,
	PatchToFilterFreq,
	PatchToFilterQ,
	PatchToOscPulseWidth,
	PatchToOscPan,
	PatchLFO1ToLFO2Frequency,
	PatchLFO1ToLFO2Gain,
	Count,
	ESynthLFOPatchType_MAX
};

enum ESynthLFOMode
{
	Sync,
	OneShot,
	Free,
	Count,
	ESynthLFOMode_MAX
};

enum ESynthLFOType
{
	Sine,
	UpSaw,
	DownSaw,
	Square,
	Triangle,
	Exponential,
	RandomSampleHold,
	Count,
	ESynthLFOType_MAX
};

enum ESynth1OscType
{
	Sine,
	Saw,
	Triangle,
	Square,
	Noise,
	Count,
	ESynth1OscType_MAX
};

enum ESourceEffectDynamicsPeakMode
{
	MeanSquared,
	RootMeanSquared,
	Peak,
	Count,
	ESourceEffectDynamicsPeakMode_MAX
};

enum ESourceEffectDynamicsProcessorType
{
	Compressor,
	Limiter,
	Expander,
	Gate,
	Count,
	ESourceEffectDynamicsProcessorType_MAX
};

enum EEnvelopeFollowerPeakMode
{
	MeanSquared,
	RootMeanSquared,
	Peak,
	Count,
	EEnvelopeFollowerPeakMode_MAX
};

enum ESourceEffectFilterParam
{
	FilterFrequency,
	FilterResonance,
	Count,
	ESourceEffectFilterParam_MAX
};

enum ESourceEffectFilterType
{
	LowPass,
	HighPass,
	BandPass,
	BandStop,
	Count,
	ESourceEffectFilterType_MAX
};

enum ESourceEffectFilterCircuit
{
	OnePole,
	StateVariable,
	Ladder,
	Count,
	ESourceEffectFilterCircuit_MAX
};

enum EStereoChannelMode
{
	MidSide,
	LeftRight,
	count,
	EStereoChannelMode_MAX
};

enum EPhaserLFOType
{
	Sine,
	UpSaw,
	DownSaw,
	Square,
	Triangle,
	Exponential,
	RandomSampleHold,
	Count,
	EPhaserLFOType_MAX
};

enum ERingModulatorTypeSourceEffect
{
	Sine,
	Saw,
	Triangle,
	Square,
	Count,
	ERingModulatorTypeSourceEffect_MAX
};

enum EStereoDelayFiltertype
{
	Lowpass,
	Highpass,
	Bandpass,
	Notch,
	Count,
	EStereoDelayFiltertype_MAX
};

enum EStereoDelaySourceEffect
{
	Normal,
	Cross,
	PingPong,
	Count,
	EStereoDelaySourceEffect_MAX
};

enum ESubmixEffectConvolutionReverbBlockSize
{
	BlockSize256,
	BlockSize512,
	BlockSize1024,
	ESubmixEffectConvolutionReverbBlockSize_MAX
};

enum ESubmixFilterAlgorithm
{
	OnePole,
	StateVariable,
	Ladder,
	Count,
	ESubmixFilterAlgorithm_MAX
};

enum ESubmixFilterType
{
	LowPass,
	HighPass,
	BandPass,
	BandStop,
	Count,
	ESubmixFilterType_MAX
};

enum ETapLineMode
{
	SendToChannel,
	Panning,
	Disabled,
	ETapLineMode_MAX
};

enum EGranularSynthSeekType
{
	FromBeginning,
	FromCurrentPosition,
	Count,
	EGranularSynthSeekType_MAX
};

enum EGranularSynthEnvelopeType
{
	Rectangular,
	Triangle,
	DownwardTriangle,
	UpwardTriangle,
	ExponentialDecay,
	ExponentialIncrease,
	Gaussian,
	Hanning,
	Lanczos,
	Cosine,
	CosineSquared,
	Welch,
	Blackman,
	BlackmanHarris,
	Count,
	EGranularSynthEnvelopeType_MAX
};

enum ECurveInterpolationType
{
	AUTOINTERP,
	LINEAR,
	CONSTANT,
	CurveInterpolationType_MAX
};

enum ESamplePlayerSeekType
{
	FromBeginning,
	FromCurrentPosition,
	FromEnd,
	Count,
	ESamplePlayerSeekType_MAX
};

enum ESynthKnobSize
{
	Medium,
	Large,
	Count,
	ESynthKnobSize_MAX
};

enum ESynthSlateColorStyle
{
	Light,
	Dark,
	Count,
	ESynthSlateColorStyle_MAX
};

enum ESynthSlateSizeType
{
	Small,
	Medium,
	Large,
	Count,
	ESynthSlateSizeType_MAX
};

enum ETargetingAOEShape
{
	Box,
	Cylinder,
	Sphere,
	Capsule,
	SourceComponent,
	ETargetingAOEShape_MAX
};

enum ETargetingTraceType
{
	Line,
	Sweep,
	ETargetingTraceType_MAX
};

enum ETemplateSectionPropertyScaleType
{
	FloatProperty,
	TransformPropertyLocationOnly,
	TransformPropertyRotationOnly,
	ETemplateSectionPropertyScaleType_MAX
};

enum EFrameNumberDisplayFormats
{
	NonDropFrameTimecode,
	DropFrameTimecode,
	Seconds,
	Frames,
	MAX_Count,
	EFrameNumberDisplayFormats_MAX
};

enum ETimedDataInputState
{
	Connected,
	Unresponsive,
	Disconnected,
	ETimedDataInputState_MAX
};

enum ETimedDataInputEvaluationType
{
	None,
	Timecode,
	PlatformTime,
	ETimedDataInputEvaluationType_MAX
};

enum ETimeSynthEventQuantization
{
	None,
	Bars8,
	Bars4,
	Bars2,
	Bar,
	HalfNote,
	HalfNoteTriplet,
	QuarterNote,
	QuarterNoteTriplet,
	EighthNote,
	EighthNoteTriplet,
	SixteenthNote,
	SixteenthNoteTriplet,
	ThirtySecondNote,
	Count,
	ETimeSynthEventQuantization_MAX
};

enum ETimeSynthEnvelopeFollowerPeakMode
{
	MeanSquared,
	RootMeanSquared,
	Peak,
	Count,
	ETimeSynthEnvelopeFollowerPeakMode_MAX
};

enum ETimeSynthFilterType
{
	LowPass,
	HighPass,
	BandPass,
	BandStop,
	Count,
	ETimeSynthFilterType_MAX
};

enum ETimeSynthFilter
{
	FilterA,
	FilterB,
	Count,
	ETimeSynthFilter_MAX
};

enum ETimeSynthEventClipQuantization
{
	Global,
	None,
	Bars8,
	Bars4,
	Bars2,
	Bar,
	HalfNote,
	HalfNoteTriplet,
	QuarterNote,
	QuarterNoteTriplet,
	EighthNote,
	EighthNoteTriplet,
	SixteenthNote,
	SixteenthNoteTriplet,
	ThirtySecondNote,
	Count,
	ETimeSynthEventClipQuantization_MAX
};

enum ETimeSynthFFTSize
{
	Min_64,
	Small_256,
	Medium_512,
	Large_1024,
	ETimeSynthFFTSize_MAX
};

enum ETimeSynthBeatDivision
{
	One,
	Two,
	Four,
	Eight,
	Sixteen,
	Count,
	ETimeSynthBeatDivision_MAX
};

enum EUdpMessageFormat
{
	None,
	Json,
	TaggedProperty,
	CborPlatformEndianness,
	CborStandardEndianness,
	EUdpMessageFormat_MAX
};

enum EFlyingEvasiveManeuversMode
{
	Disabled,
	HorizontalAndVerticalWhileMoving,
	HorizontalAndVerticalWhileStandingStill,
	EFlyingEvasiveManeuversMode_MAX
};

enum EFlyingAttackMovementMode
{
	None,
	ReachTarget,
	Evade,
	EFlyingAttackMovementMode_MAX
};

enum EAITractorBeamMode
{
	None,
	TryToKidnap,
	KeepKidnapped,
	EAITractorBeamMode_MAX
};

enum EKidnapMode
{
	None,
	FromTargetingSystem,
	LowPriorityTrackedObject,
	HighPriorityTrackedObject,
	EKidnapMode_MAX
};

enum ESlateAccessibleBehavior
{
	NotAccessible,
	Auto,
	Summary,
	Custom,
	ToolTip,
	ESlateAccessibleBehavior_MAX
};

enum ESlateVisibility
{
	Visible,
	Collapsed,
	Hidden,
	HitTestInvisible,
	SelfHitTestInvisible,
	ESlateVisibility_MAX
};

enum EVirtualKeyboardType
{
	Default,
	Number,
	Web,
	Email,
	Password,
	AlphaNumeric,
	EVirtualKeyboardType_MAX
};

enum EWidgetAnimationEvent
{
	Started,
	Finished,
	EWidgetAnimationEvent_MAX
};

enum EUMGSequencePlayMode
{
	Forward,
	Reverse,
	PingPong,
	EUMGSequencePlayMode_MAX
};

enum EWidgetTickFrequency
{
	Never,
	Auto,
	EWidgetTickFrequency_MAX
};

enum EWidgetGeometryMode
{
	Plane,
	Cylinder,
	EWidgetGeometryMode_MAX
};

enum EWidgetSpace
{
	World,
	Screen,
	EWidgetSpace_MAX
};

enum EWindowVisibility
{
	Visible,
	SelfHitTestInvisible,
	EWindowVisibility_MAX
};

enum ETickMode
{
	Disabled,
	Enabled,
	Automatic,
	ETickMode_MAX
};

enum EWidgetBlendMode
{
	Opaque,
	Masked,
	Transparent,
	EWidgetBlendMode_MAX
};

enum EWidgetTimingPolicy
{
	RealTime,
	GameTime,
	EWidgetTimingPolicy_MAX
};

enum EDragPivot
{
	MouseDown,
	TopLeft,
	TopCenter,
	TopRight,
	CenterLeft,
	CenterCenter,
	CenterRight,
	BottomLeft,
	BottomCenter,
	BottomRight,
	EDragPivot_MAX
};

enum EDynamicBoxType
{
	Horizontal,
	Vertical,
	Wrap,
	VerticalWrap,
	Radial,
	Overlay,
	EDynamicBoxType_MAX
};

enum ESlateSizeRule
{
	Automatic,
	Fill,
	ESlateSizeRule_MAX
};

enum EWidgetDesignFlags
{
	None,
	Designing,
	ShowOutline,
	ExecutePreConstruct,
	EWidgetDesignFlags_MAX
};

enum EBindingKind
{
	Function,
	Property,
	EBindingKind_MAX
};

enum EWidgetInteractionSource
{
	World,
	Mouse,
	CenterScreen,
	Custom,
	EWidgetInteractionSource_MAX
};

enum ETireStates
{
	Default,
	Popped,
	ETireStates_MAX
};

enum ETireSurfaces
{
	Road,
	Dirt,
	Grass,
	Air,
	Water,
	ETireSurfaces_MAX
};

enum EFortDagwoodSimEvent
{
	EnterLandscape,
	ExitLandscape,
	EnterRoad,
	ExitRoad,
	Explode,
	FlipImpact,
	StartBoost,
	FinishBoost,
	EFortDagwoodSimEvent_MAX
};

enum EVehicleClass
{
	Sedan,
	Sport,
	PickupTruck,
	SemiTruck,
	EVehicleClass_MAX
};

enum ETireLocations
{
	FrontRight,
	FrontLeft,
	BackRight,
	BackLeft,
	ETireLocations_MAX
};

enum EPoppedTireReactionStates
{
	None,
	VeerLeft,
	VeerRight,
	Wiggle,
	Yaw90,
	FlipPitch,
	FlipRoll,
	EPoppedTireReactionStates_MAX
};

enum EFuelLeakType
{
	None,
	GenericFromBetweenRearTires,
	EFuelLeakType_MAX
};

enum EControlsPrototypes
{
	CameraSteering,
	NonCameraSteering,
	HybridCameraSteering,
	MaxCount,
	EControlsPrototypes_MAX
};

enum EVectorVMOp
{
	done,
	add,
	sub,
	mul,
	div,
	mad,
	lerp,
	rcp,
	rsq,
	sqrt,
	neg,
	abs,
	exp,
	exp2,
	log,
	log2,
	sin,
	cos,
	tan,
	asin,
	acos,
	atan,
	atan2,
	ceil,
	floor,
	fmod,
	frac,
	trunc,
	clamp,
	min,
	max,
	pow,
	round,
	sign,
	step,
	random,
	noise,
	cmplt,
	cmple,
	cmpgt,
	cmpge,
	cmpeq,
	cmpneq,
	select,
	addi,
	subi,
	muli,
	divi,
	clampi,
	mini,
	maxi,
	absi,
	negi,
	signi,
	randomi,
	cmplti,
	cmplei,
	cmpgti,
	cmpgei,
	cmpeqi,
	cmpneqi,
	bit_and,
	bit_or,
	bit_xor,
	bit_not,
	bit_lshift,
	bit_rshift,
	logic_and,
	logic_or,
	logic_xor,
	logic_not,
	f2i,
	i2f,
	f2b,
	b2f,
	i2b,
	b2i,
	inputdata_float,
	inputdata_int32,
	inputdata_half,
	inputdata_noadvance_float,
	inputdata_noadvance_int32,
	inputdata_noadvance_half,
	outputdata_float,
	outputdata_int32,
	outputdata_half,
	acquireindex,
	external_func_call,
	exec_index,
	noise2D,
	noise3D,
	enter_stat_scope,
	exit_stat_scope,
	update_id,
	acquire_id,
	NumOpcodes
};

enum EVectorVMOperandLocation
{
	Register,
	Constant,
	Num,
	EVectorVMOperandLocation_MAX
};

enum EVectorVMBaseTypes
{
	Float,
	Int,
	Bool,
	Num,
	EVectorVMBaseTypes_MAX
};

enum EVkLinkPublishMode
{
	Live,
	Playtest,
	EVkLinkPublishMode_MAX
};

enum EConsumerRole
{
	Server,
	Client,
	EConsumerRole_MAX
};

enum EVkValidationFlags
{
	None,
	VF_IgnoreValidation,
	VF_AllowMissingGameFeatureDataAsset,
	VF_AllowEngineContent,
	VF_AllowGameContent,
	VF_NoPreCheckVerse,
	EVkValidationFlags_MAX
};

enum EVoteSessionState
{
	None,
	Setup,
	Voting,
	Complete,
	Shutdown,
	EVoteSessionState_MAX
};

enum EVoteSessionNetworkType
{
	NotDetermined,
	DedicatedServer,
	MeshNetwork,
	EVoteSessionNetworkType_MAX
};

enum EVoteState
{
	None,
	Setup,
	Active,
	Completed,
	EVoteState_MAX
};

enum EBuoyancyEvent
{
	EnteredWaterBody,
	ExitedWaterBody,
	EBuoyancyEvent_MAX
};

enum EWaveSpectrumType
{
	Phillips,
	PiersonMoskowitz,
	JONSWAP,
	EWaveSpectrumType_MAX
};

enum EWaterBrushBlendType
{
	AlphaBlend,
	Min,
	Max,
	Additive
};

enum EWaterBodyType
{
	River,
	Lake,
	Ocean,
	Transition,
	Num,
	EWaterBodyType_MAX
};

enum EWaterBrushFalloffMode
{
	Angle,
	Width,
	EWaterBrushFalloffMode_MAX
};

