// PX2AISteeringParamLoader.cpp

#include "PX2AISteeringParamLoader.hpp"
using namespace PX2;

ParamLoader::ParamLoader()
{
	NumAgents = 300;
	NumObstacles = 7;
	MinObstacleRadius = 10;
	MaxObstacleRadius = 30;

	NumCellsX = 7;
	NumCellsY = 7;

	NumSamplesForSmoothing = 10;

	SteeringForceTweaker = 1.0f;
	MaxSteeringForce = 1.0f * SteeringForceTweaker;
	MaxSpeed = 0.2f;
	VehicleMass = 1.0f;
	VehicleScale = 3.0f;

	SeparationWeight = 1.0f * SteeringForceTweaker;
	AlignmentWeight = 1.0f * SteeringForceTweaker;
	CohesionWeight = 2.0f * SteeringForceTweaker;
	ObstacleAvoidanceWeight = 5.0f * SteeringForceTweaker;
	WallAvoidanceWeight = 1.0f * SteeringForceTweaker;
	WanderWeight = 1.0f * SteeringForceTweaker;
	SeekWeight = 1.0f * SteeringForceTweaker;
	FleeWeight = 1.0f * SteeringForceTweaker;
	ArriveWeight = 1.0f * SteeringForceTweaker;
	PursuitWeight = 1.0f * SteeringForceTweaker;
	OffsetPursuitWeight = 1.0f * SteeringForceTweaker;
	InterposeWeight = 1.0f * SteeringForceTweaker;
	HideWeight = 1.0f * SteeringForceTweaker;
	EvadeWeight = 1.0f * SteeringForceTweaker;
	FollowPathWeight = 10.0f * SteeringForceTweaker;

	ViewDistance = 50.0f;
	MinDetectionBoxLength = 0.4f;
	WallDetectionFeelerLength = 40.0f;

	prWallAvoidance = 0.5f;
	prObstacleAvoidance = 0.5f;
	prSeparation = 0.2f;
	prAlignment = 0.3f;
	prCohesion = 0.6f;
	prWander = 0.8f;
	prSeek = 0.8f;
	prFlee = 0.6f;
	prEvade = 1.0f;
	prHide = 0.8f;
	prArrive =0.5f;

	MaxTurnRatePerSecond = Mathf::PI;
}
//------------------------------------------------------------------------
ParamLoader* ParamLoader::Instance()
{
	static ParamLoader instance;
	return &instance;
}
//------------------------------------------------------------------------