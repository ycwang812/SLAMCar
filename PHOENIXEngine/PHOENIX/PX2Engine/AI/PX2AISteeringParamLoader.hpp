// PX2AISteeringParamLoader.hpp

#ifndef PX2AISTEERINGPARAMLOADER_HPP
#define PX2AISTEERINGPARAMLOADER_HPP

#include "PX2Math.hpp"

namespace PX2
{

#define Prm (*ParamLoader::Instance())

	class PX2_ENGINE_ITEM ParamLoader
	{
	private:
		ParamLoader();

	public:

		static ParamLoader* Instance();

		int	NumAgents;
		int	NumObstacles;
		double MinObstacleRadius;
		double MaxObstacleRadius;

		//number of horizontal cells used for spatial partitioning
		int   NumCellsX;
		//number of vertical cells used for spatial partitioning
		int   NumCellsY;

		//how many samples the smoother will use to average a value
		int   NumSamplesForSmoothing;

		//used to tweak the combined steering force (simply altering the MaxSteeringForce
		//will NOT work!This tweaker affects all the steering force multipliers
		//too).
		double SteeringForceTweaker;

		double MaxSteeringForce;
		double MaxSpeed;
		double VehicleMass;

		double VehicleScale;
		double MaxTurnRatePerSecond;

		double SeparationWeight;
		double AlignmentWeight;
		double CohesionWeight;
		double ObstacleAvoidanceWeight;
		double WallAvoidanceWeight;
		double WanderWeight;
		double SeekWeight;
		double FleeWeight;
		double ArriveWeight;
		double PursuitWeight;
		double OffsetPursuitWeight;
		double InterposeWeight;
		double HideWeight;
		double EvadeWeight;
		double FollowPathWeight;

		//how close a neighbour must be before an agent perceives it (considers it
		//to be within its neighborhood)
		double ViewDistance;

		//used in obstacle avoidance
		double MinDetectionBoxLength;

		//used in wall avoidance
		double WallDetectionFeelerLength;

		//these are the probabilities that a steering behavior will be used
		//when the prioritized dither calculate method is used
		double prWallAvoidance;
		double prObstacleAvoidance;
		double prSeparation;
		double prAlignment;
		double prCohesion;
		double prWander;
		double prSeek;
		double prFlee;
		double prEvade;
		double prHide;
		double prArrive;
	};
}

#endif