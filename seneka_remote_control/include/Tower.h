#ifndef TOWER_H
#define TOWER_H

#include <ros/ros.h>

class Tower
{
	private:

		enum Mode
		{
			SINGLE,	// single entire trunk rotation
			CUSTOM,	// single trunk rotation to target position
			ENDLESS	// endless trunk rotation
		};

		enum Direction
		{
			POSITIVE,	// trunk rotates in mathematical positive direction
			NEGATIVE	// trunk rotates in mathematical negative direction
		};

		bool		bEnable;			// start/stop switch
		bool		bTurnLeft;
		bool		bTurnRight;
		float		fVelocity;			// [] = %
		int			nTarget_position;	// [] = deg
		int			nStepSize;			// [] = deg
		Mode		eMode;
		Direction	eDirection;

	public:

		Tower();
};

#endif