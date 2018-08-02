/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>
#include "Path.h"

class Trajectory
{
public:
	// Generates a time-optimal trajectory
	Trajectory(const Path &path, const Eigen::VectorXd &maxVelocity, const Eigen::VectorXd &maxAcceleration, double timeStep = 0.001);
	
	~Trajectory(void);

	// Call this method after constructing the object to make sure the trajectory generation succeeded without errors.
	// If this method returns false, all other methods have undefined behavior.
	bool isValid() const;

	// Returns the optimal duration of the trajectory
	double getDuration() const;

	// Return the position/configuration or velocity vector of the robot for a given point in time within the trajectory.
	Eigen::VectorXd getPosition(double time) const;
	Eigen::VectorXd getVelocity(double time) const;

	// Outputs the phase trajectory and the velocity limit curve in 2 files for debugging purposes.
	void outputPhasePlaneTrajectory() const;

private:
	struct TrajectoryStep {
		TrajectoryStep() {}
		TrajectoryStep(double pathPos, double pathVel) :
			pathPos(pathPos),
			pathVel(pathVel)
		{}
		double pathPos;
		double pathVel;
		double time;
	};

	bool getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool integrateForward(std::list<TrajectoryStep> &trajectory, double acceleration);
	void integrateBackward(std::list<TrajectoryStep> &startTrajectory, double pathPos, double pathVel, double acceleration);
	double getMinMaxPathAcceleration(double pathPosition, double pathVelocity, bool max);
	double getMinMaxPhaseSlope(double pathPosition, double pathVelocity, bool max);
	double getAccelerationMaxPathVelocity(double pathPos) const;
	double getVelocityMaxPathVelocity(double pathPos) const;
	double getAccelerationMaxPathVelocityDeriv(double pathPos);
	double getVelocityMaxPathVelocityDeriv(double pathPos);
	
	std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;
	
	Path path;
	Eigen::VectorXd maxVelocity;
	Eigen::VectorXd maxAcceleration;
	unsigned int n;
	bool valid;
	std::list<TrajectoryStep> trajectory;
	std::list<TrajectoryStep> endTrajectory; // non-empty only if the trajectory generation failed.

	static const double eps;
	const double timeStep;

	mutable double cachedTime;
	mutable std::list<TrajectoryStep>::const_iterator cachedTrajectorySegment;
};