/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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

#include "Trajectory.h"
#include <limits>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

const double Trajectory::eps = 0.000001;

static double squared(double d) {
	return d * d;
}

Trajectory::Trajectory(const Path &path, const VectorXd &maxVelocity, const VectorXd &maxAcceleration, double timeStep) :
	path(path),
	maxVelocity(maxVelocity),
	maxAcceleration(maxAcceleration),
	n(maxVelocity.size()),
	valid(true),
	timeStep(timeStep),
	cachedTime(numeric_limits<double>::max())
{
	trajectory.push_back(TrajectoryStep(0.0, 0.0));
	double afterAcceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
	while(valid && !integrateForward(trajectory, afterAcceleration) && valid) {
		double beforeAcceleration;
		TrajectoryStep switchingPoint;
		if(getNextSwitchingPoint(trajectory.back().pathPos, switchingPoint, beforeAcceleration, afterAcceleration)) {
			break;
		}
		integrateBackward(trajectory, switchingPoint.pathPos, switchingPoint.pathVel, beforeAcceleration);
	}

	if(valid) {
		double beforeAcceleration = getMinMaxPathAcceleration(path.getLength(), 0.0, false);
		integrateBackward(trajectory, path.getLength(), 0.0, beforeAcceleration);
	}

	if(valid) {
		// calculate timing
		list<TrajectoryStep>::iterator previous = trajectory.begin();
		list<TrajectoryStep>::iterator it = previous;
		it->time = 0.0;
		it++;
		while(it != trajectory.end()) {
			it->time = previous->time + (it->pathPos - previous->pathPos) / ((it->pathVel + previous->pathVel) / 2.0);
			previous = it;
			it++;
		}
	}
}

Trajectory::~Trajectory(void) {
}

void Trajectory::outputPhasePlaneTrajectory() const {
	ofstream file1("maxVelocity.txt");
	const double stepSize = path.getLength() / 100000.0;
	for(double s = 0.0; s < path.getLength(); s += stepSize) {
		double maxVelocity = getAccelerationMaxPathVelocity(s);
		if(maxVelocity == numeric_limits<double>::infinity())
			maxVelocity = 10.0;
		file1 << s << "  " << maxVelocity << "  " << getVelocityMaxPathVelocity(s) << endl;
	}
	file1.close();

	ofstream file2("trajectory.txt");
	for(list<TrajectoryStep>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) {
		file2 << it->pathPos << "  " << it->pathVel << endl;
	}
	for(list<TrajectoryStep>::const_iterator it = endTrajectory.begin(); it != endTrajectory.end(); it++) {
		file2 << it->pathPos << "  " << it->pathVel << endl;
	}
	file2.close();
}

// returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	TrajectoryStep accelerationSwitchingPoint(pathPos, 0.0);
	double accelerationBeforeAcceleration, accelerationAfterAcceleration;
	bool accelerationReachedEnd;
	do {
		accelerationReachedEnd = getNextAccelerationSwitchingPoint(accelerationSwitchingPoint.pathPos, accelerationSwitchingPoint, accelerationBeforeAcceleration, accelerationAfterAcceleration);
		double test = getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos);
	} while(!accelerationReachedEnd && accelerationSwitchingPoint.pathVel > getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos));
	
	TrajectoryStep velocitySwitchingPoint(pathPos, 0.0);
	double velocityBeforeAcceleration, velocityAfterAcceleration;
	bool velocityReachedEnd;
	do {
		velocityReachedEnd = getNextVelocitySwitchingPoint(velocitySwitchingPoint.pathPos, velocitySwitchingPoint, velocityBeforeAcceleration, velocityAfterAcceleration);
	} while(!velocityReachedEnd && velocitySwitchingPoint.pathPos <= accelerationSwitchingPoint.pathPos
		&& (velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos - eps)
		|| velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos + eps)));

	if(accelerationReachedEnd && velocityReachedEnd) {
		return true;
	}
	else if(!accelerationReachedEnd && (velocityReachedEnd || accelerationSwitchingPoint.pathPos <= velocitySwitchingPoint.pathPos)) {
		nextSwitchingPoint = accelerationSwitchingPoint;
		beforeAcceleration = accelerationBeforeAcceleration;
		afterAcceleration = accelerationAfterAcceleration;
		return false;
	}
	else {
		nextSwitchingPoint = velocitySwitchingPoint;
		beforeAcceleration = velocityBeforeAcceleration;
		afterAcceleration = velocityAfterAcceleration;
		return false;
	}
}

bool Trajectory::getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	double switchingPathPos = pathPos;
	double switchingPathVel;
	while(true) {
		bool discontinuity;
		switchingPathPos = path.getNextSwitchingPoint(switchingPathPos, discontinuity);

		if(switchingPathPos > path.getLength() - eps) {
			return true;
		}
		
		if(discontinuity) {
			const double beforePathVel = getAccelerationMaxPathVelocity(switchingPathPos - eps);
			const double afterPathVel = getAccelerationMaxPathVelocity(switchingPathPos + eps);
			switchingPathVel = min(beforePathVel, afterPathVel);
			beforeAcceleration = getMinMaxPathAcceleration(switchingPathPos - eps, switchingPathVel, false);
			afterAcceleration = getMinMaxPathAcceleration(switchingPathPos + eps, switchingPathVel, true);
			
			if((beforePathVel > afterPathVel
				|| getMinMaxPhaseSlope(switchingPathPos - eps, switchingPathVel, false) > getAccelerationMaxPathVelocityDeriv(switchingPathPos - 2.0*eps))
				&& (beforePathVel < afterPathVel
				|| getMinMaxPhaseSlope(switchingPathPos + eps, switchingPathVel, true) < getAccelerationMaxPathVelocityDeriv(switchingPathPos + 2.0*eps)))
			{
				break;
			}
		}
		else {
			switchingPathVel = getAccelerationMaxPathVelocity(switchingPathPos);
			beforeAcceleration = 0.0;
			afterAcceleration = 0.0;

			if(getAccelerationMaxPathVelocityDeriv(switchingPathPos - eps) < 0.0 && getAccelerationMaxPathVelocityDeriv(switchingPathPos + eps) > 0.0) {
				break;
			}
		}
	}
	
	nextSwitchingPoint = TrajectoryStep(switchingPathPos, switchingPathVel);
	return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
	const double stepSize = 0.001;
	const double accuracy = 0.000001;

	bool start = false;
	pathPos -= stepSize;
	do {
		pathPos += stepSize;


		if(getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) >= getVelocityMaxPathVelocityDeriv(pathPos)) {
			start = true;
		}
	} while((!start || getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) > getVelocityMaxPathVelocityDeriv(pathPos))
		&& pathPos < path.getLength());

	if(pathPos >= path.getLength()) {
		return true; // end of trajectory reached
	}

	double beforePathPos = pathPos - stepSize;
	double afterPathPos = pathPos;
	while(afterPathPos - beforePathPos > accuracy) {
		pathPos = (beforePathPos + afterPathPos) / 2.0;
		if(getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) > getVelocityMaxPathVelocityDeriv(pathPos)) {
			beforePathPos = pathPos;
		}
		else {
			afterPathPos = pathPos;
		}
	}

	beforeAcceleration = getMinMaxPathAcceleration(beforePathPos, getVelocityMaxPathVelocity(beforePathPos), false);
	afterAcceleration = getMinMaxPathAcceleration(afterPathPos, getVelocityMaxPathVelocity(afterPathPos), true);
	nextSwitchingPoint = TrajectoryStep(afterPathPos, getVelocityMaxPathVelocity(afterPathPos));
	return false;
}

// returns true if end of path is reached
bool Trajectory::integrateForward(list<TrajectoryStep> &trajectory, double acceleration) {
	
	double pathPos = trajectory.back().pathPos;
	double pathVel = trajectory.back().pathVel;
	
	list<pair<double, bool> > switchingPoints = path.getSwitchingPoints();
	list<pair<double, bool> >::iterator nextDiscontinuity = switchingPoints.begin();

	while(true)
	{
		while(nextDiscontinuity != switchingPoints.end() && (nextDiscontinuity->first <= pathPos || !nextDiscontinuity->second)) {
			nextDiscontinuity++;
		}

		double oldPathPos = pathPos;
		double oldPathVel = pathVel;
		
		pathVel += timeStep * acceleration;
		pathPos += timeStep * 0.5 * (oldPathVel + pathVel);

		if(nextDiscontinuity != switchingPoints.end() && pathPos > nextDiscontinuity->first) {
			pathVel = oldPathVel + (nextDiscontinuity->first - oldPathPos) * (pathVel - oldPathVel) / (pathPos - oldPathPos);
			pathPos = nextDiscontinuity->first;
		}

		if(pathPos > path.getLength()) {
			trajectory.push_back(TrajectoryStep(pathPos, pathVel));
			return true;
		}
		else if(pathVel < 0.0) {
			valid = false;
			cout << "error" << endl;
			return true;
		}

		if(pathVel > getVelocityMaxPathVelocity(pathPos)
			&& getMinMaxPhaseSlope(oldPathPos, getVelocityMaxPathVelocity(oldPathPos), false) <= getVelocityMaxPathVelocityDeriv(oldPathPos))
		{
			pathVel = getVelocityMaxPathVelocity(pathPos);
		}

		trajectory.push_back(TrajectoryStep(pathPos, pathVel));
		acceleration = getMinMaxPathAcceleration(pathPos, pathVel, true);

		if(pathVel > getAccelerationMaxPathVelocity(pathPos) || pathVel > getVelocityMaxPathVelocity(pathPos)) {
			// find more accurate intersection with max-velocity curve using bisection
			TrajectoryStep overshoot = trajectory.back();
			trajectory.pop_back();
			double before = trajectory.back().pathPos;
			double beforePathVel = trajectory.back().pathVel;
			double after = overshoot.pathPos;
			double afterPathVel = overshoot.pathVel;
			while(after - before > eps) {
				const double midpoint = 0.5 * (before + after);
				double midpointPathVel = 0.5 * (beforePathVel + afterPathVel);

				if(midpointPathVel > getVelocityMaxPathVelocity(midpoint)
					&& getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <= getVelocityMaxPathVelocityDeriv(before))
				{
					midpointPathVel = getVelocityMaxPathVelocity(midpoint);
				}

				if(midpointPathVel > getAccelerationMaxPathVelocity(midpoint) || midpointPathVel > getVelocityMaxPathVelocity(midpoint)) {
					after = midpoint;
					afterPathVel = midpointPathVel;
				}
				else {
					before = midpoint;
					beforePathVel = midpointPathVel;
				}
			}
			trajectory.push_back(TrajectoryStep(before, beforePathVel));
		
			if(getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after)) {
				if(after > nextDiscontinuity->first) {
					return false;
				}
				else if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, true) > getAccelerationMaxPathVelocityDeriv(trajectory.back().pathPos)) {
					return false;
				}
			}
			else {
				if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, false) > getVelocityMaxPathVelocityDeriv(trajectory.back().pathPos)) {
					return false;
				}
			}
		}
	}
}

void Trajectory::integrateBackward(list<TrajectoryStep> &startTrajectory, double pathPos, double pathVel, double acceleration) {
	list<TrajectoryStep>::iterator start2 = startTrajectory.end();
	start2--;
	list<TrajectoryStep>::iterator start1 = start2;
	start1--;
	list<TrajectoryStep> trajectory;
	double slope;
	assert(start1->pathPos <= pathPos);

	while(start1 != startTrajectory.begin() || pathPos >= 0.0)
	{
		if(start1->pathPos <= pathPos) {
			trajectory.push_front(TrajectoryStep(pathPos, pathVel));
			pathVel -= timeStep * acceleration;
			pathPos -= timeStep * 0.5 * (pathVel + trajectory.front().pathVel);
			acceleration = getMinMaxPathAcceleration(pathPos, pathVel, false);
			slope = (trajectory.front().pathVel - pathVel) / (trajectory.front().pathPos - pathPos);
			
			if(pathVel < 0.0) {
				valid = false;
				cout << "Error while integrating backward: Negative path velocity" << endl;
				endTrajectory = trajectory;
				return;
			}
		}
		else {
			start1--;
			start2--;
		}

		// check for intersection between current start trajectory and backward trajectory segments
		const double startSlope = (start2->pathVel - start1->pathVel) / (start2->pathPos - start1->pathPos);
		const double intersectionPathPos = (start1->pathVel - pathVel + slope * pathPos - startSlope * start1->pathPos) / (slope - startSlope);
		if(max(start1->pathPos, pathPos) - eps <= intersectionPathPos && intersectionPathPos <= eps + min(start2->pathPos, trajectory.front().pathPos)) {
			const double intersectionPathVel = start1->pathVel + startSlope * (intersectionPathPos - start1->pathPos);
			startTrajectory.erase(start2, startTrajectory.end());
			startTrajectory.push_back(TrajectoryStep(intersectionPathPos, intersectionPathVel));
			startTrajectory.splice(startTrajectory.end(), trajectory);
			return;
		}
	}

	valid = false;
	cout << "Error while integrating backward: Did not hit start trajectory" << endl;
	endTrajectory = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double pathPos, double pathVel, bool max) {
	VectorXd configDeriv = path.getTangent(pathPos);
	VectorXd configDeriv2 = path.getCurvature(pathPos);
	double factor = max ? 1.0 : -1.0;
	double maxPathAcceleration = numeric_limits<double>::max();
	for(unsigned int i = 0; i < n; i++) {
		if(configDeriv[i] != 0.0) {
			maxPathAcceleration = min(maxPathAcceleration,
				maxAcceleration[i]/abs(configDeriv[i]) - factor * configDeriv2[i] * pathVel*pathVel / configDeriv[i]);
		}
	}
	return factor * maxPathAcceleration;
}

double Trajectory::getMinMaxPhaseSlope(double pathPos, double pathVel, bool max) {
	return getMinMaxPathAcceleration(pathPos, pathVel, max) / pathVel;
}

double Trajectory::getAccelerationMaxPathVelocity(double pathPos) const {
	double maxPathVelocity = numeric_limits<double>::infinity();
	const VectorXd configDeriv = path.getTangent(pathPos);
	const VectorXd configDeriv2 = path.getCurvature(pathPos);
	for(unsigned int i = 0; i < n; i++) {
		if(configDeriv[i] != 0.0) {
			for(unsigned int j = i + 1; j < n; j++) {
				if(configDeriv[j] != 0.0) {
					double A_ij = configDeriv2[i] / configDeriv[i] - configDeriv2[j] / configDeriv[j];
					if(A_ij != 0.0) {
						maxPathVelocity = min(maxPathVelocity,
							sqrt((maxAcceleration[i] / abs(configDeriv[i]) + maxAcceleration[j] / abs(configDeriv[j]))
							/ abs(A_ij)));
					}
				}
			}
		}
		else if(configDeriv2[i] != 0.0) {
			maxPathVelocity = min(maxPathVelocity, sqrt(maxAcceleration[i] / abs(configDeriv2[i])));
		}
	}
	return maxPathVelocity;
}


double Trajectory::getVelocityMaxPathVelocity(double pathPos) const {
	const VectorXd tangent = path.getTangent(pathPos);
	double maxPathVelocity = numeric_limits<double>::max();
	for(unsigned int i = 0; i < n; i++) {
		maxPathVelocity = min(maxPathVelocity, maxVelocity[i] / abs(tangent[i]));
	}
	return maxPathVelocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double pathPos) {
	return (getAccelerationMaxPathVelocity(pathPos + eps) - getAccelerationMaxPathVelocity(pathPos - eps)) / (2.0 * eps);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double pathPos) {
	const VectorXd tangent = path.getTangent(pathPos);
	double maxPathVelocity = numeric_limits<double>::max();
	unsigned int activeConstraint;
	for(unsigned int i = 0; i < n; i++) {
		const double thisMaxPathVelocity = maxVelocity[i] / abs(tangent[i]);
		if(thisMaxPathVelocity < maxPathVelocity) {
			maxPathVelocity = thisMaxPathVelocity;
			activeConstraint = i;
		}
	}
	return - (maxVelocity[activeConstraint] * path.getCurvature(pathPos)[activeConstraint])
		/ (tangent[activeConstraint] * abs(tangent[activeConstraint]));
}

bool Trajectory::isValid() const {
	return valid;
}

double Trajectory::getDuration() const {
	return trajectory.back().time;
}

list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const {
	if(time >= trajectory.back().time) {
		list<TrajectoryStep>::const_iterator last = trajectory.end();
		last--;
		return last;
	}
	else {
		if(time < cachedTime) {
			cachedTrajectorySegment = trajectory.begin();
		}
		while(time >= cachedTrajectorySegment->time) {
			cachedTrajectorySegment++;
		}
		cachedTime = time;
		return cachedTrajectorySegment;
	}
}

VectorXd Trajectory::getPosition(double time) const {
	list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
	list<TrajectoryStep>::const_iterator previous = it;
	previous--;
	
	double timeStep = it->time - previous->time;
	const double acceleration = 2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

	timeStep = time - previous->time;
	const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration; 
	
	return path.getConfig(pathPos);
}

VectorXd Trajectory::getVelocity(double time) const {
	list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
	list<TrajectoryStep>::const_iterator previous = it;
	previous--;
		
	double timeStep = it->time - previous->time;
	const double acceleration = 2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

	timeStep = time - previous->time;
	const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration; 
	const double pathVel = previous->pathVel + timeStep * acceleration;
	
	return path.getTangent(pathPos) * pathVel;
}
