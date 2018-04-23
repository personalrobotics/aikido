/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
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
 *
 * This file implements a steering method for the double-integrator minimum
 * time problem as described in section IV of [1].
 *
 * [1] Tobias Kunz, Mike Stilman. Probabilistically Complete Kinodynamic
 *     Planning for Robot Manipulators with Acceleration Limits. IEEE/RSJ
 *     International Conference on Intelligent Robots and Systems (IROS), 2014.
 */
// TODO: Compare with original code to catch non-architectural changes

#pragma once

#include <iostream>
#include <utility>
#include <array>
#include <cmath>
#include <Eigen/Core>
#include <tuple>

template <int dof>
class DoubleIntegrator
{
public:
    typedef Eigen::Matrix<double, 2 * dof, 1> StateVector;
    typedef Eigen::Matrix<double, dof, 1> Vector;
    Vector maxAccelerations_;
    Vector maxVelocities_;

    DoubleIntegrator(Vector maxAccelerations, Vector maxVelocities)
      : maxAccelerations_(maxAccelerations), maxVelocities_(maxVelocities)
    {
    }

    class Trajectory1D
    {
        // 3 segments of constant acceleration: position/velocity[i] :=
        // position/velocity at time[i]
        double times[4];
        double positions[4];
        double velocities[4];

    public:
        int getSegment(double t) const
        {
            assert(0.0 <= t);

            int i = 0;
            while (i < 3 && t >= times[i + 1])
                i++;
            return i;
        }

        void getState(double t, double &position, double &velocity) const
        {
            int i = getSegment(t);
            if (i == 3)
            {
                position = positions[3];
                velocity = velocities[3];
            }
            else
            {
                const double s = (t - times[i]) / (times[i + 1] - times[i]);
                velocity = (1.0 - s) * velocities[i] + s * velocities[i + 1];
                position = (1.0 - s * s) * positions[i] + s * s * positions[i + 1] +
                           (t - times[i]) * (1.0 - s) * velocities[i];
            }
            assert(velocity == velocity);
            assert(position == position);
        }

        void setStartAndGoal(double startPosition, double startVelocity, double goalPosition, double goalVelocity)
        {
            positions[0] = startPosition;
            velocities[0] = startVelocity;
            positions[3] = goalPosition;
            velocities[3] = goalVelocity;
        }

        void setPPTrajectory(double duration1, double duration2, double acceleration1)
        {
            setPLPTrajectory(duration1, 0.0, duration2, velocities[0] + duration1 * acceleration1);
        }

        void setPLPTrajectory(double duration1, double duration2, double duration3, double limitVelocity)
        {
            // Check for NaNs
            assert(duration1 == duration1);
            assert(duration2 == duration2);
            assert(duration3 == duration3);
            assert(limitVelocity == limitVelocity);

            times[0] = 0.0;
            times[1] = duration1;
            times[2] = duration1 + duration2;
            times[3] = duration1 + duration2 + duration3;

            velocities[1] = velocities[2] = limitVelocity;
            positions[1] = positions[0] + 0.5 * (velocities[0] + velocities[1]) * duration1;
            positions[2] = positions[3] - 0.5 * (velocities[2] + velocities[3]) * duration3;
        }

        double getDuration() const
        {
            return times[3];
        }

        bool satisfiesVelocityLimits(double minVelocity, double maxVelocity) const
        {
            return (minVelocity <= velocities[1] && velocities[1] <= maxVelocity);
        }

        bool satisfiesPositionLimits(double minPosition, double maxPosition) const
        {
            if (velocities[0] * velocities[1] < 0.0)
            {
                const double startExtremalPosition =
                    positions[0] + 0.5 * velocities[0] * velocities[0] / (velocities[0] - velocities[1]) * times[1];
                if (startExtremalPosition < minPosition || maxPosition < startExtremalPosition)
                    return false;
            }

            if (velocities[2] * velocities[3] < 0.0)
            {
                const double goalExtremalPosition =
                    positions[3] -
                    0.5 * velocities[3] * velocities[3] / (velocities[3] - velocities[2]) * (times[3] - times[2]);
                if (goalExtremalPosition < minPosition || maxPosition < goalExtremalPosition)
                    return false;
            }

            return true;
        }
    };

    struct Trajectory : public std::array<Trajectory1D, dof>
    {
        double getDuration() const
        {
            return (*this)[0].getDuration();
        }

        StateVector getState(double time) const
        {
            StateVector state;
            for (int i = 0; i < dof; i++)
            {
                (*this)[i].getState(time, state[i], state[dof + i]);
            }
            return state;
        }
    };

    // private:

    static inline double sign(double d)
    {
        return d >= 0.0 ? 1.0 : -1.0;
    }

    static inline double squared(double d)
    {
        return d * d;
    }

    static void getPPTime(double startVelocity, double goalVelocity, double distance, double acceleration1,
                          double *tMin1, double *tMin2, double *tMax1, double *tMax2)
    {
        if (startVelocity == goalVelocity && distance == 0.0)
        {
            if (tMin1)
            {
                *tMin1 = 0.0;
                *tMin2 = 0.0;
            }
            if (tMax1)
            {
                *tMax1 = 0.0;
                *tMax2 = 0.0;
            }
            return;
        }

        const double a = acceleration1;
        const double b = 2.0 * startVelocity;
        // const double c = (squared(goalVelocity) - squared(startVelocity)) / 2.0 /
        // acceleration2 - distance;
        const double c =
            0.5 * (startVelocity + goalVelocity) * ((goalVelocity - startVelocity) / -acceleration1) - distance;

        const double radicand = b * b - 4.0 * a * c;
        assert(radicand >= 0.0);

        // numerically stable solution to the quadratic equation
        const double q = -0.5 * (b + sign(b) * sqrt(radicand));

        if (a * sign(b) >= 0.0)
        {
            if (tMin1)
                *tMin1 = q / a;
            if (tMax1)
                *tMax1 = c / q;
        }
        else
        {
            if (tMin1)
                *tMin1 = c / q;
            if (tMax1)
                *tMax1 = q / a;
        }

        if (tMin1)
        {
            *tMin2 = *tMin1 - (goalVelocity - startVelocity) / acceleration1;
            if (*tMin1 < 0.0 || *tMin2 < 0.0)
            {
                *tMin1 = std::numeric_limits<double>::infinity();
                *tMin2 = std::numeric_limits<double>::infinity();
            }
        }

        if (tMax1)
        {
            *tMax2 = *tMax1 - (goalVelocity - startVelocity) / acceleration1;
            if (*tMax1 < 0.0 || *tMax2 < 0.0)
            {
                *tMax1 = std::numeric_limits<double>::infinity();
                *tMax2 = std::numeric_limits<double>::infinity();
            }
        }
    }

    static double getPLPTime(double startVelocity, double goalVelocity, double distance, double maxVelocity,
                             double acceleration1)
    {
        // const bool check = std::abs(startVelocity) <= maxVelocity &&
        // std::abs(goalVelocity) <= maxVelocity;
        // if(!check)
        // {
        // 	std::cout << "Start velocity: " << startVelocity << " | Goal velocity: "
        // 		<< goalVelocity << " | Max velocity: " << maxVelocity <<
        // std::endl;
        // }
        // assert(std::abs(startVelocity) <= maxVelocity && std::abs(goalVelocity)
        // <= maxVelocity);

        const double boundaryVelocity = sign(acceleration1) * maxVelocity;
        const double timeToBoundary1 = (boundaryVelocity - startVelocity) / acceleration1;
        const double timeToBoundary2 = (goalVelocity - boundaryVelocity) / -acceleration1;
        const double distanceToBoundary =
            0.5 * (2.0 * squared(boundaryVelocity) - squared(startVelocity) - squared(goalVelocity)) / acceleration1;
        const double boundaryTime = (distance - distanceToBoundary) / boundaryVelocity;

        if (boundaryTime < 0.0)
            return std::numeric_limits<double>::infinity();
        else
        {
            return timeToBoundary1 + boundaryTime + timeToBoundary2;
        }
    }

    static Trajectory1D getMinAcceleration(double startPosition, double startVelocity, double goalPosition,
                                           double goalVelocity, double time, double maxVelocity)
    {
        Trajectory1D trajectory;
        trajectory.setStartAndGoal(startPosition, startVelocity, goalPosition, goalVelocity);

        const double a = time * time;
        const double b = 2.0 * (startVelocity + goalVelocity) * time - 4.0 * (goalPosition - startPosition);
        const double c = -squared(goalVelocity - startVelocity);

        if (b == 0.0 && c == 0.0)
        {
            trajectory.setPLPTrajectory(0.0, time, 0.0, startVelocity);
            return trajectory;
        }

        // numerically stable solution to the quadratic equation
        const double acceleration = (-b - sign(b) * sqrt(b * b - 4.0 * a * c)) / (2.0 * a);

        const double t1 = 0.5 * ((goalVelocity - startVelocity) / acceleration + time);
        assert(t1 >= -0.000001);
        assert(t1 <= time + 0.000001);

        trajectory.setPPTrajectory(t1, time - t1, acceleration);

        if (!trajectory.satisfiesVelocityLimits(-maxVelocity, maxVelocity))
        {
            // Calculate PLP solution
            const double velocityLimit = sign(acceleration) * maxVelocity;
            if (startVelocity == velocityLimit && goalVelocity == velocityLimit)
            {
                trajectory.setPLPTrajectory(0.0, time, 0.0, velocityLimit);
                return trajectory;
            }
            const double acceleration =
                0.5 * (squared(velocityLimit - startVelocity) + squared(goalVelocity - velocityLimit)) /
                (velocityLimit * time - (goalPosition - startPosition));
            const double t1 = (velocityLimit - startVelocity) / acceleration;
            const double t2 = (goalVelocity - velocityLimit) / -acceleration;
            assert(t1 == t1);
            trajectory.setPLPTrajectory(t1, time - t1 - t2, t2, velocityLimit);
        }
        return trajectory;
    }

    static void adjustForInfeasibleIntervals(int i, double time, double maxTime,
                                             //const Eigen::Ref<const Vector> &startVelocities,
                                             //const Eigen::Ref<const Vector> &goalVelocities,
                                             //const Eigen::Ref<const Vector> &distances,
                                             //const Eigen::Ref<const Vector> &firstAccelerations,
                                             //const Eigen::Ref<const Vector> &maxAccelerations,
                                             //const Eigen::Ref<const Vector> &maxVelocities,
                                             const Vector& startVelocities,
                                             const Vector& goalVelocities,
                                             const Vector& distances,
                                             const Vector& firstAccelerations,
                                             const Vector& maxAccelerations,
                                             const Vector& maxVelocities,
                                             double &minTime,
                                             std::pair<double, double> *infeasibleIntervals, int &limitDof)
    {
        if (time >= maxTime)
        {
            minTime = time;
            return;
        }

        if (time < minTime)
        {
            infeasibleIntervals[i] =
                getInfeasibleInterval(startVelocities[i], goalVelocities[i], distances[i], firstAccelerations[i],
                                      maxAccelerations[i], maxVelocities[i]);
        }
        else
        {
            minTime = time;
            if (limitDof != -1)
            {
                infeasibleIntervals[limitDof] = getInfeasibleInterval(
                    startVelocities[limitDof], goalVelocities[limitDof], distances[limitDof],
                    firstAccelerations[limitDof], maxAccelerations[limitDof], maxVelocities[limitDof]);
            }
            limitDof = i;
        }

        int j = 0;
        while (j <= i && minTime < maxTime)
        {
            if (j != limitDof)
            {
                if (infeasibleIntervals[j].first < minTime && minTime < infeasibleIntervals[j].second)
                {
                    minTime = infeasibleIntervals[j].second;
                    j = 0;
                    if (limitDof != -1)
                    {
                        infeasibleIntervals[limitDof] = getInfeasibleInterval(
                            startVelocities[limitDof], goalVelocities[limitDof], distances[limitDof],
                            firstAccelerations[limitDof], maxAccelerations[limitDof], maxVelocities[limitDof]);
                        limitDof = -1;
                    }
                }
            }
            j++;
        }
    }

public:
    static double getMinTime1D(double startVelocity, double goalVelocity, double distance, double maxAcceleration,
                               double maxVelocity, double maxTime, double &acceleration1)
    {
        // determine distance travelled while accelerating from start velocity to
        // goal velocity
        const double accelerationTime = std::abs(goalVelocity - startVelocity) / maxAcceleration;
        if (accelerationTime >= maxTime)
            return accelerationTime;
        const double accelerationDistance = 0.5 * (startVelocity + goalVelocity) * accelerationTime;

        // determine whether to accelerate at the upper or lower limit first
        const double additionalDistance = distance - accelerationDistance;

        acceleration1 = sign(additionalDistance) * maxAcceleration;

        double time1, time2;
        getPPTime(startVelocity, goalVelocity, distance, acceleration1, NULL, NULL, &time1, &time2);
        double time = time1 + time2;

        assert(time >= 0.0);
        assert(time != std::numeric_limits<double>::infinity());
        // if(time == std::numeric_limits<double>::infinity())
        // time = 100;
        if (time < maxTime && std::abs(startVelocity + acceleration1 * time1) >= maxVelocity)
        {
            //std::cout << "DI violate the velocity limit " << std::abs(startVelocity + acceleration1 * time1) << std::endl;
            time = getPLPTime(startVelocity, goalVelocity, distance, maxVelocity, acceleration1);
        }
        // if(time == std::numeric_limits<double>::infinity())
        // time = 100;
        // std::cout << "T1= " << time1 << " T2=" << time2 << " Total=" << time <<
        // std::endl;
        return time;
    }

    static std::pair<double, double> getInfeasibleInterval(double startVelocity, double goalVelocity, double distance,
                                                           double acceleration1, double maxAcceleration,
                                                           double maxVelocity)
    {
        std::pair<double, double> infeasibleInterval;

        if (startVelocity * goalVelocity <= 0.0 || acceleration1 * startVelocity < 0.0)
        {
            // If minimum-time solution goes through zero-velocity, there is no
            // infeasible time interval, because we can stop and wait at zero-velocity
            infeasibleInterval.first = std::numeric_limits<double>::infinity();
            infeasibleInterval.second = std::numeric_limits<double>::infinity();
        }
        // for all cases below: sign(startVelocity) == sign(goalVelocity) ==
        // sign(distance) == sign(additionalDistance) == sign(acceleration1) ==
        // -sign(acceleration2)
        else
        {
            double zeroTime1 = std::abs(startVelocity) / maxAcceleration;
            double zeroTime2 = std::abs(goalVelocity) / maxAcceleration;
            double zeroDistance = zeroTime1 * startVelocity / 2.0 + zeroTime2 * goalVelocity / 2.0;
            if (std::abs(zeroDistance) < std::abs(distance))
            {
                infeasibleInterval.first = std::numeric_limits<double>::infinity();
                infeasibleInterval.second = std::numeric_limits<double>::infinity();
            }
            else
            {
                double timeLow1, timeLow2, timeHigh1, timeHigh2;
                getPPTime(startVelocity, goalVelocity, distance, -acceleration1, &timeLow1, &timeLow2, &timeHigh1,
                          &timeHigh2);
                infeasibleInterval.first = timeLow1 + timeLow2;
                infeasibleInterval.second = timeHigh1 + timeHigh2;
                if (infeasibleInterval.second == std::numeric_limits<double>::infinity())
                {
                    std::cout << "infinity 1" << std::endl;
                    infeasibleInterval.first = 0.0;
                    infeasibleInterval.second = std::numeric_limits<double>::infinity();
                    return infeasibleInterval;
                    assert(false);
                }

                if (std::abs(startVelocity + timeHigh1 * -acceleration1) >= maxVelocity)
                {
                    infeasibleInterval.second =
                        getPLPTime(startVelocity, goalVelocity, distance, maxVelocity, -acceleration1);
                    if (infeasibleInterval.second == std::numeric_limits<double>::infinity())
                    {
                        std::cout << "infinity 2" << std::endl;
                        assert(false);
                    }
                }
            }
        }

        return infeasibleInterval;
    }

    double getMinTime(const StateVector &state1, const StateVector &state2,
                       double maxTime = std::numeric_limits<double>::infinity()) const
    {
        const Vector distances = state2.template head<dof>() - state1.template head<dof>();
        const Vector vel1 = state1.template tail<dof>();
        const Vector vel2 = state2.template tail<dof>();
        double minTime = 0.0;
        int limitDof = -1;  // DOF for which the min time but not the infeasible
                            // interval has been calculated yet
        std::pair<double, double> infeasibleIntervals[dof];
        Vector firstAccelerations;

        for (unsigned int i = 0; i < dof && minTime < maxTime; ++i)
        {
            const double time = getMinTime1D(state1[dof + i], state2[dof + i], distances[i], maxAccelerations_[i],
                                             maxVelocities_[i], maxTime, firstAccelerations[i]);
            /*
            adjustForInfeasibleIntervals(i, time, maxTime, state1.template tail<dof>(), state2.template tail<dof>(),
                                         distances, firstAccelerations, maxAccelerations_, maxVelocities_, minTime,
                                         infeasibleIntervals, limitDof);
                                         */
            adjustForInfeasibleIntervals(i, time, maxTime,
                                         vel1, vel2, distances, //does not change
                                         firstAccelerations,
                                         maxAccelerations_, maxVelocities_,  //does not change
                                         minTime,
                                         infeasibleIntervals, limitDof);
        }
        assert(minTime < std::numeric_limits<double>::infinity());
        return minTime;
    }

    /*
    void getGradient(const StateVector &state, const StateVector &grad, double h) const
    {
        //compute cost of state
        std::vector<double> oneDofTimesLoc(0.0, dof);
        std::vector<double> oneDofTimesVel(0.0, dof);

        std::vector<double> firstAccelLoc(0.0, dof);
        std::vector<double> firstAccelVel(0.0, dof);
        const Vector vels = state1.template tail<dof>();
        double maxTime = std::numeric_limits<double>::infinity();
        for (int i(0); i < dof; ++i)
        {
            double vel = vels[i];
            oneDofTimesLoc[i] = getMinTime1D(vel, vel, h,
                                             maxAccelerations_[i], maxVelocities_[i],
                                             maxTime,
                                             firstAccelLoc[i]);
        }
        for (int i(0); i < dof; ++i)
        {
            double vel = vels[i];
            oneDofTimesVel[i] = getMinTime1D(vel, vel+h, 0,
                                             maxAccelerations_[i], maxVelocities_[i],
                                             maxTime,
                                             firstAccelVel[i]);
        }

        Vector distances;
        //compute first elements of grad (location)
        for (int i(0); i < dof; ++i)
        {
            distances[i] = h;
            Vector firstAccelerations;
            double minTime = 0.0;
            for (unsigned int j = 0; j < dof && minTime < maxTime; ++j)
            {
                double time = (i==j) ? oneDofTimesLoc[i] : 0;
                firstAccelerations[j] = (i==j) ? firstAccelLoc[i] : maxAccelerations_[i];

                adjustForInfeasibleIntervals(j, time, maxTime,
                                             vel, vel, distances, //does not change
                                             firstAccelerations,
                                             maxAccelerations_, maxVelocities_,  //does not change
                                             minTime,
                                             infeasibleIntervals, limitDof);
            }
            grad[i] = minTime;
            distances[i] = 0;
        }


        VectorXd grad(curr_state.size());
        VectorXd state_plus(curr_state);
        double cost = getCost(curr_state);

        for (int dim = 0; dim < curr_state.size(); dim++)
        {
            state_plus(dim) = curr_state(dim) + h;
            grad(dim) = (getCost(state_plus) - cost) / (h);
            state_plus(dim) = curr_state(dim) - h;

        }

        const Vector distances = state2.template head<dof>() - state1.template head<dof>();
        const Vector vel1 = state1.template tail<dof>();
        const Vector vel2 = state2.template tail<dof>();
        double minTime = 0.0;
        int limitDof = -1;  // DOF for which the min time but not the infeasible
                            // interval has been calculated yet
        std::pair<double, double> infeasibleIntervals[dof];
        Vector firstAccelerations;

        for (unsigned int i = 0; i < dof && minTime < maxTime; ++i)
        {
            const double time = getMinTime1D(state1[dof + i], state2[dof + i], distances[i], maxAccelerations_[i],
                                             maxVelocities_[i], maxTime, firstAccelerations[i]);

            adjustForInfeasibleIntervals(i, time, maxTime,
                                         vel1, vel2, distances, //does not change
                                         firstAccelerations,
                                         maxAccelerations_, maxVelocities_,  //does not change
                                         minTime,
                                         infeasibleIntervals, limitDof);
        }
        assert(minTime < std::numeric_limits<double>::infinity());
        return minTime;
    }

    */

    double getMinTimeIfSmallerThan(const StateVector &state1, const StateVector &state2, double timeThreshold) const
    {
        const Vector distances = state2.template head<dof>() - state1.template head<dof>();
        double minTime = 0.0;
        int limitDof = -1;  // DOF for which the min time but not the infeasible
                            // interval has been calculated yet
        std::pair<double, double> infeasibleIntervals[dof];
        Vector firstAccelerations;
        double maxTimeDummy = std::numeric_limits<double>::infinity();

        for (unsigned int i = 0; i < dof && minTime < maxTimeDummy; ++i)
        {
            const double time = getMinTime1D(state1[dof + i], state2[dof + i], distances[i], maxAccelerations_[i],
                                             maxVelocities_[i], maxTimeDummy, firstAccelerations[i]);
            if(time >= timeThreshold)
            {
                return std::numeric_limits<double>::infinity();
            }

            adjustForInfeasibleIntervals(i, time, maxTimeDummy, state1.template tail<dof>(), state2.template tail<dof>(),
                                         distances, firstAccelerations, maxAccelerations_, maxVelocities_, minTime,
                                         infeasibleIntervals, limitDof);
            if(minTime >= timeThreshold)
            {
                return std::numeric_limits<double>::infinity();
            }
        }
        return minTime;
    }

    std::tuple<double, double, double>
    getMinTimeAndIntervals(const StateVector &state1, const StateVector &state2,
                           double maxTime = std::numeric_limits<double>::infinity()) const
    {
        const Vector distances = state2.template head<dof>() - state1.template head<dof>();
        double minTime = 0.0;
        int limitDof = -1;  // DOF for which the min time but not the infeasible
                            // interval has been calculated yet
        std::pair<double, double> infeasibleIntervals[dof];
        Vector firstAccelerations;

        for (unsigned int i = 0; i < dof && minTime < maxTime; ++i)
        {
            const double time = getMinTime1D(state1[dof + i], state2[dof + i], distances[i], maxAccelerations_[i],
                                             maxVelocities_[i], maxTime, firstAccelerations[i]);
            adjustForInfeasibleIntervals(i, time, maxTime, state1.template tail<dof>(), state2.template tail<dof>(),
                                         distances, firstAccelerations, maxAccelerations_, maxVelocities_, minTime,
                                         infeasibleIntervals, limitDof);
        }
        assert(minTime < std::numeric_limits<double>::infinity());
        return std::make_tuple(minTime, std::get<0>(infeasibleIntervals[limitDof]),
                               std::get<1>(infeasibleIntervals[limitDof]));
    }

    Trajectory getTrajectory(const StateVector &state1, const StateVector &state2, double time) const
    {
        Trajectory trajectory;
        for (unsigned int i = 0; i < dof; i++)
            trajectory[i] =
                getMinAcceleration(state1[i], state1[dof + i], state2[i], state2[dof + i], time, maxVelocities_[i]);
        return trajectory;
    }

    Trajectory getTrajectory(const StateVector &state1, const StateVector &state2) const
    {
        const double time = getMinTime(state1, state2);
        return getTrajectory(state1, state2, time);
    }


};
