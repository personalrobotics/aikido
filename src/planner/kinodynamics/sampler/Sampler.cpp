#include <vector>
#include <memory>
#include <stdexcept>
#ifdef USE_NLOPT
#include <nlopt.hpp>
#endif
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <aikido/planner/kinodynamics/sampler/Sampler.hpp>
#include <aikido/planner/kinodynamics/dimt/Params.h>
#include <aikido/planner/kinodynamics/ompl/MyOptimizationObjective.hpp>


namespace ompl
{
    namespace base
    {
        ///
        /// Get the state limits of the space
        ///
        /// @return Tuple(state_max, state_min)
        ///
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> MyInformedSampler::getStateLimits() const
        {
            // Get the bounds from the vector
            const auto space = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();
            Eigen::VectorXd high = get_eigen_vector(space->getBounds().high);
            Eigen::VectorXd low = get_eigen_vector(space->getBounds().low);

            return std::make_tuple(low, high);
        }

        ///
        /// Get the start state
        ///
        /// @return Start state defined in the constructor
        ///
        Eigen::VectorXd MyInformedSampler::getStartState() const
        {
            Eigen::VectorXd startState(param.dimensions);
            get_eigen_vector(problem_->getStartState(0), startState);
            return startState;
        }

        ///
        /// Get the goal state
        ///
        /// @return Start state defined in the constructor
        ///
        Eigen::VectorXd MyInformedSampler::getGoalState() const
        {
            const auto goal_state = problem_->getGoal()->as<ompl::base::GoalState>();
            Eigen::VectorXd goalState(param.dimensions);
            get_eigen_vector(goal_state->getState(), goalState);
            return goalState;
        }

        Eigen::VectorXd MyInformedSampler::getGradient(const Eigen::VectorXd &curr_state)
        {

            // Assert that the matrix is not empty
            assert(curr_state.size() != 0 or curr_state.size() != 0);

            // Derivative constant
            double h = 0.001;

            // Loop through and calculate the gradients
            /*VectorXd grad(curr_state.size());
            VectorXd state_plus(curr_state);
            VectorXd state_min(curr_state);
            for (int dim = 0; dim < curr_state.size(); dim++)
            {
                state_plus(dim) = curr_state(dim) + h;
                state_min(dim) = curr_state(dim) - h;
                grad(dim) = (getCost(state_plus) - getCost(state_min)) / (2 * h);
                state_plus(dim) = curr_state(dim) - h;
                state_min(dim) = curr_state(dim) + h;
            }*/

            Eigen::VectorXd grad(curr_state.size());
            Eigen::VectorXd state_plus(curr_state);
            double cost = getCost(curr_state);

            for (int dim = 0; dim < curr_state.size(); dim++)
            {
                state_plus(dim) = curr_state(dim) + h;
                grad(dim) = (getCost(state_plus) - cost) / (h);
                state_plus(dim) = curr_state(dim) - h;

            }

            return grad;
        }

        Eigen::VectorXd MyInformedSampler::getInvJacobian(const Eigen::VectorXd &curr_state) const
        {
            // Assert that the matrix is not empty
            assert(curr_state.size() != 0 or curr_state.size() != 0);

            // Derivative constant
            double h = 0.001;

            // Loop through and calculate the gradients
            Eigen::VectorXd inv_jacobian(curr_state.size());
            for (int dim = 0; dim < curr_state.size(); dim++)
            {
                Eigen::VectorXd state_plus = curr_state;
                Eigen::VectorXd state_min = curr_state;
                state_plus(dim) = curr_state(dim) + h;
                state_min(dim) = curr_state(dim) - h;
                double delta = getCost(state_plus) - getCost(state_min);
                // std::cout << "DELTA " << delta << std::endl;
                if (delta == 0.0)
                {
                    inv_jacobian(dim) = 0.0;
                }
                else
                {
                    inv_jacobian(dim) = (2 * h) / (delta);
                }
            }

            return inv_jacobian;
        }

        ///
        /// Get the cost for a specific state
        ///
        /// @param curr_state Current state to get the cost for
        /// @return Cost at that state
        ///
        double MyInformedSampler::getCost(const Eigen::VectorXd &curr_state) const
        {

            // Assert that the problem definition has an optimization objective defined
            assert(problem_->hasOptimizationObjective());

            const ompl::base::State *start_state = problem_->getStartState(0);
            const ompl::base::State *goal_state = problem_->getGoal()->as<ompl::base::GoalState>()->getState();


            get_ompl_state(curr_state, tmpState_);

            const ompl::base::OptimizationObjectivePtr optim_obj = problem_->getOptimizationObjective();

            double cost =  optim_obj->motionCost(start_state, tmpState_).value() +
                    optim_obj->motionCost(tmpState_, goal_state).value();


            return cost;
        }

        bool MyInformedSampler::isInLevelSet(const Eigen::VectorXd &curr_state, double thresholdCost, double& stateCost) const
        {
            // Assert that the problem definition has an optimization objective defined
            assert(problem_->hasOptimizationObjective());
            const ompl::base::OptimizationObjectivePtr optim_obj = problem_->getOptimizationObjective();
            const ompl::base::DimtObjective* dimt_obj = dynamic_cast<ompl::base::DimtObjective*>(optim_obj.get());
            assert(dimt_obj!=nullptr);

            const ompl::base::State *start_state = problem_->getStartState(0);
            const ompl::base::State *goal_state = problem_->getGoal()->as<ompl::base::GoalState>()->getState();

            get_ompl_state(curr_state, tmpState_);

            double costToCome = dimt_obj->getCostIfSmallerThan(start_state, tmpState_, Cost(thresholdCost)).value();
            if (costToCome == std::numeric_limits<double>::infinity() )
            {
                return false;
            }
            double costToGo = dimt_obj->getCostIfSmallerThan(tmpState_, goal_state, Cost(thresholdCost-costToCome)).value();
            if (costToGo == std::numeric_limits<double>::infinity() )
            {
                return false;
            }
            stateCost = costToCome + costToGo;
            return true;
        }

        /// Can implement as many private functions as you want to help do the sampling
        double MyInformedSampler::getRandomDimension(const double max, const double min)
        {
            return uniRndGnr_.sample(min, max);
        }

        ///
        /// Function to sample a state uniformly from the entire space before you have
        /// a solution
        ///
        /// @param statePtr Pointer to the state to sample
        ///
        bool MyInformedSampler::sampleFullSpace(State *statePtr)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();

            double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;
            for (int i = 0; i < param.dimensions; i++)
            {
                val[i] = getRandomDimension(max_vals[i], min_vals[i]);
            }

            return true;
        }

        ///
        /// Function to sample uniformly from the informed subset
        ///
        /// @param statePtr Pointer to the state to sample
        /// @param maxCost Best cost found so far
        ///
        bool MyInformedSampler::sampleInformedSpace(State *statePtr, const Cost maxCost)
        {
            /*
            if (started_ == false)  // Set the timer when the function is called for the first time
            {
                started_ = true;
                // set time
                tStart_ = std::chrono::high_resolution_clock::now();
                // push back zero time and initial cost
                durationVec_.push_back(tStart_ - tStart_);
                costVec_.push_back(maxCost.value());
            }
            else
            {
                // Measure time and save it with cost
                tNow_ = std::chrono::high_resolution_clock::now();
                durationVec_.push_back(tNow_ - tStart_);
                costVec_.push_back(maxCost.value());
            }

            if (!same_cost(maxCost.value(), prevCost_) or sampleIndex_ >= sampleBatchSize_)
            {
                if (maxCost.value() != prevCost_)
                    updateLevelSet(maxCost.value());

                std::chrono::high_resolution_clock::duration duration;
                batchSamples_ = sample(sampleBatchSize_, duration);

                prevCost_ = maxCost.value();
                sampleIndex_ = 0;
            }

            auto sample = batchSamples_.row(sampleIndex_);

            double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;
            for (int i = 0; i < sample.size() - 1; i++)
            {
                val[i] = sample(i);
            }

            sampleIndex_++;

            return true;
            */
            updateLevelSet(maxCost.value());
            Eigen::VectorXd sample(si_->getStateDimension()+1);
            double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;
            if(false == sampleInLevelSet(sample))
            {
                return false;
            }

            for (int i = 0; i < sample.size() - 1; i++)
            {
                val[i] = sample(i);
            }

            return true;
        }

        ///
        /// Sample uniformly from the informed space
        ///
        /// @param statePtr Pointer of the state you're sampling
        /// @param maxCost Max cost of the informed subspace
        /// @return true if a sample is gotten false, if not
        ///
        bool MyInformedSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            if (maxCost.value() == std::numeric_limits<double>::infinity())
            {
                return sampleFullSpace(statePtr);
            }
            else
            {
                return sampleInformedSpace(statePtr, maxCost);
            }
        }

        ///
        /// Just call sampleUniform(statePtr, maxCost) - there is no mincost
        ///
        /// @param statePtr Pointer of the state you're sampling
        /// @param maxCost Max cost of the informed subspace
        /// @param minCost Minimum cost of the informed subspace
        /// @return true if a sample is gotten false, if not
        ///
        bool MyInformedSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            return sampleUniform(statePtr, maxCost);
        }

        ///
        /// Function that lets the planner know if we have an informed measure
        ///
        /// @return True if we have implemented an informed measure, false if not
        ///
        bool MyInformedSampler::hasInformedMeasure() const
        {
            return false;
        }

        ///
        /// Function to return the measure of the informed space
        ///
        /// @param currentCost - Current cost of the best path
        /// @return Measure of the informed space
        ///
        double MyInformedSampler::getInformedMeasure(const Cost &currentCost) const
        {
            return InformedSampler::space_->getMeasure();
        }

        ///
        /// Determines if a sample is within the boundaries of the space
        ///
        /// @param state State to test
        /// @return Boolean that is true if it is in the boundaries of the space
        ///
        bool MyInformedSampler::isInBound(const Eigen::VectorXd &state) const
        {

            for (unsigned int i = 0; i < state.size(); i++)
                if (state(i) > stateMax_(i) || state(i) < stateMin_(i))
                    return false;
            return true;
        }


        ///
        /// Get one random uniform sample from the space
        ///
        /// @return Random uniform vector of length size
        ///
        Eigen::VectorXd MyInformedSampler::getRandomSample()
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();

            int size = getSpaceDimension();
            Eigen::VectorXd sample(size);
            for (int i = 0; i < size; i++)
            {
                // Get a random distribution between the values of the joint
                double min = min_vals(i);
                double max = max_vals(i);
                sample(i) = uniRndGnr_.sample(min, max);
            }

            return sample;
        }

        ///
        /// Surf down the cost function to get to the levelset
        ///
        /// @param start Vector to start
        /// @return A state in the level set
        ///
        Eigen::VectorXd MyInformedSampler::newtonRaphson(const Eigen::VectorXd &start, double levelSet_)
        {
            Eigen::VectorXd end = start;
            double cost = getCost(end);

            int steps = 0;
            const int maxSteps = 10;
            int maxTrials = 10;
            while (cost > levelSet_ && maxTrials > 0)
            {
                double last_cost = cost;
                Eigen::VectorXd inv_jacobian = getInvJacobian(end);
                end = end - inv_jacobian * cost;
                cost = getCost(end);
                steps++;

                // If the number of steps reaches some threshold, start over
                if (steps > maxSteps)
                {
                    steps = 0;
                    end = getRandomSample();
                    cost = getCost(end);
                }

                maxTrials --;
            }

            return end;
        }

#ifdef USE_NLOPT
        double MyInformedSampler::inequalConstraint(const std::vector<double> &x,
                                        std::vector<double> &grad, void *data)
        {
            MyInformedSampler* sampler = (MyInformedSampler*)(data);
            Eigen::Map<const Eigen::VectorXd> state(x.data(), x.size());
            double delta = sampler->getCost(state) - sampler->getLevelSet();
            return delta;
        }

        static double min_func(const std::vector<double> &x,
                               std::vector<double> &grad, void *data)
        {
            double* ref = (double*) data;
            double dist = 0.0;
            for(unsigned i=0;i<x.size();++i)
            {
                dist += (x.data()[i] - ref[i])*(x.data()[i] - ref[i]);
            }
            return dist;
        }
#endif

        Eigen::VectorXd MyInformedSampler::findSolutionInLevelSet(Eigen::VectorXd& init, double levelSet_)
        {
            if(isInLevelSet(init, levelSet_))
            {
                return init;
            }

#ifdef USE_NLOPT
            nlopt::opt optProb( nlopt::LN_COBYLA, getSpaceDimension() );
            optProb.add_inequality_constraint(inequalConstraint, this, 1e-4);
            optProb.set_min_objective(min_func, init.data());
            optProb.set_maxeval(20);
            optProb.set_xtol_rel(1e-4);
            std::vector<double> x;
            for(unsigned int i=0;i<getSpaceDimension();++i)
            {
                x.push_back(init[i]);
            }
            std::vector<double> result = optProb.optimize(x);
            return Eigen::Map<Eigen::VectorXd>(result.data(), result.size());
#else
            return newtonRaphson(init, levelSet_);
#endif
        }

        Eigen::MatrixXd MyInformedSampler::sample(const uint numSamples,
                                                 std::chrono::high_resolution_clock::duration &duration)
        {
            // Reset acceptance rate
            resetAcceptanceRatio();

            Eigen::MatrixXd samples(numSamples, getSpaceDimension() + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;
            while (numAcceptedSamples_ < numSamples && timeElapsedDouble < batchTimelimit_)
            {
                Eigen::VectorXd newsample( getSpaceDimension() + 1 );
                if(sampleInLevelSet(newsample))
                {
                    std::cout << " num " << numAcceptedSamples_ << std::endl;
                    samples.row(numAcceptedSamples_-1) = newsample;
                }

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2-t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();
            }

            duration = timeElapsed;

            if(numAcceptedSamples_ < numSamples)
            {
                return samples.topRows(numAcceptedSamples_);
            }

            return samples;
        }

        double MyInformedSampler::getAcceptanceRatio()
        {
                acceptanceRatio_ = static_cast<double>(numAcceptedSamples_) /
                          static_cast<double>(numAcceptedSamples_+numRejectedSamples_);
            return acceptanceRatio_;
        }

        void MyInformedSampler::resetAcceptanceRatio()
        {
            acceptanceRatio_ = 1.0;
            numAcceptedSamples_ = 0;
            numRejectedSamples_ = 0;

        }

    }  // base
}  // oml
