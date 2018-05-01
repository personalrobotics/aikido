#pragma once

#include <tuple>
#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include "ompl/base/ProblemDefinition.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "aikido/planner/kinodynamics/sampler/RandomGenerator.hpp"

namespace ompl
{
    namespace base
    {   
        class MyInformedSampler : public InformedSampler
        {
        private:
            /// Holds the previous cost so that we know
            double prevCost_ = -1;

            /// Holds the index of the sample we've gotten
            int sampleIndex_ = 0;

            /// Number of samples to call at each iteration
            int sampleBatchSize_;

            /// Samples in the batch
            Eigen::MatrixXd batchSamples_;

            // Timing stuff
            std::chrono::high_resolution_clock::time_point tStart_;
            std::chrono::high_resolution_clock::time_point tNow_;
            std::vector<std::chrono::high_resolution_clock::duration> durationVec_;
            std::vector<double> costVec_;
            bool started_;

            ///
            /// Function to sample a state uniformly from the entire space before you have
            /// a solution
            ///
            /// @param statePtr Pointer to the state to sample
            ///
            virtual bool sampleFullSpace(State *statePtr);

            ///
            /// Function to sample uniformly from the informed subset
            ///
            /// @param statePtr Pointer to the state to sample
            /// @param maxCost Best cost found so far
            ///
            virtual bool sampleInformedSpace(State *statePtr, const Cost maxCost);

            ///
            /// Function to get a random sample in the limits
            ///
            /// @param max Max value for the dimension
            /// @param min Min value for the dimension
            /// @return Uniform sample in one of the dimensions between min and max
            ///
            double getRandomDimension(const double max, const double min);


#ifdef USE_NLOPT
            static double inequalConstraint(const std::vector<double> &x,
                                            std::vector<double> &grad, void *data);
#endif
        protected:
            ompl::base::SpaceInformationPtr si_;

            ompl::base::ProblemDefinitionPtr problem_;

            double levelSet_;
            double timelimit_;
            double batchTimelimit_;

            /// random generator
            UniformRealRandomGenerator uniRndGnr_;

            Eigen::VectorXd stateMax_;
            Eigen::VectorXd stateMin_;

            double acceptanceRatio_;
            uint64_t numAcceptedSamples_;
            uint64_t numRejectedSamples_; 

            ompl::base::State* tmpState_ ;

        public:
            ///
            /// Constructor
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            ///
            MyInformedSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem,
                              const double levelSet, const unsigned int maxNumberCalls,
                              const int sampleBatchSize, const double timelimit = 300)
              : InformedSampler(problem, maxNumberCalls)
              , sampleBatchSize_(sampleBatchSize)
              , si_(si)
              , problem_(problem)
              , levelSet_(levelSet)
              , timelimit_(timelimit)
              , batchTimelimit_(7200)
              , acceptanceRatio_(1.0)
              , numAcceptedSamples_(0)
              , numRejectedSamples_(0)
            {
                started_ = false;
                std::tie(stateMin_, stateMax_) = getStateLimits();
                tmpState_ = si_->allocState();
            }

            virtual ~MyInformedSampler()
            {
                si_->freeState(tmpState_);
            }

            ///
            /// Sample uniformly from the informed space
            ///
            /// @param statePtr Pointer of the state you're sampling
            /// @param maxCost Max cost of the informed subspace
            /// @return true if a sample is gotten false, if not
            ///
            virtual bool sampleUniform(State *statePtr, const Cost &maxCost) override;

            ///
            /// Just call sampleUniform(statePtr, maxCost) - there is no mincost
            ///
            /// @param statePtr Pointer of the state you're sampling
            /// @param maxCost Max cost of the informed subspace
            /// @param minCost Minimum cost of the informed subspace
            /// @return true if a sample is gotten false, if not
            ///
            virtual bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

            ///
            /// Function that lets the planner know if we have an informed measure
            ///
            /// @return True if we have implemented an informed measure, false if not
            ///
            virtual bool hasInformedMeasure() const override;

            ///
            /// Function to return the measure of the informed space
            ///
            /// @param currentCost - Current cost of the best path
            /// @return Measure of the informed space
            ///
            virtual double getInformedMeasure(const Cost &currentCost) const override;

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param numSamples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is
            /// displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const uint numSamples,
                                           std::chrono::high_resolution_clock::duration &duration);

            ///
            /// Get one sample for the problem space
            /// \param[out] sample
            /// \return True if the sample is in the level set
            ///
            virtual bool sampleInLevelSet(Eigen::VectorXd& sample)  = 0;

            ///
            /// Get the problem definition for the problem
            ///
            /// @return The problem definition
            ///
            ompl::base::ProblemDefinitionPtr problem() const
            {
                return problem_;
            }

            ///
            /// Get the space information pointer for the problem
            ///
            /// @return The space information
            ///
            ompl::base::SpaceInformationPtr si() const
            {
                return si_;
            }

            ///
            /// Update the level set of the problem definition
            ///
            /// @param levelSet The new level set
            ///
            virtual void updateLevelSet(const double levelSet)
            {
                levelSet_ = levelSet;
            }

            ///
            /// Get the state limits of the space
            ///
            /// @return Tuple(state_max, state_min)
            ///
            std::tuple<Eigen::VectorXd, Eigen::VectorXd> getStateLimits() const;

            ///
            /// Get the start state
            ///
            /// @return Start state defined in the constructor
            ///
            Eigen::VectorXd getStartState() const;

            ///
            /// Get the goal state
            ///
            /// @return Start state defined in the constructor
            ///
            Eigen::VectorXd getGoalState() const;

            ///
            /// Get the level set
            ///
            /// @return Get the level set of the cost function you want to sample from
            ///
            double getLevelSet() const
            {
                return levelSet_;
            }

            ///
            /// Get the dimension of the space
            ///
            /// @return Get the level set of the cost function you want to sample from
            ///
            uint getSpaceDimension() const
            {
                return si_->getStateSpace()->getDimension();
            }

            ///
            /// Get the cost for a specific state
            ///
            /// @param curr_state Current state to get the cost for
            /// @return Cost at that state
            ///
            virtual double getCost(const Eigen::VectorXd &curr_state) const;

            ///
            /// Determines if a sample is within the cost function level set
            ///
            /// @param state State to test
            /// @return Boolean that is true if it is in the level set
            ///
            virtual bool isInLevelSet(const Eigen::VectorXd &state, double& stateCost) const
            {
                return isInLevelSet(state, levelSet_, stateCost);
            }

            bool isInLevelSet(const Eigen::VectorXd &curr_state, double thresholdCost, double& stateCost) const;

            ///
            /// Get the gradient of the cost function at a specific state
            ///
            /// @param curr_state Current state to get the cost for
            /// @return Gradient of the function at the current state (same dimension as
            /// current state)
            ///
            virtual Eigen::VectorXd getGradient(const Eigen::VectorXd &curr_state);

            ///
            /// Get the Inverse Jacobian of the cost function at a specific state
            ///
            /// @param curr_state Current state to get the cost for
            /// @return Inverse Jacobian of the function at the current state (same
            /// dimension as current state)
            ///
            virtual Eigen::VectorXd getInvJacobian(const Eigen::VectorXd &curr_state) const;

            ///
            /// Determines if a sample is within the boundaries of the space
            ///
            /// @param state State to test
            /// @return Boolean that is true if it is in the boundaries of the space
            ///
            virtual bool isInBound(const Eigen::VectorXd &state) const;

            ///
            /// Get one random uniform sample from the space
            ///
            /// @return Random uniform vector from the space
            ///
            virtual Eigen::VectorXd getRandomSample();

            ///
            /// Surf down the cost function to get to the levelset
            ///
            /// @param start Starting state
            /// @return Path to the level set
            ///
            virtual Eigen::VectorXd newtonRaphson(const Eigen::VectorXd &start, double levelSet_);


            Eigen::VectorXd findSolutionInLevelSet(Eigen::VectorXd& init, double levelSet_);


            void setSingleSampleTimelimit(double timelimit) { timelimit_ = timelimit; }
            double getSingleSampleTimelimit() { return timelimit_; }
            void setBatchSampleTimelimit(double timelimit) { batchTimelimit_ = timelimit; }
            double getBatchSampleTimelimit() { return batchTimelimit_; }

            /// Get acceptance ratio
            double getAcceptanceRatio();

            void resetAcceptanceRatio();

        };  // MyInformedSampler

        using MyInformedSamplerPtr = std::shared_ptr<ompl::base::MyInformedSampler>;
    }  // base
}  // ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
