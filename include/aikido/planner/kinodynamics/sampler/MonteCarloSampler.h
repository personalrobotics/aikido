#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/kinodynamics/sampler/Sampler.h>

namespace ompl
{
    namespace base
    {
        class LangevinSampler : public MyInformedSampler
        {
        public:
            LangevinSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                              const unsigned int maxNumberCalls, const int sampleBatchSize, const double step)
              : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize), step_(step), currentStep_(-1)
            {
                lastSample_ = Eigen::VectorXd(getSpaceDimension());
            }

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

        protected:
            double step_;
            int currentStep_;
            /// Last sample
            Eigen::VectorXd lastSample_;
        };

        class MonteCarloSampler : public MyInformedSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Step size of gradient descent
            ///
            MonteCarloSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                              const unsigned int maxNumberCalls, const int sampleBatchSize, const double alpha)
              : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize), alpha_(alpha), currentStep_(-1)
            {
                lastSample_ = Eigen::VectorXd(getSpaceDimension());
            }

            ///
            /// Get alpha - learning rate for gradient descent
            ///
            double getAlpha() const
            {
                return alpha_;
            }

            ///
            /// Surf down the cost function to get to the levelset
            ///
            /// @param alpha Learning rate
            /// @return Path to the level set
            ///
            virtual Eigen::VectorXd gradDescent(const double alpha = 0.01);

            ///
            /// Get the energy of the state from the cost function
            ///
            /// @param curr_state Current state to get the energy for
            /// @return Energy of the state
            ///
            virtual double getEnergy(const Eigen::VectorXd &curr_state) const;

            double getEnergyGradient(double cost) const;

            double getEnergy(double cost) const;

            virtual Eigen::VectorXd getGradient(const Eigen::VectorXd &curr_state);

            ///
            /// Get the probability of the state from the cost function
            ///
            /// @param energy Energy of the state
            /// @return Probability of the state
            ///
            virtual double getProb(const double energy) const;

            ///
            /// Get the probability of the state from the cost function
            ///
            /// @param curr_state Current state to get the energy for
            /// @return Probability of the state
            ///
            virtual double getProb(const Eigen::VectorXd &curr_state) const;

            ///
            /// Get a normal random vector for the momentum
            ///
            /// @param mean Mean of the normal distribution
            /// @paramm sigma Sigma of the normal distribution
            /// @return A vector of momentum sampled from a random distribution
            ///
            Eigen::VectorXd sampleNormal(const double mean, const double sigma);


            ///
            /// Get current step
            ///
            int getCurrentStep() const
            {
                return currentStep_;
            }

            ///
            /// Get last sample
            ///
            Eigen::VectorXd getLastSample() const
            {
                return lastSample_;
            }

            ///
            /// Update current step
            ///
            void updateCurrentStep(int currentStep)
            {
                currentStep_ = currentStep;
            }

            ///
            /// Update current step
            ///
            void updateCurrentStep()
            {
                currentStep_++;
            }

        protected:
            /// Learning rate for gradient descent
            double alpha_;

            NormalRealRandomGenerator normRndGnr_;

            int currentStep_;

            /// Last sample
            Eigen::VectorXd lastSample_;

            ///
            /// Function to determine if any of the joint limits are violated
            /// @param sample Sample to check
            /// @return Boolean that is true if any are in violation
            ///
            bool anyDimensionInViolation(const Eigen::VectorXd &sample) const;
        };  // MonteCarloSampler

        class HMCSampler : public MonteCarloSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Learning rate for gradient descent
            /// @param L Distance of integration for HMC step
            /// @param epsilon Integration constant for HMC
            /// @param sigma Sampling the momentum as a normal distribution
            /// @param steps Number of steps to run HMC for each chain
            ///
            HMCSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                       const unsigned int maxNumberCalls, const int sampleBatchSize, const double &alpha,
                       const double &L, const double &epsilon, const double &sigma, const int &steps)
              : MonteCarloSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize, alpha)
              , L_(L)
              , epsilon_(epsilon)
              , sigma_(sigma)
              , steps_(steps)
            {
            }

            ///
            /// Get L - Distance of integration for HMC step
            ///
            double getL() const
            {
                return L_;
            }

            ///
            /// Get epsilon - Integration constant for HMC
            ///
            double getEpsilon() const
            {
                return epsilon_;
            }

            ///
            /// Get sigma - Sampling the momentum as a normal distribution
            ///
            double getSigma() const
            {
                return sigma_;
            }

            ///
            /// Get steps - Number of steps to run HMC for each chain
            ///
            double getSteps() const
            {
                return steps_;
            }

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;


        private:
            /// Distance of integration for HMC step
            double L_;

            /// Integration constant for HMC
            double epsilon_;

            /// Sigma for sampling the momentum as a normal distribution
            double sigma_;

            /// Number of steps to run HMC
            double steps_;

        };

        class MCMCSampler : public MonteCarloSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Learning rate for gradient descent
            /// @param sigma Sigma for sampling the step
            /// @param steps Number of steps to run MCMC for each chain
            ///
            MCMCSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                        const unsigned int maxNumberCalls, const int sampleBatchSize, const double alpha,
                        const double sigma, const double steps)
              : MonteCarloSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize, alpha)
              , sigma_(sigma)
              , steps_(steps)
            {
            }

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

            ///
            /// Get Sigma for sampling the step
            ///
            double getSigma() const
            {
                return sigma_;
            }

            ///
            /// Get number of steps to run for each MCMC chain
            ///
            double getSteps() const
            {
                return steps_;
            }


        private:
            /// Sigma for sampling the step
            double sigma_;

            /// Number of steps to run MCMC for each chain
            double steps_;

        };
    }  // base
}  // ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_MONTE_CARLO_SAMPLERS_
