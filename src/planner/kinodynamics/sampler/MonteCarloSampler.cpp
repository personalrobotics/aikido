#include <math.h> /* exp, tanh, log */
#include <limits>
#include <algorithm>
#include <aikido/planner/kinodynamics/dimt/Params.h>
#include <aikido/planner/kinodynamics/sampler/MonteCarloSampler.hpp>

namespace
{
    // Verbose constant
    const bool VERBOSE = true;

    ///
    /// Sigmoid function
    ///
    /// @param x Input
    /// @param a Controls shape of sigmoid
    /// @param c Controls shape of sigmoid
    /// @return Output of sigmoid function
    ///
    inline double sigmoid(const double &x, const double &a = 200, const double &c = 0)
    {
        return 1 / (1 + exp(-a * (x - c)));
    }

}  // namespace

namespace ompl
{
    namespace base
    {
        bool LangevinSampler::sampleInLevelSet(Eigen::VectorXd& sample)
        {
            return false;
        }

        ///
        /// Function to determine if any of the joint limits are violated
        /// @param sample Sample to check
        /// @return Boolean that is true if any are in violation
        ///
        bool MonteCarloSampler::anyDimensionInViolation(const Eigen::VectorXd &sample) const
        {
            const std::tuple<Eigen::VectorXd, Eigen::VectorXd> limits = getStateLimits();
            const auto min_vals = std::get<0>(limits);
            const auto max_vals = std::get<1>(limits);

            for (uint i = 0; i < sample.size(); i++)
            {
                if (sample[i] > max_vals[i] or sample[i] < min_vals[i])
                {
                    return true;
                }
            }

            return false;
        }

        ///
        /// Get the energy of the state from the cost function
        ///
        /// @param curr_state Current state to get the energy for
        /// @return Energy of the function
        ///
        double MonteCarloSampler::getEnergy(const Eigen::VectorXd &curr_state) const
        {
            const double cost = getCost(curr_state);
            // double E_grad = tanh(cost);
            const double E_grad = log(1 + log(1 + cost));
            const double E_informed = 100 * sigmoid(cost - getLevelSet());

            double E_region = 0;
            std::tuple<Eigen::VectorXd, Eigen::VectorXd> limits = getStateLimits();
            for (int i = 0; i < curr_state.size(); i++)
            {
                E_region += 100 * sigmoid(std::get<0>(limits)(i) - curr_state(i));  // Lower Limits
                E_region += 100 * sigmoid(curr_state(i) - std::get<1>(limits)(i));  // Higher Limits
            }
            return E_region + E_grad + E_informed;

            //return E_grad + E_informed;
        }

        double MonteCarloSampler::getEnergy(double cost) const
        {
            const double E_grad = log(1 + log(1 + cost));
            const double E_informed = 100 * sigmoid(cost - getLevelSet());
            return E_grad + E_informed;
        }

        double MonteCarloSampler::getEnergyGradient(double cost) const
        {
            static const double costStep = 0.0001;
            double energy1 = getEnergy(cost);
            double energy2 = getEnergy(cost + costStep);
            return (energy2 - energy1) / costStep;
        }

        Eigen::VectorXd MonteCarloSampler::getGradient(const Eigen::VectorXd &curr_state)
        {
            /*
            Eigen::VectorXd grad = MyInformedSampler::getGradient(curr_state);
            double cost = getCost(curr_state);
            double energyGrad = getEnergyGradient(cost);
            return energyGrad * grad;
            */
            return MyInformedSampler::getGradient(curr_state);
        }

        ///
        /// Get the probability of the state from the cost function
        ///
        /// @param energy Energy of the state
        /// @return Probability of the state
        ///
        double MonteCarloSampler::getProb(const double energy) const
        {
            return exp(-energy);
        }

        //
        // Get the probability of the state from the cost function
        //
        // @param curr_state Current state to get the energy for
        // @return Probability of the state
        //
        double MonteCarloSampler::getProb(const Eigen::VectorXd &curr_state) const
        {
            return exp(-MonteCarloSampler::getEnergy(curr_state));
        }

        //
        // Surf down the cost function to get to the levelset
        //
        // @param alpha Learning rate
        // @return Path to the level set
        //
        Eigen::VectorXd MonteCarloSampler::gradDescent(const double alpha)
        {
            // If the number of steps reaches some threshold, start over
            const double thresh = 20;
            Eigen::VectorXd start = MonteCarloSampler::getRandomSample();
            double cost = getCost(start);
            double prev_cost = cost;

            int steps = 0;
            while (cost > getLevelSet())
            {
                Eigen::VectorXd grad = getGradient(start);
                start = start - alpha * grad;

                cost = getCost(start);

                if (VERBOSE)
                    std::cout << cost << std::endl;

                steps++;


                // if(steps > thresh || cost > prev_cost)
                if (steps > thresh || cost > 10 * getLevelSet() || grad.norm() < 0.001)
                {
                    if (VERBOSE)
                        std::cout << "Restarting!" << std::endl;
                    // recursing gives segfaults so just change the start position instead
                    // return grad_descent(alpha);
                    start = MonteCarloSampler::getRandomSample();
                    steps = 0;
                }
                prev_cost = cost;
            }

            return start;
        }


        //
        // Get a normal random vector for the momentum
        //
        // @return A vector of momentum sampled from a random distribution
        //
        Eigen::VectorXd MonteCarloSampler::sampleNormal(const double mean, const double sigma)
        {
            int size = getSpaceDimension();
            Eigen::VectorXd sample(size);
            for (int i = 0; i < size; i++)
            {
                sample(i) = normRndGnr_.sample(mean, sigma);
            }

            return sample;
        }

        //
        // HMCSampler
        //

        bool HMCSampler::sampleInLevelSet(Eigen::VectorXd& sample)
        {      
            // last sample
            Eigen::VectorXd q = Eigen::VectorXd(getStartState().size());
            double sampleCost = std::numeric_limits<double>::infinity();
            q << lastSample_;
            const int maxTrialNum = 10;
            int currentTrialNum = 0;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;

            bool inLevelSet = true;
            do
            {
                if (timeElapsedDouble > timelimit_)
                {
                    inLevelSet = false;
                    break;
                }

                currentTrialNum++;
                if( currentTrialNum > maxTrialNum )
                {
                    currentTrialNum = 0;
                    currentStep_ = -1;
                }

                if (getCurrentStep() < 0)
                {
                    Eigen::VectorXd start = getRandomSample();
                    q = findSolutionInLevelSet(start, getLevelSet());

                }

                // Make a half step for momentum at the beginning
                Eigen::VectorXd grad = getGradient(q);
                //if (VERBOSE)
                //    std::cout << "Got the gradient" << std::endl;

                /*
                // Ensure that the gradient isn't two large
                while (grad.maxCoeff() > 1e2)
                {
                    if (VERBOSE)
                        std::cout << "WARNING: Gradient too high" << std::endl;

                    Eigen::VectorXd start = getRandomSample();
                    q = findSolutionInLevelSet(start, getLevelSet());
                    grad = getGradient(q);
                }
                */

                updateCurrentStep();

                // Sample the momentum and set up the past and current state and momentum
                Eigen::VectorXd q_last = q;
                Eigen::VectorXd p = MonteCarloSampler::sampleNormal(0, getSigma());
                Eigen::VectorXd p_last = p;

                //if (VERBOSE)
                //    std::cout << "Sampled the momentum" << std::endl;

                p = p - getEpsilon() * grad / 2;

                // Alternate Full steps for q and p
                for (int i = 0; i < getL(); i++)
                {
                    grad = getGradient(q);
                    q = q + getEpsilon() * p;
                    if (i != getL())
                        p = p - getEpsilon() * grad;
                }

                //if (VERBOSE)
                //    std::cout << "Integrated Along momentum" << std::endl;

                // Make a half step for momentum at the end
                grad = getGradient(q);
                p = p - getEpsilon() * grad / 2;

                // Negate the momentum at the end of the traj to make proposal
                // symmetric
                p = -p;

                // Evaluate potential and kinetic energies at start and end of traj
                double U_last = getEnergy(q_last);
                double K_last = p_last.norm() / 2;
                double U_proposed = getEnergy(q);
                double K_proposed = p_last.norm() / 2;

                //if (VERBOSE)
                //    std::cout << "Got energies" << std::endl;

                // Accept or reject the state at the end of trajectory
                double alpha = std::min(1.0, std::exp(U_last - U_proposed + K_last - K_proposed));
                if (uniRndGnr_.sample() > alpha)
                {
                    q = q_last;
                }

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2-t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while(!isInLevelSet(q, sampleCost) );

            if(inLevelSet)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }

            sample << q, sampleCost;

            lastSample_ << q;
            if (getCurrentStep() >= getSteps())
            {
                updateCurrentStep(-1);
            }

            //std::cout << "ACCEPTED  " << numAcceptedSamples_ << " REJECTED " << numRejectedSamples_ << std::endl;

            return inLevelSet;
        }

        bool MCMCSampler::sampleInLevelSet(Eigen::VectorXd& sample)
        {
            // last sample
            Eigen::VectorXd q = Eigen::VectorXd(getStartState().size());
            double sampleCost = std::numeric_limits<double>::infinity();
            q << lastSample_;
            const int maxTrialNum = 10;
            int currentTrialNum = 0;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;

            bool inLevelSet = true;
            do
            {
                if (timeElapsedDouble > timelimit_)
                {
                    inLevelSet = false;
                    break;
                }

                currentTrialNum++;
                if( currentTrialNum > maxTrialNum )
                {
                    currentTrialNum = 0;
                    currentStep_ = -1;
                }

                if (getCurrentStep() < 0)
                {
                    Eigen::VectorXd start = getRandomSample();
                    q = findSolutionInLevelSet(start, getLevelSet());
                }

                // Make a half step for momentum at the beginning
                Eigen::VectorXd grad = getGradient(q);
                //if (VERBOSE)
                //    std::cout << "Got the gradient" << std::endl;

                /*
                // Ensure that the gradient isn't two large
                while (grad.maxCoeff() > 1e2)
                {        
                    if (VERBOSE)
                        std::cout << "WARNING: Gradient too high" << std::endl;

                    Eigen::VectorXd start = getRandomSample();
                    q = findSolutionInLevelSet(start, getLevelSet());
                    grad = getGradient(q);
                }*/

                updateCurrentStep();

                Eigen::VectorXd q_proposed = q + sampleNormal(0, getSigma());
                double prob_proposed = getProb(q_proposed);
                double prob_before = getProb(q);

                // if(prob_proposed / prob_before >= rand_uni() and
                //    // !any_dimensions_in_violation(q_proposed))
                if (prob_proposed / prob_before >= uniRndGnr_.sample())
                {
                    q = q_proposed;
                }

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2-t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while(!isInLevelSet(q, sampleCost) );

            if(inLevelSet)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }

            sample << q, sampleCost;

            lastSample_ << q;
            if (getCurrentStep() >= getSteps())
            {
                updateCurrentStep(-1);
            }

            return inLevelSet;
        }

    }  // base
}  // ompl
