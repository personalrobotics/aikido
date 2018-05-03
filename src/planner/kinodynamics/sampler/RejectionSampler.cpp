#include <limits>
#include <math.h>
#include <aikido/planner/kinodynamics/sampler/RejectionSampler.hpp>
#include <aikido/planner/kinodynamics/ompl/DimtStateSpace.hpp>

namespace ompl
{
    namespace base
    {
        bool RejectionSampler::sampleInLevelSet(Eigen::VectorXd& sample)
        {
            Eigen::VectorXd newSample = getRandomSample(stateMin_, stateMax_, getSpaceDimension());
            double newSampleCost = std::numeric_limits<double>::infinity();
            if (isInLevelSet(newSample, newSampleCost))
            {
                sample << newSample, newSampleCost;
                numAcceptedSamples_++;
                return true;
            }

            numRejectedSamples_++;

            /*
            static int cnt = 0;
            if (cnt++%100000 == 0)
                std::cout << "ACCEPTED  " << numAcceptedSamples_ << " REJECTED " << numRejectedSamples_ << std::endl;
            */

            return false;
        }

        /// Can implement as many private functions as you want to help do the sampling
        Eigen::VectorXd RejectionSampler::getRandomSample(const Eigen::VectorXd min, const Eigen::VectorXd max, const int size)
        {
            Eigen::VectorXd sample(size);

            for (int i = 0; i < size; i++)
            {
                sample(i) = uniRndGnr_.sample(min[i], max[i]);
            }
            return sample;
        }

        bool HierarchicalRejectionSampler::sampleInLevelSet(Eigen::VectorXd& sample)
        {
            Eigen::VectorXd newSample( getSpaceDimension() );
            double newSampleCost = std::numeric_limits<double>::infinity();
            HRS(0, dimension_ - 1, newSample, timelimit_);

            if (isInLevelSet(newSample, newSampleCost))
            {
                sample << newSample, newSampleCost;
                numAcceptedSamples_++;
                return true;
            }

            sample << newSample, newSampleCost;
            numRejectedSamples_++;
            return false;
        }

        ///
        /// Get one sample using a recursive algorithm of heirarchical rejection
        /// sampling
        ///
        /// @param start_index Start index of the hierarchical sample
        /// @param end_index End index of the hierarchical sample
        /// @param sample Reference to a sample that gets changed in place
        /// @return (c_start, c_goal)
        ///
        std::tuple<double, double> HierarchicalRejectionSampler::HRS(const int startIndex, const int endIndex,
                                                                     Eigen::VectorXd &sample, double timeLeft)
        {
            // Initialize the costs
            double cStart = std::numeric_limits<double>::infinity();
            double cGoal = std::numeric_limits<double>::infinity();

            // std::cout << " [ " << sample[0] << ", " << sample[1] <<  ", " <<
            //     sample[2] << ", " << sample[3] << " ]" << std::endl;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;

            if (startIndex == endIndex)
            {
                while (Cost(cStart) + Cost(cGoal) > getLevelSet())
                {
                    sampleLeaf(sample, startIndex);

                    cStart = calculateLeaf(getStartState(), sample, startIndex);
                    cGoal = calculateLeaf(sample, getGoalState(), startIndex);
                    //std::cout << getLevelSet() << std::endl;
                    t2 = std::chrono::high_resolution_clock::now();
                    timeElapsed = t2-t1;
                    timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();
                    if (timeElapsedDouble > timeLeft )
                    {
                        break;
                    }
                }
            }
            else
            {
                int mid_index = std::floor(startIndex + endIndex) / 2;
                double c_dash_start = std::numeric_limits<double>::infinity();
                double c_dash_goal = std::numeric_limits<double>::infinity();

                while (Cost(cStart) + Cost(cGoal) > getLevelSet())
                {
                    t2 = std::chrono::high_resolution_clock::now();
                    timeElapsed = t2-t1;
                    timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();
                    timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();
                    if (timeElapsedDouble > timeLeft )
                    {
                        break;
                    }

                    std::tie(cStart, cGoal) = HRS(startIndex, mid_index, sample, timeLeft-timeElapsedDouble);

                    t2 = std::chrono::high_resolution_clock::now();
                    timeElapsed = t2-t1;
                    timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

                    std::tie(c_dash_start, c_dash_goal) = HRS(mid_index + 1, endIndex, sample, timeLeft-timeElapsedDouble);
                    cStart =
                        combineCosts(getStartState(), sample, startIndex, mid_index, endIndex, cStart, c_dash_start);
                    cGoal =
                        combineCosts(sample, getGoalState(), startIndex, mid_index, endIndex, cGoal, c_dash_goal);


                }
            }

            return std::make_tuple(cStart, cGoal);
        }

        ///
        /// GeometricHierarchicalRejectionSampler
        ///

        ///
        /// Calculates the cost of a leaf node
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom
        /// @return Cost to go from x1 to x2
        ///
        double GeometricHierarchicalRejectionSampler::calculateLeaf(const Eigen::VectorXd &x1,
                                                                    const Eigen::VectorXd &x2, const int i)
        {
            return std::pow(x1[i] - x2[i], 2);
        }

        ///
        /// Combines the cost of two states
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom for first state
        /// @param m Mid degree of freedom
        /// @param j Index of  the degree of freedom of the second state
        /// @param c1 Cost one
        /// @param c2 Cost two
        /// @return Combination of the costs
        ///
        double GeometricHierarchicalRejectionSampler::combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                                   const int i, const int m, const int j,
                                                                   const double c1, const double c2) const
        {
            return c1 + c2;
        }

        ///
        /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
        ///
        /// @param sample A vector to the sample
        /// @param dof An index to the degree of freedom to sample
        /// @return A random vector in the space
        ///
        void GeometricHierarchicalRejectionSampler::sampleLeaf(Eigen::VectorXd &sample, const int dof)
        {
            sample[dof] = uniRndGnr_.sample(min_[dof], max_[dof]);
        }

        ///
        /// Dimt Hierarchical Rejection Sampler
        ///

        ///
        /// Calculates the cost of a leaf node
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom
        /// @return Cost to go from x1 to x2
        ///
        double DimtHierarchicalRejectionSampler::calculateLeaf(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                               const int i)
        {
            double cost, infeasible_min, infeasible_max;
            std::tie(cost, infeasible_min, infeasible_max) =
                dimt_->getMinTimeAndIntervals1Dof(x1[i*2], x1[i*2+1], x2[i*2], x2[i*2+1], i);

            infeasibleIntervals_[i] = std::make_pair(infeasible_min, infeasible_max);
            costs_[i] = cost;

            return costs_[i];
        }

        std::pair<std::size_t, double> max_in_range(const std::vector<double> &costs, const std::size_t start, const std::size_t end)
        {
            double max = -1;
            std::size_t index = -1;
            for (std::size_t i = start; i <= end; i++)
            {
                if (costs[i] > max)
                {
                    max = costs[i];
                    index = i;
                }
            }

            return std::make_pair(index, max);
        }

        std::pair<bool, double> find_infeasible_intervals(const std::vector<std::pair<double, double>> &intervals,
                                                          const double max_val, const std::size_t start, const std::size_t end)
        {
            for (std::size_t i = start; i <= end; i++)
            {
                if (max_val > std::get<0>(intervals[i]) and max_val < std::get<1>(intervals[i]))
                {
                    return std::make_pair(true, std::get<1>(intervals[i]));
                }
            }

            return std::make_pair(false, max_val);
        }

        ///
        /// Combines the cost of two states
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom for first state
        /// @param m Mid degree of freedom
        /// @param j Index of  the degree of freedom of the second state
        /// @param c1 Cost one
        /// @param c2 Cost two
        /// @return Combination of the costs
        ///
        double DimtHierarchicalRejectionSampler::combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                              const int i, const int m, const int j, const double c1,
                                                              const double c2) const
        {
            // Find the index and the max value of cost from the costs in the range
            std::size_t index;
            double max_val;
            std::tie(index, max_val) = max_in_range(costs_, i, j);

            bool is_invalid = true;
            while (is_invalid)
            {
                std::tie(is_invalid, max_val) = find_infeasible_intervals(infeasibleIntervals_, max_val, i, j);
            }
            return max_val;
        }

        ///
        /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
        ///
        /// @param sample A vector to the sample
        /// @param dof An index to the degree of freedom to sample
        /// @return A random vector in the space
        ///
        void DimtHierarchicalRejectionSampler::sampleLeaf(Eigen::VectorXd &sample, const int dof)
        {
            sample[dof * 2] = uniRndGnr_.sample(min_[dof * 2], max_[dof * 2]);
            sample[dof * 2 + 1] = uniRndGnr_.sample(min_[dof * 2 + 1], max_[dof * 2 + 1]);
        }
    }  // namespace ompl
}  // namespace base
