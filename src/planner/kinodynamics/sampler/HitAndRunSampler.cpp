#include <aikido/planner/kinodynamics/sampler/HitAndRunSampler.h>

const bool VERBOSE = false;

namespace ompl
{
    namespace base
    {
        Eigen::VectorXd GibbsSampler::getRandomSample(double min, double max, const int dim)
        {
            // Updates the member variable of the class as well
            prev_sample_(dim) = uniRndGnr_.sample(min, max);
            return prev_sample_;
        }

        bool GibbsSampler::sampleInLevelSet(Eigen::VectorXd &sample)
        {
            double sampleCost = std::numeric_limits<double>::infinity();
            int trys = 0;
            bool inLevelset = true;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;
            int skip = 0;
            Eigen::VectorXd q;
            do
            {
                if (timeElapsedDouble >= timelimit_)
                {
                    inLevelset = false;
                    break;
                }
                if (trys > 10000)
                {
                    skip++;
                    trys = 0;
                }
                q = getRandomSample(stateMin_[(numAcceptedSamples_ + skip) % getSpaceDimension()],
                                    stateMax_[(numAcceptedSamples_ + skip) % getSpaceDimension()],
                                    (numAcceptedSamples_ + skip) % getSpaceDimension());
                trys++;

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2 - t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while (!isInLevelSet(q, sampleCost));
            prev_sample_ = q;

            sample << q, sampleCost;
            if (inLevelset)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }

            return inLevelset;
        }

        void GibbsSampler::updateLevelSet(const double level_set)
        {
            prev_sample_ = getStartState();
            MyInformedSampler::updateLevelSet(level_set);
            // std::cout << "Updated Level Set" << std::endl;
        }

        bool HitAndRunSampler::sampleInLevelSet(Eigen::VectorXd &sample)
        {
            double lamda_lower_bound = 0.0, lamda_upper_bound = 1.0;
            Eigen::VectorXd dir(getSpaceDimension());
            double newSampleCost = std::numeric_limits<double>::infinity();
            Eigen::VectorXd newSample;

            int trys = -1;
            bool retry = false;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;

            bool inLevelset = true;
            do
            {
                if (timeElapsedDouble > timelimit_)
                {
                    inLevelset = false;
                    break;
                }
                retry = false;
                if (trys > numOfTries_ || trys == -1)
                {
                    // Sample random direction in S^dim
                    double sum = 0;
                    for (uint i = 0; i < getSpaceDimension(); i++)
                    {
                        dir[i] = normRndGnr_.sample();
                        sum = sum + dir[i] * dir[i];
                    }
                    dir = dir / sum;
                    lamda_upper_bound = diagonalLength_;
                    lamda_lower_bound = -diagonalLength_;
                    // skip++;
                    trys = 0;
                }
                // Generate random sample along dir
                double lamda;
                double power = 4;
                if (lamda_lower_bound < 0 && lamda_upper_bound > 0)
                {
                    double rand_no1 = uniRndGnr_.sample(0, 1);
                    double rand_no2 = uniRndGnr_.sample(0, 1);
                    lamda = rand_no1 < 0.5 ? lamda_lower_bound * pow(rand_no2,1/power) : lamda_upper_bound * pow(rand_no2,1/power);
                }
                else
                {
                    double rand_no = uniRndGnr_.sample(0, 1);
                    if (lamda_lower_bound > 0)
                        lamda = pow(pow(lamda_lower_bound, power) +
                                     rand_no * (pow(lamda_upper_bound, power) - pow(lamda_lower_bound, power)),1/power);
                    else if (lamda_upper_bound < 0)
                      lamda = -pow(pow(lamda_upper_bound,power) +
                                        rand_no * (pow(lamda_lower_bound,power) -
                                                   pow(lamda_upper_bound,power)),1/power);
                }

                // lamda = uniRndGnr_.sample(lamda_lower_bound, lamda_upper_bound);
                newSample = prev_sample_ + lamda * dir;
                if (!isInBound(newSample))
                {
                    if (lamda > 0)
                        lamda_upper_bound = lamda;
                    else
                        lamda_lower_bound = lamda;
                    retry = true;
                }
                else if (!isInLevelSet(newSample, newSampleCost))
                    retry = true;
                trys++;

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2 - t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while (retry);

            sample << newSample, newSampleCost;
            pushPrevSamples(newSample);

            if (inLevelset)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }
            return inLevelset;
        }
    }  // namespace base
}  // namespace ompl
